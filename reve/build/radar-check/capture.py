import os
import termios
import select
import time
import struct
import csv
import json
from pathlib import Path

output = Path(__file__).resolve().parent

def open_uart(path, speed):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    original = termios.tcgetattr(fd)
    attrs = termios.tcgetattr(fd)
    attrs[0] = attrs[1] = attrs[3] = 0
    attrs[2] = speed | termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[4] = attrs[5] = speed
    attrs[6][termios.VMIN] = attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd, original

def read_for(fd, seconds):
    result = bytearray()
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if select.select([fd], [], [], min(0.05, max(0, end - time.monotonic())))[0]:
            result.extend(os.read(fd, 65536))
            if fd == cli and result.rstrip().endswith(b"mmwDemo:/>"):
                break
    return bytes(result)

cli, cli_original = open_uart('/dev/ttyUSB0', termios.B115200)
data, data_original = open_uart('/dev/ttyUSB1', termios.B921600)
try:
    with (output / 'configuration.log').open('w') as log:
        for line in (output / 'capture_aop.cfg').read_text().splitlines():
            command = line.strip()
            if not command or command.startswith('%'):
                continue
            os.write(cli, (command + '\n').encode())
            response = read_for(cli, 2 if command == 'sensorStart' else 0.15).decode(errors='replace')
            log.write(command + '\n' + response + '\n')
            log.flush()
            print(command, repr(response), flush=True)
            if 'Error' in response or 'not recognized' in response or 'Done' not in response:
                raise RuntimeError('Configuration rejected: ' + command)
    raw = read_for(data, 10)
    (output / 'raw_uart.bin').write_bytes(raw)
finally:
    os.write(cli, b'sensorStop\n')
    print('STOP', read_for(cli, 1).decode(errors='replace'), flush=True)
    for fd, original in [(cli, cli_original), (data, data_original)]:
        termios.tcsetattr(fd, termios.TCSANOW, original)
        os.close(fd)

magic = bytes.fromhex('0201040306050807')
offset = 0
frames = []
rows = []
while True:
    offset = raw.find(magic, offset)
    if offset < 0 or offset + 40 > len(raw):
        break
    header = struct.unpack_from('<8I', raw, offset + 8)
    version, length, platform, frame, cycles, count, ntlv, subframe = header
    if length < 40 or length > 1000000:
        offset += 1
        continue
    if offset + length > len(raw):
        break
    cursor = offset + 40
    points, side = [], []
    types = []
    for _ in range(ntlv):
        kind, size = struct.unpack_from('<II', raw, cursor)
        cursor += 8
        if cursor + size > offset + length:
            raise RuntimeError('Invalid TLV length')
        payload = raw[cursor:cursor + size]
        types.append(kind)
        if kind == 1:
            points = list(struct.iter_unpack('<4f', payload))
        if kind == 7:
            side = list(struct.iter_unpack('<2h', payload))
        cursor += size
    if len(points) != count:
        raise RuntimeError('Point count mismatch')
    for index, point in enumerate(points):
        snr, noise = (side[index][0] * 0.1, side[index][1] * 0.1) if index < len(side) else ('', '')
        rows.append((frame, cycles, index, *point, snr, noise))
    frames.append({'frame': frame, 'points': count, 'types': types, 'bytes': length})
    offset += length
with (output / 'points.csv').open('w') as stream:
    writer = csv.writer(stream)
    writer.writerow(['frame', 'cpu_cycles', 'point', 'x_m', 'y_m', 'z_m', 'doppler_mps', 'snr_db', 'noise_db'])
    writer.writerows(rows)
summary = {'raw_bytes': len(raw), 'frames': len(frames), 'points': len(rows), 'tlv_types': sorted({kind for frame in frames for kind in frame['types']}), 'missing_frames': sum(max(0, b['frame'] - a['frame'] - 1) for a, b in zip(frames, frames[1:])), 'first_frames': frames[:5]}
(output / 'summary.json').write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
print('FIRST POINTS', rows[:5])
