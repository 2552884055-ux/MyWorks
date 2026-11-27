#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <limits>
#include <regex>
#include <string>
#include <system_error>

#include <ros/ros.h>
#include <sensor_msgs/Range.h>

namespace {

constexpr std::array<const char*, 4> kDirections{{"front", "rear", "left", "right"}};
constexpr std::size_t kMaxLineBytes = 80;
const std::regex kLegacyFrame(R"([ \t]*(?:-1|[0-9]{1,6})(?:[ \t]*,[ \t]*(?:-1|[0-9]{1,6})){3}[ \t\r]*)");
const std::regex kTof1Frame(
    R"(\$(TOF1,([0-9]{1,10}),([0-9]{1,10}),([0-9]{1,6}),([0-3]),([0-9]{1,6}),([0-3]),([0-9]{1,6}),([0-3]),([0-9]{1,6}),([0-3]))\*([0-9A-Fa-f]{4})\r?)");

std::uint16_t tofCrc(const std::string& payload) {
  std::uint16_t crc = 0xFFFF;
  for (const unsigned char byte : payload) {
    crc ^= static_cast<std::uint16_t>(byte) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
  }
  return crc;
}

using Distances = std::array<int, kDirections.size()>;

bool parseTof1(const std::string& line, Distances& distances_mm) {
  std::smatch fields;
  if (!std::regex_match(line, fields, kTof1Frame)) {
    ROS_WARN_THROTTLE(5.0, "Ignoring malformed TOF1 serial line");
    return false;
  }
  if (tofCrc(fields[1].str()) != std::stoul(fields[12].str(), nullptr, 16)) {
    ROS_WARN_THROTTLE(5.0, "Ignoring TOF1 frame with invalid CRC");
    return false;
  }
  constexpr auto max_counter = std::numeric_limits<std::uint32_t>::max();
  if (std::stoull(fields[2].str()) > max_counter || std::stoull(fields[3].str()) > max_counter) {
    ROS_WARN_THROTTLE(5.0, "Ignoring TOF1 frame with an out-of-range counter");
    return false;
  }

  for (std::size_t i = 0; i < distances_mm.size(); ++i) {
    const auto distance_field = 4 + 2 * i;
    distances_mm[i] = fields[distance_field + 1].str() == "0"
                          ? std::stoi(fields[distance_field].str()) : -1;
  }
  return true;
}

bool parseLegacyCsv(const std::string& line, Distances& distances_mm) {
  if (!std::regex_match(line, kLegacyFrame)) {
    ROS_WARN_THROTTLE(5.0, "Ignoring malformed legacy ToF serial line");
    return false;
  }
  std::size_t start = 0;
  for (std::size_t i = 0; i < distances_mm.size(); ++i) {
    distances_mm[i] = std::stoi(line.substr(start));
    if (i + 1 < distances_mm.size()) {
      start = line.find(',', start) + 1;
    }
  }
  return true;
}

speed_t serialSpeed(int baud) {
  switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: return B0;
  }
}

[[noreturn]] void serialError(const char* operation) {
  throw std::system_error(errno, std::generic_category(), operation);
}

class SerialPort {
 public:
  SerialPort(const std::string& port, speed_t speed) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      serialError("open");
    }
    try {
      termios attrs{};
      if (tcgetattr(fd_, &attrs) < 0) {
        serialError("tcgetattr");
      }
      cfmakeraw(&attrs);
      attrs.c_iflag &= ~(IXON | IXOFF | IXANY);
      attrs.c_cflag &= ~(CSTOPB | CRTSCTS);
      attrs.c_cflag |= CLOCAL | CREAD;
      if (cfsetispeed(&attrs, speed) < 0 || cfsetospeed(&attrs, speed) < 0 ||
          tcsetattr(fd_, TCSANOW, &attrs) < 0 || tcflush(fd_, TCIFLUSH) < 0) {
        serialError("configure serial port");
      }
    } catch (...) {
      ::close(fd_);
      throw;
    }
  }

  ~SerialPort() { ::close(fd_); }
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  ssize_t readSome(char* buffer, std::size_t capacity) const {
    pollfd event{fd_, POLLIN, 0};
    const int ready = ::poll(&event, 1, 250);
    if (ready == 0 || (ready < 0 && errno == EINTR)) {
      return 0;
    }
    if (ready < 0) {
      serialError("poll");
    }
    if (event.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      throw std::system_error(std::make_error_code(std::errc::io_error), "serial device disconnected");
    }
    const ssize_t count = ::read(fd_, buffer, capacity);
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      return 0;
    }
    if (count < 0) {
      serialError("read");
    }
    if (count == 0) {
      throw std::system_error(std::make_error_code(std::errc::io_error), "serial device disconnected");
    }
    return count;
  }

 private:
  int fd_;
};

void publishRanges(const Distances& distances_mm, const std::array<ros::Publisher, 4>& publishers,
                 const std::array<std::string, 4>& frame_ids, sensor_msgs::Range message) {
  message.header.stamp = ros::Time::now();
  for (std::size_t i = 0; i < publishers.size(); ++i) {
    const int mm = distances_mm[i];
    message.header.frame_id = frame_ids[i];
    message.range = mm == -1 ? std::numeric_limits<float>::quiet_NaN() : mm / 1000.0f;
    publishers[i].publish(message);
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "tof");
  ros::NodeHandle private_nh("~");
  const auto port = private_nh.param<std::string>("port", "/dev/ttyUSB0");
  const auto protocol = private_nh.param<std::string>("protocol", "tof1");
  const int baud = private_nh.param("baud", 115200);
  const auto min_range = private_nh.param("min_range", 0.03);
  const auto max_range = private_nh.param("max_range", 2.0);
  const auto field_of_view = private_nh.param("field_of_view", 0.44);
  const speed_t speed = serialSpeed(baud);

  if (protocol != "tof1" && protocol != "legacy_csv") {
    ROS_FATAL("Unsupported ToF serial protocol: %s", protocol.c_str());
    return 1;
  }
  const auto parse = protocol == "tof1" ? parseTof1 : parseLegacyCsv;

  if (speed == B0) {
    ROS_FATAL("Unsupported serial baud rate: %d", baud);
    return 1;
  }
  if (!(std::isfinite(min_range) && std::isfinite(max_range) &&
        0.0 <= min_range && min_range < max_range &&
        0.0 < field_of_view && field_of_view <= std::acos(-1.0))) {
    ROS_FATAL("Invalid ToF range or field of view parameters");
    return 1;
  }

  std::array<ros::Publisher, 4> publishers;
  std::array<std::string, 4> frame_ids;
  for (std::size_t i = 0; i < kDirections.size(); ++i) {
    const std::string direction = kDirections[i];
    publishers[i] = private_nh.advertise<sensor_msgs::Range>(direction, 10);
    frame_ids[i] = private_nh.param(direction + "_frame", "tof_" + direction);
  }
  sensor_msgs::Range message;
  message.radiation_type = sensor_msgs::Range::INFRARED;
  message.min_range = min_range;
  message.max_range = max_range;
  message.field_of_view = field_of_view;

  while (ros::ok()) {
    try {
      const SerialPort serial(port, speed);
      ROS_INFO("Reading ToF ranges from %s at %d baud (%s)", port.c_str(), baud, protocol.c_str());
      std::string line;
      line.reserve(kMaxLineBytes);
      bool discard = false;
      std::array<char, 1024> bytes;
      while (ros::ok()) {
        const ssize_t count = serial.readSome(bytes.data(), bytes.size());
        for (ssize_t i = 0; i < count; ++i) {
          const char byte = bytes[i];
          if (byte == '\n') {
            if (!discard) {
              Distances distances_mm;
              if (parse(line, distances_mm)) {
                publishRanges(distances_mm, publishers, frame_ids, message);
              }
            }
            line.clear();
            discard = false;
          } else if (!discard) {
            if (line.size() == kMaxLineBytes) {
              line.clear();
              discard = true;
            } else {
              line.push_back(byte);
            }
          }
        }
      }
    } catch (const std::system_error& error) {
      ROS_WARN_THROTTLE(5.0, "ToF serial error on %s: %s", port.c_str(), error.what());
    }
    if (ros::ok()) {
      ros::WallDuration(1.0).sleep();
    }
  }
  return 0;
}
