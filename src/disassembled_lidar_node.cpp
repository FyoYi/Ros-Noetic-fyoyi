/*
 * 拆机激光雷达 ROS 驱动节点。
 *
 * 通过 termios 打开串口，解析 360 拖地机器人“小章鱼”拆机激光雷达的
 * MiniLidar 数据包，并发布标准 sensor_msgs/LaserScan 到 /scan。
 * 协议按固定 60 字节包处理：包头 55 AA，类型 0x23，每包 16 个采样点，
 * 每个采样点 3 字节，距离取低 14 位，并过滤无效采样点。
 *
 * 这个节点只负责串口读取和 LaserScan 发布，不负责 TF、建图或导航。
 */
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;

uint16_t readLe16(const std::vector<uint8_t>& data, size_t offset)
{
  return static_cast<uint16_t>(data[offset]) | (static_cast<uint16_t>(data[offset + 1]) << 8);
}

double normalizeDegrees(double angle)
{
  while (angle >= 360.0)
  {
    angle -= 360.0;
  }
  while (angle < 0.0)
  {
    angle += 360.0;
  }
  return angle;
}

speed_t baudToTermios(int baud)
{
  switch (baud)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B230400;
  }
}

class DisassembledLidarNode
{
public:
  DisassembledLidarNode()
    : private_nh_("~")
  {
    private_nh_.param("port", port_, std::string("/dev/ttyUSB0"));
    private_nh_.param("baud", baud_, 230400);
    private_nh_.param("frame_id", frame_id_, std::string("laser"));
    private_nh_.param("scan_topic", scan_topic_, std::string("/scan"));
    private_nh_.param("range_min", range_min_, 0.05);
    private_nh_.param("range_max", range_max_, 8.0);
    private_nh_.param("angle_min", angle_min_, -kPi);
    private_nh_.param("angle_max", angle_max_, kPi);
    private_nh_.param("scan_bins", scan_bins_, 720);
    private_nh_.param("publish_partial_scan", publish_partial_scan_, false);
    private_nh_.param("inverted", inverted_, false);
    private_nh_.param("angle_offset", angle_offset_, 0.0);
    private_nh_.param("sync0", sync0_, 0x55);
    private_nh_.param("sync1", sync1_, 0xAA);
    private_nh_.param("debug_raw", debug_raw_, true);
    private_nh_.param("sample_offset", sample_offset_, 8);
    private_nh_.param("sample_stride", sample_stride_, 3);
    private_nh_.param("packet_size", packet_size_, 60);
    private_nh_.param("packet_type", packet_type_, 0x23);
    private_nh_.param("samples_per_packet", samples_per_packet_, 0x10);

    if (scan_bins_ < 90)
    {
      scan_bins_ = 90;
    }

    ranges_.assign(scan_bins_, std::numeric_limits<float>::infinity());
    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 2);

    openSerial();
  }

  ~DisassembledLidarNode()
  {
    if (fd_ >= 0)
    {
      close(fd_);
    }
  }

  void spin()
  {
    ros::WallRate idle_rate(200.0);
    uint8_t read_buffer[512];
    while (ros::ok())
    {
      const ssize_t nread = read(fd_, read_buffer, sizeof(read_buffer));
      if (nread > 0)
      {
        bytes_read_ += static_cast<uint64_t>(nread);
        buffer_.insert(buffer_.end(), read_buffer, read_buffer + nread);
        collectRawDebug(read_buffer, static_cast<size_t>(nread));
        parseBuffer();
      }
      else if (nread < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      {
        ROS_ERROR_THROTTLE(2.0, "DisassembledLidar: serial read failed: %s", std::strerror(errno));
      }
      printDiagnostics();
      ros::spinOnce();
      idle_rate.sleep();
    }
  }

private:
  void openSerial()
  {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
      ROS_FATAL("DisassembledLidar: failed to open %s: %s", port_.c_str(), std::strerror(errno));
      ros::shutdown();
      return;
    }

    termios options;
    if (tcgetattr(fd_, &options) != 0)
    {
      ROS_FATAL("DisassembledLidar: tcgetattr failed: %s", std::strerror(errno));
      ros::shutdown();
      return;
    }

    cfmakeraw(&options);
    const speed_t speed = baudToTermios(baud_);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &options) != 0)
    {
      ROS_FATAL("DisassembledLidar: tcsetattr failed: %s", std::strerror(errno));
      ros::shutdown();
      return;
    }

    tcflush(fd_, TCIOFLUSH);
    ROS_INFO("DisassembledLidar: opened %s at %d baud, publishing %s", port_.c_str(), baud_, scan_topic_.c_str());
  }

  void parseBuffer()
  {
    const size_t max_buffer = 8192;
    if (buffer_.size() > max_buffer)
    {
      buffer_.erase(buffer_.begin(), buffer_.end() - max_buffer);
    }

    while (buffer_.size() >= static_cast<size_t>(packet_size_))
    {
      size_t start = 0;
      while (start + 1 < buffer_.size() &&
             !(buffer_[start] == static_cast<uint8_t>(sync0_) && buffer_[start + 1] == static_cast<uint8_t>(sync1_)))
      {
        ++start;
      }
      if (start > 0)
      {
        buffer_.erase(buffer_.begin(), buffer_.begin() + start);
      }
      if (buffer_.size() < static_cast<size_t>(packet_size_))
      {
        return;
      }

      if (buffer_[2] != static_cast<uint8_t>(packet_type_) ||
          buffer_[3] != static_cast<uint8_t>(samples_per_packet_))
      {
        ++bad_packets_;
        buffer_.erase(buffer_.begin());
        continue;
      }

      std::vector<uint8_t> packet(buffer_.begin(), buffer_.begin() + packet_size_);
      buffer_.erase(buffer_.begin(), buffer_.begin() + packet_size_);
      ++packets_seen_;
      parsePacket(packet);
    }
  }

  void collectRawDebug(const uint8_t* data, size_t size)
  {
    if (!debug_raw_)
    {
      return;
    }
    for (size_t i = 0; i < size && raw_debug_sample_.size() < 96; ++i)
    {
      raw_debug_sample_.push_back(data[i]);
    }
    for (size_t i = 1; i < size; ++i)
    {
      const uint16_t pair = (static_cast<uint16_t>(data[i - 1]) << 8) | data[i];
      ++pair_counts_[pair];
    }
  }

  void parsePacket(const std::vector<uint8_t>& packet)
  {
    const int lsn = packet[3];
    const uint16_t fsa_raw = readLe16(packet, 6);
    const uint16_t lsa_raw = readLe16(packet, 56);

    const double first_angle = normalizeDegrees((static_cast<double>(fsa_raw) - 0xA000) / 64.0);
    const double last_angle = normalizeDegrees((static_cast<double>(lsa_raw) - 0xA000) / 64.0);
    double delta = last_angle >= first_angle ? last_angle - first_angle : 360.0 - first_angle + last_angle;
    if (lsn > 0)
    {
      delta /= static_cast<double>(lsn);
    }
    else
    {
      delta = 0.0;
    }

    bool crossed_zero = false;
    int valid_points = 0;
    for (int i = 0; i < lsn; ++i)
    {
      const size_t sample_offset = static_cast<size_t>(sample_offset_) + static_cast<size_t>(i) * sample_stride_;
      if (sample_offset + 1 >= packet.size())
      {
        continue;
      }
      const uint8_t distance_lsb = packet[sample_offset];
      const uint8_t distance_msb = packet[sample_offset + 1];
      if ((distance_msb & 0xC0) == 0x80)
      {
        continue;
      }

      const uint16_t distance_mm = readLe16(packet, sample_offset) & 0x3FFF;
      const float range = static_cast<float>(static_cast<double>(distance_mm) * 0.001);
      if (range < range_min_ || range > range_max_)
      {
        continue;
      }

      double angle_deg = normalizeDegrees(first_angle + static_cast<double>(i) * delta + angle_offset_ * 180.0 / kPi);
      if (inverted_)
      {
        angle_deg = normalizeDegrees(360.0 - angle_deg);
      }
      crossed_zero = crossed_zero || (first_angle > last_angle);
      insertRange(angle_deg * kPi / 180.0, range);
      ++valid_points;
    }
    valid_points_seen_ += static_cast<uint64_t>(valid_points);

    if (publish_partial_scan_ || crossed_zero)
    {
      publishScan();
      ++scans_published_;
      std::fill(ranges_.begin(), ranges_.end(), std::numeric_limits<float>::infinity());
    }
  }

  void printDiagnostics()
  {
    const ros::WallTime now = ros::WallTime::now();
    if (!last_diag_wall_.isZero() && (now - last_diag_wall_).toSec() < 2.0)
    {
      return;
    }
    last_diag_wall_ = now;
    ROS_INFO("DisassembledLidar: bytes=%llu packets=%llu scans=%llu valid_points=%llu bad=%llu buffer=%zu",
             static_cast<unsigned long long>(bytes_read_),
             static_cast<unsigned long long>(packets_seen_),
             static_cast<unsigned long long>(scans_published_),
             static_cast<unsigned long long>(valid_points_seen_),
             static_cast<unsigned long long>(bad_packets_),
             buffer_.size());
    if (debug_raw_ && packets_seen_ == 0 && !raw_debug_sample_.empty())
    {
      ROS_WARN("DisassembledLidar: no packets found with sync %02X %02X; raw sample: %s",
               sync0_ & 0xFF,
               sync1_ & 0xFF,
               formatBytes(raw_debug_sample_).c_str());
      ROS_WARN("DisassembledLidar: common byte pairs: %s", formatCommonPairs().c_str());
      raw_debug_sample_.clear();
    }
  }

  std::string formatBytes(const std::vector<uint8_t>& bytes) const
  {
    std::ostringstream stream;
    stream << std::hex << std::uppercase;
    for (size_t i = 0; i < bytes.size(); ++i)
    {
      if (i > 0)
      {
        stream << ' ';
      }
      const int value = bytes[i];
      if (value < 16)
      {
        stream << '0';
      }
      stream << value;
    }
    return stream.str();
  }

  std::string formatCommonPairs() const
  {
    std::vector<std::pair<uint16_t, uint64_t>> pairs;
    for (size_t i = 0; i < pair_counts_.size(); ++i)
    {
      if (pair_counts_[i] > 0)
      {
        pairs.push_back(std::make_pair(static_cast<uint16_t>(i), pair_counts_[i]));
      }
    }
    std::sort(pairs.begin(), pairs.end(), [](const std::pair<uint16_t, uint64_t>& a,
                                             const std::pair<uint16_t, uint64_t>& b) {
      return a.second > b.second;
    });

    std::ostringstream stream;
    stream << std::hex << std::uppercase;
    const size_t count = std::min<size_t>(8, pairs.size());
    for (size_t i = 0; i < count; ++i)
    {
      if (i > 0)
      {
        stream << ", ";
      }
      const uint16_t pair = pairs[i].first;
      const int first = (pair >> 8) & 0xFF;
      const int second = pair & 0xFF;
      if (first < 16)
      {
        stream << '0';
      }
      stream << first << ' ';
      if (second < 16)
      {
        stream << '0';
      }
      stream << second << std::dec << "(" << pairs[i].second << ")" << std::hex << std::uppercase;
    }
    return stream.str();
  }

  void insertRange(double angle, float range)
  {
    while (angle > kPi)
    {
      angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
      angle += 2.0 * kPi;
    }
    if (angle < angle_min_ || angle > angle_max_)
    {
      return;
    }

    const double ratio = (angle - angle_min_) / (angle_max_ - angle_min_);
    int index = static_cast<int>(std::round(ratio * static_cast<double>(scan_bins_ - 1)));
    index = std::max(0, std::min(scan_bins_ - 1, index));
    if (!std::isfinite(ranges_[index]) || range < ranges_[index])
    {
      ranges_[index] = range;
    }
  }

  void publishScan()
  {
    const ros::Time stamp = ros::Time::now();
    sensor_msgs::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = frame_id_;
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = (angle_max_ - angle_min_) / static_cast<double>(scan_bins_ - 1);
    scan.time_increment = 0.0;
    scan.scan_time = last_scan_stamp_.isZero() ? 0.1 : (stamp - last_scan_stamp_).toSec();
    scan.range_min = range_min_;
    scan.range_max = range_max_;
    scan.ranges = ranges_;
    scan_pub_.publish(scan);
    last_scan_stamp_ = stamp;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher scan_pub_;
  int fd_ = -1;
  std::vector<uint8_t> buffer_;
  std::vector<float> ranges_;
  ros::Time last_scan_stamp_;
  ros::WallTime last_diag_wall_;
  std::vector<uint8_t> raw_debug_sample_;
  std::vector<uint64_t> pair_counts_ = std::vector<uint64_t>(65536, 0);
  uint64_t bytes_read_ = 0;
  uint64_t packets_seen_ = 0;
  uint64_t scans_published_ = 0;
  uint64_t valid_points_seen_ = 0;
  uint64_t bad_packets_ = 0;

  std::string port_;
  std::string frame_id_;
  std::string scan_topic_;
  int baud_ = 230400;
  int scan_bins_ = 720;
  int sync0_ = 0xAA;
  int sync1_ = 0x55;
  int sample_offset_ = 8;
  int sample_stride_ = 3;
  int packet_size_ = 60;
  int packet_type_ = 0x23;
  int samples_per_packet_ = 0x10;
  double range_min_ = 0.05;
  double range_max_ = 8.0;
  double angle_min_ = -kPi;
  double angle_max_ = kPi;
  double angle_offset_ = 0.0;
  bool publish_partial_scan_ = false;
  bool inverted_ = false;
  bool debug_raw_ = true;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disassembled_lidar_node");
  DisassembledLidarNode node;
  node.spin();
  return 0;
}
