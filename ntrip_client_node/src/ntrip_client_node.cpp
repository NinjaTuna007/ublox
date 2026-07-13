// Copyright 2023 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <curl/curl.h>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rtcm_msgs/msg/message.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "ntrip_client_node/visibility_control.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace ublox_dgnss
{
struct CurlHandle
{
  CURL * handle;
  CurlHandle()
  : handle(curl_easy_init()) {}
  ~CurlHandle() {curl_easy_cleanup(handle);}
};

class NTRIPClientNode : public rclcpp::Node
{
public:
  NTRIP_CLIENT_NODE_PUBLIC
  explicit NTRIPClientNode(const rclcpp::NodeOptions & options)
  : Node("ntrip_client",
      rclcpp::NodeOptions(options)),
    curlHandle_(std::make_shared<CurlHandle>())
  {
    RCLCPP_INFO(this->get_logger(), "starting %s", get_name());

    declare_parameter("use_https", true);
    declare_parameter("host", "ntrip.data.gnss.ga.gov.au");
    declare_parameter("port", 443);
    declare_parameter("mountpoint", "MBCH00AUS0");
    declare_parameter("username", "noname");
    declare_parameter("password", "password");
    declare_parameter("log_level", "INFO");
    declare_parameter("maxage_conn", 30);
    declare_parameter("ntrip_version", "");
    declare_parameter("gga_fix_topic", "");
    declare_parameter("gga_send_period_sec", 1.0);

    use_https_ = get_parameter("use_https").as_bool();
    host_ = get_parameter("host").as_string();
    port_ = get_parameter("port").as_int();
    mountpoint_ = get_parameter("mountpoint").as_string();
    username_ = get_parameter("username").as_string();
    password_ = get_parameter("password").as_string();
    log_level_ = get_parameter("log_level").as_string();
    maxage_conn_ = get_parameter("maxage_conn").as_int();
    ntrip_version_ = get_parameter("ntrip_version").as_string();
    gga_fix_topic_ = get_parameter("gga_fix_topic").as_string();
    gga_send_period_sec_ = get_parameter("gga_send_period_sec").as_double();

    // Initialize pending values before registering callback to avoid race
    pending_use_https_ = use_https_;
    pending_host_ = host_;
    pending_port_ = port_;
    pending_mountpoint_ = mountpoint_;
    pending_username_ = username_;
    pending_password_ = password_;
    pending_log_level_ = log_level_;
    pending_maxage_conn_ = maxage_conn_;
    pending_ntrip_version_ = ntrip_version_;

    // Register parameter change callback for runtime reconfiguration
    parameters_callback_handle_ =
      this->add_on_set_parameters_callback(
      std::bind(
        &NTRIPClientNode::on_set_parameters_callback,
        this, _1));

    std::string url = ConnectionUrl();

    RCLCPP_INFO(this->get_logger(), "ntrip connection url: '%s'", url.c_str());

    std::string userpwd = username_ + ":" + password_;
    RCLCPP_DEBUG(this->get_logger(), "userpwd: '%s'", userpwd.c_str());

    // Create the publisher for rtcm_msgs::msg::Message
    rtcm_pub_ = this->create_publisher<rtcm_msgs::msg::Message>("/ntrip_client/rtcm", 10);

    if (!gga_fix_topic_.empty()) {
      const auto fix_qos = rclcpp::SensorDataQoS();
      fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gga_fix_topic_, fix_qos,
        std::bind(&NTRIPClientNode::fix_callback, this, _1));
      RCLCPP_INFO(
        this->get_logger(), "NTRIP GGA enabled from fix topic '%s'",
        gga_fix_topic_.c_str());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "gga_fix_topic is empty; VRS casters such as SWEPOS may not send RTCM");
    }

    curl_global_init(CURL_GLOBAL_DEFAULT);

    auto handle = curlHandle_->handle;
    if (handle) {
      // Static curl options that never change
      curl_easy_setopt(handle, CURLOPT_HTTP09_ALLOWED, true);
      curl_easy_setopt(handle, CURLOPT_HTTP_VERSION, CURL_HTTP_VERSION_1_0);
      curl_easy_setopt(handle, CURLOPT_USERAGENT, "NTRIP ros2/ublox_dgnss");
      curl_easy_setopt(handle, CURLOPT_FAILONERROR, true);
      curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, &NTRIPClientNode::WriteCallback);
      curl_easy_setopt(handle, CURLOPT_WRITEDATA, this);
      curl_easy_setopt(handle, CURLOPT_XFERINFOFUNCTION, &NTRIPClientNode::XferInfoCallback);
      curl_easy_setopt(handle, CURLOPT_XFERINFODATA, this);
      curl_easy_setopt(handle, CURLOPT_NOPROGRESS, 0L);
      curl_easy_setopt(handle, CURLOPT_TCP_KEEPALIVE, 1L);
      curl_easy_setopt(handle, CURLOPT_SOCKOPTFUNCTION, &NTRIPClientNode::SockoptCallback);
      curl_easy_setopt(handle, CURLOPT_SOCKOPTDATA, this);

      // Dynamic options extracted to ApplyCurlOptions() for runtime reconfiguration
      ApplyCurlOptions();

      // Start the streaming in a separate thread
      streaming_exit_.store(false);
      streamingThread_ = std::thread(&NTRIPClientNode::DoStreaming, this);
    }
  }

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  std::shared_ptr<CurlHandle> curlHandle_;
  struct curl_slist * http_headers_ = nullptr;
  std::thread streamingThread_;

  std::atomic<bool> streaming_exit_{false};

  // Runtime parameter reconfiguration support
  std::mutex params_mutex_;
  std::atomic<bool> reconfigure_needed_{false};

  // Pending parameter values (written by param callback, read by streaming thread)
  bool pending_use_https_;
  std::string pending_host_;
  int pending_port_;
  std::string pending_mountpoint_;
  std::string pending_username_;
  std::string pending_password_;
  std::string pending_log_level_;
  long pending_maxage_conn_;
  std::string pending_ntrip_version_;

  // NTRIP castor connection
  bool use_https_;
  std::string host_;
  int port_;
  std::string mountpoint_;
  std::string username_;
  std::string password_;
  std::string log_level_;
  long maxage_conn_;
  std::string ntrip_version_;
  std::string gga_fix_topic_;
  double gga_send_period_sec_{1.0};

  rclcpp::Publisher<rtcm_msgs::msg::Message>::SharedPtr rtcm_pub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

  std::mutex fix_mutex_;
  sensor_msgs::msg::NavSatFix latest_fix_;
  bool has_fix_{false};

  std::mutex stream_mutex_;
  curl_socket_t stream_socket_{CURL_SOCKET_BAD};
  std::atomic<bool> stream_socket_valid_{false};

  std::string ConnectionUrl()
  {
    std::string url;
    if (use_https_) {
      url = "https://" + host_ + ":" + std::to_string(port_) + "/" + mountpoint_;
    } else {
      url = "http://" + host_ + ":" + std::to_string(port_) + "/" + mountpoint_;
    }

    return url;
  }

  // Parameter callback for runtime reconfiguration
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::lock_guard<std::mutex> lock(params_mutex_);

    for (const auto & param : parameters) {
      const auto & name = param.get_name();

      if (name == "use_https") {
        pending_use_https_ = param.as_bool();
      } else if (name == "host") {
        if (param.as_string().empty()) {
          result.successful = false;
          result.reason = "host cannot be empty";
          return result;
        }
        pending_host_ = param.as_string();
      } else if (name == "port") {
        auto port = param.as_int();
        if (port < 1 || port > 65535) {
          result.successful = false;
          result.reason = "port must be between 1 and 65535";
          return result;
        }
        pending_port_ = static_cast<int>(port);
      } else if (name == "mountpoint") {
        if (param.as_string().empty()) {
          result.successful = false;
          result.reason = "mountpoint cannot be empty";
          return result;
        }
        pending_mountpoint_ = param.as_string();
      } else if (name == "username") {
        pending_username_ = param.as_string();
      } else if (name == "password") {
        pending_password_ = param.as_string();
      } else if (name == "log_level") {
        pending_log_level_ = param.as_string();
      } else if (name == "maxage_conn") {
        pending_maxage_conn_ = param.as_int();
      } else if (name == "ntrip_version") {
        pending_ntrip_version_ = param.as_string();
      }
    }

    reconfigure_needed_.store(true);
    RCLCPP_INFO(this->get_logger(), "Parameter change queued for next streaming cycle");

    return result;
  }

  void UpdateHttpHeaders()
  {
    curl_slist_free_all(http_headers_);
    http_headers_ = nullptr;

    if (!ntrip_version_.empty()) {
      const std::string header = "Ntrip-Version: " + ntrip_version_;
      http_headers_ = curl_slist_append(http_headers_, header.c_str());
    }

    curl_easy_setopt(
      curlHandle_->handle, CURLOPT_HTTPHEADER,
      http_headers_ != nullptr ? http_headers_ : nullptr);
  }

  // Apply dynamic curl options (called from constructor and DoStreaming)
  void ApplyCurlOptions()
  {
    auto handle = curlHandle_->handle;
    std::string url = ConnectionUrl();
    std::string userpwd = username_ + ":" + password_;

    curl_easy_setopt(handle, CURLOPT_URL, url.c_str());
    curl_easy_setopt(handle, CURLOPT_USERPWD, userpwd.c_str());

    if (log_level_ != "INFO") {
      curl_easy_setopt(handle, CURLOPT_VERBOSE, 1L);
    } else {
      curl_easy_setopt(handle, CURLOPT_VERBOSE, 0L);
    }

    curl_easy_setopt(handle, CURLOPT_MAXAGE_CONN, maxage_conn_);
    UpdateHttpHeaders();
  }

  static int SockoptCallback(
    void * clientp, curl_socket_t curlfd, curlsocktype purpose)
  {
    auto * node = reinterpret_cast<NTRIPClientNode *>(clientp);
    if (purpose == CURLSOCKTYPE_IPCXN) {
      std::lock_guard<std::mutex> lock(node->stream_mutex_);
      node->stream_socket_ = curlfd;
      node->stream_socket_valid_.store(true);
    }
    return CURL_SOCKOPT_OK;
  }

  void ClearStreamSocket()
  {
    std::lock_guard<std::mutex> lock(stream_mutex_);
    stream_socket_ = CURL_SOCKET_BAD;
    stream_socket_valid_.store(false);
  }

  void fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(fix_mutex_);
    latest_fix_ = *msg;
    has_fix_ = true;
  }

  static std::string LatDdToDmm(double decimal_degrees)
  {
    const double abs_deg = std::fabs(decimal_degrees);
    double degrees_whole = 0.0;
    const double degrees_frac = std::modf(abs_deg, &degrees_whole);
    double minutes_whole = 0.0;
    const double minutes_frac = std::modf(degrees_frac * 60.0, &minutes_whole);
    const int subminutes = static_cast<int>(minutes_frac * 1e5);

    char buf[16];
    std::snprintf(
      buf, sizeof(buf), "%02d%02d.%05d",
      static_cast<int>(degrees_whole),
      static_cast<int>(minutes_whole),
      subminutes);
    return buf;
  }

  static std::string LonDdToDmm(double decimal_degrees)
  {
    const double abs_deg = std::fabs(decimal_degrees);
    double degrees_whole = 0.0;
    const double degrees_frac = std::modf(abs_deg, &degrees_whole);
    double minutes_whole = 0.0;
    const double minutes_frac = std::modf(degrees_frac * 60.0, &minutes_whole);
    const int subminutes = static_cast<int>(minutes_frac * 1e5);

    char buf[16];
    std::snprintf(
      buf, sizeof(buf), "%03d%02d.%05d",
      static_cast<int>(degrees_whole),
      static_cast<int>(minutes_whole),
      subminutes);
    return buf;
  }

  static uint8_t NmeaChecksum(const std::string & sentence_no_checksum)
  {
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence_no_checksum.size(); ++i) {
      checksum ^= static_cast<uint8_t>(sentence_no_checksum[i]);
    }
    return checksum;
  }

  std::string BuildGgaSentence()
  {
    sensor_msgs::msg::NavSatFix fix;
    {
      std::lock_guard<std::mutex> lock(fix_mutex_);
      if (!has_fix_) {
        return "";
      }
      fix = latest_fix_;
    }

    const double stamp_sec =
      static_cast<double>(fix.header.stamp.sec) +
      static_cast<double>(fix.header.stamp.nanosec) * 1e-9;
    const time_t sec = static_cast<time_t>(stamp_sec);
    const double frac = stamp_sec - std::floor(stamp_sec);
    const int centiseconds = static_cast<int>(frac * 100.0);

    struct tm utc_tm {};
    gmtime_r(&sec, &utc_tm);

    char utc_buf[16];
    std::snprintf(
      utc_buf, sizeof(utc_buf), "%02d%02d%02d.%02d",
      utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec, centiseconds);

    const char lat_dir = fix.latitude < 0.0 ? 'S' : 'N';
    const char lon_dir = fix.longitude < 0.0 ? 'W' : 'E';
    const std::string lat = LatDdToDmm(fix.latitude);
    const std::string lon = LonDdToDmm(fix.longitude);

    int fix_quality = 0;
    switch (fix.status.status) {
      case sensor_msgs::msg::NavSatStatus::STATUS_FIX:
        fix_quality = 1;
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX:
        fix_quality = 2;
        break;
      case sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX:
        fix_quality = 5;
        break;
      default:
        fix_quality = 0;
        break;
    }

    const std::string sentence_no_checksum =
      "$GPGGA," + std::string(utc_buf) + "," + lat + "," + lat_dir + "," + lon + "," +
      lon_dir + "," + std::to_string(fix_quality) +
      ",05,1.0,100.0,M,-32.0,M,,0000";
    const uint8_t checksum = NmeaChecksum(sentence_no_checksum);

    char checksum_buf[8];
    std::snprintf(checksum_buf, sizeof(checksum_buf), "%02x", checksum);
    return sentence_no_checksum + "*" + checksum_buf + "\r\n";
  }

  void SendGgaIfPossible()
  {
    if (gga_fix_topic_.empty()) {
      return;
    }

    const std::string sentence = BuildGgaSentence();
    if (sentence.empty()) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for NavSatFix on '%s' before sending GGA", gga_fix_topic_.c_str());
      return;
    }

    curl_socket_t socket = CURL_SOCKET_BAD;
    {
      std::lock_guard<std::mutex> lock(stream_mutex_);
      if (!stream_socket_valid_.load()) {
        return;
      }
      socket = stream_socket_;
    }

    const ssize_t sent = ::send(
      socket, sentence.data(), sentence.size(), MSG_NOSIGNAL);
    if (sent < 0) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to send GGA to NTRIP caster");
      return;
    }

    RCLCPP_DEBUG(get_logger(), "Sent GGA to NTRIP caster (%zd bytes)", sent);
  }

  void GgaSenderLoop(std::atomic<bool> * active)
  {
    const auto period = std::chrono::duration<double>(gga_send_period_sec_);
    while (active->load() && !streaming_exit_.load()) {
      SendGgaIfPossible();
      std::this_thread::sleep_for(period);
    }
  }

  static int XferInfoCallback(
    void * userdata, curl_off_t /*dltotal*/, curl_off_t /*dlnow*/,
    curl_off_t /*ultotal*/, curl_off_t /*ulnow*/)
  {
    auto * node = reinterpret_cast<NTRIPClientNode *>(userdata);
    if (node->streaming_exit_.load() || node->reconfigure_needed_.load()) {
      return 1;
    }
    return 0;
  }

  static size_t WriteCallback(char * ptr, size_t size, size_t nmemb, void * userdata)
  {
    NTRIPClientNode * node = reinterpret_cast<NTRIPClientNode *>(userdata);

    // code doesnt work in Humble
    // if (node->get_logger().get_effective_level() == rclcpp::Logger::Level::Debug) {
    // Convert the received data to a hexadecimal string
    std::stringstream hexStream;
    hexStream << std::hex << std::setfill('0');
    for (size_t i = 0; i < size * nmemb; i++) {
      hexStream << std::setw(2) << static_cast<int>(ptr[i]);
    }
    std::string hexString = hexStream.str();

    // Log the hexadecimal string as a debug message
    RCLCPP_DEBUG(
      node->get_logger(), "Received size: %ld nmemb: %ld data: %s", size, nmemb,
      hexString.c_str());
    // }

    // Create an instance of the message and populate
    auto message = std::make_unique<rtcm_msgs::msg::Message>();
    message->header.stamp = node->get_clock()->now();
    message->header.frame_id = node->mountpoint_;

    // Set the data from the char* ptr
    const size_t nbytes = size * nmemb;
    if (nbytes == 0) {
      return 0;
    }

    // Ignore HTTP/NTRIP header text; RTCM3 frames begin with 0xD3.
    bool has_rtcm = false;
    for (size_t i = 0; i < nbytes; ++i) {
      if (static_cast<unsigned char>(ptr[i]) == 0xD3) {
        has_rtcm = true;
        break;
      }
    }
    if (!has_rtcm) {
      RCLCPP_DEBUG(
        node->get_logger(), "Skipping non-RTCM chunk (%zu bytes)", nbytes);
      return nbytes;
    }

    message->message.assign(ptr, ptr + nbytes);

    // Publish the message
    node->rtcm_pub_->publish(std::move(message));

    return size * nmemb;
  }

  // DoStreaming with runtime parameter reconfiguration support
  void DoStreaming()
  {
    while (!streaming_exit_.load()) {
      ClearStreamSocket();

      std::atomic<bool> gga_active{false};
      std::thread gga_thread;
      if (!gga_fix_topic_.empty()) {
        gga_active.store(true);
        gga_thread = std::thread(&NTRIPClientNode::GgaSenderLoop, this, &gga_active);
      }

      // Perform the request
      CURLcode res = curl_easy_perform(curlHandle_->handle);

      gga_active.store(false);
      if (gga_thread.joinable()) {
        gga_thread.join();
      }
      ClearStreamSocket();

      if (streaming_exit_.load()) {
        break;
      }

      // Reconfigure after perform returns (handle is idle) — tightest response to param changes
      if (reconfigure_needed_.load()) {
        std::lock_guard<std::mutex> lock(params_mutex_);

        // Copy pending values to active values
        use_https_ = pending_use_https_;
        host_ = pending_host_;
        port_ = pending_port_;
        mountpoint_ = pending_mountpoint_;
        username_ = pending_username_;
        password_ = pending_password_;
        log_level_ = pending_log_level_;
        maxage_conn_ = pending_maxage_conn_;
        ntrip_version_ = pending_ntrip_version_;

        // Reconfigure curl with new values
        ApplyCurlOptions();

        reconfigure_needed_.store(false);

        RCLCPP_INFO(
          this->get_logger(),
          "Reconfigured NTRIP connection: %s", ConnectionUrl().c_str());

        // Skip stale res handling — immediately perform with new config
        continue;
      }

      if (res == CURLE_ABORTED_BY_CALLBACK) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      // Check for any errors
      if (res != CURLE_OK) {
        // Retrieve and log the effective URL
        char * effectiveUrl = nullptr;
        curl_easy_getinfo(curlHandle_->handle, CURLINFO_EFFECTIVE_URL, &effectiveUrl);
        RCLCPP_ERROR(
          this->get_logger(), "Failed to perform streaming request for URL: %s",
          effectiveUrl != nullptr ? effectiveUrl : "(unknown)");

        // Retrieve and log the response code
        long responseCode = 0;
        curl_easy_getinfo(curlHandle_->handle, CURLINFO_RESPONSE_CODE, &responseCode);
        RCLCPP_ERROR(this->get_logger(), "Response code: %ld", responseCode);

        RCLCPP_ERROR(
          this->get_logger(), "Failed to perform streaming request: %s",
          curl_easy_strerror(res));

        rclcpp::sleep_for(std::chrono::seconds(1));
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "NTRIP stream ended; reconnecting to %s", ConnectionUrl().c_str());
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
    }
  }

public:
  NTRIP_CLIENT_NODE_LOCAL
  ~NTRIPClientNode()
  {
    streaming_exit_.store(true);

    // Wait for the streaming thread to finish
    streamingThread_.join();

    curlHandle_.reset();
    curl_slist_free_all(http_headers_);
    curl_global_cleanup();
    RCLCPP_INFO(this->get_logger(), "finished");
  }
};
}  // namespace ublox_dgnss

RCLCPP_COMPONENTS_REGISTER_NODE(ublox_dgnss::NTRIPClientNode)
