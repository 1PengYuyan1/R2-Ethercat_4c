#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <cmath>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "rclcpp/rclcpp.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <asm/termbits.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C"{
#endif
#include <poll.h>

#include "hipnuc_dec.h"

#define GRA_ACC          (9.8)
#define DEG_TO_RAD       (0.01745329)
#define UTESLA_TO_TESLA  (0.000001)
#define BUF_SIZE         (1024)
#ifdef __cplusplus
}
#endif



namespace hipnuc_driver
{
	using namespace std::chrono_literals;
	using namespace std;
	static hipnuc_raw_t raw;

	class IMUPublisher : public rclcpp::Node
	{
		public:
			
			IMUPublisher(const rclcpp::NodeOptions &options) : Node("IMU_publisher", options)	
			{
				this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
				this->declare_parameter<int>("baud_rate", 115200);
				this->declare_parameter<std::string>("frame_id", "base_link");
				this->declare_parameter<std::string>("imu_topic", "/IMU_data");
				this->declare_parameter<std::string>("euler_topic", "/euler_data");
				this->declare_parameter<std::string>("magnetic_topic", "/magnetic_data");
				this->declare_parameter<std::string>("temperature_topic", "/temperature_data");
				this->declare_parameter<std::string>("pressure_topic", "/pressure_data");
				this->declare_parameter<bool>("imu_switch", true);
				this->declare_parameter<bool>("euler_switch", false);
				this->declare_parameter<bool>("magnetic_switch", false);
				this->declare_parameter<bool>("temperature_switch", false);
				this->declare_parameter<bool>("pressure_switch", false);
				this->declare_parameter<bool>("hardware_attitude_reset_on_start", true);
				this->declare_parameter<int>("hardware_attitude_reset_delay_ms", 300);
				this->declare_parameter<bool>("reset_attitude_on_start", true);
				this->declare_parameter<int>("reset_attitude_delay_ms", 500);

				this->get_parameter("serial_port", serial_port);
				this->get_parameter("baud_rate", baud_rate);
				this->get_parameter("frame_id", frame_id);
				this->get_parameter("imu_topic", imu_topic);
				this->get_parameter("euler_topic", euler_topic);
				this->get_parameter("magnetic_topic", magnetic_topic);
				this->get_parameter("temperature_topic", temperature_topic);
				this->get_parameter("pressure_topic", pressure_topic);
				this->get_parameter("imu_switch", imu_switch);
				this->get_parameter("euler_switch", euler_switch);
				this->get_parameter("magnetic_switch", magnetic_switch);
				this->get_parameter("temperature_switch", temperature_switch);
				this->get_parameter("pressure_switch", pressure_switch);
				this->get_parameter("hardware_attitude_reset_on_start", hardware_attitude_reset_on_start);
				this->get_parameter("hardware_attitude_reset_delay_ms", hardware_attitude_reset_delay_ms);
				this->get_parameter("reset_attitude_on_start", reset_attitude_on_start);
				this->get_parameter("reset_attitude_delay_ms", reset_attitude_delay_ms);

				RCLCPP_INFO(this->get_logger(),"serial_port: %s\r\n", serial_port.c_str());
				RCLCPP_INFO(this->get_logger(), "baud_rate: %d\r\n", baud_rate);
				RCLCPP_INFO(this->get_logger(), "frame_id: %s\r\n", frame_id.c_str());
				RCLCPP_INFO(this->get_logger(), "imu_topic: %s\r\n", imu_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "euler_topic: %s\r\n", euler_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "magnetic_topic: %s\r\n", magnetic_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "temperature_topic: %s\r\n", temperature_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "pressure_topic: %s\r\n", pressure_topic.c_str());
				RCLCPP_INFO(this->get_logger(), "imu_switch: %d\r\n", imu_switch);
				RCLCPP_INFO(this->get_logger(), "euler_switch: %d\r\n", euler_switch);
				RCLCPP_INFO(this->get_logger(), "magnetic_switch: %d\r\n", magnetic_switch);
				RCLCPP_INFO(this->get_logger(), "temperature_switch: %d\r\n", temperature_switch);
				RCLCPP_INFO(this->get_logger(), "pressure_switch: %d\r\n", pressure_switch);
				RCLCPP_INFO(this->get_logger(), "hardware_attitude_reset_on_start: %d\r\n", hardware_attitude_reset_on_start);
				RCLCPP_INFO(this->get_logger(), "hardware_attitude_reset_delay_ms: %d\r\n", hardware_attitude_reset_delay_ms);
				RCLCPP_INFO(this->get_logger(), "reset_attitude_on_start: %d\r\n", reset_attitude_on_start);
				RCLCPP_INFO(this->get_logger(), "reset_attitude_delay_ms: %d\r\n", reset_attitude_delay_ms);
				
				imu_pub = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());
				euler_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(euler_topic, rclcpp::SensorDataQoS());
				magnetic_pub = this->create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic, rclcpp::SensorDataQoS());
				temp_pub = this->create_publisher<sensor_msgs::msg::Temperature>(temperature_topic, rclcpp::SensorDataQoS());
				pressure_pub = this->create_publisher<sensor_msgs::msg::FluidPressure>(pressure_topic, rclcpp::SensorDataQoS());
				
				fd = open_ttyport(serial_port, baud_rate);

				if (fd < 0) {
					RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port.c_str());
					return;
				}

				if (hardware_attitude_reset_on_start)
				{
					send_hardware_attitude_reset();
					if (hardware_attitude_reset_delay_ms > 0)
						std::this_thread::sleep_for(std::chrono::milliseconds(hardware_attitude_reset_delay_ms));
				}

				attitude_reset_start_time = std::chrono::steady_clock::now();
				running.store(true);
				reader_thread = std::thread([this]() {this->read_loop(); });
			}

			~IMUPublisher() override
			{
				running.store(false);

				if (reader_thread.joinable())
					reader_thread.join();
				
				if (fd >= 0) {
					close(fd);
					fd = -1;
				}
			}

		private: 
			void read_loop()
			{
				struct pollfd p{};
				p.fd = fd;
				p.events = POLLIN;

				while(rclcpp::ok() && running.load())
				{
					int rpoll = poll(&p, 1, 2);
					if (rpoll == 0)
						continue;
					
					if (rpoll < 0) {
						if (errno == EINTR) 
							continue;

						RCLCPP_ERROR(this->get_logger(), "poll() failed: %s", std::strerror(errno));
						break;
					}

					if (!(p.revents & POLLIN))
						continue;

					int n = read(fd, buf, sizeof(buf));
					if (n < 0) {
						if (errno == EAGAIN || errno == EWOULDBLOCK) 
							continue;

						RCLCPP_ERROR(this->get_logger(), "read() failed: %s", std::strerror(errno));
						break;
					}

					if (n == 0) {
						std::this_thread::sleep_for(10ms);
						continue;
					}

					for (int i = 0; i < n; i++) {
						int rev = hipnuc_input(&raw, buf[i]);
						if (!rev) continue;

                        if (raw.hi83.tag == 0x83)
                        {
                            uint32_t bm = raw.hi83.data_bitmap;

                            if (bm & HI83_BMAP_QUAT)
                            {
                                imu_msg.orientation.w = raw.hi83.quat[0];
                                imu_msg.orientation.x = raw.hi83.quat[1];
                                imu_msg.orientation.y = raw.hi83.quat[2];
                                imu_msg.orientation.z = raw.hi83.quat[3];
                            }
                            if (bm & HI83_BMAP_GYR_B)
                            {
                                imu_msg.angular_velocity.x = raw.hi83.gyr_b[0];
                                imu_msg.angular_velocity.y = raw.hi83.gyr_b[1];
                                imu_msg.angular_velocity.z = raw.hi83.gyr_b[2];
                            }
                            if (bm & HI83_BMAP_ACC_B)
                            {
                                imu_msg.linear_acceleration.x = raw.hi83.acc_b[0];
                                imu_msg.linear_acceleration.y = raw.hi83.acc_b[1];
                                imu_msg.linear_acceleration.z = raw.hi83.acc_b[2];
                            }
							if (bm & HI83_BMAP_MAG_B)
							{
								magnetic_msg.magnetic_field.x = raw.hi83.mag_b[0] * UTESLA_TO_TESLA ;
								magnetic_msg.magnetic_field.y = raw.hi83.mag_b[1] * UTESLA_TO_TESLA ;
								magnetic_msg.magnetic_field.z = raw.hi83.mag_b[2] * UTESLA_TO_TESLA ;
							}

							if (bm & HI83_BMAP_RPY)
							{
								euler_msg.vector.x = raw.hi83.rpy[0] * DEG_TO_RAD;
								euler_msg.vector.y = raw.hi83.rpy[1] * DEG_TO_RAD;
								euler_msg.vector.z = raw.hi83.rpy[2] * DEG_TO_RAD;
							}

							if (bm & HI83_BMAP_AIR_PRESSURE)
							{
								pre_msg.fluid_pressure = raw.hi83.air_pressure;
							}

							if (bm & HI83_BMAP_TEMPERATURE)
							{
								temp_msg.temperature = raw.hi83.temperature;
							}

							if (!(raw.hi83.main_status & (1 << 11)) && (bm & HI83_BMAP_UTC))
							{
								imu_msg.header.stamp.sec = utc_to_unix(2000 + raw.hi83.utc.year, raw.hi83.utc.month, raw.hi83.utc.day, raw.hi83.utc.hour, raw.hi83.utc.min, raw.hi83.utc.sec_ms / 1000 % 60);
								imu_msg.header.stamp.nanosec = raw.hi83.utc.sec_ms % 1000 * 1000000;

								magnetic_msg.header.stamp.sec = utc_to_unix(2000 + raw.hi83.utc.year, raw.hi83.utc.month, raw.hi83.utc.day, raw.hi83.utc.hour, raw.hi83.utc.min, raw.hi83.utc.sec_ms / 1000 % 60);
								magnetic_msg.header.stamp.nanosec = raw.hi83.utc.sec_ms % 1000 * 1000000;

								euler_msg.header.stamp.sec = utc_to_unix(2000 + raw.hi83.utc.year, raw.hi83.utc.month, raw.hi83.utc.day, raw.hi83.utc.hour, raw.hi83.utc.min, raw.hi83.utc.sec_ms / 1000 % 60);
								euler_msg.header.stamp.nanosec = raw.hi83.utc.sec_ms % 1000 * 1000000;

								temp_msg.header.stamp.sec = utc_to_unix(2000 + raw.hi83.utc.year, raw.hi83.utc.month, raw.hi83.utc.day, raw.hi83.utc.hour, raw.hi83.utc.min, raw.hi83.utc.sec_ms / 1000 % 60);
								temp_msg.header.stamp.nanosec = raw.hi83.utc.sec_ms % 1000 * 1000000;

								pre_msg.header.stamp.sec = utc_to_unix(2000 + raw.hi83.utc.year, raw.hi83.utc.month, raw.hi83.utc.day, raw.hi83.utc.hour, raw.hi83.utc.min, raw.hi83.utc.sec_ms / 1000 % 60);
								pre_msg.header.stamp.nanosec = raw.hi83.utc.sec_ms % 1000 * 1000000;

							}
							else
							{
								imu_msg.header.stamp = rclcpp::Clock().now();
								magnetic_msg.header.stamp = rclcpp::Clock().now();
								euler_msg.header.stamp = rclcpp::Clock().now();
								temp_msg.header.stamp = rclcpp::Clock().now();
								pre_msg.header.stamp = rclcpp::Clock().now();
							}
                        }
                        else if (raw.hi91.tag == 0x91)
                        {
                            imu_msg.orientation.w = raw.hi91.quat[0];
                            imu_msg.orientation.x = raw.hi91.quat[1]; 
                            imu_msg.orientation.y = raw.hi91.quat[2];
                            imu_msg.orientation.z = raw.hi91.quat[3];
                            imu_msg.angular_velocity.x = raw.hi91.gyr[0] * DEG_TO_RAD;
                            imu_msg.angular_velocity.y = raw.hi91.gyr[1] * DEG_TO_RAD;
                            imu_msg.angular_velocity.z = raw.hi91.gyr[2] * DEG_TO_RAD;
                            imu_msg.linear_acceleration.x = raw.hi91.acc[0] * GRA_ACC;
                            imu_msg.linear_acceleration.y = raw.hi91.acc[1] * GRA_ACC;
                            imu_msg.linear_acceleration.z = raw.hi91.acc[2] * GRA_ACC;

							euler_msg.vector.x = raw.hi91.roll  * DEG_TO_RAD;
							euler_msg.vector.y = raw.hi91.pitch * DEG_TO_RAD;
							euler_msg.vector.z = raw.hi91.yaw   * DEG_TO_RAD;

							magnetic_msg.magnetic_field.x = raw.hi91.mag[0] * UTESLA_TO_TESLA ;
							magnetic_msg.magnetic_field.y = raw.hi91.mag[1] * UTESLA_TO_TESLA ;
							magnetic_msg.magnetic_field.z = raw.hi91.mag[2] * UTESLA_TO_TESLA ;

							temp_msg.temperature = raw.hi91.temp;

							pre_msg.fluid_pressure = raw.hi91.air_pressure;

							imu_msg.header.stamp = rclcpp::Clock().now();
							magnetic_msg.header.stamp = rclcpp::Clock().now();
							euler_msg.header.stamp = rclcpp::Clock().now();
							temp_msg.header.stamp = rclcpp::Clock().now();
							pre_msg.header.stamp = rclcpp::Clock().now();
                        }
						else 
							continue;

						if (imu_switch) {
							imu_msg.header.frame_id = frame_id;
							auto publish_imu_msg = imu_msg;
							if (prepare_imu_for_publish(publish_imu_msg))
								imu_pub->publish(publish_imu_msg);
						}
                       
						if (magnetic_switch) {
							magnetic_msg.header.frame_id = frame_id;
							magnetic_pub->publish(magnetic_msg);
						}
						
							if (euler_switch) {
								euler_msg.header.frame_id = frame_id;
								auto publish_euler_msg = euler_msg;
								apply_attitude_zero_to_euler(publish_euler_msg);
								euler_pub->publish(publish_euler_msg);
							}

						if (temperature_switch) {
							temp_msg.header.frame_id = frame_id;
							temp_pub->publish(temp_msg);
						}

						if (pressure_switch) {
							pre_msg.header.frame_id = frame_id;
							pressure_pub->publish(pre_msg);
						}	
					}
				}
			}

			int open_ttyport(const std::string &tty_port, int baud)
			{
				const char *port_device = tty_port.c_str();
				int serial_port = open(port_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
				if (serial_port < 0)
				{
					RCLCPP_ERROR(this->get_logger(), "Error opening serial port %s: %s", port_device, std::strerror(errno));
					return -1;
				}

				struct termios2 tty {};

				if (ioctl(serial_port, TCGETS2, &tty) != 0)
				{
					RCLCPP_ERROR(this->get_logger(), "TCGETS2 failed: %s ", std::strerror(errno));
					close(serial_port);
					return -1;
				}

				tty.c_cflag &= ~CBAUD;
				tty.c_cflag |= BOTHER;

				tty.c_ispeed = baud;
				tty.c_ospeed = baud;

				tty.c_cflag |= CS8;
				tty.c_cflag &= ~PARENB;
				tty.c_cflag &= ~CSTOPB;
				tty.c_cflag &= ~CRTSCTS;

				tty.c_lflag &= ~ICANON;
				tty.c_lflag &= ~ECHO;
				tty.c_lflag &= ~ECHOE;
				tty.c_lflag &= ~ECHONL;
				tty.c_lflag &= ~ISIG;

				tty.c_iflag &= ~(IXON | IXOFF | IXANY);
				tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

				tty.c_cc[VTIME] = 10;
				tty.c_cc[VMIN] = 0;

				if (ioctl(serial_port, TCSETS2, &tty) != 0)
				{
					RCLCPP_ERROR(this->get_logger(), "TCSETS2 failed : %s", std::strerror(errno));
					close(serial_port);
					return -1;
				}

				return serial_port;
			}

			bool write_serial_command(const std::string &command)
			{
				const std::string wire_command = command + "\r\n";
				size_t written = 0;
				while (written < wire_command.size())
				{
					const ssize_t n = write(fd, wire_command.data() + written, wire_command.size() - written);
					if (n > 0)
					{
						written += static_cast<size_t>(n);
						continue;
					}

					if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
					{
						std::this_thread::sleep_for(10ms);
						continue;
					}

					RCLCPP_WARN(this->get_logger(), "Failed to write IMU command '%s': %s",
					            command.c_str(), std::strerror(errno));
					return false;
				}

				return true;
			}

			void send_hardware_attitude_reset()
			{
				const std::string heading_command = "CONFIG ATT RST 1";
				const std::string pitch_roll_command = "CONFIG ATT RST 2";
				if (write_serial_command(heading_command))
				{
					RCLCPP_WARN(this->get_logger(),
					            "IMU hardware attitude reset command sent: %s",
					            heading_command.c_str());
				}
				std::this_thread::sleep_for(50ms);
				if (write_serial_command(pitch_roll_command))
				{
					RCLCPP_WARN(this->get_logger(),
					            "IMU hardware attitude reset command sent: %s",
					            pitch_roll_command.c_str());
				}
			}

			static double normalize_angle(double angle)
			{
				while (angle > M_PI)
					angle -= 2.0 * M_PI;
				while (angle < -M_PI)
					angle += 2.0 * M_PI;
				return angle;
			}

			static bool normalize_quaternion_values(double &w, double &x, double &y, double &z)
			{
				const double norm = std::sqrt(w * w + x * x + y * y + z * z);
				if (norm < 1e-6)
					return false;

				w /= norm;
				x /= norm;
				y /= norm;
				z /= norm;
				return true;
			}

			static bool normalize_quaternion(sensor_msgs::msg::Imu &msg)
			{
				double w = msg.orientation.w;
				double x = msg.orientation.x;
				double y = msg.orientation.y;
				double z = msg.orientation.z;
				if (!normalize_quaternion_values(w, x, y, z))
					return false;

				msg.orientation.w = w;
				msg.orientation.x = x;
				msg.orientation.y = y;
				msg.orientation.z = z;
				return true;
			}

			void apply_attitude_zero(sensor_msgs::msg::Imu &msg) const
			{
				const double w = msg.orientation.w;
				const double x = msg.orientation.x;
				const double y = msg.orientation.y;
				const double z = msg.orientation.z;

				const double iw = attitude_zero_w;
				const double ix = -attitude_zero_x;
				const double iy = -attitude_zero_y;
				const double iz = -attitude_zero_z;

				double nw = iw * w - ix * x - iy * y - iz * z;
				double nx = iw * x + ix * w + iy * z - iz * y;
				double ny = iw * y - ix * z + iy * w + iz * x;
				double nz = iw * z + ix * y - iy * x + iz * w;
				const double norm = std::sqrt(nw * nw + nx * nx + ny * ny + nz * nz);
				if (norm < 1e-6)
					return;

				msg.orientation.w = nw / norm;
				msg.orientation.x = nx / norm;
				msg.orientation.y = ny / norm;
				msg.orientation.z = nz / norm;
			}

			bool prepare_imu_for_publish(sensor_msgs::msg::Imu &msg)
			{
				if (!reset_attitude_on_start)
					return true;

				if (!normalize_quaternion(msg))
					return false;

				if (!attitude_zero_initialized)
				{
					const auto elapsed_ms =
						std::chrono::duration_cast<std::chrono::milliseconds>(
							std::chrono::steady_clock::now() - attitude_reset_start_time).count();
					if (elapsed_ms < reset_attitude_delay_ms)
						return false;

					attitude_zero_w = msg.orientation.w;
					attitude_zero_x = msg.orientation.x;
					attitude_zero_y = msg.orientation.y;
					attitude_zero_z = msg.orientation.z;
					attitude_zero_initialized = true;
					RCLCPP_WARN(this->get_logger(),
					            "IMU software attitude reset initialized");
				}

				apply_attitude_zero(msg);
				return true;
			}

			void apply_attitude_zero_to_euler(geometry_msgs::msg::Vector3Stamped &msg)
			{
				if (!reset_attitude_on_start)
					return;

				if (!euler_zero_initialized && attitude_zero_initialized)
				{
					euler_zero_x = msg.vector.x;
					euler_zero_y = msg.vector.y;
					euler_zero_z = msg.vector.z;
					euler_zero_initialized = true;
				}

				if (euler_zero_initialized)
				{
					msg.vector.x = normalize_angle(msg.vector.x - euler_zero_x);
					msg.vector.y = normalize_angle(msg.vector.y - euler_zero_y);
					msg.vector.z = normalize_angle(msg.vector.z - euler_zero_z);
				}
			}

			time_t utc_to_unix(int year, int month, int day, int hour, int minute, int second) 
			{
				struct tm tm_time;
				time_t timestamp;

				tm_time.tm_year = year - 1900;
				tm_time.tm_mon = month - 1;
				tm_time.tm_mday = day;
				tm_time.tm_hour = hour;
				tm_time.tm_min = minute;
				tm_time.tm_sec = second;
				tm_time.tm_isdst = 0;  
				

				#ifdef _WIN32
				timestamp = _mkgmtime(&tm_time);  
				#else
				timestamp = timegm(&tm_time);
				#endif
				
				return timestamp;
			}

			int fd = 0;
			uint8_t buf[BUF_SIZE] = {0};

			std::string serial_port;
			int baud_rate;
			std::string frame_id;
			bool imu_switch;
			bool euler_switch;
				bool magnetic_switch;
				bool temperature_switch;
				bool pressure_switch;
				bool hardware_attitude_reset_on_start;
				int hardware_attitude_reset_delay_ms;
				bool reset_attitude_on_start;
				int reset_attitude_delay_ms;
				bool attitude_zero_initialized = false;
				double attitude_zero_w = 1.0;
				double attitude_zero_x = 0.0;
				double attitude_zero_y = 0.0;
				double attitude_zero_z = 0.0;
				bool euler_zero_initialized = false;
				double euler_zero_x = 0.0;
				double euler_zero_y = 0.0;
				double euler_zero_z = 0.0;
				std::chrono::steady_clock::time_point attitude_reset_start_time;
			std::string imu_topic;
			std::string euler_topic;
			std::string magnetic_topic;
			std::string temperature_topic;
			std::string pressure_topic;

			sensor_msgs::msg::Imu imu_msg;
			sensor_msgs::msg::MagneticField magnetic_msg;
			geometry_msgs::msg::Vector3Stamped euler_msg;
			sensor_msgs::msg::Temperature temp_msg;
			sensor_msgs::msg::FluidPressure pre_msg;

			rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
			rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub;
			rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub;
			rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub;
			rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub;
			
			std::atomic<bool> running{false};
			std::thread reader_thread;
	};
}


int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	options.use_intra_process_comms(true);
	rclcpp::spin(std::make_shared<hipnuc_driver::IMUPublisher>(options));
	rclcpp::shutdown();

	return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hipnuc_driver::IMUPublisher)
