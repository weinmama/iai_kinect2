#ifndef BUILD_KINECT2_BRIDGE_COMPONENT_H
#define BUILD_KINECT2_BRIDGE_COMPONENT_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>
#include <sys/stat.h>
#if defined(__linux__)

#include <sys/prctl.h>
#elif defined(__APPLE__)
#include <pthread.h>
#endif

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>

#include <kinect2_bridge_component/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>
#include <kinect2_registration/kinect2_console.h>

namespace composition
{
	class Kinect2Bridge : public rclcpp::Node
	{
	public:

		Kinect2Bridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
		bool start();
		void stop();
		bool initialize();
		void threadDispatcher(const size_t id);
		void receiveIrDepth();
		void receiveColor();
		bool receiveFrames(libfreenect2::SyncMultiFrameListener* listener, libfreenect2::FrameMap& frames);
		std_msgs::msg::Header createHeader(rclcpp::Time& last, rclcpp::Time& other);

	private:
		template<typename ParameterType>
		void declare_and_set_parameter(std::string name, ParameterType& variable, ParameterType defaultValue)
		{
			declare_parameter(name, defaultValue);
			get_parameter(name, variable);
			RCLCPP_DEBUG_STREAM (get_logger(), "-- Parameter:" << name << " = " << variable);
		}

		bool initRegistration(const std::string& method, const int32_t device, const double maxDepth);
		bool initPipeline(const std::string& method, const int32_t device);

		std::vector<int> compressionParams;
		std::string compression16BitExt, compression16BitString, baseNameTF;

		cv::Size sizeColor, sizeIr, sizeLowRes;
		libfreenect2::Frame color;
		cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
		cv::Mat rotation, translation;
		cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

		std::vector<std::thread> threads;
		std::mutex lockIrDepth, lockColor;
		std::mutex lockSync, lockPub, lockTime, lockStatus;
		std::mutex lockRegLowRes, lockRegHighRes, lockRegSD;

		bool publishTF;
		std::thread controlThread, tfPublisher, deviceStatusThread;

		libfreenect2::Freenect2 freenect2;
		libfreenect2::Freenect2Device* device;
		libfreenect2::SyncMultiFrameListener* listenerColor, * listenerIrDepth;
		libfreenect2::PacketPipeline* packetPipeline;
		libfreenect2::Registration* registration;
		libfreenect2::Freenect2Device::ColorCameraParams colorParams;
		libfreenect2::Freenect2Device::IrCameraParams irParams;

		DepthRegistration* depthRegLowRes, * depthRegHighRes;

		size_t frameColor, frameIrDepth, pubFrameColor, pubFrameIrDepth;
		rclcpp::Time lastColor, lastDepth;

		double depthShift, elapsedTimeColor, elapsedTimeIrDepth;
		bool running, deviceActive, clientConnected, isSubscribedColor, isSubscribedDepth;

		enum Image
		{
			IR_SD = 0,
			IR_SD_RECT,

			DEPTH_SD,
			DEPTH_SD_RECT,
			DEPTH_HD,
			DEPTH_QHD,

			COLOR_SD_RECT,
			COLOR_HD,
			COLOR_HD_RECT,
			COLOR_QHD,
			COLOR_QHD_RECT,

			MONO_HD,
			MONO_HD_RECT,
			MONO_QHD,
			MONO_QHD_RECT,

			COUNT
		};

		enum Status
		{
			UNSUBSCRIBED = 0,
			RAW,
			COMPRESSED,
			BOTH
		};

		std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> imagePubs;
		std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> compressedPubs;
		rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoHDPub, infoQHDPub, infoIRPub;
		sensor_msgs::msg::CameraInfo infoHD, infoQHD, infoIR;
		std::vector<Status> status;
		bool nextColor, nextIrDepth;
		double deltaT;

		void initConfig(const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth);
		void initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png);
		void initTopics(const int32_t queueSize, const std::string& base_name);
		bool initDevice(std::string& sensor);
		void initCalibration(const std::string& calib_path, const std::string& sensor);
		bool loadCalibrationFile(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distortion) const;
		bool loadCalibrationPoseFile(const std::string& filename, cv::Mat& rotation, cv::Mat& translation) const;
		bool loadCalibrationDepthFile(const std::string& filename, double& depthShift) const;
		void createCameraInfo();
		void
		createCameraInfo(const cv::Size& size, const cv::Mat& cameraMatrix, const cv::Mat& distortion, const cv::Mat& rotation, const cv::Mat& projection,
				sensor_msgs::msg::CameraInfo& cameraInfo) const;
		void maintainDeviceStatus();
		bool hasSubscription(bool& isSubscribedColor, bool& isSubscribedDepth);
		void createImage(const cv::Mat& image, const std_msgs::msg::Header& header, const Image type, sensor_msgs::msg::Image& msgImage) const;
		void createCompressed(const cv::Mat& image, const std_msgs::msg::Header& header, const Image type, sensor_msgs::msg::CompressedImage& msgImage) const;
		void publishStaticTF();
		void processIrDepth(const cv::Mat& depth, std::vector<cv::Mat>& images, const std::vector<Kinect2Bridge::Status>& status);
		void processColor(std::vector<cv::Mat>& images, const std::vector<Kinect2Bridge::Status>& status);
		void control();
		void
		publishImages(const std::vector<cv::Mat>& images, const std_msgs::msg::Header& header, const std::vector<Status>& status, const size_t frame, size_t& pubFrame, const size_t begin,
				const size_t end);
	};
};

#endif //BUILD_KINECT2_BRIDGE_COMPONENT_H
