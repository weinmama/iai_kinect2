/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <kinect2_registration/kinect2_registration.h>
#include <kinect2_registration/kinect2_console.h>
#include "rclcpp/rclcpp.hpp"


#ifdef DEPTH_REG_CPU
#include "depth_registration_cpu.h"
#endif

#ifdef DEPTH_REG_OPENCL
#include "depth_registration_opencl.h"
#endif

DepthRegistration::DepthRegistration()
{
}

DepthRegistration::~DepthRegistration()
{
}

bool DepthRegistration::init(const cv::Mat& cameraMatrixRegistered, const cv::Size& sizeRegistered, const cv::Mat& cameraMatrixDepth, const cv::Size& sizeDepth,
		const cv::Mat& distortionDepth, const cv::Mat& rotation, const cv::Mat& translation,
		const float zNear, const float zFar, const int deviceId)
{
	this->cameraMatrixRegistered = cameraMatrixRegistered;
	this->cameraMatrixDepth = cameraMatrixDepth;
	this->rotation = rotation;
	this->translation = translation;
	this->sizeRegistered = sizeRegistered;
	this->sizeDepth = sizeDepth;
	this->zNear = zNear;
	this->zFar = zFar;

	cv::initUndistortRectifyMap(cameraMatrixDepth, distortionDepth, cv::Mat(), cameraMatrixRegistered, sizeRegistered, CV_32FC1, mapX, mapY);

	return init(deviceId);
}

DepthRegistration* DepthRegistration::New(Method method)
{
	if (method == DEFAULT)
	{
#ifdef DEPTH_REG_OPENCL
		method = OPENCL;
#elif defined DEPTH_REG_CPU
		method = CPU;
#endif
	}

	switch (method)
	{
	case DEFAULT:
		RCLCPP_ERROR(rclcpp::get_logger("DepthRegistration"), "No default registration method available!");
		break;
	case CPU:
#ifdef DEPTH_REG_CPU
		RCLCPP_INFO(rclcpp::get_logger("DepthRegistration"),"Using CPU registration method!");
		return new DepthRegistrationCPU();
#else
		RCLCPP_ERROR(rclcpp::get_logger("DepthRegistration"),"CPU registration method not available!");
	  break;
#endif
	case OPENCL:
#ifdef DEPTH_REG_OPENCL
	RCLCPP_INFO(rclcpp::get_logger("DepthRegistration"), "Using OpenCL registration method!");
		return new DepthRegistrationOpenCL();
#else
		RCLCPP_ERROR(rclcpp::get_logger("DepthRegistration"), "OpenCL registration method not available!");
		break;
#endif
	}
	return NULL;
}

const std::string getFunctionName(const std::string& name)
{
	size_t end = name.rfind('(');
	if (end == std::string::npos)
	{
		end = name.size();
	}
	size_t begin = 1 + name.rfind(' ', end);
	return name.substr(begin, end - begin);
}
