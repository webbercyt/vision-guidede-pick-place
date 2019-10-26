#include "depth_camera_realsense.h"
#include <librealsense2/rsutil.h>
#include "utils.h"

#define OPEN_LASER_WHEN_NEED /*1*/0
#define USE_OPEN_CLOSE /*1*/0
static const int FRAME_PER_SECOND = 30;

RealSenseD400Series::RealSenseD400Series(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name):
	DepthCamera(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
}

RealSenseD400Series::~RealSenseD400Series()
{
	if (is_dig_alloc_succeed_ 
#if USE_OPEN_CLOSE
		&& is_laser_running_
#endif
		)
	{
		pipeline_->stop();
	}
}

bool RealSenseD400Series::singleGrab(bool display_image)
{
	if (display_image)
	{
		if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
		{
			MdispLut(mil_display_, M_DEFAULT);
		}

		MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	startLaserEmitting();
	is_cur_frame_corrupted_ = !fetchFrame();
	stopLaserEmitting();

	return !is_cur_frame_corrupted_;
}

void RealSenseD400Series::continuousGrabHelper()
{
	setDisplayImage(getDispImageComponents()[workspace_ptr_->camera().at(node_name_).depth_camera().display_image()]);
	MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	startLaserEmitting();
	while (!end_grab_)
	{
		fetchFrame();
	}
	stopLaserEmitting();
}

bool RealSenseD400Series::setCameraOutput(CameraOutputMode mode)
{
	switch (mode)
	{
	case SHOW_ONLY:
	case CALIBRATION:
	{
		do_align_ = false;
		break;
	}
	case LOCALIZATON:
	{
		do_align_ = true;
		break;
	}
	default:
		break;
	}

	return true;
}

void RealSenseD400Series::config()
{
	is_dig_alloc_succeed_ = false;

	pipeline_ = std::make_unique<rs2::pipeline>();
	align_to_color_ = std::make_unique<rs2::align>(RS2_STREAM_COLOR);

	do_align_ = true;
	depth_scale_ = 1.0;

	int image_size_x = getImageSizeX();
	int image_size_y = getImageSizeY();

	config_.enable_stream(RS2_STREAM_COLOR, image_size_x, image_size_y, RS2_FORMAT_Y16);
	config_.enable_stream(RS2_STREAM_DEPTH, image_size_x, image_size_y, RS2_FORMAT_Z16);
	//RS2_FORMAT_Z16: 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value.
	profile_ = pipeline_->start(config_);

#if not /*USE_OPEN_CLOSE*/1 
	// open senseor
	rs2::device selected_device = profile_.get_device();
	auto depth_sensor = selected_device.first<rs2::depth_sensor>();

	//auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
	if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
	{
		depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
	}
	if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
	{
		// Query min and max values:
		auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
		depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
	}
#endif
	is_laser_running_ = true;

	// @ capture 1 frame to get aligned camera intrinsic
	rs2::frameset frameset = pipeline_->wait_for_frames();
	frameset = align_to_color_->process(frameset);
	rs2_intrinsics intr = frameset.get_depth_frame().get_profile().as<rs2::video_stream_profile>().get_intrinsics();

	// set parameters
	auto process_3d_params = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->mutable_process_3d_params();

	// intrinsic
	process_3d_params->set_intrinsic_height(intr.height);
	process_3d_params->set_intrinsic_width(intr.width);

	process_3d_params->clear_intrinsic_param();
	process_3d_params->add_intrinsic_param(intr.ppx);
	process_3d_params->add_intrinsic_param(intr.ppy);
	process_3d_params->add_intrinsic_param(intr.fx);
	process_3d_params->add_intrinsic_param(intr.fy);
	process_3d_params->add_intrinsic_param(intr.model);

	for (int i = 0; i < 5; i++)
	{
		process_3d_params->add_intrinsic_param(intr.coeffs[i]);
	}

	// Go over the device's sensors
	for (rs2::sensor& sensor : profile_.get_device().query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			depth_scale_ = dpt.get_depth_scale() * 1000.0f;	// convert from m to mm
			break;
		}
	}

	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_offset(0.0);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_scale(/*1.0 /*/ depth_scale_);

	MbufAlloc2d(mil_system_, image_size_x, image_size_y, 8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &mil_image_);
	MbufAlloc2d(mil_system_, image_size_x, image_size_y, 16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &mil_range_image_);

	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_x(google::protobuf::int32(image_size_x));
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_y(google::protobuf::int32(image_size_y));

	setDisplayImage("MONO");
	/*setDisplayImage("RANGE");*/

	setFov();

	is_dig_alloc_succeed_ = true;
	stopLaserEmitting();
}

bool RealSenseD400Series::fetchFrame()
{
	//clock_t t = clock();

	rs2::frameset frameset = pipeline_->wait_for_frames();

	if (do_align_)
	{
		// Align all frames to color viewport
		frameset = align_to_color_->process(frameset);
	}

	auto depth = frameset.get_depth_frame();
	auto color = frameset.get_color_frame();

	int w = getImageSizeX();
	int h = getImageSizeY();

	if (color)
	{
		auto frame = color.as<rs2::video_frame>();
		MIL_ID temp = MbufAlloc2d(mil_system_, w, h, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MbufPut2d(temp, 0, 0, w, h, (void*)color.get_data());
		MimShift(temp, mil_image_, -8);
		MbufFree(temp);
	}
	
	if (depth)
	{
		auto frame = depth.as<rs2::video_frame>();
		MbufPut2d(mil_range_image_, 0, 0, w, h, (void*)depth.get_data());

		// filter depth for display depth setting
		updateDepthRange();
		if (view_depth_min_ > 0 && view_depth_max_ > view_depth_min_)
		{
			int min_value = int((double)view_depth_min_ / depth_scale_);
			int max_value = int((double)view_depth_max_ / depth_scale_);

			// auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];
			MimClip(mil_range_image_, mil_range_image_, M_OUT_RANGE, min_value, max_value, 0, 0);
		}
	}
	
	/*t = clock() - t;
	printf("[mil convert] It took me %d clicks (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);*/

	if (!color || !depth)
		return  false;
	else
		return true;
}

void RealSenseD400Series::startLaserEmitting()
{
#if OPEN_LASER_WHEN_NEED
	if (!is_laser_running_)
	{
		//clock_t t = clock();

#if USE_OPEN_CLOSE
		pipeline_->start(config_);
		// Temp fix, use gain value to set sleep time
		auto t = (int)workspace_ptr_->camera().at(node_name_).gain();
		Sleep(t);
		//Sleep(800);
#else
		rs2::device selected_device = profile_.get_device();
		auto depth_sensor = selected_device.first<rs2::depth_sensor>();
		/*auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();*/
		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
		}
		if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
		{
			// Query min and max values:
			auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
		}
#endif
		is_laser_running_ = true;

		/*clock_t total_time = clock() - t;
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "startLaserEmitting: " + std::to_string(((float)total_time) / CLOCKS_PER_SEC) + " s"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;*/
	}
#endif
}

void RealSenseD400Series::stopLaserEmitting()
{
#if OPEN_LASER_WHEN_NEED
	if (is_laser_running_)
	{
		//clock_t t = clock();

#if USE_OPEN_CLOSE
		pipeline_->stop();
#else
		auto depth_sensor = profile_.get_device().first<rs2::depth_sensor>();
		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f); // Disable emitter
		}
		if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
		{
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser
		}
#endif
		is_laser_running_ = false;

		/*clock_t total_time = clock() - t;
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "stopLaserEmitting: " + std::to_string(((float)total_time) / CLOCKS_PER_SEC) + " s"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;*/
	}
#endif
}

RealSenseD415Camera::RealSenseD415Camera(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name):
	RealSenseD400Series(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
	config();
}

RealSenseD415Camera::~RealSenseD415Camera()
{
}

void RealSenseD415Camera::setFov()
{
	// https://store.intelrealsense.com/buy-intel-realsense-depth-camera-d415.html
	// use the fov of rgb camera since we project depth iamge to the rgb
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_h(69.4);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_v(42.5);
}

int RealSenseD415Camera::getImageSizeX()
{
	return D415_IMAGE_SIZE_X;
}

int RealSenseD415Camera::getImageSizeY()
{
	return D415_IMAGE_SIZE_Y;
}

RealSenseDataWrapper::RealSenseDataWrapper(MIL_ID mil_system,
	const Workspace_DepthCamera workspace_depth_camera,
	MIL_ID mil_calibration) :
	DepthCameraDataWrapper(mil_system,
		workspace_depth_camera,
		mil_calibration)
{
	intrinsics = new rs2_intrinsics();

	auto process_3d_params = workspace_depth_camera.process_3d_params();
	intrinsics->height = process_3d_params.intrinsic_height();
	intrinsics->width = process_3d_params.intrinsic_width();

	int size = process_3d_params.intrinsic_param_size();

	if (size == 10)
	{
		intrinsics->ppx = (float)process_3d_params.intrinsic_param(0);
		intrinsics->ppy = (float)process_3d_params.intrinsic_param(1);
		intrinsics->fx = (float)process_3d_params.intrinsic_param(2);
		intrinsics->fy = (float)process_3d_params.intrinsic_param(3);
		intrinsics->model = (rs2_distortion)(int)process_3d_params.intrinsic_param(4);

		for (int i = 5; i < 10; i++)
		{
			intrinsics->coeffs[i - 5] = (float)process_3d_params.intrinsic_param(i);
		}
	}
	else
	{
		// Error number of intrinsic_param !
	}

	depth_data_ = DepthCameraDataWrapper::setupDepthData(workspace_depth_camera);
}

RealSenseDataWrapper::~RealSenseDataWrapper()
{
	delete intrinsics;
}

void RealSenseDataWrapper::composePointCloudPoint(MIL_ID range_image, double max_depth)
{
	MIL_UINT16* range_data = (MIL_UINT16*)MbufInquire(range_image, M_HOST_ADDRESS, M_NULL);
	MIL_INT range_pitch = MbufInquire(range_image, M_PITCH, M_NULL);

	MIL_INT image_size_x = MbufInquire(range_image, M_SIZE_X, M_NULL);
	MIL_INT image_size_y = MbufInquire(range_image, M_SIZE_Y, M_NULL);

	int index = 0;
	for (MIL_INT y = 0; y < image_size_y; y++)
	{
		for (MIL_INT x = 0; x < image_size_x; x++)
		{
			if (range_data[x] != 0)
			{
				/*
					@ example of 3d point project from 2d depth map:

						float upixel[2] = { 640, 360 };
						float upoint[3];
						auto udist = depth.get_distance((int)upixel[0], (int)upixel[1]);	// unit: meter
						rs2_intrinsics intr = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
						rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);
				*/

				float pixel[2] = { (float)x, (float)y };
				float dist = (float)((float)range_data[x] * range_scale_ + range_offset_ / 1000.0);	// convert unit from 'mm' to 'm'
				float point3d[3];
				rs2_deproject_pixel_to_point(point3d, intrinsics, pixel, dist);

				if (max_depth > 0)
				{
					if (point3d[2] < max_depth)
					{
						x_world_[index] = point3d[0];
						y_world_[index] = point3d[1];
						z_world_[index] = point3d[2];
						index++;
					}
				}
				else
				{
					x_world_[index] = point3d[0];
					y_world_[index] = point3d[1];
					z_world_[index] = point3d[2];
					index++;
				}
			}
		}
		range_data += range_pitch;
	}
}

bool RealSenseDataWrapper::get3DPointUsingNeighborFilter(
	int input_x,
	int input_y,
	double& output_x,
	double& output_y,
	double& output_z,
	MIL_ID cond_range_image,
	int neighbor_x_offset_neg,
	int neighbor_x_offset_pos,
	int neighbor_y_offset_neg,
	int neighbor_y_offset_pos)
{
#define PI 3.14159265

	MIL_INT cols = MbufInquire(cond_range_image, M_SIZE_X, M_NULL);
	MIL_INT rows = MbufInquire(cond_range_image, M_SIZE_Y, M_NULL);

	if (input_x < 0 ||
		input_y < 0 ||
		input_x > cols ||
		input_y > rows ||
		input_x - neighbor_x_offset_neg < 0 ||
		input_y - neighbor_y_offset_neg < 0 ||
		input_x + neighbor_x_offset_pos > cols ||
		input_y + neighbor_y_offset_pos > rows)
		return false;

	int x_start = input_x - neighbor_x_offset_neg;
	int x_end = input_x + neighbor_x_offset_pos;

	int y_start = input_y - neighbor_y_offset_neg;
	int y_end = input_y + neighbor_y_offset_pos;

	MIL_ID smooth_image = MbufClone(cond_range_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(smooth_image, 0);
	// smooth image before use 
	MimConvolve(cond_range_image, smooth_image, /*M_DERICHE_FILTER*//*M_SHEN_FILTER(M_SMOOTH, 70)*/M_SMOOTH);
	//MbufCopy(cond_range_image, smooth_image);

	MIL_UINT16* range_data = (MIL_UINT16*)MbufInquire(smooth_image, M_HOST_ADDRESS, M_NULL);
	MIL_INT range_pitch = MbufInquire(smooth_image, M_PITCH, M_NULL);
	range_data += range_pitch * y_start;

	double sigma = 0.0; // deviation
	double nb = 0;

	std::vector<std::pair<double, std::vector<float>>> point_weight_vector;
	for (MIL_INT y = y_start; y < y_end; y += 1)
	{
		for (MIL_INT x = x_start; x < x_end; x++)
		{
			if (range_data[x] != 0)
			{
				double r_square = (double)(pow((x - input_x), 2) + pow((y - input_y), 2));
				sigma += r_square;

				float pixel[2] = { (float)x, (float)y };
				float dist = (float)((float)range_data[x] * range_scale_ + range_offset_ / 1000.0);	// convert unit from 'mm' to 'm'
				float point3d[3];
				rs2_deproject_pixel_to_point(point3d, intrinsics, pixel, dist);

				// temporary weight is r = sqrt((xi - x)^2 + (yi - y)^2)
				std::vector<float> point3d_vec;
				for (int i = 0; i < 3; i++)
				{
					point3d_vec.push_back(point3d[i]);
				}

				point_weight_vector.push_back(std::pair<double, std::vector<float>>(sqrt(r_square), point3d_vec));
				nb += 1;
			}
		}
		range_data += range_pitch;
	}
	MbufFree(smooth_image);

	if (nb == 0)
		return false;

	sigma = sqrt(sigma / nb);
	double s = 2.0 * sigma * sigma;

	double sum = 0.0;
	for (int i = 0; i < point_weight_vector.size(); i++)
	{
		double r = point_weight_vector[i].first;
		double weight = (exp(-(r * r) / s)) / (PI * s);
		point_weight_vector[i].first = weight;	// update weight
		sum += weight;
	}

	output_x = 0;
	output_y = 0;
	output_z = 0;

	for (int i = 0; i < point_weight_vector.size(); i++)
	{
		double weight = point_weight_vector[i].first / sum;
		output_x += point_weight_vector[i].second[0] * weight;
		output_y += point_weight_vector[i].second[1] * weight;
		output_z += point_weight_vector[i].second[2] * weight;
	}

	return true;
}