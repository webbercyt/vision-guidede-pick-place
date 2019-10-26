#include "depth_camera_ty.h"
#include "utils.h"

#define RESIZE_SCALE 1/*2*/

CallbackData::CallbackData()
{
	depth_calib = new TY_CAMERA_CALIB_INFO();
	color_calib = new TY_CAMERA_CALIB_INFO();
}

CallbackData::~CallbackData()
{
	delete depth_calib;
	delete color_calib;
}

bool TYCamera::openDevice()
{
	std::vector<TY_DEVICE_BASE_INFO> selected;
	int ret = selectDevice(TY_INTERFACE_ALL/*TY_INTERFACE_USB*/, "", "", 1, selected);

	// check select status
	if (ret == TY_STATUS_ERROR)
		return false;	// error

	ASSERT(selected.size() > 0);
	TY_DEVICE_BASE_INFO& selectedDev = selected[0];

	ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &h_iface_));
	ASSERT_OK(TYOpenDevice(h_iface_, selectedDev.id, &h_device_));

	return true;
}

bool TYCamera::isGrabPreCheckSucceed()
{
	return true;
}

void TYCamera::handleFrame(TY_FRAME_DATA* frame, void* userdata)
{
	updateDepthRange();
	CallbackData* pData = (CallbackData*)userdata;
	
	cv::Mat depth, color;
	parseFrame(*frame, &depth, 0, 0, &color, NULL/*hColorIspHandle_*/);

	if(RESIZE_SCALE != 1)
		cv::resize(color, color, cv::Size(0, 0), 1.0 / (double)RESIZE_SCALE, 1.0 / (double)RESIZE_SCALE);

	if (!depth.empty() && !color.empty()) {

		bool map_depth_to_color = 0;
		cv::Mat undistort_color, out;

		doRegister(pData->depth_calib, pData->color_calib, depth, color, pData->needUndistort, undistort_color, out, 1);

#if DEBUG_RUNTIME
		clock_t t = clock();
#endif
		// compose mil data
		cv::Mat mono;
		cv::cvtColor(undistort_color, mono, cv::COLOR_BGR2GRAY);

		MbufPut2d(mil_image_, 0, 0, mono.cols, mono.rows, mono.data);
		MbufPut2d(mil_range_image_, 0, 0, out.cols, out.rows, out.data);

		// filter depth for display depth setting
		if (view_depth_min_ > 0 && view_depth_max_ > view_depth_min_)
		{
			// refer to https://percipiodc.readthedocs.io/en/latest/apiguides/index.html
			// range image pixel unit: mm (2 Byte)
			MimClip(mil_range_image_, mil_range_image_, M_OUT_RANGE, view_depth_min_, view_depth_max_, 0, 0);
		}

#if DEBUG_RUNTIME
		t = clock() - t;
		printf("[mil convert] It took me %d clicks (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
#endif
	}

	ASSERT_OK(TYEnqueueBuffer(pData->h_device_, frame->userBuffer, frame->bufferSize));
}

TYCamera::TYCamera(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	DepthCamera(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
}

TYCamera::~TYCamera()
{
	if (is_dig_alloc_succeed_)
	{
		ASSERT_OK(TYStopCapture(h_device_));
		ASSERT_OK(TYCloseDevice(h_device_));
		ASSERT_OK(TYCloseInterface(h_iface_));
		ASSERT_OK(TYDeinitLib());
	}
	else
	{
		TYStopCapture(h_device_);
		TYCloseDevice(h_device_);
		TYCloseInterface(h_iface_);
		TYDeinitLib();
	}

	if (frame_buffer_[0]) delete frame_buffer_[0];
	if (frame_buffer_[1]) delete frame_buffer_[1];
}

int TYCamera::setExposureTime(MIL_DOUBLE exposure_time)
{
	static const int exposure_min = 1;
	static const int exposure_max = 1088;

	int expo = int(exposure_time);
	
	if (exposure_time > exposure_max)
		expo = exposure_max;

	if (exposure_time < exposure_min)
		expo = exposure_min;

	/*TY_STATUS status = */TYSetInt(h_device_, TY_COMPONENT_RGB_CAM, TY_INT_EXPOSURE_TIME, expo);

	return ERR_NULL;
}

int TYCamera::setGain(MIL_DOUBLE gain)
{
	return ERR_NULL;
}

bool TYCamera::singleGrab(bool display_image)
{
	if (!isGrabPreCheckSucceed())
	{
		return false;
	}

	if (display_image)
	{
		if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
			MdispLut(mil_display_, M_DEFAULT);
		MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	TY_FRAME_DATA frame;
	int err = TYFetchFrame(h_device_, &frame, -1);
	if (err != TY_STATUS_OK) {
		is_cur_frame_corrupted_ = true;
	}
	else
	{
		is_cur_frame_corrupted_ = false;

#if DEBUG_RUNTIME
		clock_t t = clock();
#endif
		handleFrame(&frame, &cb_data_);
#if DEBUG_RUNTIME
		t = clock() - t;
		printf("It took me %d clicks (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
#endif
	}

	return !is_cur_frame_corrupted_;
}

void TYCamera::continuousGrabHelper()
{
	setDisplayImage(getDispImageComponents()[workspace_ptr_->camera().at(node_name_).depth_camera().display_image()]);
	MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	while (!end_grab_)
	{
		TY_FRAME_DATA frame;
		int err = TYFetchFrame(h_device_, &frame, -1);
		if (err != TY_STATUS_OK) {
			is_cur_frame_corrupted_ = true;
		}
		else
			is_cur_frame_corrupted_ = false;

		handleFrame(&frame, &cb_data_);
	}
}

void TYCamera::config()
{
	frame_buffer_[0] = nullptr;
	frame_buffer_[1] = nullptr;

	ASSERT_OK(TYInitLib());
	if (!openDevice())
		return;

	int32_t allComps;
	ASSERT_OK(TYGetComponentIDs(h_device_, &allComps));
	if (!(allComps & TY_COMPONENT_RGB_CAM)) {
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::no_rgb_camera)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// Configure components
	int32_t componentIDs = TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_RGB_CAM;
	ASSERT_OK(TYEnableComponents(h_device_, componentIDs));

	// ASSERT_OK( TYSetEnum(h_device_, TY_COMPONENT_RGB_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_YUYV_640x480) );
	bool hasUndistortSwitch, hasDistortionCoef;
	ASSERT_OK(TYHasFeature(h_device_, TY_COMPONENT_RGB_CAM, TY_BOOL_UNDISTORTION, &hasUndistortSwitch));
	ASSERT_OK(TYHasFeature(h_device_, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_DISTORTION, &hasDistortionCoef));
	if (hasUndistortSwitch) {
		ASSERT_OK(TYSetBool(h_device_, TY_COMPONENT_RGB_CAM, TY_BOOL_UNDISTORTION, true));
	}

	// Prepare image buffer
	uint32_t frameSize;
	ASSERT_OK(TYGetFrameBufferSize(h_device_, &frameSize));
	// Allocate & enqueue buffers
	frame_buffer_[0] = new char[frameSize];
	frame_buffer_[1] = new char[frameSize];
	ASSERT_OK(TYEnqueueBuffer(h_device_, frame_buffer_[0], frameSize));
	ASSERT_OK(TYEnqueueBuffer(h_device_, frame_buffer_[1], frameSize));

	bool hasTriggerParam = false;
	ASSERT_OK(TYHasFeature(h_device_, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTriggerParam));
	if (hasTriggerParam) {
		// Disable trigger mode
		TY_TRIGGER_PARAM trigger;
		trigger.mode = TY_TRIGGER_MODE_OFF;
		ASSERT_OK(TYSetStruct(h_device_, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)));
	}

	cb_data_.index = 0;
	cb_data_.h_device_ = h_device_;
	cb_data_.needUndistort = !hasUndistortSwitch && hasDistortionCoef;

	// Read depth calib info
	ASSERT_OK(TYGetStruct(h_device_, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA
		, cb_data_.depth_calib, sizeof(TY_CAMERA_CALIB_INFO)));

	// Read color calib info
	ASSERT_OK(TYGetStruct(h_device_, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA
		, cb_data_.color_calib, sizeof(TY_CAMERA_CALIB_INFO)));

	is_dig_alloc_succeed_ = true;

	// set up mil buffer
	int32_t w, h;
	ASSERT_OK(TYGetInt(h_device_, TY_COMPONENT_RGB_CAM, TY_INT_WIDTH, &w));
	ASSERT_OK(TYGetInt(h_device_, TY_COMPONENT_RGB_CAM, TY_INT_HEIGHT, &h));

	MIL_INT size_w = (MIL_INT)((double)w / (double)RESIZE_SCALE);
	MIL_INT size_h = (MIL_INT)((double)h / (double)RESIZE_SCALE);

	setExposureTime(workspace_ptr_->camera().at(node_name_).exposure());
	MbufAlloc2d(mil_system_, size_w, size_h, 8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &mil_image_);

	ASSERT_OK(TYGetInt(h_device_, TY_COMPONENT_DEPTH_CAM, TY_INT_WIDTH, &w));
	ASSERT_OK(TYGetInt(h_device_, TY_COMPONENT_DEPTH_CAM, TY_INT_HEIGHT, &h));
	MbufAlloc2d(mil_system_, size_w, size_h, 16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &mil_range_image_);

	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_x(google::protobuf::int32(size_w));
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_y(google::protobuf::int32(size_h));

	// mil params.
	auto depth_camera_mutable_ptr = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera();
	depth_camera_mutable_ptr->set_range_image_offset(0.0);
	depth_camera_mutable_ptr->set_range_image_scale(1.0);

	// save type name & color calib for point cloud projection
	auto process_3d_params = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->mutable_process_3d_params();

	// intrinsic
	process_3d_params->set_intrinsic_height(cb_data_.color_calib->intrinsicHeight);
	process_3d_params->set_intrinsic_width(cb_data_.color_calib->intrinsicWidth);

	int size = sizeof(cb_data_.color_calib->intrinsic.data) / sizeof(float);
	process_3d_params->clear_intrinsic_param();
	for (int i = 0; i < size; i++)
		process_3d_params->add_intrinsic_param(cb_data_.color_calib->intrinsic.data[i]);

	setDisplayImage(getDispImageComponents()[workspace_ptr_->camera().at(node_name_).depth_camera().display_image()]);

	// Start capture
	ASSERT_OK(TYStartCapture(h_device_));
}

FM810IXCamera::FM810IXCamera(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name):
	TYCamera(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
	config();
	setFov();
}

FM810IXCamera::~FM810IXCamera()
{
}

void FM810IXCamera::setFov()
{
	// https://percipiodc.readthedocs.io/en/latest/hwreference/FM810-IX-U2.html
	// use the fov of rgb camera since we project depth iamge to the rgb
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_h(53);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_v(37);
}

bool FM810IXCamera::openDevice()
{
	std::vector<TY_DEVICE_BASE_INFO> selected;
	int ret = selectDevice(TY_INTERFACE_USB, "", "", 1, selected);

	// check select status
	if (ret == TY_STATUS_ERROR)
		return false;	// error

	ASSERT(selected.size() > 0);
	TY_DEVICE_BASE_INFO& selectedDev = selected[0];

	ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &h_iface_));
	ASSERT_OK(TYOpenDevice(h_iface_, selectedDev.id, &h_device_));

	return true;
}

FM810GIXE1Camera::FM810GIXE1Camera(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	TYCamera(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
	config();
	setFov();

	MIL_INT size_x = MbufInquire(mil_image_, M_SIZE_X, M_NULL);
	MIL_INT size_y = MbufInquire(mil_image_, M_SIZE_Y, M_NULL);
	MbufAllocColor(mil_system, 3, size_x, size_y,
		8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &bayer_color_buf_);
}

FM810GIXE1Camera::~FM810GIXE1Camera()
{
	ASSERT_OK(TYISPRelease(&hColorIspHandle_));

	if (bayer_color_buf_) MbufFree(bayer_color_buf_);
}

void FM810GIXE1Camera::setFov()
{
	// https://percipiodc.readthedocs.io/en/latest/hwreference/FM810-GIX-E1.html
	// use the fov of rgb camera since we project depth iamge to the rgb
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_h(56);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_v(46);
}

bool FM810GIXE1Camera::openDevice()
{
	std::string camera_ip = workspace_ptr_->camera().at(node_name_).ip();
	std::vector<TY_DEVICE_BASE_INFO> selected;
	int ret = selectDevice(TY_INTERFACE_ETHERNET, "", /*"169.254.11.30"*/camera_ip, 1, selected);

	// check select status
	if (ret == TY_STATUS_ERROR)
		return false;	// error

	ASSERT(selected.size() > 0);
	TY_DEVICE_BASE_INFO& selectedDev = selected[0];

	ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &h_iface_));
	ASSERT_OK(TYOpenDevice(h_iface_, selectedDev.id, &h_device_));

	ASSERT_OK(TYISPCreate(&hColorIspHandle_));

	return true;
}

void FM810GIXE1Camera::handleFrame(TY_FRAME_DATA* frame, void* userdata)
{
	return TYCamera::handleFrame(frame, userdata);

	updateDepthRange();
	CallbackData* pData = (CallbackData*)userdata;

	cv::Mat depth, raw;
	parseFrameRaw(*frame, &depth, 0, 0, &raw, NULL/*hColorIspHandle_*/);

	if (RESIZE_SCALE != 1)
		cv::resize(raw, raw, cv::Size(0, 0), 1.0 / (double)RESIZE_SCALE, 1.0 / (double)RESIZE_SCALE);

	if (!depth.empty() && !raw.empty())
	{
		// bayer demosaicing
		MIL_ID bayer_raw_buf = MbufClone(mil_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
		MbufPut2d(bayer_raw_buf, 0, 0, raw.cols, raw.rows, raw.data);
		MbufBayer(bayer_raw_buf, bayer_color_buf_, M_DEFAULT, M_BAYER_GB + M_ADAPTIVE + M_COLOR_CORRECTION);

		// sharpen L channel
		MimConvert(bayer_color_buf_, bayer_color_buf_, M_RGB_TO_HLS);
		MIL_ID channel_L = MbufClone(mil_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
		MbufCopyColor(bayer_color_buf_, channel_L, 1);
		MimConvolve(channel_L, channel_L, M_SHARPEN_8);
		MbufCopyColor(channel_L, bayer_color_buf_, 1);
		MbufFree(channel_L);
		MimConvert(bayer_color_buf_, bayer_color_buf_, M_HLS_TO_RGB);
		
		// convert rgb to mono to do undistort
		MimConvert(bayer_color_buf_, bayer_raw_buf, M_RGB_TO_Y);
		std::vector<MIL_UINT8> data;
		MbufGet(bayer_raw_buf, data);
		raw = cv::Mat(raw.rows, raw.cols, CV_8U, &data[0]).clone();
		MbufFree(bayer_raw_buf);

		bool map_depth_to_color = 0;
		cv::Mat undistort, out;
		doRegisterMono(pData->depth_calib, pData->color_calib, depth, raw, pData->needUndistort, undistort, out, 1);

		// compose mil data
		MbufPut2d(mil_image_, 0, 0, undistort.cols, undistort.rows, undistort.data);

#if DEBUG_RUNTIME
		clock_t t = clock();
#endif

		MbufPut2d(mil_range_image_, 0, 0, out.cols, out.rows, out.data);

		// filter depth for display depth setting
		if (view_depth_min_ > 0 && view_depth_max_ > view_depth_min_)
		{
			// refer to https://percipiodc.readthedocs.io/en/latest/apiguides/index.html
			// range image pixel unit: mm (2 Byte)
			MimClip(mil_range_image_, mil_range_image_, M_OUT_RANGE, view_depth_min_, view_depth_max_, 0, 0);
		}

#if DEBUG_RUNTIME
		t = clock() - t;
		printf("[mil convert] It took me %d clicks (%f seconds).\n", t, ((float)t) / CLOCKS_PER_SEC);
#endif
	}

	ASSERT_OK(TYEnqueueBuffer(pData->h_device_, frame->userBuffer, frame->bufferSize));
}

TYDataWrapper::TYDataWrapper(MIL_ID mil_system, const Workspace_DepthCamera workspace_depth_camera, MIL_ID mil_calibration) :
	DepthCameraDataWrapper(mil_system,
		workspace_depth_camera,
		mil_calibration)
{
	pc_project_calib_ = new TY_CAMERA_CALIB_INFO();

	auto process_3d_params = workspace_depth_camera.process_3d_params();
	pc_project_calib_->intrinsicHeight = process_3d_params.intrinsic_height();
	pc_project_calib_->intrinsicWidth = process_3d_params.intrinsic_width();

	int size = process_3d_params.intrinsic_param().size();
	for (int i = 0; i < size; i++)
		pc_project_calib_->intrinsic.data[i] = float(process_3d_params.intrinsic_param(i));

	// no use of extrinsic & distortion
	for (int i = 0; i < 16; i++)
		pc_project_calib_->extrinsic.data[i] = 0;

	for (int i = 0; i < 12; i++)
		pc_project_calib_->distortion.data[i] = 0;

	depth_data_ = DepthCameraDataWrapper::setupDepthData(workspace_depth_camera);
}

TYDataWrapper::~TYDataWrapper()
{
	delete pc_project_calib_;
}

bool TYDataWrapper::get3DPointUsingNeighborFilter(
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

	MIL_UINT16* range_data = (MIL_UINT16*)MbufInquire(cond_range_image, M_HOST_ADDRESS, M_NULL);
	MIL_INT range_pitch = MbufInquire(cond_range_image, M_PITCH, M_NULL);
	range_data += range_pitch * y_start;

	MIL_DOUBLE mean_value = 0.0;
	MIL_INT nb = 0;
	std::vector<TY_PIXEL_DESC> p;
	p.reserve((y_end - y_start) * (x_end - x_start));

	for (MIL_INT y = y_start; y < y_end; y += 1)
	{
		for (MIL_INT x = x_start; x < x_end; x++)
		{
			if (range_data[x] != 0)
			{
				mean_value += (MIL_DOUBLE)(range_data[x]);
				nb++;

				// Calculate the world position of the point in the depth image.
				MIL_DOUBLE range_world;
				if (range_scale_ != 1.0 || range_offset_ != 0.0)
					range_world = range_data[x] * range_scale_ + range_offset_;
				else
					range_world = range_data[x];

				p.push_back(TY_PIXEL_DESC());
				p.back().x = int16_t(x);
				p.back().y = int16_t(y);
				p.back().depth = int16_t(range_world);
			}
		}
		range_data += range_pitch;
	}

	if (nb == 0)
		return false;

	mean_value /= (MIL_DOUBLE)(nb);

	std::vector<TY_VECT_3F> p3dp;
	p3dp.resize(p.size());
	TYMapDepthToPoint3d(/*&color_calib*/pc_project_calib_,
		(int)cols, (int)rows,
		&p[0], uint32_t(p.size()),
		&p3dp[0]);

	double x_sum = 0;
	double y_sum = 0;
	double z_sum = 0;
	for (int i = 0; i < p3dp.size(); i++)
	{
		x_sum += p3dp[i].x;
		y_sum += p3dp[i].y;
		z_sum += p3dp[i].z;
	}

	output_x = x_sum / p3dp.size();
	output_y = y_sum / p3dp.size();
	output_z = z_sum / p3dp.size();

	return true;
}

void TYDataWrapper::composePointCloudPoint(MIL_ID range_image, double max_depth)
{
	MIL_UINT16* range_data = (MIL_UINT16*)MbufInquire(range_image, M_HOST_ADDRESS, M_NULL);
	MIL_INT range_pitch = MbufInquire(range_image, M_PITCH, M_NULL);

	MIL_INT image_size_x = MbufInquire(range_image, M_SIZE_X, M_NULL);
	MIL_INT image_size_y = MbufInquire(range_image, M_SIZE_Y, M_NULL);

	std::vector<TY_PIXEL_DESC> p;
	p.reserve(image_size_y * image_size_x);
	for (MIL_INT y = 0; y < image_size_y; y++)
	{
		for (MIL_INT x = 0; x < image_size_x; x++)
		{
			if (range_data[x] != 0)
			{
				// Calculate the world position of the point in the depth image.
				MIL_DOUBLE range_world = range_data[x] * range_scale_ + range_offset_;

				p.push_back(TY_PIXEL_DESC());
				p.back().x = int16_t(x);
				p.back().y = int16_t(y);
				p.back().depth = int16_t(range_world);
			}
		}
		range_data += range_pitch;
	}

	if (p.size() < 1)
		return;

	MIL_INT cols = MbufInquire(range_image, M_SIZE_X, M_NULL);
	MIL_INT rows = MbufInquire(range_image, M_SIZE_Y, M_NULL);

	std::vector<TY_VECT_3F> p3dp;
	p3dp.resize(p.size());

	TYMapDepthToPoint3d(pc_project_calib_,
		(int)cols, (int)rows,
		&p[0], uint32_t(p.size()),
		&p3dp[0]);

	if (max_depth > 0)
	{
		int index = 0;
		for (int i = 0; i < p3dp.size(); i++)
		{
			if (p3dp[i].z <= max_depth)
			{
				x_world_[index] = p3dp[i].x;
				y_world_[index] = p3dp[i].y;
				z_world_[index] = p3dp[i].z;
				index++;
			}
		}
	}
	else
	{
		for (int i = 0; i < p3dp.size(); i++)
		{
			x_world_[i] = p3dp[i].x;
			y_world_[i] = p3dp[i].y;
			z_world_[i] = p3dp[i].z;
		}
	}
}