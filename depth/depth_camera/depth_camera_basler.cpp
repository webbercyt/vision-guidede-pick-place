#include "depth_camera_basler.h"
#include "utils.h"

#define PI 3.14159265

BaslerToFCamera::BaslerToFCamera(MIL_ID mil_application,
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
		node_name),
	mil_grabbed_images_buffer_(M_NULL),
	require_images_(false)
{
	// deal with ip address
	auto camera_ip = workspace_ptr_->camera().at(node_name_).ip();

	// change camera ip from string format to decimal format
	unsigned char a, b, c, d;
	sscanf_s(camera_ip.c_str(), "%hhu.%hhu.%hhu.%hhu", &a, &b, &c, &d);
	unsigned long camera_ip_address = (a << 24) | (b << 16) | (c << 8) | d;

	MIL_INT digitizer_num = MsysInquire(mil_system, M_DIGITIZER_NUM, M_NULL);
	if (digitizer_num == 0)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "camera allocation failed!"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	MIL_INT dev_index = -1;
	for (MIL_INT n = 0; n < digitizer_num; n++)
	{
		MIL_INT64 gev_device_ip_address = 0;

		std::shared_ptr<wchar_t> wc_digiter_index = utils::string2Wchar_tPtr(std::to_string(n));
		MsysControlFeature(mil_system,
			M_GENTL_INTERFACE0 + M_FEATURE_VALUE,
			MIL_TEXT("DeviceSelector"),
			M_TYPE_STRING,
			wc_digiter_index.get());

		MsysInquireFeature(mil_system,
			M_GENTL_INTERFACE0 + M_FEATURE_VALUE,
			MIL_TEXT("GevDeviceIPAddress"),
			M_TYPE_INT64,
			&gev_device_ip_address);

		if (camera_ip_address == gev_device_ip_address)
		{
			dev_index = n;
			break;
		}
	}

	if (dev_index != -1 && MdigAlloc(mil_system, dev_index, MIL_TEXT("M_DEFAULT"), M_DEV_NUMBER, &mil_digitizer_) != M_NULL) {

		is_dig_alloc_succeed_ = true;

		config();

		// start continuous grab
		MdigProcess(mil_digitizer_, &mil_grabbed_images_buffer_[0], 2, M_START, M_DEFAULT, continuousGrabHook, this);
	}
	else {
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "camera allocation failed!"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}

	setFov();
}

BaslerToFCamera::~BaslerToFCamera()
{
	if (is_dig_alloc_succeed_)
		MdigProcess(mil_digitizer_, &mil_grabbed_images_buffer_[0], 2, M_STOP + M_WAIT, M_DEFAULT, continuousGrabHook, this);

	for (MIL_INT b = mil_grabbed_images_buffer_.size() - 1; b >= 0; b--)
		MbufFree(mil_grabbed_images_buffer_[b]);
}

bool BaslerToFCamera::singleGrab(bool display_image)
{
	if (!isGrabPreCheckSucceed())
	{
		return false;
	}

	MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	is_cur_frame_corrupted_ = true;
	std::unique_lock<std::mutex> lock(require_images_lock_);
	require_images_ = true;
	lock.unlock();

	while (1)
	{
		lock.lock();
		if (!require_images_)
			break;
		lock.unlock();
	}

	return !is_cur_frame_corrupted_;
}

void BaslerToFCamera::continuousGrabHelper()
{
	MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	while (!end_grab_)
	{
		std::unique_lock<std::mutex> lock(require_images_lock_);

		if (!require_images_)
		{
			require_images_ = true;
		}

		lock.unlock();
	}
}

void BaslerToFCamera::setFov()
{
	//the fov is set from https://www.baslerweb.com/en/products/cameras/3d-cameras/time-of-flight-camera/tof640-20gm_850nm/
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_h(57);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_v(43);
}

bool BaslerToFCamera::isGrabPreCheckSucceed() {

	if (Camera::isGrabPreCheckSucceed())
	{
		if (mil_range_image_ == M_NULL)
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Grab image failed! Range image buffer is NULL."); 
			if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}
	}

	return true;
}

void BaslerToFCamera::config()
{
	auto depth_camera_mutable_ptr = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera();

	// close confidence image grab
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("ComponentSelector"), M_TYPE_STRING, MIL_TEXT("Confidence"));
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("ComponentEnable"), M_TYPE_STRING, MIL_TEXT("0"));
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("PixelFormat"), M_TYPE_STRING, MIL_TEXT("Confidence16"));

	// open intensity image grab
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("ComponentSelector"), M_TYPE_STRING, MIL_TEXT("Intensity"));
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("ComponentEnable"), M_TYPE_STRING, MIL_TEXT("1"));
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("PixelFormat"), M_TYPE_STRING, MIL_TEXT("Mono16"));

	// open range image grab
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("ComponentSelector"), M_TYPE_STRING, MIL_TEXT("Range"));
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("ComponentEnable"), M_TYPE_STRING, MIL_TEXT("1"));
	MdigControlFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("PixelFormat"), M_TYPE_STRING, MIL_TEXT("Coord3D_C16"));

	MIL_INT64 min_depth_in_mm;
	MIL_INT64 max_depth_in_mm;
	MdigInquireFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("DepthMin"), M_TYPE_INT64, &min_depth_in_mm);
	MdigInquireFeature(mil_digitizer_, M_FEATURE_VALUE, MIL_TEXT("DepthMax"), M_TYPE_INT64, &max_depth_in_mm);

	depth_camera_mutable_ptr->set_range_image_offset(double(min_depth_in_mm));
	depth_camera_mutable_ptr->set_range_image_scale(double(max_depth_in_mm - min_depth_in_mm) / 65535.0);

	// set up image process parameters
	MIL_INT image_size_x = MdigInquire(mil_digitizer_, M_SIZE_X, M_NULL);
	MIL_INT image_size_y = MdigInquire(mil_digitizer_, M_SIZE_Y, M_NULL);

	MIL_DOUBLE frame_rate;
	MdigInquire(mil_digitizer_, M_SELECTED_FRAME_RATE, &frame_rate);
	require_images_wait_time_ = MIL_INT(1000.0 / frame_rate);

	MIL_ID grab_type = MdigInquire(mil_digitizer_, M_TYPE, M_NULL);
	MbufAlloc2d(mil_system_, image_size_x, image_size_y, 8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP + M_GRAB, &mil_image_);
	MbufAlloc2d(mil_system_, image_size_x, image_size_y, grab_type, M_IMAGE + M_PROC + M_DISP + M_GRAB, &mil_range_image_);
	depth_camera_mutable_ptr->set_range_image_size_x(google::protobuf::int32(image_size_x));
	depth_camera_mutable_ptr->set_range_image_size_y(google::protobuf::int32(image_size_y));

	mil_grabbed_images_buffer_.resize(2);
	MbufAllocDefault(mil_system_, mil_digitizer_, M_CONTAINER + M_3D_SCENE + M_GRAB + M_PROC, M_DEFAULT, M_DEFAULT, &mil_grabbed_images_buffer_[0]);
	MbufAllocDefault(mil_system_, mil_digitizer_, M_CONTAINER + M_3D_SCENE + M_GRAB + M_PROC, M_DEFAULT, M_DEFAULT, &mil_grabbed_images_buffer_[1]);

	// set mono image as default iamge for display
	setDisplayImage("MONO");
}

MIL_INT MFTYPE BaslerToFCamera::continuousGrabHook(MIL_INT hook_type, MIL_ID mil_hook_id, void* hook_data_ptr)
{
	BaslerToFCamera* cam_proc = (BaslerToFCamera*)hook_data_ptr;
	return cam_proc->grabContinuous(mil_hook_id);
}

MIL_INT BaslerToFCamera::grabContinuous(MIL_ID mil_hook_id)
{
	std::unique_lock<std::mutex> lock(require_images_lock_);
	if (require_images_)
	{
		MIL_INT index;
		MdigGetHookInfo(mil_hook_id, M_MODIFIED_BUFFER + M_BUFFER_INDEX, &index);

		MIL_ID mil_int_image = MbufInquire(mil_grabbed_images_buffer_[index], M_COMPONENT_ID(M_INTENSITY), M_NULL);
		MIL_ID mil_range_image = MbufInquire(mil_grabbed_images_buffer_[index], M_COMPONENT_ID(M_RANGE), M_NULL);

		if (mil_int_image && mil_range_image)
		{
			MimShift(mil_int_image, mil_image_, -8);
			MbufCopy(mil_range_image, mil_range_image_);

			is_cur_frame_corrupted_ = false;
		}
		else {
			is_cur_frame_corrupted_ = true;
		}

		require_images_ = false;
	}

	return 0;
}

BaslerToFDataWrapper::BaslerToFDataWrapper(MIL_ID mil_system,
	const Workspace_DepthCamera workspace_depth_camera,
	MIL_ID mil_calibration) :
	DepthCameraDataWrapper(mil_system,
		workspace_depth_camera,
		mil_calibration)
{
	x_unit_.resize(0);
	y_unit_.resize(0);
	z_unit_.resize(0);
}

void BaslerToFDataWrapper::composePointCloudPoint(MIL_ID range_image, double max_depth)
{
	MIL_INT image_size_x = MbufInquire(range_image, M_SIZE_X, M_NULL);
	MIL_INT image_size_y = MbufInquire(range_image, M_SIZE_Y, M_NULL);

	if (x_unit_.size() == 0 &&
		y_unit_.size() == 0 &&
		z_unit_.size() == 0)
	{
		// Fill the pixel coordinates.
		MIL_INT total_pixels = image_size_x * image_size_y;
		std::vector<MIL_DOUBLE> x_pixel(total_pixels);
		std::vector<MIL_DOUBLE> y_pixel(total_pixels);
		x_unit_.resize(total_pixels);
		y_unit_.resize(total_pixels);
		z_unit_.resize(total_pixels);

		for (MIL_INT y = 0, i = 0; y < image_size_y; y++)
		{
			for (MIL_INT x = 0; x < image_size_x; x++, i++)
			{
				x_pixel[i] = (MIL_DOUBLE)x;
				y_pixel[i] = (MIL_DOUBLE)y;
			}
		}

		// Get the unit vectors of each pixel of the range image.
		McalTransformCoordinate3dList(mil_calibration_,
			M_PIXEL_COORDINATE_SYSTEM,
			M_CAMERA_COORDINATE_SYSTEM,
			total_pixels,
			&x_pixel[0],
			&y_pixel[0],
			M_NULL,
			&x_unit_[0],
			&y_unit_[0],
			&z_unit_[0],
			M_UNIT_DIRECTION_VECTOR);
	}

	MIL_UINT16* range_data = (MIL_UINT16*)MbufInquire(range_image, M_HOST_ADDRESS, M_NULL);
	MIL_INT range_pitch = MbufInquire(range_image, M_PITCH, M_NULL);
	MIL_INT unit_index = 0;
	MIL_INT world_index = 0;

	if (max_depth > 0)
	{
		for (MIL_INT y = 0; y < image_size_y; y++)
		{
			for (MIL_INT x = 0; x < image_size_x; x++)
			{
				if (range_data[x] != 0 && z_unit_[unit_index] != 0)
				{
					MIL_DOUBLE range_world = range_data[x] * range_scale_ + range_offset_;
					MIL_DOUBLE z_world = range_world * z_unit_[unit_index];
					if (z_world <= max_depth)
					{
						// Calculate the world position of the point in the depth image.				
						x_world_[world_index] = range_world * x_unit_[unit_index];
						y_world_[world_index] = range_world * y_unit_[unit_index];
						z_world_[world_index] = z_world;
						world_index++;
					}
				}
				unit_index++;
			}
			range_data += range_pitch;
		}
	}
	else
	{
		for (MIL_INT y = 0; y < image_size_y; y++)
		{
			for (MIL_INT x = 0; x < image_size_x; x++)
			{
				if (range_data[x] != 0 && z_unit_[unit_index] != 0)
				{
					// Calculate the world position of the point in the depth image.
					MIL_DOUBLE range_world = range_data[x] * range_scale_ + range_offset_;
					x_world_[unit_index] = range_world * x_unit_[unit_index];
					y_world_[unit_index] = range_world * y_unit_[unit_index];
					z_world_[unit_index] = range_world * z_unit_[unit_index];
				}
				unit_index++;
			}
			range_data += range_pitch;
		}
	}
}
