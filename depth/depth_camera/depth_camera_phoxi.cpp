#include "depth_camera_phoxi.h"
#include <windows.h>
#include "utils.h"
#include "opencv2/opencv.hpp" 

#define USE_SUPPORTED_MODE /*0*/1	// 0: 2064 * 1544	1: 1032 * 772
#define MAX_POINT_CLOUD_MAP_SIZE 1
#define PC_QUEUE_ZERO_INDEX 0

// The exposure time options from phoXi control app
static const double EXPOSURE_TIME[12] = {
	10.24, 
	14.335, 
	20.48, 
	24.576, 
	30.72, 
	34.816, 
	40.96, 
	49.15, 
	75.776, 
	79.872, 
	90.112, 
	100.352};

int PhotoXiScanner::pc_buffer_id_ = PC_QUEUE_ZERO_INDEX;
std::mutex PhotoXiScanner::point_cloud_map_lock_;
std::deque<std::pair<MIL_ID, PointCloud32f>> PhotoXiScanner::point_cloud_deque_;

/*----------------------------------------
			 PhotoXiScanner
----------------------------------------*/
PhotoXiScanner::PhotoXiScanner(MIL_ID mil_application,
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
		node_name),
	current_pc_lut_id_(M_NULL)
{
	connect();
	conifg();
}

PhotoXiScanner::~PhotoXiScanner()
{
	disconnect();
}

bool PhotoXiScanner::singleGrab(bool display_image)
{
	if (!isGrabPreCheckSucceed()) {
		return false;
	}

	if (display_image)
	{
		if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
			MdispLut(mil_display_, M_DEFAULT);
		MdispSelectWindow(mil_display_, mil_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	if (softwareTrigger())
	{
		is_cur_frame_corrupted_ = false;
		return true;
	}
	else 
	{
		is_cur_frame_corrupted_ = true;
		return false;
	}
}

void PhotoXiScanner::continuousGrabHelper()
{
	if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
		MdispLut(mil_display_, M_DEFAULT);

	MdispSelectWindow(mil_display_, mil_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	std::lock_guard<std::mutex> guard(image_grab_lock_);
	while (!end_grab_)
	{
		softwareTrigger();
	}
}

bool PhotoXiScanner::stopContinuousGrab()
{
	end_grab_ = true;
	std::lock_guard<std::mutex> guard(image_grab_lock_);
	return Camera::stopContinuousGrab();
}

int PhotoXiScanner::setExposureTime(MIL_DOUBLE exposure_time)
{
	if (!isConnected()) {
		return ERR_CAMERA_NOT_CONNECTED;
	}

	double _exposure_time = 0;
	int size = ((sizeof EXPOSURE_TIME) / (sizeof *EXPOSURE_TIME));

	double diff = std::numeric_limits<double>::max();
	for (int i = 0; i < size; i++)
	{
		double temp_diff = abs(exposure_time - EXPOSURE_TIME[i]);
		if (temp_diff < diff)
		{
			_exposure_time = EXPOSURE_TIME[i];
			diff = temp_diff;
		}
		else if (temp_diff >= diff)
		{
			break;
		}
	}

	// get capturing settings
	PhoXiCapturingSettings capturing_settings = phoXi_device_->CapturingSettings;
	if (!phoXi_device_->CapturingSettings.isLastOperationSuccessful())
		return ERR_CAMERA_GET_FEATURE_FAIL;

	// set capturing settings
	capturing_settings.SinglePatternExposure = _exposure_time;

	phoXi_device_->CapturingSettings = capturing_settings;
	if (!phoXi_device_->CapturingSettings.isLastOperationSuccessful())
		return ERR_CAMERA_SET_FEATURE_FAIL;

	phoXi_device_->CapturingSettings->SinglePatternExposure = _exposure_time;

	return ERR_NULL;
}

MIL_ID PhotoXiScanner::getRangeImageID()
{
	return current_pc_lut_id_;
}

bool PhotoXiScanner::saveRangeImage(std::string file_name, MIL_ID key)
{
	if (point_cloud_deque_.size() < 0)
		return false;

	MIL_ID valid_key = -1;
	if (key == M_NULL)
	{
		valid_key = current_pc_lut_id_;
	}
	else
	{
		valid_key = key;
	}

	bool has_index = false;
	int index = 0;
	for (int i = 0; i < point_cloud_deque_.size(); i++)
	{
		if (point_cloud_deque_[i].first == valid_key)
		{
			has_index = true;
			index = i;
			break;
		}
	}

	if (!has_index) return false;

	MIL_ID PCLUT = MbufAllocColor(mil_system_,
		3, image_size_x_, image_size_y_,
		32 + M_FLOAT, M_ARRAY, M_NULL);

	MIL_FLOAT *x_pos = (MIL_FLOAT *)malloc(image_size_x_ * image_size_y_ * sizeof(MIL_FLOAT));
	MIL_FLOAT *y_pos = (MIL_FLOAT *)malloc(image_size_x_ * image_size_y_ * sizeof(MIL_FLOAT));
	MIL_FLOAT *z_pos = (MIL_FLOAT *)malloc(image_size_x_ * image_size_y_ * sizeof(MIL_FLOAT));

	memset(x_pos, 0, image_size_x_ * image_size_y_ * sizeof(MIL_FLOAT));
	memset(y_pos, 0, image_size_x_ * image_size_y_ * sizeof(MIL_FLOAT));
	memset(z_pos, 0, image_size_x_ * image_size_y_ * sizeof(MIL_FLOAT));

	PointCloud32f pc = point_cloud_deque_[index].second;

	Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
	int index_offset = 0;
	for (int y = 0; y < pc.Size.Height; y++)
	{
		for (int x = 0; x < pc.Size.Width; x++)
		{
			Point3_32f pt = pc[y][x];
			if (pt != ZeroPoint) {
				x_pos[x + index_offset] = float(pt.x);
				y_pos[x + index_offset] = float(pt.y);
				z_pos[x + index_offset] = float(pt.z);
			}
		}
		index_offset += pc.Size.Width;
	}

	MbufPutColor2d(PCLUT, M_SINGLE_BAND, 0, 0, 0, image_size_x_, image_size_y_, x_pos);
	MbufPutColor2d(PCLUT, M_SINGLE_BAND, 1, 0, 0, image_size_x_, image_size_y_, y_pos);
	MbufPutColor2d(PCLUT, M_SINGLE_BAND, 2, 0, 0, image_size_x_, image_size_y_, z_pos);

	free(x_pos);
	free(y_pos);
	free(z_pos);

	// check folder existance
	if (utils::dirExists(utils::dirNameFromPath(file_name)) == -2) // this is a file instead of folder
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Please check image path ", file_name, '\n'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	else // create folder no matter it exist or not
	{
		if (utils::createFolder(utils::dirNameFromPath(file_name)))
		{
			MbufSave(utils::string2Wchar_tPtr(file_name).get(), PCLUT);
			MbufFree(PCLUT);
			return true;
		}
		else
		{
			MbufFree(PCLUT);
			return false;
		}
	}
}

bool PhotoXiScanner::setCameraOutput(CameraOutputMode mode)
{
	switch (mode)
	{
	case SHOW_ONLY:
	{
		PhoXiCapturingSettings capturing_settings = phoXi_device_->CapturingSettings;
		if (!phoXi_device_->CapturingSettings.isLastOperationSuccessful())
			return false;

		if (!capturing_settings.CameraOnlyMode)
		{
			capturing_settings.CameraOnlyMode = true;
			phoXi_device_->CapturingSettings = capturing_settings;
		}

		FrameOutputSettings output_settings = phoXi_device_->OutputSettings;

		if (output_settings.SendPointCloud ||
			output_settings.SendDepthMap ||
			output_settings.SendNormalMap ||
			output_settings.SendConfidenceMap ||
			!output_settings.SendTexture)
		{
			output_settings.SendPointCloud = false;
			output_settings.SendDepthMap = false;
			output_settings.SendNormalMap = false;
			output_settings.SendConfidenceMap = false;
			output_settings.SendTexture = true;

			phoXi_device_->OutputSettings = output_settings;
		}
		break;
	}
	case CALIBRATION:
	{
		PhoXiCapturingSettings capturing_settings = phoXi_device_->CapturingSettings;
		if (!phoXi_device_->CapturingSettings.isLastOperationSuccessful())
			return false;

		if (!capturing_settings.CameraOnlyMode)
		{
			capturing_settings.CameraOnlyMode = true;
			phoXi_device_->CapturingSettings = capturing_settings;
		}

		FrameOutputSettings output_settings = phoXi_device_->OutputSettings;

		if (output_settings.SendPointCloud ||
			output_settings.SendDepthMap ||
			output_settings.SendNormalMap ||
			output_settings.SendConfidenceMap ||
			!output_settings.SendTexture)
		{
			output_settings.SendPointCloud = false;
			output_settings.SendDepthMap = false;
			output_settings.SendNormalMap = false;
			output_settings.SendConfidenceMap = false;
			output_settings.SendTexture = true;

			phoXi_device_->OutputSettings = output_settings;
		}
		break;
	}
	case LOCALIZATON:
	{
		PhoXiCapturingSettings capturing_settings = phoXi_device_->CapturingSettings;
		if (!phoXi_device_->CapturingSettings.isLastOperationSuccessful())
			return false;

		if (capturing_settings.CameraOnlyMode)
		{
			capturing_settings.CameraOnlyMode = false;
			phoXi_device_->CapturingSettings = capturing_settings;
		}

		FrameOutputSettings output_settings = phoXi_device_->OutputSettings;

		if (!output_settings.SendPointCloud ||
			output_settings.SendDepthMap ||
			output_settings.SendNormalMap ||
			output_settings.SendConfidenceMap ||
			!output_settings.SendTexture)
		{
			output_settings.SendPointCloud = true;
			output_settings.SendDepthMap = false/*true*/;
			output_settings.SendNormalMap = false;
			output_settings.SendConfidenceMap = false;
			output_settings.SendTexture = true/*false*/;

			phoXi_device_->OutputSettings = output_settings;
		}

		break;
	}
	default:
		break;
	}

	return true;
}

PointCloud32f PhotoXiScanner::getPCLut(MIL_ID key)
{
	static const int INVALID_INDEX = -1;

	PointCloud32f ret_pc;
	std::unique_lock<std::mutex> lock(point_cloud_map_lock_);

	int index = INVALID_INDEX;
	for (int i = 0; i < point_cloud_deque_.size(); i++)
	{
		if (point_cloud_deque_[i].first == key)
		{
			index = i;
			break;
		}
	}

	if (index != INVALID_INDEX)
		ret_pc = point_cloud_deque_[index].second;
	else
		ret_pc = PointCloud32f(PhoXiSize(0, 0));

	lock.unlock();
	return ret_pc;
}

void PhotoXiScanner::connect() {
	//You can connect to any device connected to local network (with compatible ip4 settings)
	//The connection can be made in multiple ways

	PhoXiFactory phoXi_factory_;
	phoXi_factory_.MinimizePhoXiControl();

	std::vector <PhoXiDeviceInformation> device_list = phoXi_factory_.GetDeviceList();

	bool virtual_camera = false/*true*/;

	for (std::size_t i = 0; i < device_list.size(); i++) 
	{
		if (!device_list[i].Status.Ready)
			continue;

		if (virtual_camera)
		{
			if (0 >= device_list.size()) {
				std::cout << "Bad Index, or not number!" << std::endl;
				return;
			}

			phoXi_device_ = phoXi_factory_.Create(device_list[0]);
			if (phoXi_device_) {
				if (phoXi_device_->Connect()) {
					std::cout << "Connection to the device " << device_list[0].HWIdentification << " was Successful!"
						<< std::endl;
				}
				else {
					std::cout << "Connection to the device " << device_list[0].HWIdentification << " was Unsuccessful!"
						<< std::endl;
				}
			}
		}
		else
		{
			if (device_list[i].FirmwareVersion == "")
				continue;

			std::string camera_ip = workspace_ptr_->camera().at(node_name_).ip();

			//phoXi_device_ = phoXi_factory_.CreateAndConnect(device_list[i].HWIdentification, device_list[i].Type, camera_ip);
			phoXi_device_ = phoXi_factory_.CreateAndConnect(device_list[i].HWIdentification);

			if (phoXi_device_) {
				std::cout << "Connection to the device " << (std::string) phoXi_device_->HardwareIdentification
					<< " was Successful!" << std::endl;
				break;
			}
			else {
				std::cout << "There is no attached device, or the device is not ready!" << std::endl;
			}
		}
	}

	if (phoXi_device_ && phoXi_device_->isConnected())
		is_dig_alloc_succeed_ = true;
}

void PhotoXiScanner::conifg() 
{
	if (!is_dig_alloc_succeed_)
		return;

	if (!phoXi_device_->CapturingSettings.isEnabled() || 
		!phoXi_device_->CapturingSettings.CanSet() || 
		!phoXi_device_->CapturingSettings.CanGet()) 
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Conifg is Not supported in the Device Hardware"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// set the trigger mode to Software Trigger
	if (phoXi_device_->TriggerMode != pho::api::PhoXiTriggerMode::Software)
	{
		if (phoXi_device_->isAcquiring())
		{
			if (!phoXi_device_->StopAcquisition())
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Error in Stop Acquistion"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return;
			}
		}
		phoXi_device_->TriggerMode = pho::api::PhoXiTriggerMode::Software;

		if (!phoXi_device_->TriggerMode.isLastOperationSuccessful())
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Trigger error: ", phoXi_device_->TriggerMode.GetLastErrorMessage()); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return;
		}
	}

	// start acqusition
	if (!phoXi_device_->isAcquiring())
	{
		if (!phoXi_device_->StartAcquisition())
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Error in Start Acquistion"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return;
		}
	}

	if (!setCameraOutput(CameraOutputMode::SHOW_ONLY))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Error in switch to camera only mode"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}

	if (!phoXi_device_->CapturingSettings.isLastOperationSuccessful())
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Capture error: ", phoXi_device_->CapturingSettings.GetLastErrorMessage()); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	//Try to change device resolution
	if (phoXi_device_->SupportedCapturingModes.isEnabled() && phoXi_device_->SupportedCapturingModes.CanGet()
		&& phoXi_device_->CapturingMode.isEnabled() && phoXi_device_->CapturingMode.CanSet()
		&& phoXi_device_->CapturingMode.CanGet())
	{
		//Get all supported modes
		std::vector<pho::api::PhoXiCapturingMode> supported_capturing_modes = phoXi_device_->SupportedCapturingModes;
		if (!phoXi_device_->SupportedCapturingModes.isLastOperationSuccessful())
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Set capture mode error: ", phoXi_device_->SupportedCapturingModes.GetLastErrorMessage()); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return;
		}

		int support_mode_index = USE_SUPPORTED_MODE;
		if (USE_SUPPORTED_MODE >= supported_capturing_modes.size())
			support_mode_index = 0;

		if (!(supported_capturing_modes[support_mode_index] == phoXi_device_->CapturingMode))
		{
			phoXi_device_->CapturingMode = supported_capturing_modes[support_mode_index];
			if (!phoXi_device_->CapturingMode.isLastOperationSuccessful())
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Set capture mode error: ", phoXi_device_->CapturingMode.GetLastErrorMessage()); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return;
			}
		}

		if (mil_image_)
		{
			MbufFree(mil_image_);
		}

		
		pho::api::PhoXiCapturingMode current_capturing_mode = phoXi_device_->CapturingMode;
		image_size_x_ = current_capturing_mode.Resolution.Width;
		image_size_y_ = current_capturing_mode.Resolution.Height;

		MbufAlloc2d(mil_system_,
			image_size_x_,
			image_size_y_,
			8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &mil_image_);

		workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_x(google::protobuf::int32(image_size_x_));
		workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_y(google::protobuf::int32(image_size_y_));
	}

	// set exposure time
	int exposure_time = workspace_ptr_->camera().at(node_name_).exposure();
	setExposureTime((double)exposure_time);
}

void PhotoXiScanner::disconnect() 
{
	if (is_dig_alloc_succeed_)
	{
		if (phoXi_device_->isAcquiring())
			phoXi_device_->StopAcquisition();

		phoXi_device_->Disconnect(/*true*/false);
	}
}

bool PhotoXiScanner::softwareTrigger()
{
	if (is_dig_alloc_succeed_)
	{
		if (phoXi_device_->TriggerMode != pho::api::PhoXiTriggerMode::Software) {
			return false;
		}

		if (!phoXi_device_->isAcquiring()) {
			if (!phoXi_device_->StartAcquisition()) {
				return false;
			}
		}

		if (!phoXi_device_->isAcquiring())
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Grab image failed! Camera connection is incorrect."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		phoXi_device_->ClearBuffer();		 
		if (phoXi_device_->isAcquiring()) 
		{
			int frame_id = phoXi_device_->TriggerFrame();
			if (frame_id < 0) return false;

			PFrame Frame = phoXi_device_->GetSpecificFrame(frame_id, PhoXiTimeout(5000));
			if (Frame)
			{
				if (!Frame->Empty()) 
				{
					if (!Frame->PointCloud.Empty())
					{
						std::unique_lock<std::mutex> lock(point_cloud_map_lock_);
						while (point_cloud_deque_.size() >= MAX_POINT_CLOUD_MAP_SIZE)
						{
							point_cloud_deque_.pop_front();
							pc_buffer_id_ = PC_QUEUE_ZERO_INDEX;
						}

						current_pc_lut_id_ = ++pc_buffer_id_;
						point_cloud_deque_.push_back(std::pair<MIL_ID, PointCloud32f>(current_pc_lut_id_, Frame->PointCloud));
						lock.unlock();
					}
					if (!Frame->Texture.Empty()) 
					{
						Frame->Texture.At(0, 0) = 0;
						Frame->Texture.At(1, 0) = 4095;

						cv::Mat Destination = cv::Mat(cv::Size(Frame->Texture.Size.Width, Frame->Texture.Size.Height), cv::DataType<Intensity_32f::ElementChannelType>::type);
						for (int y = 0; y < Destination.rows; y++) 
						{
							memcpy(Destination.ptr<Intensity_32f::ElementChannelType>(y),
								Frame->Texture[y],
								Frame->Texture.Size.Width * Intensity_32f::ElementSize);
						}

						// clip max value to 4095 for bright view
						cv::threshold(Destination, Destination, 4095, 4095, cv::THRESH_TRUNC);
						cv::normalize(Destination, Destination, 0, 255, cv::NORM_MINMAX, CV_8UC1);
						MbufPut2d(mil_image_, 0, 0, Destination.cols, Destination.rows, Destination.data);
					}
					if (!Frame->DepthMap.Empty())
					{
						Frame->DepthMap.At(0, 0) = 0;
						Frame->DepthMap.At(1, 0) = 4095;

						cv::Mat Destination = cv::Mat(cv::Size(Frame->DepthMap.Size.Width, Frame->DepthMap.Size.Height), cv::DataType<Intensity_32f::ElementChannelType>::type);
						for (int y = 0; y < Destination.rows; y++)
						{
							memcpy(Destination.ptr<Intensity_32f::ElementChannelType>(y),
								Frame->DepthMap[y],
								Frame->DepthMap.Size.Width * Intensity_32f::ElementSize);
						}

						// clip max value to 4095 for bright view
						//cv::threshold(Destination, Destination, 4095, 4095, cv::THRESH_TRUNC);
						cv::normalize(Destination, Destination, 0, 255, cv::NORM_MINMAX, CV_8UC1);
						MbufPut2d(mil_image_, 0, 0, Destination.cols, Destination.rows, Destination.data);
					}
				}
				else 
				{
					std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Frame is empty."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
					return false;
				}
			}
			else 
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Failed to retrieve the frame!"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false;
			}
		}
	}

	return true;
}

/*----------------------------------------
			 PhotoXiScannerM
----------------------------------------*/
PhotoXiScannerM::PhotoXiScannerM(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name):
	PhotoXiScanner(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
	if (!is_dig_alloc_succeed_)
		return;

	// set from https://www.photoneo.com/products/phoxi-scan-m/
	static double NEAR_Z = 458.0;
	static double SWEET_Z = 650.0;
	static double FAR_Z = 1118.0;

	static double NEAR_X = 317.0;
	static double SWEET_X = 590.0;
	static double FAR_X = 826.0;

	static double NEAR_Y = 303.0;
	static double SWEET_Y = 421.0;
	static double FAR_Y = 877.0;

	// mil params.
	auto depth_camera_mutable_ptr = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera();
	depth_camera_mutable_ptr->set_range_image_offset(0.0);
	depth_camera_mutable_ptr->set_range_image_scale(1.0);

	auto view_depth_max = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_max();
	auto view_depth_min = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_min();

	if (view_depth_max < NEAR_Z)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "The view depth max must be greater than", NEAR_Z, 
			", automatically set to:", SWEET_Z); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		view_depth_max = (int)SWEET_Z;
	}

	double view_half_h = 0.0, view_half_v = 0.0;
	if (view_depth_max >= NEAR_Z && view_depth_max < SWEET_Z)
	{
		// (Z - Near_Z) / (Sweet_Z - Near_Z) = (X - Near_X) / (Sweet_X - Near_X)
		double z = view_depth_max;
		double z_ratio = (z - NEAR_Z) / (SWEET_Z - NEAR_Z);

		double x = z_ratio * (SWEET_X - NEAR_X) + NEAR_X;
		double y = z_ratio * (SWEET_Y - NEAR_Y) + NEAR_Y;
		view_half_h = x / 2;
		view_half_v = y / 2;
	}
	else if (view_depth_max >= SWEET_Z /*&& view_depth_max <= FAR_Z*/)
	{
		double z = view_depth_max;
		double z_ratio = (z - SWEET_Z) / (FAR_Z - SWEET_Z);

		double x = z_ratio * (FAR_X - SWEET_X) + SWEET_X;
		double y = z_ratio * (FAR_Y - SWEET_Y) + SWEET_Y;
		view_half_h = x / 2;
		view_half_v = y / 2;
	}

	depth_camera_mutable_ptr->set_view_start_pos_x(-view_half_h);
	depth_camera_mutable_ptr->set_view_start_pos_y(-view_half_v);
	depth_camera_mutable_ptr->set_view_start_pos_z(view_depth_min);

	depth_camera_mutable_ptr->set_view_size_x(view_half_h * 2.0);
	depth_camera_mutable_ptr->set_view_size_y(view_half_v * 2.0);
	depth_camera_mutable_ptr->set_view_size_z(view_depth_max - view_depth_min);

	double depth_map_size_x = (double)image_size_x_;
	double depth_map_size_y = (double)image_size_y_;

	if (view_half_h / view_half_v > depth_map_size_x / depth_map_size_y)
	{
		// decrease y
		depth_map_size_y = depth_map_size_x / (view_half_h / view_half_v);
	}
	else if (view_half_h / view_half_v < depth_map_size_x / depth_map_size_y)
	{
		// decrease x
		depth_map_size_x = depth_map_size_y * (view_half_h / view_half_v);
	}

	double depth_map_mm_per_pixel =
		view_half_h / depth_map_size_x + view_half_v / depth_map_size_y;

	depth_camera_mutable_ptr->set_depth_map_mm_per_pixel(depth_map_mm_per_pixel);
	depth_camera_mutable_ptr->set_depth_map_size_x(google::protobuf::int32(depth_map_size_x));
	depth_camera_mutable_ptr->set_depth_map_size_y(google::protobuf::int32(depth_map_size_y));
}

/*----------------------------------------
			 PhotoXiScannerL
----------------------------------------*/
PhotoXiScannerL::PhotoXiScannerL(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	PhotoXiScanner(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
	if (!is_dig_alloc_succeed_)
		return;

	// set from https://www.photoneo.com/products/phoxi-scan-l/
	static double NEAR_Z = 870.0;
	static double SWEET_Z = 1239.0;
	static double FAR_Z = 2150.0;

	static double NEAR_X = 600.0;
	static double SWEET_X = 1082.0;
	static double FAR_X = 1644.0;

	static double NEAR_Y = 568.0;
	static double SWEET_Y = 802.0;
	static double FAR_Y = 1388.0;

	// mil params.
	auto depth_camera_mutable_ptr = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera();
	depth_camera_mutable_ptr->set_range_image_offset(0.0);
	depth_camera_mutable_ptr->set_range_image_scale(1.0);

	auto view_depth_max = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_max();
	auto view_depth_min = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_min();

	if (view_depth_max < NEAR_Z)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "The view depth max must be greater than", NEAR_Z,
			", automatically set to:", SWEET_Z); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		view_depth_max = (int)SWEET_Z;
	}

	double view_half_h = 0.0, view_half_v = 0.0;
	if (view_depth_max >= NEAR_Z && view_depth_max < SWEET_Z)
	{
		// (Z - Near_Z) / (Sweet_Z - Near_Z) = (X - Near_X) / (Sweet_X - Near_X)
		double z = view_depth_max;
		double z_ratio = (z - NEAR_Z) / (SWEET_Z - NEAR_Z);

		double x = z_ratio * (SWEET_X - NEAR_X) + NEAR_X;
		double y = z_ratio * (SWEET_Y - NEAR_Y) + NEAR_Y;
		view_half_h = x / 2;
		view_half_v = y / 2;
	}
	else if (view_depth_max >= SWEET_Z /*&& view_depth_max <= FAR_Z*/)
	{
		double z = view_depth_max;
		double z_ratio = (z - SWEET_Z) / (FAR_Z - SWEET_Z);

		double x = z_ratio * (FAR_X - SWEET_X) + SWEET_X;
		double y = z_ratio * (FAR_Y - SWEET_Y) + SWEET_Y;
		view_half_h = x / 2;
		view_half_v = y / 2;
	}

	depth_camera_mutable_ptr->set_view_start_pos_x(-view_half_h);
	depth_camera_mutable_ptr->set_view_start_pos_y(-view_half_v);
	depth_camera_mutable_ptr->set_view_start_pos_z(view_depth_min);

	depth_camera_mutable_ptr->set_view_size_x(view_half_h * 2.0);
	depth_camera_mutable_ptr->set_view_size_y(view_half_v * 2.0);
	depth_camera_mutable_ptr->set_view_size_z(view_depth_max - view_depth_min);

	double depth_map_size_x = (double)image_size_x_;
	double depth_map_size_y = (double)image_size_y_;

	if (view_half_h / view_half_v > depth_map_size_x / depth_map_size_y)
	{
		// decrease y
		depth_map_size_y = depth_map_size_x / (view_half_h / view_half_v);
	}
	else if (view_half_h / view_half_v < depth_map_size_x / depth_map_size_y)
	{
		// decrease x
		depth_map_size_x = depth_map_size_y * (view_half_h / view_half_v);
	}

	double depth_map_mm_per_pixel = 
		view_half_h / depth_map_size_x + view_half_v / depth_map_size_y;

	depth_camera_mutable_ptr->set_depth_map_mm_per_pixel(depth_map_mm_per_pixel);
	depth_camera_mutable_ptr->set_depth_map_size_x(google::protobuf::int32(depth_map_size_x));
	depth_camera_mutable_ptr->set_depth_map_size_y(google::protobuf::int32(depth_map_size_y));
}

/*----------------------------------------
			 PhoXiDataWrapper
----------------------------------------*/
PhoXiDataWrapper::PhoXiDataWrapper(MIL_ID mil_system,
	const Workspace_DepthCamera workspace_depth_camera,
	MIL_ID mil_calibration) :
	DepthCameraDataWrapper(mil_system,
		workspace_depth_camera,
		mil_calibration)
{
	depth_data_ = PhoXiDataWrapper::setupDepthData(workspace_depth_camera);
}

bool PhoXiDataWrapper::createDepthMap(MIL_ID pc_lut_id,
	MIL_ID mask_image,
	MIL_ID depth_map,
	MIL_ID pc_label,
	double max_depth,
	double,
	bool inquire_camera,
	bool,
	bool,
	SPoseDataQuat transform)
{
	// create depth map from pc
	if (inquire_camera)
	{
		// @ load from memory

		Point3_32f ZeroPoint(0.0f, 0.0f, 0.0f);
		PointCloud32f pc = PhotoXiScanner::getPCLut(pc_lut_id);

		if (pc.Size == PhoXiSize(0, 0))
			return false;

		if (x_world_.size() == 0 &&
			y_world_.size() == 0 &&
			z_world_.size() == 0)
		{
			int total_pixels = pc.Size.Height * pc.Size.Width;
			x_world_.resize(total_pixels);
			y_world_.resize(total_pixels);
			z_world_.resize(total_pixels);
		}

		std::fill(x_world_.begin(), x_world_.end(), M_INVALID_POINT);
		std::fill(y_world_.begin(), y_world_.end(), M_INVALID_POINT);
		std::fill(z_world_.begin(), z_world_.end(), M_INVALID_POINT);

		if (mask_image != M_NULL)
		{
			MIL_UINT8* mask_data = (MIL_UINT8*)MbufInquire(mask_image, M_HOST_ADDRESS, M_NULL);
			MIL_INT mask_pitch = MbufInquire(mask_image, M_PITCH, M_NULL);

			int index = 0;
			for (int y = 0; y < pc.Size.Height; y++)
			{
				for (int x = 0; x < pc.Size.Width; x++)
				{
					if (mask_data[x] != 0)
					{
						Point3_32f pt = pc[y][x];
						if (pt == ZeroPoint)
							continue;

						x_world_[index] = float(pt.x);
						y_world_[index] = float(pt.y);
						z_world_[index] = float(pt.z);
						index++;
					}
				}
				mask_data += mask_pitch;
			}
		}
		else
		{
			int index = 0;
			for (int y = 0; y < pc.Size.Height; y++)
			{
				for (int x = 0; x < pc.Size.Width; x++)
				{
					Point3_32f pt = pc[y][x];
					if (pt != ZeroPoint)
					{
						x_world_[index] = float(pt.x);
						y_world_[index] = float(pt.y);
						z_world_[index] = float(pt.z);
						index++;
					}
				}
			}
		}
	}
	else
	{
		// @ load from composed MIL color image
		MIL_INT image_size_x = MbufInquire(pc_lut_id, M_SIZE_X, M_NULL);
		MIL_INT image_size_y = MbufInquire(pc_lut_id, M_SIZE_Y, M_NULL);

		MIL_FLOAT *x_pos = (MIL_FLOAT *)malloc(image_size_x * image_size_y * sizeof(MIL_FLOAT));
		MIL_FLOAT *y_pos = (MIL_FLOAT *)malloc(image_size_x * image_size_y * sizeof(MIL_FLOAT));
		MIL_FLOAT *z_pos = (MIL_FLOAT *)malloc(image_size_x * image_size_y * sizeof(MIL_FLOAT));

		memset(x_pos, 0, image_size_x * image_size_y * sizeof(MIL_FLOAT));
		memset(y_pos, 0, image_size_x * image_size_y * sizeof(MIL_FLOAT));
		memset(z_pos, 0, image_size_x * image_size_y * sizeof(MIL_FLOAT));

		MbufGetColor2d(pc_lut_id, M_SINGLE_BAND, 0, 0, 0, image_size_x, image_size_y, x_pos);
		MbufGetColor2d(pc_lut_id, M_SINGLE_BAND, 1, 0, 0, image_size_x, image_size_y, y_pos);
		MbufGetColor2d(pc_lut_id, M_SINGLE_BAND, 2, 0, 0, image_size_x, image_size_y, z_pos);

		if (x_world_.size() == 0 &&
			y_world_.size() == 0 &&
			z_world_.size() == 0)
		{
			int total_pixels = (int)(image_size_x * image_size_y);
			x_world_.resize(total_pixels);
			y_world_.resize(total_pixels);
			z_world_.resize(total_pixels);
		}

		if (mask_image != M_NULL)
		{
			MIL_UINT8* mask_data = (MIL_UINT8*)MbufInquire(mask_image, M_HOST_ADDRESS, M_NULL);
			MIL_INT mask_pitch = MbufInquire(mask_image, M_PITCH, M_NULL);

			int index = 0;
			mask_data += mask_pitch;
			for (int y = 0; y < image_size_y; y++)
			{
				for (int x = 0; x < image_size_x; x++)
				{
					if (mask_data[x] != 0)
					{
						if (x_pos[index] || y_pos[index] || z_pos[index])
						{
							x_world_[index] = x_pos[index];
							y_world_[index] = y_pos[index];
							z_world_[index] = z_pos[index];
						}

					}
					index++;
				}
				mask_data += mask_pitch;
			}
		}
		else
		{
			int index = 0;
			for (int y = 0; y < image_size_y; y++)
			{
				for (int x = 0; x < image_size_x; x++)
				{
					if (x_pos[index] || y_pos[index] || z_pos[index])
					{
						x_world_[index] = x_pos[index];
						y_world_[index] = y_pos[index];
						z_world_[index] = z_pos[index];
					}
					index++;
				}
			}
		}

		free(x_pos);
		free(y_pos);
		free(z_pos);
	}

	if (max_depth > 0.0)
	{
		MIL_INT actual_point_nb = x_world_.size();

		std::vector<MIL_DOUBLE> x_buf(actual_point_nb);
		std::vector<MIL_DOUBLE> y_buf(actual_point_nb);
		std::vector<MIL_DOUBLE> z_buf(actual_point_nb);

		McalSetCoordinateSystem(mil_calibration_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM, M_TRANSLATION + M_ASSIGN,
			M_NULL, transform.x, transform.y, transform.z, M_DEFAULT);

		McalSetCoordinateSystem(mil_calibration_, M_RELATIVE_COORDINATE_SYSTEM, M_RELATIVE_COORDINATE_SYSTEM, M_ROTATION_QUATERNION + M_COMPOSE_WITH_CURRENT,
			M_NULL, transform.q1, transform.q2, transform.q3, transform.q4);

		McalTransformCoordinate3dList(mil_calibration_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
			actual_point_nb, x_world_, y_world_, z_world_, x_buf, y_buf, z_buf, M_DEFAULT);

		for (int i = 0; i < z_buf.size(); i++)
		{
			if (z_buf[i] < -max_depth)
			{
				x_world_[i] = M_INVALID_POINT;
				y_world_[i] = M_INVALID_POINT;
				z_world_[i] = M_INVALID_POINT;
			}
		}
	}

	// Put the relative coordinate system on the camera coordinate system.
	McalSetCoordinateSystem(mil_calibration_,
		M_RELATIVE_COORDINATE_SYSTEM,
		M_CAMERA_COORDINATE_SYSTEM,
		M_IDENTITY,
		M_NULL,
		M_DEFAULT,
		M_DEFAULT,
		M_DEFAULT,
		M_DEFAULT);

	// Add the points to the point cloud.
	M3dmapPut(point_cloud_, PC_LABEL(pc_label), M_POSITION, 64 + M_FLOAT,
		x_world_.size(), &x_world_[0], &y_world_[0], &z_world_[0], mil_calibration_, M_DEFAULT);

	McalSetCoordinateSystem(mil_calibration_,
		M_RELATIVE_COORDINATE_SYSTEM,
		M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY,
		M_NULL,
		M_DEFAULT,
		M_DEFAULT,
		M_DEFAULT,
		M_DEFAULT);

	// Creates the depth map from the point cloud
	M3dmapControl(point_cloud_, M_GENERAL, M_EXTRACTION_OVERLAP, M_MAX);
	M3dmapSetBox(point_cloud_, M_EXTRACTION_BOX, M_CORNER_AND_DIMENSION,
		depth_data_.depth_view_x_pose_,
		depth_data_.depth_view_y_pose_,
		depth_data_.depth_view_z_pose_,
		depth_data_.depth_view_x_size_,
		depth_data_.depth_view_y_size_,
		depth_data_.depth_view_z_size_);

	MIL_INT point_nb = M3dmapGet(point_cloud_,
		PC_LABEL(pc_label),
		M_POSITION,
		M_EXCLUDE_INVALID_POINTS + M_INCLUDE_POINTS_INSIDE_BOX_ONLY,
		M_FLOAT + 64,
		M_NULL,
		M_NULL,
		M_NULL,
		M_NULL,
		M_NULL);

	if (point_nb < 1)
	{
		std::string o_msg = printCerrMsg(LEVEL_DETAIL_INFO, "No data inside point cloud."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	if (depth_map)
		createDepthMapFromPointCloud(depth_map, 0, false, false, pc_label);

	return true;
}

SDepthData PhoXiDataWrapper::setupDepthData(config::Workspace_DepthCamera workspace_depth_camera)
{
	SDepthData ret = SDepthData();

	ret.depth_view_x_pose_ = workspace_depth_camera.view_start_pos_x();
	ret.depth_view_y_pose_ = workspace_depth_camera.view_start_pos_y();
	ret.depth_view_z_pose_ = workspace_depth_camera.view_start_pos_z();
	ret.depth_view_x_size_ = workspace_depth_camera.view_size_x();
	ret.depth_view_y_size_ = workspace_depth_camera.view_size_y();
	ret.depth_view_z_size_ = workspace_depth_camera.view_size_z();

	ret.depth_map_mm_per_pixel_ = workspace_depth_camera.depth_map_mm_per_pixel();

	ret.depth_map_size_x_ = workspace_depth_camera.depth_map_size_x();
	ret.depth_map_size_y_ = workspace_depth_camera.depth_map_size_y();

	return ret;
}