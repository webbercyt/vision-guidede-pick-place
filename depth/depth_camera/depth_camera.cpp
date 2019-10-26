#include "depth_camera.h"
#include "depth_camera_basler.h"
#include "depth_camera_ty.h"
#include "depth_camera_phoxi.h"
#include "depth_camera_realsense.h"

#include "utils.h"

static std::vector<std::string> display_components = {"MONO", "RANGE"};

SDepthData::SDepthData()
{
	depth_view_x_pose_ = 0;
	depth_view_x_size_ = 0;
	depth_view_y_pose_ = 0;
	depth_view_y_size_ = 0;
	depth_view_z_pose_ = 0;
	depth_view_z_size_ = 0;

	depth_map_mm_per_pixel_ = 0;
	depth_map_size_x_ = 0;
	depth_map_size_y_ = 0;
}

enum DepthCameraMdoel {
	CM_NULL,
	CM_BaslerToFCamera, 
	CM_FM810IXCamera,
	CM_FM810GIXE1Camera,
	CM_PhotoXiScannerM,
	CM_PhotoXiScannerL,
	CM_REALSENSE_D415
};

namespace DepthCameraManager
{
	static std::vector<std::pair<DepthCameraMdoel, const std::string>> model_model_name_vector = {
		std::pair<DepthCameraMdoel, const std::string>(CM_BaslerToFCamera, "BL-ToF"),
		std::pair<DepthCameraMdoel, const std::string>(CM_FM810IXCamera, "TY-810 USB"),
		std::pair<DepthCameraMdoel, const std::string>(CM_FM810GIXE1Camera, "TY-810/850 GIGE"),
		std::pair<DepthCameraMdoel, const std::string>(CM_PhotoXiScannerM, "PHN-M"),
		std::pair<DepthCameraMdoel, const std::string>(CM_PhotoXiScannerL, "PHN-L"),
		std::pair<DepthCameraMdoel, const std::string>(CM_REALSENSE_D415, "RS-D415"),
	};

	std::shared_ptr<Camera> createDepthCamPtr(
		MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name)
	{
		std::string depth_camera_model_name = workspace_ptr->camera().at(node_name).depth_camera().camera_model_name();
		DepthCameraMdoel depth_camera_model = CM_NULL;
		for (auto it : model_model_name_vector)
		{
			if (depth_camera_model_name == it.second)
			{
				depth_camera_model = it.first;
				break;
			}
		}

		switch (depth_camera_model)
		{
		case CM_NULL:
		{
			// virtual camera
			if (workspace_ptr->camera().at(node_name).virtual_camera())
			{
				return std::make_shared<VirtualDepthCamera>(
					mil_application,
					mil_system,
					mil_display,
					mil_window_id,
					workspace_ptr,
					node_name);
			}
		}
		case CM_BaslerToFCamera:
		{
			return std::make_shared<BaslerToFCamera>(
				mil_application,
				mil_system,
				mil_display,
				mil_window_id,
				workspace_ptr,
				node_name);
			break;
		}
		case CM_FM810IXCamera:
		{
			return std::make_shared<FM810IXCamera>(
				mil_application,
				mil_system,
				mil_display,
				mil_window_id,
				workspace_ptr,
				node_name);
			break;
		}
		case CM_FM810GIXE1Camera:
		{
			return std::make_shared<FM810GIXE1Camera>(
				mil_application,
				mil_system,
				mil_display,
				mil_window_id,
				workspace_ptr,
				node_name);
			break;
		}
		case CM_PhotoXiScannerM:
		{
			return std::make_shared<PhotoXiScannerM>(
				mil_application,
				mil_system,
				mil_display,
				mil_window_id,
				workspace_ptr,
				node_name);
			break;
		}
		case CM_PhotoXiScannerL:
		{
			return std::make_shared<PhotoXiScannerL>(
				mil_application,
				mil_system,
				mil_display,
				mil_window_id,
				workspace_ptr,
				node_name);
			break;
		}
		case CM_REALSENSE_D415:
		{
			return std::make_shared<RealSenseD415Camera>(
				mil_application,
				mil_system,
				mil_display,
				mil_window_id,
				workspace_ptr,
				node_name);
			break;
		}
		default:
			return nullptr;
			break;
		}
	}

	std::vector<std::string> getCameraModelNames()
	{
		std::vector<std::string> ret;
		for (auto it : model_model_name_vector)
		{
			ret.push_back(it.second);
		}
		return ret;
	}
}

/* ================================
			   DPCLib
================================ */
namespace DPCLib
{
	static const MIL_INT DISPLAY_SIZE_X = 1280;
	static const MIL_INT DISPLAY_SIZE_Y = 960;

	bool cropPointCloudToFixBox(MIL_ID point_cloud, MIL_ID pc_label, MIL_ID calibration)
	{
		MIL_INT max_point_nb = M3dmapGet(point_cloud, PC_LABEL(pc_label), M_POSITION, M_EXCLUDE_INVALID_POINTS,
			M_FLOAT + 64, M_NULL, M_NULL, M_NULL, M_NULL, M_NULL);

		if (max_point_nb < 1)
			return false;

		std::vector<MIL_DOUBLE> x(max_point_nb);
		std::vector<MIL_DOUBLE> y(max_point_nb);
		std::vector<MIL_DOUBLE> z(max_point_nb);

		// Get the points inside the extraction box.
		MIL_INT actual_point_nb = M3dmapGet(point_cloud, PC_LABEL(pc_label), M_POSITION, M_INCLUDE_POINTS_INSIDE_BOX_ONLY,
			M_FLOAT + 64, max_point_nb, &x[0], &y[0], &z[0], M_NULL);
		if (actual_point_nb < 1)
			return false;

		// Put back the points in the same point cloud. 
		// Since the number of points changed, the point cloud must be cleared before calling M3dmapPut().
		M3dmapClear(point_cloud, PC_LABEL(pc_label), /*M_DELETE*/M_CLEAR, M_DEFAULT);

		McalSetCoordinateSystem(calibration,
			M_RELATIVE_COORDINATE_SYSTEM,
			M_CAMERA_COORDINATE_SYSTEM,
			M_IDENTITY,
			M_NULL,
			M_DEFAULT,
			M_DEFAULT,
			M_DEFAULT,
			M_DEFAULT);

		M3dmapPut(point_cloud, PC_LABEL(pc_label), M_POSITION, M_FLOAT + 64, actual_point_nb, &x[0], &y[0], &z[0], calibration, M_DEFAULT);

		McalSetCoordinateSystem(calibration,
			M_RELATIVE_COORDINATE_SYSTEM,
			M_ABSOLUTE_COORDINATE_SYSTEM,
			M_IDENTITY,
			M_NULL,
			M_DEFAULT,
			M_DEFAULT,
			M_DEFAULT,
			M_DEFAULT);

		// Compute a robust bounding box of all points.
		M3dmapControl(point_cloud, M_GENERAL, M_BOUNDING_BOX_ALGORITHM, M_ROBUST);
		M3dmapSetBox(point_cloud, M_EXTRACTION_BOX, M_BOUNDING_BOX, M_ALL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

		return true;
	}

	void getPointCloudBoudingBox(
		MIL_ID point_clond,
		MIL_DOUBLE& min_x,
		MIL_DOUBLE& min_y,
		MIL_DOUBLE& min_z,
		MIL_DOUBLE& max_x,
		MIL_DOUBLE& max_y,
		MIL_DOUBLE& max_z)
	{
		M3dmapInquire(point_clond, M_GENERAL, M_EXTRACTION_BOX_MIN_X + M_TYPE_MIL_DOUBLE, &min_x);
		M3dmapInquire(point_clond, M_GENERAL, M_EXTRACTION_BOX_MIN_Y + M_TYPE_MIL_DOUBLE, &min_y);
		M3dmapInquire(point_clond, M_GENERAL, M_EXTRACTION_BOX_MIN_Z + M_TYPE_MIL_DOUBLE, &min_z);
		M3dmapInquire(point_clond, M_GENERAL, M_EXTRACTION_BOX_MAX_X + M_TYPE_MIL_DOUBLE, &max_x);
		M3dmapInquire(point_clond, M_GENERAL, M_EXTRACTION_BOX_MAX_Y + M_TYPE_MIL_DOUBLE, &max_y);
		M3dmapInquire(point_clond, M_GENERAL, M_EXTRACTION_BOX_MAX_Z + M_TYPE_MIL_DOUBLE, &max_z);
	}

	void mapDepthImageTo8Bits(MIL_ID mil_system, MIL_ID src, MIL_ID dst)
	{
		// The remapping operation must exclude missing data pixels.
		// Create a raster region containing valid pixels only.
		MIL_ID region = MbufAlloc2d(mil_system,
			MbufInquire(src, M_SIZE_X, M_NULL),
			MbufInquire(src, M_SIZE_Y, M_NULL),
			8 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);

		MimBinarize(src, region, M_FIXED + M_NOT_EQUAL, 65535, M_NULL);

		// Associate the raster region to the depth map.
		MbufSetRegion(src, region, M_DEFAULT, M_RASTERIZE, M_DEFAULT);

		// Missing data pixels are mapped to 0 in MilTgtImage.
		MbufClear(dst, 0.0);

		// Remap depth-map's 16 bits dynamic range of valid data to 8 bits.
		MimRemap(M_DEFAULT, src, dst, /*M_FIT_SRC_DATA*/M_FIT_SRC_RANGE);

		// Remove region and free temporary buffer.
		MbufSetRegion(src, M_NULL, M_DEFAULT, M_DELETE, M_DEFAULT);
		MbufFree(region);
	}

	std::shared_ptr<DepthCameraDataWrapper> createDepthMapGeneratorPtr(
		MIL_ID mil_system,
		const Workspace_DepthCamera workspace_depth_camera,
		MIL_ID mil_calibration)
	{
		std::string depth_camera_model_name = workspace_depth_camera.camera_model_name();

		DepthCameraMdoel depth_camera_model = CM_NULL;
		for (auto it : DepthCameraManager::model_model_name_vector)
		{
			if (depth_camera_model_name == it.second)
			{
				depth_camera_model = it.first;
				break;
			}
		}

		switch (depth_camera_model)
		{
		case CM_NULL:
		{
			// virtual camera
			return std::make_shared<TYDataWrapper>(
				mil_system,
				workspace_depth_camera,
				mil_calibration);
			break;
		}			
		case CM_BaslerToFCamera:
		{
			return std::make_shared<BaslerToFDataWrapper>(
				mil_system,
				workspace_depth_camera,
				mil_calibration);
			break;
		}
		case CM_FM810IXCamera:
		case CM_FM810GIXE1Camera:
		{
			return std::make_shared<TYDataWrapper>(
				mil_system,
				workspace_depth_camera,
				mil_calibration);
			break;
		}
		case CM_PhotoXiScannerM:
		case CM_PhotoXiScannerL:
		{
			return std::make_shared<PhoXiDataWrapper>(
				mil_system,
				workspace_depth_camera,
				mil_calibration);
			break;
		}
		case CM_REALSENSE_D415:
		{
			return std::make_shared<RealSenseDataWrapper>(
				mil_system,
				workspace_depth_camera,
				mil_calibration);
			break;
		}
		default:
			return nullptr;
			break;
		}

		return nullptr;
	}

	SDepthData getCameraDepthData(config::Workspace_DepthCamera workspace_depth_camera)
	{
		SDepthData ret = SDepthData();

		std::string depth_camera_model_name = workspace_depth_camera.camera_model_name();
		DepthCameraMdoel depth_camera_model = CM_NULL;
		for (auto it : DepthCameraManager::model_model_name_vector)
		{
			if (depth_camera_model_name == it.second)
			{
				depth_camera_model = it.first;
				break;
			}
		}

		switch (depth_camera_model)
		{
		case CM_NULL:
		case CM_BaslerToFCamera:
		case CM_FM810IXCamera:
		case CM_FM810GIXE1Camera:
		case CM_REALSENSE_D415:
			ret = DepthCameraDataWrapper::setupDepthData(workspace_depth_camera);
			break;
		case CM_PhotoXiScannerM:
		case CM_PhotoXiScannerL:
			ret = PhoXiDataWrapper::setupDepthData(workspace_depth_camera);
			break;
		default:
			break;
		}
		return ret;
	}

	void getGroundEraseParams(
		config::Workspace* const workspace_ptr,
		const std::string feature_group_name,
		MIL_ID calibration,
		SPoseDataQuat pose_base_to_tool,
		double& max_depth,
		SPoseDataQuat& ground_rotation)
	{
		std::string camera_name = workspace_ptr->training().at(workspace_ptr->feature_group().at(feature_group_name).training_name()).camera_name();

		MIL_DOUBLE z;
		auto mt = workspace_ptr->camera().at(camera_name).mounting_type();

		// get the rotation
		if (workspace_ptr->camera().at(camera_name).mounting_type() == MOUNTED_ON_FIXED)
		{
			z = workspace_ptr->camera().at(camera_name).depth_camera().view_depth_max();

			McalGetCoordinateSystem(calibration,
				M_CAMERA_COORDINATE_SYSTEM,
				M_TOOL_COORDINATE_SYSTEM,
				M_ROTATION_QUATERNION,
				M_NULL,
				&ground_rotation.q1,
				&ground_rotation.q2,
				&ground_rotation.q3,
				&ground_rotation.q4);
		}
		else if (workspace_ptr->camera().at(camera_name).mounting_type() == MOUNTED_ON_ROBOT_ARM /*&& ROBOT_MODE*/)
		{
			MIL_DOUBLE x, y;
			McalSetCoordinateSystem(calibration,
				M_TOOL_COORDINATE_SYSTEM,
				M_ROBOT_BASE_COORDINATE_SYSTEM,
				M_TRANSLATION + M_ASSIGN,
				M_NULL,
				pose_base_to_tool.x,
				pose_base_to_tool.y,
				pose_base_to_tool.z,
				M_DEFAULT);

			McalSetCoordinateSystem(calibration,
				M_TOOL_COORDINATE_SYSTEM,
				M_TOOL_COORDINATE_SYSTEM,
				M_ROTATION_QUATERNION + M_COMPOSE_WITH_CURRENT,
				M_NULL,
				pose_base_to_tool.q1,
				pose_base_to_tool.q2,
				pose_base_to_tool.q3,
				pose_base_to_tool.q4);

			McalGetCoordinateSystem(calibration,
				M_ABSOLUTE_COORDINATE_SYSTEM,
				M_CAMERA_COORDINATE_SYSTEM,
				M_TRANSLATION,
				M_NULL,
				&x,
				&y,
				&z,
				M_NULL);

			McalSetCoordinateSystem(calibration, M_CAMERA_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
				M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		}

		auto groud_depth_offset = workspace_ptr->feature_group().at(feature_group_name).depth_feature_group().ground_depth_offset();
		max_depth = z + MIL_DOUBLE(groud_depth_offset);
	}
}

/*----------------------------------------
				DepthCamera
----------------------------------------*/
DepthCamera::DepthCamera(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	Camera(workspace_ptr)
{
	mil_application_ = mil_application;
	mil_system_ = mil_system;
	mil_display_ = mil_display;
	mil_window_id_ = mil_window_id;
	node_name_ = node_name;
	mil_image_ = M_NULL;
	mil_range_image_ = M_NULL;
	mil_digitizer_ = M_NULL;
	mil_display_lut_ = NULL;
	is_dig_alloc_succeed_ = false;
	is_cur_frame_corrupted_ = true;

	MgraAlloc(mil_system_, &mil_graphic_context_);
	MgraAllocList(mil_system_, M_DEFAULT, &mil_graphic_context_list_);
}

DepthCamera::~DepthCamera()
{
	if (mil_range_image_) MbufFree(mil_range_image_);
	if (mil_display_lut_) MbufFree(mil_display_lut_);
}

bool DepthCamera::isConnected() const
{
	return is_dig_alloc_succeed_;
}

int DepthCamera::setExposureTime(MIL_DOUBLE exposure_time)
{
	return ERR_NULL;
}

int DepthCamera::setGain(MIL_DOUBLE gain)
{
	return ERR_NULL;
}

bool DepthCamera::saveRangeImage(std::string file_name, MIL_ID key)
{
	if (is_cur_frame_corrupted_)
	{
		return false;
	}

	// check folder existance
	if (utils::dirExists(utils::dirNameFromPath(file_name)) == -2) // this is a file instead of folder
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::check_path), file_name, '\n'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	else // create folder no matter it exist or not
	{
		if (utils::createFolder(utils::dirNameFromPath(file_name)))
		{
			MbufSave(utils::string2Wchar_tPtr(file_name).get(), mil_range_image_);
			return true;
		}
		else
		{
			return false;
		}
	}
}

MIL_ID DepthCamera::getRangeImageID()
{
	return mil_range_image_;
}

std::vector<std::string> DepthCamera::getDispImageComponents()
{
	return display_components;
}

int DepthCamera::setDisplayImage(std::string component_name)
{
	if (component_name == "MONO")
	{
		if (MdispInquire(mil_display_, M_SELECTED, M_NULL) != M_NULL)
			MdispSelectWindow(mil_display_, M_NULL, (MIL_WINDOW_HANDLE)mil_window_id_);

		if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
			MdispLut(mil_display_, M_DEFAULT);

		if (mil_display_lut_)
		{
			MbufFree(mil_display_lut_);
			mil_display_lut_ = M_NULL;
		}

		mil_display_image_ = mil_image_;
	}
	else if (component_name == "RANGE")
	{
		if (MdispInquire(mil_display_, M_SELECTED, M_NULL) != M_NULL)
			MdispSelectWindow(mil_display_, M_NULL, (MIL_WINDOW_HANDLE)mil_window_id_);

		if (mil_display_lut_) MbufFree(mil_display_lut_);
		MbufAllocColor(mil_system_, 3, MIL_UINT16_MAX, 1, 8 + M_UNSIGNED, M_LUT, &mil_display_lut_);

		// Create the color map LUT.
		MbufClear(mil_display_lut_, M_COLOR_BLACK);

		double range_offset = workspace_ptr_->camera().at(node_name_).depth_camera().range_image_offset();
		double range_scale = workspace_ptr_->camera().at(node_name_).depth_camera().range_image_scale();
		auto view_depth_min = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_min();
		auto view_depth_max = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_max();

		MIL_INT max_gray = (MIL_INT)((view_depth_max - range_offset) / range_scale);
		max_gray = max_gray > (MIL_UINT16_MAX - 1) ? MIL_UINT16_MAX - 1 : max_gray;
		MIL_ID MilDispLutChild = MbufChild1d(mil_display_lut_, view_depth_min, max_gray - view_depth_min, M_NULL);
		MgenLutFunction(MilDispLutChild, M_COLORMAP_JET, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		MbufFree(MilDispLutChild);
		MdispLut(mil_display_, mil_display_lut_);

		mil_display_image_ = mil_range_image_;
	}
	else
	{
		return -1;
	}

	// update display index
	int component_index = 0;
	for (int i = 0; i < display_components.size(); i++)
	{
		if (display_components[i] == component_name)
		{
			component_index = i;
			break;
		}
	}
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_display_image(component_index);

	return ERR_NULL;
}

bool DepthCamera::savePauseImage(std::string file_name)
{
	int index = workspace_ptr_->camera().at(node_name_).depth_camera().display_image();
	if (display_components[index] == "MONO")
	{
		return saveImage(file_name);
	}
	else if (display_components[index] == "RANGE")
	{
		return saveRangeImage(file_name);
	}
	else
	{
		return false;
	}
}

void DepthCamera::updateDepthRange()
{
	view_depth_max_ = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_max();
	view_depth_min_ = workspace_ptr_->camera().at(node_name_).depth_camera().view_depth_min();
}

/*----------------------------------------
			VirtualDepthCamera
----------------------------------------*/
VirtualDepthCamera::VirtualDepthCamera(MIL_ID mil_application,
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
	// load virtual camera images
	std::string str_virtual_camera_folder_path = workspace_ptr_->camera().at(node_name_).virtual_camera_folderpath();
	std::string virtual_image_path_2d = str_virtual_camera_folder_path + "/" + workspace_ptr_->camera().at(node_name_).depth_camera().virtual_image_name_2d();
	std::string virtual_image_path_depth = str_virtual_camera_folder_path + "/" + workspace_ptr_->camera().at(node_name_).depth_camera().virtual_image_name_depth();

	if (!utils::fileExists(virtual_image_path_2d))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Image not exists [[", virtual_image_path_2d, ']]'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	if (!utils::fileExists(virtual_image_path_depth))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Image not exists [[", virtual_image_path_depth, ']]'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	MbufRestore(utils::string2Wchar_tPtr(virtual_image_path_2d).get(), mil_system_, &mil_image_);
	MbufRestore(utils::string2Wchar_tPtr(virtual_image_path_depth).get(), mil_system_, &mil_range_image_);

	MIL_INT size_x_2d = MbufInquire(mil_image_, M_SIZE_X, M_NULL);
	MIL_INT size_y_2d = MbufInquire(mil_image_, M_SIZE_Y, M_NULL);
	MIL_INT size_x_depth = MbufInquire(mil_range_image_, M_SIZE_X, M_NULL);
	MIL_INT size_y_depth = MbufInquire(mil_range_image_, M_SIZE_Y, M_NULL);

	if (size_x_2d != size_x_depth || size_y_2d != size_y_depth)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Unequal sizes of sample images !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// use "PERCIPIO" camera images as samples
	setFov();
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_offset(0.0);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_scale(1.0);

	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_x(google::protobuf::int32(size_x_depth));
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_range_image_size_y(google::protobuf::int32(size_y_depth));

	// set parameters
	auto process_3d_params = workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->mutable_process_3d_params();

	// intrinsic
	process_3d_params->set_intrinsic_height(960);
	process_3d_params->set_intrinsic_width(1280);

	process_3d_params->clear_intrinsic_param();
	process_3d_params->add_intrinsic_param(1153.893798828125);
	process_3d_params->add_intrinsic_param(0);
	process_3d_params->add_intrinsic_param(630.46685791015625);
	process_3d_params->add_intrinsic_param(0);
	process_3d_params->add_intrinsic_param(1154.11328125);
	process_3d_params->add_intrinsic_param(435.15142822265625);
	process_3d_params->add_intrinsic_param(0);
	process_3d_params->add_intrinsic_param(0);
	process_3d_params->add_intrinsic_param(1);

	is_cur_frame_corrupted_ = false;
	is_dig_alloc_succeed_ = true;
}

VirtualDepthCamera::~VirtualDepthCamera()
{
}

bool VirtualDepthCamera::singleGrab(bool display_image)
{
	return true;
}

void VirtualDepthCamera::continuousGrabHelper()
{
	setDisplayImage(getDispImageComponents()[workspace_ptr_->camera().at(node_name_).depth_camera().display_image()]);
	MdispSelectWindow(mil_display_, mil_display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	while (!end_grab_)
	{
		Sleep(100);
	}
}

void VirtualDepthCamera::setFov()
{
	// same as Percipio 810 gige
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_h(56);
	workspace_ptr_->mutable_camera()->at(node_name_).mutable_depth_camera()->set_fov_v(46);
}

/*----------------------------------------
		  DepthCameraDataWrapper
----------------------------------------*/
DepthCameraDataWrapper::DepthCameraDataWrapper(MIL_ID mil_system,
	const Workspace_DepthCamera workspace_depth_camera,
	MIL_ID mil_calibration) :
	mil_system_(mil_system),
	mil_calibration_(mil_calibration)
{
	range_offset_ = workspace_depth_camera.range_image_offset();
	range_scale_ = workspace_depth_camera.range_image_scale();

	M3dmapAllocResult(mil_system, M_POINT_CLOUD_CONTAINER, M_DEFAULT, &point_cloud_);
	M3dmapClear(point_cloud_, M_ALL, /*M_DELETE*/M_CLEAR, M_DEFAULT);
}

DepthCameraDataWrapper::~DepthCameraDataWrapper()
{
	M3dmapFree(point_cloud_);
}

bool DepthCameraDataWrapper::createDepthMap(MIL_ID range_image,
	MIL_ID mask_image,
	MIL_ID depth_map,
	MIL_ID pc_label,
	double max_depth,
	double size_depth_opening,
	bool inquire_camera,
	bool enabel_range_denoising,
	bool enable_depth_smoothing,
	SPoseDataQuat transform)
{
	MIL_ID image_range_model =
		MbufClone(range_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);

	if (mask_image != M_NULL)
		MimArith(range_image, mask_image, image_range_model, M_MULT/*M_OR*/);
	else
		MbufCopy(range_image, image_range_model);

	// Preprocess the range image to generate a validation image
	if (false/*enabel_range_denoising*/)
	{
		// Clean the depth image. The third derivative is sensitive to noise.
		MIL_ID image_range_proc = MbufClone(image_range_model, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
		MimRank(image_range_model, image_range_proc, M_5X5_RECT, M_MEDIAN, M_GRAYSCALE);

		// Create the threshold image.
		MIL_ID image_range_thr = MbufClone(image_range_model, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
		MimArith(image_range_proc, 16/*third deriv factor*/, image_range_thr, M_DIV_CONST);

		// Get the positive part of the third derivative of the image.      
		MimConvolve(image_range_proc, image_range_proc, M_EDGE_DETECT_SOBEL_FAST);	// 1st derivative
		MimConvolve(image_range_proc, image_range_proc, M_LAPLACIAN_8);				// 2nd derivative
		MimConvolve(image_range_proc, image_range_proc, M_SMOOTH);					// 3rd derivative

		// Find the pixels whose third derivative is greater than the threshold.
		MimArith(image_range_thr, image_range_proc,
			image_range_proc, M_SUB + M_SATURATION);

		// Add a slight margin around the invalid pixels.
		MimErode(image_range_proc, image_range_proc, 1, M_BINARY);
		MbufClearCond(image_range_model, 0, 0, 0, image_range_proc, M_EQUAL, 0);
		MbufFree(image_range_proc);
		MbufFree(image_range_thr);
	}

	// create depth map from pc
	if (x_world_.size() == 0 &&
		y_world_.size() == 0 &&
		z_world_.size() == 0)
	{
		MIL_INT image_size_x = MbufInquire(range_image, M_SIZE_X, M_NULL);
		MIL_INT image_size_y = MbufInquire(range_image, M_SIZE_Y, M_NULL);

		x_world_.resize(image_size_x * image_size_y);
		y_world_.resize(image_size_x * image_size_y);
		z_world_.resize(image_size_x * image_size_y);
	}

	std::fill(x_world_.begin(), x_world_.end(), M_INVALID_POINT);
	std::fill(y_world_.begin(), y_world_.end(), M_INVALID_POINT);
	std::fill(z_world_.begin(), z_world_.end(), M_INVALID_POINT);

	composePointCloudPoint(image_range_model, max_depth);

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
	M3dmapControl(point_cloud_, M_GENERAL, M_EXTRACTION_OVERLAP, M_MAX/*M_DEFAULT*//*M_MIN*/);
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
		MbufFree(image_range_model);
		std::string o_msg = printCerrMsg(LEVEL_DETAIL_INFO, "No data inside point cloud."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	if (depth_map)
		createDepthMapFromPointCloud(depth_map, size_depth_opening, enabel_range_denoising, enable_depth_smoothing, pc_label);

	MbufFree(image_range_model);
	return true;
}

MIL_ID DepthCameraDataWrapper::getPointCloudID()
{
	return point_cloud_;
}

bool DepthCameraDataWrapper::cropPointCloudToFixBox(MIL_ID pc_label)
{
	return DPCLib::cropPointCloudToFixBox(point_cloud_, pc_label, mil_calibration_);
}

void DepthCameraDataWrapper::resetPCBoundingBox()
{
	M3dmapSetBox(point_cloud_, M_EXTRACTION_BOX, M_CORNER_AND_DIMENSION,
		depth_data_.depth_view_x_pose_,
		depth_data_.depth_view_y_pose_,
		depth_data_.depth_view_z_pose_,
		depth_data_.depth_view_x_size_,
		depth_data_.depth_view_y_size_,
		depth_data_.depth_view_z_size_);
}

void DepthCameraDataWrapper::clearPointCloud()
{
	/*M3dmapClear(point_cloud_, M_ALL, M_DELETE, M_DEFAULT);*/
	M3dmapClear(point_cloud_, M_ALL, M_CLEAR, M_DEFAULT);
}

void DepthCameraDataWrapper::getPCBoundingBox(
	double& extract_box_x,
	double& extract_box_y,
	double& extract_box_z,
	double& extract_box_size_x,
	double& extract_box_size_y,
	double& extract_box_size_z)
{
	extract_box_x = depth_data_.depth_view_x_pose_;
	extract_box_y = depth_data_.depth_view_y_pose_;
	extract_box_z = depth_data_.depth_view_z_pose_;
	extract_box_size_x = depth_data_.depth_view_x_size_;
	extract_box_size_y = depth_data_.depth_view_y_size_;
	extract_box_size_z = depth_data_.depth_view_z_size_;
}

bool DepthCameraDataWrapper::get3DPointUsingNeighborFilter(
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
	std::string o_msg = printCerrMsg(LEVEL_BASIC_INFO, "Please Override to allow this function."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	return false;
}

SDepthData DepthCameraDataWrapper::setupDepthData(config::Workspace_DepthCamera workspace_depth_camera)
{
	static const int MAX_DEPTH_MAP_SIZE_X = 2560;
	static const int MAX_DEPTH_MAP_SIZE_Y = 2560;

	SDepthData ret = SDepthData();

	// set up image process parameters
	auto fov_h = workspace_depth_camera.fov_h();
	auto fov_v = workspace_depth_camera.fov_v();
	if (fov_h <= 0 || fov_v <= 0)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Error FOV input(s): ", fov_h, fov_v); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}

	auto src_image_size_x = workspace_depth_camera.range_image_size_x();
	auto src_image_size_y = workspace_depth_camera.range_image_size_y();

	if (src_image_size_x <= 0 || src_image_size_y <= 0)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Error image size input(s): ", src_image_size_x, src_image_size_y); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}

	// compute 3d point cloud bounding box from FOV & depth view range
	auto depth_max = workspace_depth_camera.view_depth_max();
	auto depth_min = workspace_depth_camera.view_depth_min();

	double view_half_h = tan((fov_h)* PI / 180.0) * depth_max * 0.5;
	double view_half_v = tan((fov_v)* PI / 180.0) * depth_max * 0.5;

	ret.depth_view_x_pose_ = -view_half_h;
	ret.depth_view_y_pose_ = -view_half_v;
	ret.depth_view_z_pose_ = depth_min;

	ret.depth_view_x_size_ = view_half_h * 2.0;
	ret.depth_view_y_size_ = view_half_v * 2.0;
	ret.depth_view_z_size_ = depth_max - depth_min;

	double depth_map_size_x = src_image_size_x;
	double depth_map_size_y = src_image_size_y;

	double fov_tan_h_by_v = (tan((fov_h)* PI / 180.0)) / (tan((fov_v)* PI / 180.0));
	double size_x_by_y = src_image_size_x / src_image_size_y;

	// modify depth map size to get ratio depth_map_size_x/depth_map_size_y = fov_tan_h_by_v
	if (fov_tan_h_by_v > size_x_by_y)
	{
		// decrease y
		depth_map_size_y = depth_map_size_x / fov_tan_h_by_v;
	}
	else if (fov_tan_h_by_v < size_x_by_y)
	{
		// decrease x
		depth_map_size_x = depth_map_size_y * fov_tan_h_by_v;
	}

	// clip to less than max size
	if (depth_map_size_x > MAX_DEPTH_MAP_SIZE_X)
	{
		depth_map_size_x = MAX_DEPTH_MAP_SIZE_X;
		depth_map_size_y = depth_map_size_x / fov_tan_h_by_v;
	}

	if (depth_map_size_y > MAX_DEPTH_MAP_SIZE_Y)
	{
		depth_map_size_y = MAX_DEPTH_MAP_SIZE_Y;
		depth_map_size_x = depth_map_size_y * fov_tan_h_by_v;
	}

	ret.depth_map_mm_per_pixel_ =
		(view_half_h * 2 / double((int)(depth_map_size_x)) +
			view_half_v * 2 / double((int)(depth_map_size_y))) / 2;

	ret.depth_map_size_x_ = (int)depth_map_size_x;
	ret.depth_map_size_y_ = (int)depth_map_size_y;

	return ret;
}

void DepthCameraDataWrapper::createDepthMapFromPointCloud(
	MIL_ID depth_map,
	double size_depth_opening,
	bool enabel_range_denoising,
	bool enable_depth_smoothing,
	MIL_ID pc_label)
{
	MbufClear(depth_map, 0);
	McalSetCoordinateSystem(point_cloud_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	M3dmapExtract(point_cloud_, depth_map, M_NULL, M_CORRECTED_DEPTH_MAP, /*M_ALL*/PC_LABEL(pc_label), M_DEFAULT);
	MimClip(depth_map, depth_map, M_EQUAL, 65535, M_NULL, 0, M_NULL);

	//	Fill the holes and smooth the depth map.This function
	//	gets the average of the valid pixels in the FilterSize
	//	neighborhood and sets it on the central pixel. If the 
	//	ratio of valid pixels is less than MIN_VALID_RATIO, the 
	//	pixel is set to the invalid data value.
	if (enable_depth_smoothing)
	{
		// Create the valid image.
		MIL_INT image_size_x = MbufInquire(depth_map, M_SIZE_X, M_NULL);
		MIL_INT image_size_y = MbufInquire(depth_map, M_SIZE_Y, M_NULL);

		MIL_ID image_depth_valid = MbufAlloc2d(mil_system_, image_size_x, image_size_y, 16 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MimBinarize(depth_map, image_depth_valid, M_FIXED + M_GREATER, 0, M_NULL);

		MIL_ID mil_uniform_kernel = MbufAlloc2d(mil_system_, 3, 3,
			8 + M_UNSIGNED, M_KERNEL, M_NULL);
		MbufClear(mil_uniform_kernel, 1);
		MbufControl(mil_uniform_kernel, M_NORMALIZATION_FACTOR, 3 * 3);
		MimConvolve(image_depth_valid, image_depth_valid, mil_uniform_kernel);
		MimConvolve(depth_map, depth_map, mil_uniform_kernel);
		MbufFree(mil_uniform_kernel);

		// Normalize the values.
		MIL_ID image_depth_uint32 = MbufAlloc2d(mil_system_, image_size_x, image_size_y, 32 + M_UNSIGNED, M_IMAGE + M_PROC, M_NULL);
		MimShift(depth_map, image_depth_uint32, 16);
		MimArith(image_depth_uint32, image_depth_valid, depth_map, M_DIV);

		// Invalidate the data of values that do not have enough valid values in their neighborhood.
		MIL_DOUBLE max_valid = 0.5 * MIL_UINT16_MAX;
		MimBinarize(image_depth_valid, image_depth_valid, M_GREATER, max_valid, M_NULL);
		MbufClearCond(depth_map, 0, M_NULL, M_NULL, image_depth_valid, M_EQUAL, 0);
		MbufFree(image_depth_valid);
		MbufFree(image_depth_uint32);
	}

	// Structure the depth map by closing it based on the MinDimension.
	if (/*size_depth_opening*/false)
	{
		MIL_INT kernal_size = (MIL_INT)(size_depth_opening / depth_data_.depth_map_mm_per_pixel_);
		kernal_size = kernal_size == 0 ? 1 : kernal_size;
		MIL_ID kernal_depth_opening = MbufAlloc2d(mil_system_, kernal_size, kernal_size, 32 + M_UNSIGNED, M_STRUCT_ELEMENT, M_NULL);
		MbufClear(kernal_depth_opening, 0);
		MimMorphic(depth_map, depth_map, kernal_depth_opening, M_OPEN, 1, M_GRAYSCALE);
		MbufFree(kernal_depth_opening);
	}
}