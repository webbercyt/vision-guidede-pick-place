#include "depth_localization_lib.h"
#include "depth_localization_geo_lib.h"
#include "depth_localization_seg_lib.h"
#include "depth_localization_rect_lib.h"
#include "utils.h"
#include "daoai.h"
#include "depth_camera.h"

namespace ModelProcess
{
	void setContextFromPattern(
		MIL_ID mod_context, 
		config::Workspace_FeatureGroup::PatternWindow pattern, 
		MIL_INT control_number)
	{
		MIL_INT64 mod_context_type;
		MmodInquire(mod_context, M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_context_type);

		// geo model
		if (mod_context_type == M_GEOMETRIC || mod_context_type == M_IMAGE)
		{
			DepthGeometryLocalization::setContextFromPattern(mod_context, pattern, control_number);
		}
		// shape rectangel
		else if(mod_context_type == M_RECTANGLE)
		{
			DepthRectLocalization::setContextFromPattern(mod_context, pattern, control_number);
		}
		// shape circle
		else if (mod_context_type == M_CIRCLE)
		{
			DepthCircleLocalization::setContextFromPattern(mod_context, pattern, control_number);
		}
		// segment
		else if (mod_context_type == M_SEGMENT)
		{
			DepthSegmentLocalization::setContextFromPattern(mod_context, pattern, control_number);
		}
		
		MmodPreprocess(mod_context, M_DEFAULT);
	}
}

DepthLocalization::DepthLocalization(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name) :
	Localization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr),
	pose_model_to_world_(SPoseDataQuat()),
	is_defining_model_(false),
	mil_graphic_context_roi_(M_NULL)
{
	if (!hasLisence())
		return;

	feature_group_name_ = feature_group_name;
	MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, mil_graphic_context_list_);

	if (!setupBuffers())
	{
		is_allocation_success_ = false;
		return;
	}
	
	is_allocation_success_ = true;
	setupDisplays();
}

DepthLocalization::DepthLocalization(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name,
	std::map<std::string, MIL_ID> mil_mod_context_map) :
	Localization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr),
	pose_model_to_world_(SPoseDataQuat()),
	is_defining_model_(true),
	mil_graphic_context_roi_(M_NULL)
{
	if (!hasLisence())
		return;

	feature_group_name_ = feature_group_name;

	if (!setupBuffers())
	{
		is_allocation_success_ = false;
		return;
	}

	is_allocation_success_ = true;

	//setupDisplays();
}

DepthLocalization::~DepthLocalization()
{
	McalFree(mil_cal_robot_transformed_);
	McalFree(mil_cal_ground_erase_);

	if(mil_graphic_context_roi_) MgraFree(mil_graphic_context_roi_);
}

void DepthLocalization::pushFrame(MIL_ID image, MIL_ID depth_image, SPoseDataQuat pose_base_to_tool, bool& is_stop, bool& has_result)
{
	if (!is_defining_model_)
	{
		MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, mil_graphic_context_list_);
		MdispControl(mil_display_, M_UPDATE, M_DISABLE);
	}

	pose_base_to_tool_ = pose_base_to_tool;
	src_2d_image_ = image;
	src_range_image_ = depth_image;

	switch (state_)
	{
	case LocalizationState::IDLE:
	{
		runLocalization();
		break;
	}
	case LocalizationState::IN_PROCESS:
	{
		model_collect_times_++;

		/*SPoseDataQuat pose;
		MIL_DOUBLE error;

		if (run3dModelLocalizationStrategy(pose, error))
		{
			model_poses_x_ = pose.x;
			model_poses_y_ = pose.y;
			model_poses_z_ = pose.z;
			model_rotate_.emplace(error, pose);

			handleCollectedData();
			state_ = LocalizationState::SUCCESSED;
			break;
		}
		else
		{
			state_ = LocalizationState::FAILED;
			break;
		}*/
	}
	default:
		break;
	}

	switch (state_)
	{
	case LocalizationState::IDLE:
	{
		is_stop = true;
		has_result = false;
		break;
	}
	case LocalizationState::IN_PROCESS:
	{
		is_stop = false;
		has_result = false;
		break;
	}
	case LocalizationState::FAILED:
	{
		is_stop = true;
		has_result = false;
		break;
	}
	case LocalizationState::SUCCESSED:
	{
		is_stop = true;
		has_result = true;
		break;
	}
	default:
		break;
	}

	if (!is_defining_model_)
		MdispControl(mil_display_, M_UPDATE, M_ENABLE);
}

void DepthLocalization::resetlocateState()
{
	state_ = LocalizationState::IDLE;
	MgraClear(mil_graphic_context_, mil_graphic_context_list_);

	if(mil_graphic_context_roi_)
		MgraClear(mil_graphic_context_roi_, mil_graphic_context_list_);
}

bool DepthLocalization::getModelPose(ModelPoseType model_pose_type, SPoseDataQuat& pose)
{
	if (state_ == LocalizationState::SUCCESSED)
	{
		switch (model_pose_type)
		{
		case MODEL_TO_IMAGE:
		{
			pose = pose_model_to_image_;
			return true;
		}
		case MODEL_TO_WORLD:
		{
			pose = pose_model_to_world_;
			return true;
		}
		case MODEL_TO_CAM:
		{
			pose = pose_model_to_cam_;
			return true;
		}
		default:
			break;
		}

		return false;
	}
	else
		return false;
}

bool DepthLocalization::getRobotMoveToReference(
	SPoseDataQuat tar_pose_base_to_tool,
	SPoseDataQuat tar_pose_world_to_model,
	SPoseDataQuat ref_pose_world_to_model,
	SPoseDataQuat& move)
{
	if (state_ == LocalizationState::SUCCESSED)
	{
		SPoseDataQuat tar_pose_model_to_camera = SPoseDataQuat();

		// move tool to target pose
		moveRobotPose(mil_cal_robot_transformed_, tar_pose_base_to_tool, M_TOOL_COORDINATE_SYSTEM, M_ROBOT_BASE_COORDINATE_SYSTEM);
		// move relative pose to tar_pose_world_to_model
		moveRobotPose(mil_cal_robot_transformed_, tar_pose_world_to_model, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM);
		// compute the move from relative pose to camera from mil_cal_const_
		// since mil_cal_const_ has not been modified
		getRobotPose(mil_cal_robot_transformed_, &tar_pose_model_to_camera, M_CAMERA_COORDINATE_SYSTEM, M_RELATIVE_COORDINATE_SYSTEM);
		// set relative pose to ref_pose_world_to_model
		moveRobotPose(mil_cal_robot_transformed_, ref_pose_world_to_model, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM);
		// move camera pose to target camera pose
		moveRobotPose(mil_cal_robot_transformed_, tar_pose_model_to_camera, M_CAMERA_COORDINATE_SYSTEM, M_RELATIVE_COORDINATE_SYSTEM);
		// compute the move from base to tool
		getRobotPose(mil_cal_robot_transformed_, &move, M_TOOL_COORDINATE_SYSTEM, M_ROBOT_BASE_COORDINATE_SYSTEM);

		return true;
	}
	else
		return false;
}

void DepthLocalization::setSearchOrder(int search_order)
{
	search_order_ = SearchOrder(search_order);
}

bool DepthLocalization::setupBuffers()
{
	state_ = LocalizationState::IDLE;
	
	std::string training_name = workspace_ptr_->feature_group().at(feature_group_name_).training_name();
	calibration_name_ = workspace_ptr_->training().at(training_name).calibration_name();

	std::string mil_cal_path = workspace_ptr_->workspace_folder() + "/calibration/" + calibration_name_ + "/" + workspace_ptr_->calibration().at(calibration_name_).mca_path();
	// Need to handle when calibration can't be restored
	if (!utils::fileExists(mil_cal_path))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::calibration_file_not_exist), mil_cal_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}
	std::shared_ptr<wchar_t> wc_mil_cal_path_ptr = utils::string2Wchar_tPtr(mil_cal_path);

	mil_cal_const_ = McalRestore(wc_mil_cal_path_ptr.get(), mil_system_, M_DEFAULT, M_NULL);
	McalSetCoordinateSystem(mil_cal_const_, M_CAMERA_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

	mil_cal_new_ = McalRestore(wc_mil_cal_path_ptr.get(), mil_system_, M_DEFAULT, M_NULL); // Will re-calibrate every time with new image
	McalSetCoordinateSystem(mil_cal_new_, M_CAMERA_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

	mil_cal_robot_transformed_ = McalRestore(wc_mil_cal_path_ptr.get(), mil_system_, M_DEFAULT, M_NULL);
	McalSetCoordinateSystem(mil_cal_robot_transformed_, M_CAMERA_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

	mil_cal_ground_erase_ = McalRestore(wc_mil_cal_path_ptr.get(), mil_system_, M_DEFAULT, M_NULL);
	McalSetCoordinateSystem(mil_cal_ground_erase_, M_CAMERA_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);


	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_;
	auto feature_group_ptr = &workspace_ptr_->feature_group().at(feature_group_name_);

	std::string reference_image_path = base_folder + "/" + feature_group_ptr->reference_image_path();
	if (!utils::fileExists(reference_image_path))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_image_not_load), reference_image_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(reference_image_path);
	MIL_ID reference_image;
	MbufImport(wc_image_path.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &reference_image);
	MbufInquire(reference_image, M_SIZE_X, &image_size_x_);
	MbufInquire(reference_image, M_SIZE_Y, &image_size_y_);
	MbufFree(reference_image);

	std::string camera_name = workspace_ptr_->training().at(training_name).camera_name();
	camera_mounting_type_ = workspace_ptr_->camera().at(camera_name).mounting_type();
	depth_groud_offset_ = MIL_DOUBLE(workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().ground_depth_offset())/*-90*/;
	
	search_order_ = SearchOrder::NO_ORDER;

	roi_x_ = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().roi_x();
	roi_y_ = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().roi_y();
	roi_x_size_ = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().roi_x_size();
	roi_y_size_ = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().roi_y_size();

	if (isROIValid())
	{
		if(roi_x_ + roi_x_size_ > image_size_x_)
		{
			roi_x_size_ = (int)image_size_x_ - roi_x_;
		}

		if (roi_y_ + roi_y_size_ > image_size_y_)
		{
			roi_y_size_ = (int)image_size_y_ - roi_y_;
		}
	}	

	display_mode_ = DisplayMode::DM_NONE;
	depth_order_pick_ = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().depth_order_pick();

	depth_camera_data_wrapper_ =
		std::move(DPCLib::createDepthMapGeneratorPtr(
			mil_system_,
			workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().depth_camera(),
			mil_cal_const_));

	return true; 
}

void DepthLocalization::setupDisplays()
{
	MgraAlloc(mil_system_, &mil_graphic_context_roi_);
}

bool DepthLocalization::hasLisence()
{
	if (!D3_DEPTH_ENABLED) /*Lisence check*/
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::license_miss)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		Sleep(10000);
		std::exit(0);
		return false;
	}
	else
		return true;
}

bool DepthLocalization::isROIValid()
{
	if (roi_x_ >= 0 &&
		roi_y_ >= 0 &&
		roi_x_size_ > 0 &&
		roi_y_size_ > 0)
		return true;
	else
		return false;
}

void DepthLocalization::handleCollectedData()
{
	//// do medium filtering to pose transform	
	//std::sort(model_poses_x_.begin(), model_poses_x_.end());
	//std::sort(model_poses_y_.begin(), model_poses_y_.end());
	//std::sort(model_poses_z_.begin(), model_poses_z_.end());

	//auto quat = model_rotate_.begin()->second;

	//int index = NB_MODEL_COMPUTE / 2;
	//pose_model_to_world_ = SPoseDataQuat(
	//	model_poses_x_[index],
	//	model_poses_y_[index],
	//	model_poses_z_[index],
	//	quat.q1, quat.q2, quat.q3, quat.q4); // chose the rotate with lowest plane fit error

	auto quat = model_rotate_.begin()->second;
	pose_model_to_world_ = SPoseDataQuat(
		model_poses_x_,
		model_poses_y_,
		model_poses_z_,
		quat.q1, quat.q2, quat.q3, quat.q4); // chose the rotate with lowest plane fit error
}

MIL_DOUBLE DepthLocalization::compute3DPLaneRotation(MIL_ID geometric_plane, SPoseDataQuat& pose, MIL_DOUBLE z_axis_angle)
{
	//Get the plane coefficients to determine the normal
	MIL_DOUBLE ax, ay, z0;
	M3dmapInquire(geometric_plane, M_DEFAULT, M_FIT_PARAM_AX, &ax);
	M3dmapInquire(geometric_plane, M_DEFAULT, M_FIT_PARAM_AY, &ay);
	M3dmapInquire(geometric_plane, M_DEFAULT, M_FIT_PARAM_Z0, &z0);

	// normal of plane at z = 0 : n1 = (0, 0, 1)
	// normal of the object plane : n2 = (ax, ay, z0)			
	// the normal of the plane perpendicular to both above planes :
	// n3 = (0, 0, 1) cross (ax, ay, z0) --- from n1 to n2
	MIL_DOUBLE norm = sqrt((ax * ax) + (ay * ay) + 1);
	MIL_DOUBLE axis_x = -ay;
	MIL_DOUBLE axis_y = ax;
	MIL_DOUBLE axis_z = 0;
	MIL_DOUBLE cross_norm = sqrt((axis_x * axis_x) + (axis_y * axis_y) + (axis_z * axis_z));
	MIL_DOUBLE rotate_angle_rad = asin(cross_norm / norm);
	MIL_DOUBLE rotate_angle_deg = RAD_TO_DEG * rotate_angle_rad;

	McalSetCoordinateSystem(mil_cal_new_,
		M_RELATIVE_COORDINATE_SYSTEM,
		M_CAMERA_COORDINATE_SYSTEM,
		M_IDENTITY,
		M_NULL,
		M_DEFAULT,
		M_DEFAULT,
		M_DEFAULT,
		M_DEFAULT);

	if (rotate_angle_deg)
	{
		McalSetCoordinateSystem(mil_cal_new_,
			M_RELATIVE_COORDINATE_SYSTEM,
			M_CAMERA_COORDINATE_SYSTEM,
			M_ROTATION_AXIS_ANGLE + M_COMPOSE_WITH_CURRENT,
			M_NULL,
			axis_x / cross_norm,
			axis_y / cross_norm,
			axis_z / cross_norm,
			-rotate_angle_deg);
	}

	if (z_axis_angle != 0)
	{
		McalSetCoordinateSystem(mil_cal_new_,
			M_RELATIVE_COORDINATE_SYSTEM,
			M_CAMERA_COORDINATE_SYSTEM,
			M_ROTATION_AXIS_ANGLE + M_COMPOSE_WITH_CURRENT,
			M_NULL,
			0,
			0,
			1,
			-z_axis_angle);
	}

	McalGetCoordinateSystem(mil_cal_new_,
		M_RELATIVE_COORDINATE_SYSTEM,
		M_CAMERA_COORDINATE_SYSTEM,
		M_ROTATION_QUATERNION,
		M_NULL,
		&pose.q1, &pose.q2, &pose.q3, &pose.q4);

	return rotate_angle_deg;
}

SPoseDataQuat DepthLocalization::transformPoseFromCameraToWorld(SPoseDataQuat input)
{
	// change pose from camera coordinate to world coordinate
	SPoseDataQuat ret;
	moveRobotPose(mil_cal_new_, pose_base_to_tool_, M_TOOL_COORDINATE_SYSTEM, M_ROBOT_BASE_COORDINATE_SYSTEM);
	moveRobotPose(mil_cal_new_, input, M_RELATIVE_COORDINATE_SYSTEM, M_CAMERA_COORDINATE_SYSTEM);
	getRobotPose(mil_cal_new_, &ret, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM);
	
	return ret;
}

DepthModelDefinedLocalization::DepthModelDefinedLocalization(
	MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name) :
	DepthLocalization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		feature_group_name),
	disp_depth_gra_list_(M_NULL),
	disp_depth_lut_(M_NULL),
	depth_lut_debug_(M_NULL),
	depth_disp_debug_(M_NULL),
	depth_gra_list_debug_(M_NULL)
{
	if (!setupBuffers())
	{
		is_allocation_success_ = false;
		return;
	}

	if (!setupModContext())
	{
		is_allocation_success_ = false;
		return;
	}

	setupDisplays();

	is_allocation_success_ = true;
}

DepthModelDefinedLocalization::DepthModelDefinedLocalization(
	MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name,
	std::map<std::string, MIL_ID> mil_mod_context_map) :
	DepthLocalization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		feature_group_name,
		mil_mod_context_map),
	disp_depth_gra_list_(M_NULL),
	disp_depth_lut_(M_NULL),
	depth_lut_debug_(M_NULL),
	depth_disp_debug_(M_NULL),
	depth_gra_list_debug_(M_NULL)
{
	if (!setupBuffers())
	{
		is_allocation_success_ = false;
		return;
	}

	std::string feature_name_2d = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_2d();
	std::string feature_name_depth = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_depth();
	for (auto it : mil_mod_context_map)
	{
		if (it.first.find(feature_name_2d) != std::string::npos)
		{
			if (it.second == M_NULL)
				continue;

			mod_context_map_2d_.emplace(it);

			MIL_INT64 mod_type_2d;
			MmodInquire(it.second, M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_type_2d);

			auto const model_type = *ModelProcess::model_context_result_type_map.find(mod_type_2d);
			MIL_INT64 mod_result_type = model_type.second;
			MIL_ID mod_result = MmodAllocResult(mil_system_, mod_result_type, M_NULL);
			mod_result_map_2d_.emplace(it.first, mod_result);
		}
		else if (it.first.find(feature_name_depth) != std::string::npos)
		{
			if (it.second == M_NULL)
				continue;

			mod_context_map_depth_.emplace(it);

			MIL_INT64 mod_type_depth;
			MmodInquire(it.second, M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_type_depth);

			auto const model_type = *ModelProcess::model_context_result_type_map.find(mod_type_depth);
			MIL_INT64 mod_result_type = model_type.second;
			MIL_ID mod_result = MmodAllocResult(mil_system_, mod_result_type, M_NULL);
			mod_result_map_depth_.emplace(it.first, mod_result);
		}
		else
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_error_name), it.first); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			is_allocation_success_ = false;
			return;
		}
	}

	//setupDisplays(); // comment/uncomment
	is_allocation_success_ = true;
}

DepthModelDefinedLocalization::~DepthModelDefinedLocalization()
{
	MbufFree(image_depth_);
	MbufFree(image_depth_uint8_);

//#if DEBUGING_MODE
	if (depth_gra_list_debug_) MgraFree(depth_gra_list_debug_);
	if (depth_disp_debug_)MdispFree(depth_disp_debug_);
	if (depth_lut_debug_)MbufFree(depth_lut_debug_);
//#endif

	if (disp_depth_gra_list_) MgraFree(disp_depth_gra_list_);
	if (disp_depth_lut_) MbufFree(disp_depth_lut_);

	if (!is_defining_model_)
	{
		for (auto it : mod_context_map_2d_)
		{
			MmodFree(it.second);
		}

		for (auto it : mod_context_map_depth_)
		{
			MmodFree(it.second);
		}
	}

	for (auto it : mod_result_map_2d_)
	{
		MmodFree(it.second);
	}

	for (auto it : mod_result_map_depth_)
	{
		MmodFree(it.second);
	}
}

void DepthModelDefinedLocalization::resetlocateState()
{
	DepthLocalization::resetlocateState();

	if (disp_depth_gra_list_)
		MgraClear(M_DEFAULT, disp_depth_gra_list_);

	mod_result_order_vec_.clear();
}

void DepthModelDefinedLocalization::runLocalization()
{
//#if DEBUGING_MODE
	if (depth_disp_debug_)
		MdispControl(depth_disp_debug_, M_UPDATE, M_DISABLE);
//#endif

	LocateMethod locate_method = LocateMethod(workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().locate_method());
	int locate_method_flag = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().locate_method_flag();

	switch (locate_method)
	{
	case LM_2D:
	{
		if ((locate_method_flag & locate_method) == LM_2D)
		{
			if (runLocateMethod2d())
				state_ = LocalizationState::SUCCESSED;
		}
		else
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,
				"Invalid locate method: 2d, please check the result of feature group !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		break;
	}
	case LM_DEPTH:
	{
		if ((locate_method_flag & locate_method) == LM_DEPTH)
		{
			if (runLocateMethodDepth())
				state_ = LocalizationState::SUCCESSED;
		}
		else
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,
				"Invalid locate method: Depth, please check the result of feature group !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		break;
	}
	case LM_2D_DEPTH:
	{
		if ((locate_method_flag & locate_method) == LM_2D_DEPTH)
		{
			if (runLocateMethod2dDepth())
				state_ = LocalizationState::SUCCESSED;
		}
		else
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,
				"Invalid locate method: 2d + Depth, please check the result of feature group !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		break;
	}
	default:
		break;
	}

//#if DEBUGING_MODE
	if (depth_disp_debug_)
		MdispControl(depth_disp_debug_, M_UPDATE, M_ENABLE);
//#endif
}

bool DepthModelDefinedLocalization::runLocateMethod2d()
{
	// switch to 2D display
	if (!is_defining_model_)
	{
		MdispControl(mil_display_, M_UPDATE, M_ENABLE);
		if (isROIValid())
		{
			if (mil_graphic_context_roi_)
			{
				MgraControl(mil_graphic_context_roi_, M_COLOR, M_COLOR_BLUE);
				MgraRect(mil_graphic_context_roi_, mil_graphic_context_list_,
					roi_x_,
					roi_y_,
					roi_x_ + roi_x_size_,
					roi_y_ + roi_y_size_);
			}
		}
		MdispControl(mil_display_, M_UPDATE, M_DISABLE);

		if (display_mode_ != DisplayMode::DM_2D)
		{
			if (disp_depth_gra_list_) MgraFree(disp_depth_gra_list_);
			if (disp_depth_lut_) MbufFree(disp_depth_lut_);

			disp_depth_gra_list_ = M_NULL;
			disp_depth_lut_ = M_NULL;

			if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
				MdispLut(mil_display_, M_DEFAULT);

			display_mode_ = DisplayMode::DM_2D;
		}
		MdispSelectWindow(mil_display_, src_2d_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	std::string found_model_name_2d;
	MIL_INT nb_model_found = locateModel2D(src_2d_image_, found_model_name_2d);

	if (!nb_model_found)
	{
		if (is_defining_model_) {
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_2d_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		return false;
	}

	depth_camera_data_wrapper_->clearPointCloud();

	MIL_DOUBLE rotate_angle = M_INVALID;
	SPoseDataQuat pose;
	MIL_DOUBLE error;
	int index;
	for (int i = 0; i < nb_model_found; i++)
	{
		error = 0.0;
		rotate_angle = M_INVALID;

		index = mod_result_order_vec_.size() == nb_model_found ? mod_result_order_vec_[i] : i;
		if (!createModelDepthMap(mod_context_map_2d_[found_model_name_2d], mod_result_map_2d_[found_model_name_2d], index))
			continue;

		computePoseFrom2dModel(
			mod_context_map_2d_[found_model_name_2d],
			mod_result_map_2d_[found_model_name_2d],
			pose,
			rotate_angle,
			error,
			index);

		if (rotate_angle != M_INVALID)
			break;
	}

	//@ if the result is good, transform the pose from camera coordinate to world
	if (rotate_angle != M_INVALID)
	{
		if (mil_graphic_context_)
		{
			/* Draw edges, position, over the occurrences that were found. */
			MgraColor(mil_graphic_context_, M_COLOR_GREEN);
			MmodDraw(mil_graphic_context_, mod_result_map_2d_[found_model_name_2d], mil_graphic_context_list_, M_DRAW_BOX, index, M_DEFAULT);
		}

		MIL_DOUBLE center_pos_x, center_pos_y;
		MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_POSITION_X, &center_pos_x);
		MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_POSITION_Y, &center_pos_y);
		pose_model_to_image_ = SPoseDataQuat(center_pos_x, center_pos_y, 0, 0, 0, 0, 0);
		pose_model_to_cam_ = pose;
		pose = transformPoseFromCameraToWorld(pose);

		// set up original state
		model_collect_times_ = 1;
		model_rotate_.clear();

		model_poses_x_ = pose.x;
		model_poses_y_ = pose.y;
		model_poses_z_ = pose.z;
		model_rotate_.emplace(error, pose);

		handleCollectedData();
		return true;
	}
	else
	{
		if (is_defining_model_) {
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		return false;
	}
}

bool DepthModelDefinedLocalization::runLocateMethodDepth()
{
	// switch to depth display
	if (!is_defining_model_)
	{
		if (display_mode_ != DisplayMode::DM_DEPTH)
		{
			// display for depth map
			if (disp_depth_gra_list_ == M_NULL && disp_depth_lut_ == M_NULL)
			{
				MgraAllocList(mil_system_, M_DEFAULT, &disp_depth_gra_list_);
				MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, disp_depth_gra_list_);

				MbufAllocColor(mil_system_, 3, MIL_UINT16_MAX, 1, 8 + M_UNSIGNED, M_LUT, &disp_depth_lut_);
				MbufClear(disp_depth_lut_, M_COLOR_BLACK);
				MIL_ID MilDispLutChild = MbufChild1d(disp_depth_lut_, 1, MIL_UINT16_MAX - 1, M_NULL);
				MgenLutFunction(MilDispLutChild, M_COLORMAP_JET, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
				MbufFree(MilDispLutChild);
				MdispLut(mil_display_, disp_depth_lut_);
			}

			display_mode_ = DisplayMode::DM_DEPTH;
		}

		MdispSelectWindow(mil_display_, image_depth_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	depth_camera_data_wrapper_->clearPointCloud();

	SPoseDataQuat pose;
	MIL_DOUBLE error;

	MIL_ID mod_context = M_NULL;
	MIL_ID mod_result = M_NULL;

	// create depth map & do depth model find
	if (!createModelDepthMap(M_NULL, M_NULL, 0))
		return false;

	MIL_INT nb_models = 0;
	//@ try to locate model in depth map 
	for (auto it : mod_context_map_depth_)
	{
		mod_context = it.second;
		mod_result = mod_result_map_depth_[it.first];

		nb_models = locateModelDepth(image_depth_, mod_context, mod_result);
		if (nb_models > 0)
			break;
	}

	// 1. MIL_DOUBLE: model depth value
	// 2. MIL_INT: mil index
	// 3. MIL_ID: model result
	std::multimap<MIL_DOUBLE, std::pair<MIL_INT, MIL_ID>> mod_depth_map;

	if (nb_models > 0)
	{
		for (int i = 0; i < nb_models; i++)
		{
			// compute at least one mean depth
			MIL_DOUBLE occ_mean_elevation = computeOccMeanElevation(mod_context, mod_result, i);
			mod_depth_map.emplace(occ_mean_elevation, std::pair<MIL_INT, MIL_ID>(i, mod_result));

			// compute depth for filter
			if (!depth_order_pick_)
				break;
		}
	}
	else
	{
		if (is_defining_model_) {
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_depth_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
	}

	//@ compute the model pose
	MIL_DOUBLE rotate_angle = M_INVALID;
	if (mod_depth_map.size() > 0)
	{
		auto depth_mod = mod_depth_map.begin();
		MIL_ID mod_result = depth_mod->second.second;

		// deal with partial point cloud
		MIL_ID point_cloud_temp = M3dmapAllocResult(mil_system_, M_POINT_CLOUD_CONTAINER, M_DEFAULT, M_NULL);
		M3dmapControl(point_cloud_temp, M_GENERAL, M_EXTRACTION_OVERLAP, M_MAX);

		MIL_ID point_cloud;
		MIL_INT point_cloud_label = 0;
		if (!createModelPartialPointCloud(
			mod_context,
			mod_result,
			0,
			point_cloud_label,
			point_cloud_temp,
			point_cloud_label))
		{
			point_cloud = point_cloud_temp;
		}
		else
		{
			point_cloud = depth_camera_data_wrapper_->getPointCloudID();
		}

		MIL_INT mod_index = depth_mod->second.first;
		computePoseFromDepthModel(mod_context,
			mod_result,
			pose,
			rotate_angle,
			error,
			depth_mod->first,
			0,
			MIL_DOUBLE(mod_index),
			MIL_DOUBLE(point_cloud));

		M3dmapFree(point_cloud_temp);

		//@ if the result is good, transform the pose from camera coordinate to world
		if (rotate_angle != M_INVALID)
		{
			display3DResult(depth_mod->second.second, mod_index);
			pose = transformPoseFromCameraToWorld(pose);

			// set up original state
			model_collect_times_ = 1;
			model_rotate_.clear();

			model_poses_x_ = pose.x;
			model_poses_y_ = pose.y;
			model_poses_z_ = pose.z;
			model_rotate_.emplace(error, pose);

			handleCollectedData();
			return true;
		}
	}

	return false;
}

bool DepthModelDefinedLocalization::runLocateMethod2dDepth()
{
	// switch to 2D display
	if (!is_defining_model_)
	{
		MdispControl(mil_display_, M_UPDATE, M_ENABLE);
		if (isROIValid())
		{
			if (mil_graphic_context_roi_)
			{
				MgraControl(mil_graphic_context_roi_, M_COLOR, M_COLOR_BLUE);
				MgraRect(mil_graphic_context_roi_, mil_graphic_context_list_,
					roi_x_,
					roi_y_,
					roi_x_ + roi_x_size_,
					roi_y_ + roi_y_size_);
			}
		}
		MdispControl(mil_display_, M_UPDATE, M_DISABLE);

		if (display_mode_ != DisplayMode::DM_2D)
		{
			if (disp_depth_gra_list_) MgraFree(disp_depth_gra_list_);
			if (disp_depth_lut_) MbufFree(disp_depth_lut_);

			disp_depth_gra_list_ = M_NULL;
			disp_depth_lut_ = M_NULL;

			if (MdispInquire(mil_display_, M_LUT_ID, M_NULL) != M_DEFAULT)
				MdispLut(mil_display_, M_DEFAULT);

			display_mode_ = DisplayMode::DM_2D;
		}
		MdispSelectWindow(mil_display_, src_2d_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	std::string found_model_name_2d;
	MIL_INT nb_model_found = locateModel2D(src_2d_image_, found_model_name_2d);

	std::set<MIL_ID> valid_2d_index;
	if (nb_model_found > 0)
	{
		valid_2d_index = deleteOverlap(mod_result_map_2d_[found_model_name_2d]);
		if (valid_2d_index.size() > 0)
		{
			for (auto index : valid_2d_index)
			{
				display2DResult(mod_result_map_2d_[found_model_name_2d], index);
			}
		}
	}

	if (!nb_model_found)
	{
		if (is_defining_model_) {
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_2d_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		return false;
	}

	SPoseDataQuat pose;
	MIL_DOUBLE error;

	MIL_ID mod_context = M_NULL;
	MIL_ID mod_result = M_NULL;

	// compose map vector for mod results
	std::map<std::string, std::vector<MIL_ID>> mod_depth_results_map;	// key: geo name (e.g. geo_depth_1); value: result ID to each model candinate
	for (auto it : mod_context_map_depth_)
	{
		mod_depth_results_map.emplace(it.first, std::vector<MIL_ID>());

		MIL_INT64 mod_type_depth;
		MmodInquire(mod_context_map_2d_[found_model_name_2d], M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_type_depth);
		auto const model_type = *ModelProcess::model_context_result_type_map.find(mod_type_depth);
		MIL_INT64 mod_result_type = model_type.second;

		for (int i = 0; i < nb_model_found; i++)
		{
			MIL_ID mod_result = MmodAllocResult(mil_system_, mod_result_type, M_NULL);
			mod_depth_results_map[it.first].push_back(mod_result);
		}
	}

	// select the needed depth model
	std::multimap<MIL_DOUBLE, INT> mod_index_map; // key: depth; value <mod_context_id, mod_result_id>
	std::map<MIL_INT, std::pair<MIL_ID, MIL_ID>> mod_depth_map; // key: depth; value <mod_context_id, mod_result_id>

	int index_2d;
	depth_camera_data_wrapper_->clearPointCloud();
	for (int i = 0; i < nb_model_found; i++)
	{
		// create depth map & do depth model find
		index_2d = mod_result_order_vec_.size() == nb_model_found ? mod_result_order_vec_[i] : i;
		if (valid_2d_index.find(index_2d) == valid_2d_index.end())
			continue;

		if (!createModelDepthMap(mod_context_map_2d_[found_model_name_2d], mod_result_map_2d_[found_model_name_2d], index_2d))
			break;

		bool has_result = false;
		//@ try to locate model in depth map 
		for (auto it : mod_context_map_depth_)
		{
			mod_context = it.second;
			mod_result = mod_depth_results_map[it.first][index_2d];

			if (locateModelDepth(image_depth_, mod_context, mod_result) > 0)
			{
				has_result = true;

				// compute mean depth
				MIL_DOUBLE occ_mean_elevation = computeOccMeanElevation(mod_context, mod_result, 0);
				mod_index_map.emplace(occ_mean_elevation, index_2d);
				mod_depth_map.emplace(index_2d, std::pair<MIL_ID, MIL_ID>(mod_context, mod_result));
				break;
			}
		}

		// compute depth for filter
		if (!depth_order_pick_ && has_result)
			break;
	}

	//@ compute the model pose
	MIL_DOUBLE rotate_angle = M_INVALID;
	if (mod_index_map.size() > 0)
	{
		//model_target_index_depth_ = 0;
		MIL_DOUBLE occ_mean_elevation = mod_index_map.begin()->first;
		MIL_INT index = mod_index_map.begin()->second;
		auto depth_mod = mod_depth_map[index];

		computePoseFromDepthModel(depth_mod.first,
			depth_mod.second,
			pose,
			rotate_angle,
			error,
			occ_mean_elevation,
			MIL_DOUBLE(index),
			MIL_DOUBLE(0));

		if (rotate_angle != M_INVALID)
		{
			// re-draw depth image & display result
			/*M3dmapExtract(depth_camera_data_wrapper_->getPointCloudID(), image_depth_, M_NULL, M_CORRECTED_DEPTH_MAP, M_ALL, M_DEFAULT);
			MimClip(image_depth_, image_depth_, M_EQUAL, 65535, M_NULL, 0, M_NULL);*/
			createModelDepthMap(mod_result_map_2d_[found_model_name_2d], index, index);

			if (mil_graphic_context_)
			{
				/* Draw edges, position, over the occurrences that were found. */
				MgraColor(mil_graphic_context_, M_COLOR_GREEN);
				MmodDraw(mil_graphic_context_, mod_result_map_2d_[found_model_name_2d], mil_graphic_context_list_, M_DRAW_BOX + M_DRAW_POSITION, index, M_DEFAULT);
			}

			display3DResult(depth_mod.second, M_DEFAULT);
		}
		else
		{
			if (is_defining_model_) {
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			}
		}
	}
	else
	{
		if (is_defining_model_) {
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_depth_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
	}

	for (auto it : mod_context_map_depth_)
	{
		for (int i = 0; i < nb_model_found; i++)
		{
			MmodFree(mod_depth_results_map[it.first][i]);
		}
	}

	//@ if the result is good, transform the pose from camera coordinate to world
	if (rotate_angle != M_INVALID)
	{
		MIL_DOUBLE center_pos_x, center_pos_y;
		MmodGetResult(mod_result_map_2d_[found_model_name_2d], index_2d, M_POSITION_X, &center_pos_x);
		MmodGetResult(mod_result_map_2d_[found_model_name_2d], index_2d, M_POSITION_Y, &center_pos_y);
		pose_model_to_image_ = SPoseDataQuat(center_pos_x, center_pos_y, 0, 0, 0, 0, 0);
		pose_model_to_cam_ = pose;
		pose = transformPoseFromCameraToWorld(pose);

		// set up original state
		model_collect_times_ = 1;
		model_rotate_.clear();

		model_poses_x_ = pose.x;
		model_poses_y_ = pose.y;
		model_poses_z_ = pose.z;
		model_rotate_.emplace(error, pose);

		handleCollectedData();
		return true;
	}
	else
		return false;
}

MIL_INT DepthModelDefinedLocalization::locateModel2D(MIL_ID mil_2d_image, std::string& found_model_name)
{
	if (mod_result_order_vec_.size() > 0)
		mod_result_order_vec_.clear();

	MIL_ID model_find_image = MbufClone(mil_2d_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(model_find_image, 0);
	if (isROIValid())
	{
		MIL_ID mask = MbufClone(mil_2d_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);

		MbufClear(mask, 0);
		MgraColor(M_DEFAULT, 255);
		MgraRectFill(M_DEFAULT, mask, roi_x_, roi_y_, roi_x_ + roi_x_size_, roi_y_ + roi_y_size_);
		MbufCopyCond(mil_2d_image, model_find_image, mask, M_NOT_EQUAL, 0);
		MbufFree(mask);
	}
	else
	{
		MbufCopy(mil_2d_image, model_find_image);
	}

	MIL_INT nb_model_found = 0;
	for (auto it : mod_context_map_2d_)
	{
		MIL_ID mod_context = it.second;
		MIL_ID mod_result = mod_result_map_2d_[it.first];

		MmodFind(mod_context, model_find_image, mod_result);
		MmodGetResult(mod_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

		if (nb_model_found >= 1)
		{
			found_model_name = it.first;
			/*display2DResult(mod_result, M_ALL);*/

			model_2d_center_x_.clear();
			model_2d_center_y_.clear();
			MmodGetResult(mod_result, M_DEFAULT, M_POSITION_X, model_2d_center_x_);
			MmodGetResult(mod_result, M_DEFAULT, M_POSITION_Y, model_2d_center_y_);

			if (search_order_ > SearchOrder::BEST_FIRST)
			{
				MIL_INT model_size_x = MmodInquire(mod_context, M_DEFAULT, M_BOX_SIZE_X, M_NULL);
				MIL_INT model_size_y = MmodInquire(mod_context, M_DEFAULT, M_BOX_SIZE_X, M_NULL);
				double scale = 0.5;

				std::map<int, std::pair<double, double>> model_pose;
				for (int i = 0; i < nb_model_found; i++)
				{
					model_pose.emplace(i,
						std::pair<double, double>(
							model_2d_center_x_[i],
							model_2d_center_y_[i]));
				}

				mod_result_order_vec_.reserve(nb_model_found);

				common_utils::composeSearchOrder(
					(double)image_size_x_,
					(double)image_size_y_,
					(double)model_size_x,
					(double)model_size_y,
					scale,
					model_pose,
					search_order_,
					mod_result_order_vec_);
			}

			break;
		}
		else
		{
			continue;
		}
	}
	MbufFree(model_find_image);

	return nb_model_found;
}

MIL_INT DepthModelDefinedLocalization::locateModelDepth(MIL_ID depth_image, MIL_ID mod_context, MIL_ID& mod_result)
{
	// Find a rectangle from depth map
	/*MbufClear(image_depth_uint8_, 0);
	MimShift(depth_image, image_depth_uint8_, -8);*/
	DPCLib::mapDepthImageTo8Bits(mil_system_, depth_image, image_depth_uint8_);

	/*clock_t t = clock();*/
	MmodFind(mod_context, image_depth_uint8_, mod_result);
	/*t = clock() - t;
	printf("[MmodFind] %f ms.\n", ((float)t));*/

	MIL_INT nb_models = 0;
	MmodGetResult(mod_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_models);

	return nb_models;
}

void DepthModelDefinedLocalization::display2DResult(MIL_ID mod_reuslt, MIL_INT model_index)
{
	if (mil_graphic_context_)
	{
		///* Draw edges, position, over the occurrences that were found. */
		///*MgraClear(mil_graphic_context_, mil_graphic_context_list_);*/
		//MgraColor(mil_graphic_context_, M_COLOR_GREEN);
		//MmodDraw(mil_graphic_context_, mod_reuslt, mil_graphic_context_list_, M_DRAW_POSITION, model_index, M_DEFAULT);
		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		MmodDraw(mil_graphic_context_, mod_reuslt, mil_graphic_context_list_, M_DRAW_EDGES + M_DRAW_POSITION, model_index, M_DEFAULT);
	}
}

void DepthModelDefinedLocalization::display3DResult(MIL_ID mod_result, MIL_INT model_index)
{
	static MIL_DOUBLE color = MIL_UINT16_MAX - 6400;
	MgraColor(M_DEFAULT, color);

	MIL_INT64 mod_type = MmodInquire(mod_result, M_DEFAULT, M_RESULT_TYPE + M_TYPE_MIL_INT64, M_NULL);
	if (mod_type == M_GEOMETRIC || mod_type == M_DEFAULT)
		MmodDraw(M_DEFAULT, mod_result, image_depth_, M_DRAW_EDGES + M_DRAW_POSITION + M_DRAW_BOX, model_index, M_DEFAULT);
	else
		MmodDraw(M_DEFAULT, mod_result, image_depth_, M_DRAW_EDGES + M_DRAW_POSITION /*+ M_DRAW_BOX*/, model_index, M_DEFAULT);
}

std::set<MIL_ID> DepthModelDefinedLocalization::deleteOverlap(MIL_ID mod_result)
{
	std::set<MIL_ID> ret;
	MIL_INT nb_model_found = 0;
	MmodGetResult(mod_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

	for (int i = 0; i < nb_model_found; i++)
		ret.emplace(i);

	return ret;
}

bool DepthModelDefinedLocalization::setupBuffers()
{
	// allocate image_depth_uint8_
	std::string training_name = workspace_ptr_->feature_group().at(feature_group_name_).training_name();
	std::string camera_name = workspace_ptr_->training().at(training_name).camera_name();
	SDepthData depth_data = DPCLib::getCameraDepthData(workspace_ptr_->camera().at(camera_name).depth_camera());
	MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, 16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &image_depth_);
	MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, 8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &image_depth_uint8_);

	// create depth_camera_data_wrapper_
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_;
	auto feature_group_ptr = &workspace_ptr_->feature_group().at(feature_group_name_);

	std::string reference_image_path = base_folder + "/" + feature_group_ptr->reference_image_path();
	if (!utils::fileExists(reference_image_path))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_image_not_load), reference_image_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	mod_result_order_vec_.clear();

	return true;
}

void DepthModelDefinedLocalization::setupDisplays()
{
	// display for depth map
//#if DEBUGING_MODE

	if (workspace_ptr_->system_config().show_3d_viewer())
	{
		MdispAlloc(mil_system_, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_WINDOWED, &depth_disp_debug_);
		MgraAllocList(mil_system_, M_DEFAULT, &depth_gra_list_debug_);
		MdispControl(depth_disp_debug_, M_ASSOCIATED_GRAPHIC_LIST_ID, depth_gra_list_debug_);
		MdispControl(depth_disp_debug_, M_TITLE, M_PTR_TO_DOUBLE(MIL_TEXT("Depth")));
		MdispSelect(depth_disp_debug_, image_depth_);

		MbufAllocColor(mil_system_, 3, MIL_UINT16_MAX, 1, 8 + M_UNSIGNED, M_LUT, &depth_lut_debug_);
		MbufClear(depth_lut_debug_, M_COLOR_BLACK);
		MIL_ID MilDispLutChild = MbufChild1d(depth_lut_debug_, 1, MIL_UINT16_MAX - 1, M_NULL);
		MgenLutFunction(MilDispLutChild, M_COLORMAP_JET, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		MbufFree(MilDispLutChild);
		MdispLut(depth_disp_debug_, depth_lut_debug_);
	}
//#endif
}

bool DepthModelDefinedLocalization::setupModContext()
{
	std::string mod_base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_ + "/";
	std::string feature_name_2d = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_2d();
	std::string feature_name_depth = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_depth();

	int m_number_2d = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().m_number_2d();
	int m_number_depth = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().m_number_depth();

	for (auto const& pattern_window : workspace_ptr_->feature_group().at(feature_group_name_).pattern_windows())
	{
		if (pattern_window.first.find(feature_name_2d) == std::string::npos &&
			pattern_window.first.find(feature_name_depth) == std::string::npos)
		{
			//std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_not_recognized), pattern_window.first); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			continue;
		}

		// check model context file
		std::string mod_context_path = mod_base_folder + pattern_window.first + ".we";
		if (!utils::fileExists(mod_context_path))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_not_model_context), mod_context_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		// load & store model contexts
		std::shared_ptr<wchar_t> wc_mod_context_path_ptr = utils::string2Wchar_tPtr(mod_context_path);
		MIL_ID mod_context = MmodRestore(wc_mod_context_path_ptr.get(), mil_system_, M_WITH_CALIBRATION, M_NULL);

		// create model results
		MIL_INT64 mod_context_type;
		MmodInquire(mod_context, M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_context_type);
		auto const model_context_result_type_map = *ModelProcess::model_context_result_type_map.find(mod_context_type);
		MIL_INT64 mod_result_type = model_context_result_type_map.second;
		MIL_ID mod_resutl = MmodAllocResult(mil_system_, mod_result_type, M_NULL);

		if (pattern_window.first.find(feature_name_2d) != std::string::npos)
		{
			ModelProcess::setContextFromPattern(mod_context, pattern_window.second, m_number_2d);
			mod_context_map_2d_.emplace(pattern_window.first, mod_context);
			mod_result_map_2d_.emplace(pattern_window.first, mod_resutl);
		}
		else if (pattern_window.first.find(feature_name_depth) != std::string::npos)
		{
			ModelProcess::setContextFromPattern(mod_context, pattern_window.second, m_number_depth);
			mod_context_map_depth_.emplace(pattern_window.first, mod_context);
			mod_result_map_depth_.emplace(pattern_window.first, mod_resutl);
		}
	}

	return true;
}

SolidShapeDepthLocalization::SolidShapeDepthLocalization(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name) :
	DepthModelDefinedLocalization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		feature_group_name)
{
}

SolidShapeDepthLocalization::SolidShapeDepthLocalization(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name,
	std::map<std::string, MIL_ID> mil_mod_context_map) :
	DepthModelDefinedLocalization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		feature_group_name,
		mil_mod_context_map)
{
}

bool SolidShapeDepthLocalization::createModelDepthMap(MIL_ID mod_context, MIL_ID mod_result, MIL_INT model_index)
{
	if (mod_result != M_NULL)
	{
		// create model mask image
		MIL_ID image_int_model_mask = MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, 1 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
		MbufClear(image_int_model_mask, 0);
		MgraColor(M_DEFAULT, 1);
		MmodDraw(M_DEFAULT, mod_result, image_int_model_mask, M_DRAW_EDGES, model_index, M_DEFAULT);

		MIL_DOUBLE center_x, center_y;
		MmodGetResult(mod_result, model_index, M_POSITION_X, &center_x);
		MmodGetResult(mod_result, model_index, M_POSITION_Y, &center_y);
		MgraFill(M_DEFAULT, image_int_model_mask, center_x, center_y);

		if (depth_camera_data_wrapper_->createDepthMap(
			src_range_image_,
			image_int_model_mask,
			image_depth_,
			model_index + 1,
			0,
			0,
			!is_defining_model_,
			false,
			true))
		{
			MbufFree(image_int_model_mask);
			return true;
		}
		else
		{
			MbufFree(image_int_model_mask);
			return false;
		}
	}
	else
	{
		MIL_ID image_int_model_mask = M_NULL;

		if (isROIValid())
		{
			MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, 1 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &image_int_model_mask);
			MbufClear(image_int_model_mask, 0);

			if (is_defining_model_)
			{
				MIL_ID mask = MbufClone(src_2d_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
				MbufClear(mask, 0);
				MgraColor(M_DEFAULT, 255);
				MgraRectFill(M_DEFAULT, mask, roi_x_, roi_y_, roi_x_ + roi_x_size_, roi_y_ + roi_y_size_);
				MbufCopyCond(src_2d_image_, image_int_model_mask, mask, M_NOT_EQUAL, 0);
				MbufFree(mask);
			}
			else
			{
				MgraColor(M_DEFAULT, 255);
				MgraRectFill(M_DEFAULT, image_int_model_mask, roi_x_, roi_y_, roi_x_ + roi_x_size_, roi_y_ + roi_y_size_);
			}
		}

		bool ret = depth_camera_data_wrapper_->createDepthMap(
			src_range_image_,
			image_int_model_mask,
			image_depth_,
			model_index + 1,
			0,
			0,
			!is_defining_model_,
			false,
			false);

		MbufFree(image_int_model_mask);
		return ret;
	}
}

void SolidShapeDepthLocalization::computePoseFrom2dModel(MIL_ID mod_context, MIL_ID mod_result, SPoseDataQuat& pose, MIL_DOUBLE& rotate_angle, MIL_DOUBLE& error,
	MIL_DOUBLE param_1, MIL_DOUBLE param_2)
{
	// MIL_DOUBLE param_1 : model index
	MIL_INT roi_x, roi_y, roi_size_x, roi_size_y;
	getModelSquareROIFromResult(mod_result, MIL_INT(param_1), M_NULL, roi_x, roi_y, roi_size_x, roi_size_y);

	MIL_DOUBLE center_pos_x, center_pos_y;
	MmodGetResult(mod_result, MIL_INT(param_1), M_POSITION_X, &center_pos_x);
	MmodGetResult(mod_result, MIL_INT(param_1), M_POSITION_Y, &center_pos_y);
	if (!depth_camera_data_wrapper_->get3DPointUsingNeighborFilter(
		(int)center_pos_x,
		(int)center_pos_y,
		pose.x,
		pose.y,
		pose.z,
		src_range_image_,
		(int)(roi_size_x / 2),
		(int)(roi_size_x / 2),
		(int)(roi_size_y / 2),
		(int)(roi_size_y / 2)))
	{
		return;
	}

	// do 3d plane fit
	MIL_ID geometry_model_plane = M3dmapAlloc(mil_system_, M_GEOMETRY, M_DEFAULT, M_NULL);
	M3dmapSetGeometry(geometry_model_plane, M_PLANE, M_FIT_POINT_CLOUD,
		static_cast<MIL_DOUBLE>(depth_camera_data_wrapper_->getPointCloudID()),
		M_INCLUDE_POINTS_INSIDE_BOX_ONLY, M_DEFAULT,
		/*M_ALL*/static_cast<MIL_DOUBLE>(PC_LABEL(MIL_INT(param_1) + 1)), M_DEFAULT);
	M3dmapInquire(geometry_model_plane, M_DEFAULT, M_FIT_RMS_ERROR, &error);

	MIL_DOUBLE z_axis_angle = getModelZAngleFromResult(mod_result, MIL_INT(param_1));
	rotate_angle = compute3DPLaneRotation(geometry_model_plane, pose, z_axis_angle);
	M3dmapFree(geometry_model_plane);
}

void SolidShapeDepthLocalization::computePoseFromDepthModel(MIL_ID mod_context, MIL_ID mod_result, SPoseDataQuat& pose, MIL_DOUBLE& rotate_angle, MIL_DOUBLE& error,
	MIL_DOUBLE param_1, MIL_DOUBLE param_2, MIL_DOUBLE param_3, MIL_DOUBLE param_4)
{
	static const int MIN_PIXEL_NB = 8 * 8;

	MIL_INT roi_x, roi_y, roi_size_x, roi_size_y;
	getModelSquareROIFromResult(mod_result, MIL_INT(param_3), image_depth_, roi_x, roi_y, roi_size_x, roi_size_y);

	MIL_DOUBLE center_pos_x, center_pos_y, center_pos_z;
	MmodGetResult(mod_result, MIL_INT(param_3), M_POSITION_X, &center_pos_x);
	MmodGetResult(mod_result, MIL_INT(param_3), M_POSITION_Y, &center_pos_y);

	// Get the Z results. 
	MIL_DOUBLE mean_value = 0.0;
	MIL_ID pc = (param_4 == M_NULL) ? depth_camera_data_wrapper_->getPointCloudID() : MIL_ID(param_4);
	MIL_ID image_depth = MbufClone(image_depth_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);
	MbufClear(image_depth, 0);
	M3dmapExtract(pc, image_depth, M_NULL, M_CORRECTED_DEPTH_MAP, PC_LABEL(MIL_INT(param_2) + 1), M_DEFAULT);

	MIL_UINT16* depth_data = (MIL_UINT16*)MbufInquire(image_depth, M_HOST_ADDRESS, M_NULL);
	MIL_INT depth_pitch = MbufInquire(image_depth, M_PITCH, M_NULL);
	depth_data += depth_pitch * roi_y;
	MIL_INT nb = 0;

	for (MIL_INT y = roi_y; y < roi_y + roi_size_y; y += 1)
	{
		for (MIL_INT x = roi_x; x < roi_x + roi_size_x; x++)
		{
			if (depth_data[x] != 0)
			{
				mean_value += (MIL_DOUBLE)(depth_data[x]);
				nb++;
			}
		}
		depth_data += depth_pitch;
	}

	if (nb == 0 || nb < MIN_PIXEL_NB)
	{
		MbufFree(image_depth);
		return;
	}

	mean_value /= (MIL_DOUBLE)(nb);

	// Get the Z calibration information of the depth map.
	MIL_DOUBLE gray_level_size_z, world_pos_z;
	McalInquire(image_depth, M_GRAY_LEVEL_SIZE_Z, &gray_level_size_z);
	McalInquire(image_depth, M_WORLD_POS_Z, &world_pos_z);
	MIL_DOUBLE world_elevation = world_pos_z / gray_level_size_z;
	world_elevation = (gray_level_size_z < 0.0) ? -world_elevation : world_elevation;
	MIL_DOUBLE RectElevation = fabs(gray_level_size_z) * (mean_value - world_elevation);
	center_pos_z = (gray_level_size_z < 0.0) ? -RectElevation : RectElevation;

	pose.x = center_pos_x;
	pose.y = center_pos_y;
	pose.z = center_pos_z;

	// do 3d plane fit
	MIL_ID geometry_model_plane = M3dmapAlloc(mil_system_, M_GEOMETRY, M_DEFAULT, M_NULL);
	M3dmapSetGeometry(geometry_model_plane, M_PLANE, M_FIT_POINT_CLOUD,
		static_cast<MIL_DOUBLE>(pc),
		M_INCLUDE_POINTS_INSIDE_BOX_ONLY, M_DEFAULT,
		/*M_ALL*/static_cast<MIL_DOUBLE>(PC_LABEL(MIL_INT(param_2) + 1)), M_DEFAULT);
	M3dmapInquire(geometry_model_plane, M_DEFAULT, M_FIT_RMS_ERROR, &error);

	MIL_DOUBLE z_axis_angle = getModelZAngleFromResult(mod_result, MIL_INT(param_3));
	rotate_angle = compute3DPLaneRotation(geometry_model_plane, pose, z_axis_angle);
	M3dmapFree(geometry_model_plane);
	MbufFree(image_depth);
}

DepthCircleLocalization::DepthCircleLocalization(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name) :
	SolidShapeDepthLocalization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		feature_group_name)
{
}

DepthCircleLocalization::DepthCircleLocalization(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	config::Workspace* const workspace_ptr,
	const std::string& feature_group_name,
	std::map<std::string, MIL_ID> mil_mod_context_map) :
	SolidShapeDepthLocalization(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		feature_group_name,
		mil_mod_context_map)
{
}

void DepthCircleLocalization::setContextFromPattern(
	MIL_ID mod_context,
	config::Workspace_FeatureGroup::PatternWindow pattern,
	MIL_INT control_number)
{
	MmodControl(mod_context, M_CONTEXT, M_SMOOTHNESS, pattern.m_smoothness());
	MmodControl(mod_context, M_CONTEXT, M_TIMEOUT, pattern.m_timeout());
	MmodControl(mod_context, M_DEFAULT, M_ACCEPTANCE, pattern.m_acceptance());
	MmodControl(mod_context, M_DEFAULT, M_CERTAINTY, pattern.m_certainty());
	MmodControl(mod_context, M_DEFAULT, M_NUMBER, control_number);
	MmodControl(mod_context, M_DEFAULT, M_SCALE_MIN_FACTOR, pattern.search_scale_min());
	MmodControl(mod_context, M_DEFAULT, M_SCALE_MAX_FACTOR, 1 / pattern.search_scale_min());
}

void DepthCircleLocalization::getModelSquareROIFromResult(MIL_ID mod_result, MIL_INT mod_index, MIL_ID image,
	MIL_INT& roi_x, MIL_INT& roi_y, MIL_INT& roi_size_x, MIL_INT& roi_size_y)
{
	MIL_DOUBLE radius;
	MmodGetResult(mod_result, mod_index, M_RADIUS, &radius);

	MIL_DOUBLE center_pos_x, center_pos_y;
	MmodGetResult(mod_result, mod_index, M_POSITION_X, &center_pos_x);
	MmodGetResult(mod_result, mod_index, M_POSITION_Y, &center_pos_y);

	// set ROI
	MIL_DOUBLE half_side = radius * sin(45.0 * DEG_TO_RAD);

	MIL_DOUBLE pixel_size_x = 1, pixel_size_y = 1;
	MIL_DOUBLE x_w[1] = { center_pos_x };
	MIL_DOUBLE y_w[1] = { center_pos_y };
	MIL_DOUBLE x_p[1];
	MIL_DOUBLE y_p[1];

	if (image != M_NULL)
	{
		McalInquire(image, M_PIXEL_SIZE_X, &pixel_size_x);
		McalInquire(image, M_PIXEL_SIZE_Y, &pixel_size_y);
		McalTransformCoordinateList(image, M_WORLD_TO_PIXEL, 1, x_w, y_w, x_p, y_p);
	}
	else
	{
		x_p[0] = x_w[0];
		y_p[0] = y_w[0];
	}

	MIL_DOUBLE half_side_x = half_side / pixel_size_x;
	MIL_DOUBLE half_side_y = half_side / pixel_size_y;

	roi_x = MIL_INT(x_p[0] - half_side_x);
	roi_y = MIL_INT(y_p[0] - half_side_y);
	roi_size_x = MIL_INT(half_side_x * 2);
	roi_size_y = MIL_INT(half_side_y * 2);
}
