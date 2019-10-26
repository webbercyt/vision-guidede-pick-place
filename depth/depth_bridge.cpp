#include "depth_bridge.h"
#include "protobuf_convertor.h"

static const bool ROBOT_MODE = /*false*/true;
static std::string DEBUG_DEPTH_IMAGE_PREFIX = "Depth";

namespace DepthFeatureDefine
{
	static std::vector<DepthModelType> model_type_vector = { DMT_UNDEFINED, DMT_GEOMETRIC, DMT_SHAPE, DMT_SEGMENT };
	static std::vector<DepthModelType> shape_type_vector = { DMT_SHAPE_RECT, DMT_SHAPE_CIRCLE };

	static std::vector<std::pair<DepthModelType, const std::string>> detail_model_type_vector = {
		std::pair<DepthModelType, const std::string>(DMT_SHAPE, "Shape"),
		std::pair<DepthModelType, const std::string>(DMT_SHAPE_RECT, "Rectangle"),
		std::pair<DepthModelType, const std::string>(DMT_SHAPE_CIRCLE, "Circle"),
		std::pair<DepthModelType, const std::string>(DMT_GEOMETRIC, "Geo-pattern"),
		std::pair<DepthModelType, const std::string>(DMT_SEGMENT, "Segment"),
		std::pair<DepthModelType, const std::string>(DMT_UNDEFINED, ""),
	};
}

DepthBridge::DepthBridge(const std::string workspace_path, const MIL_INT mil_window_id) :
	Bridge(workspace_path, mil_window_id)
{
}

DepthBridge::~DepthBridge()
{
	if (debug_image_) MbufFree(debug_image_);
	if (debug_range_image_) MbufFree(debug_range_image_);
}

bool DepthBridge::continuousGrab(const std::string camera_name)
{
	for (auto it : localization_ptr_map_)
	{
		it.second->resetlocateState();
	}

	return Bridge::continuousGrab(camera_name);
}

bool DepthBridge::singleGrab(const std::string camera_name, bool display_image)
{
	for (auto it : localization_ptr_map_)
	{
		it.second->resetlocateState();
	}

	return Bridge::singleGrab(camera_name, display_image);
}

bool DepthBridge::connectCamera(const std::string& camera_name)
{
	if (camera_ptr_map_.find(camera_name) == camera_ptr_map_.end()) // ptr does not exist
	{
		std::shared_ptr<Camera> cam =
			std::move(DepthCameraManager::createDepthCamPtr(
				mil_application_, 
				mil_gige_system_,
				mil_gige_display_, 
				mil_window_id_, 
				workspace_ptr_, 
				camera_name));

		if (cam == nullptr)
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Unrecognized Depth Camera Model: ", 
				workspace_ptr_->camera().at(camera_name).depth_camera().camera_model_name()); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		if (!cam->isConnected()) 
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::failed_camara_connect)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}
		else {
			camera_ptr_map_[camera_name] = std::move(cam);

			if (camera_display_image_component_map_.find(camera_name) != camera_display_image_component_map_.end())
			{
				camera_display_image_component_map_[camera_name].clear();
			}
			else
			{
				camera_display_image_component_map_.emplace(camera_name, std::vector <std::string>());
			}

			std::vector<std::string> camera_display_components = camera_ptr_map_[camera_name]->getDispImageComponents();

			for (auto component : camera_display_components)
				camera_display_image_component_map_[camera_name].push_back(component);
		}
	}
	return true;
}

bool DepthBridge::endCameraAcquisition(const std::string camera_name)
{
	if (camera_display_image_component_map_.find(camera_name) != camera_display_image_component_map_.end())
	{
		camera_display_image_component_map_[camera_name].clear();
	}
	return Bridge::endCameraAcquisition(camera_name);
}

int DepthBridge::setDisplayImage(const std::string& camera_name, std::string image_component_name)
{
	if (camera_ptr_map_.find(camera_name) != camera_ptr_map_.end())
	{
		return camera_ptr_map_[camera_name]->setDisplayImage(image_component_name);
	}
	else
	{
		return ERR_CAMERA_NOT_CONNECTED;
	}
}

std::vector<std::string> DepthBridge::getDispImageComponents(const std::string& camera_name)
{
	if (camera_display_image_component_map_.find(camera_name) != camera_display_image_component_map_.end())
	{
		return camera_display_image_component_map_[camera_name];
	}
	else
	{
		return std::vector<std::string>();
	}
}

std::vector<std::string> DepthBridge::getCameraModelNames()
{
	return DepthCameraManager::getCameraModelNames();
}

bool DepthBridge::calibrationWithRobotMovement(const std::string calibration_name)
{
	std::string training_name = workspace_ptr_->calibration().at(calibration_name).training_name();
	std::string camera_name = workspace_ptr_->training().at(training_name).camera_name();

	if (camera_name == "")
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::calibration_check_lock)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	else
	{
		if (camera_ptr_map_.find(camera_name) != camera_ptr_map_.end())
		{
			camera_ptr_map_[camera_name]->setCameraOutput(CameraOutputMode::CALIBRATION);
		}
		else
			return false;
	}

	return Bridge::calibrationWithRobotMovement(calibration_name);
}

// Feature Groups
DepthModelType DepthBridge::getCurrentModelType(const std::string feature_group_name, bool is_shape_check)
{
	if (workspace_ptr_->feature_group().find(feature_group_name)
		== workspace_ptr_->feature_group().end())
	{
		// print something
		return DMT_UNDEFINED;
	}

	const auto model_type = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().model_type();

	if (is_shape_check)
	{
		bool is_shape = false;
		for (auto it : shape_type_vector)
		{
			if ((DepthModelType)model_type == it)
			{
				is_shape = true;
				break;
			}
		}

		if (!is_shape)
		{
			return DMT_UNDEFINED;
		}
	}

	bool has_model = false;
	for (auto it : detail_model_type_vector)
	{
		if ((DepthModelType)model_type == it.first)
		{
			has_model = true;
			break;
		}
	}

	if (has_model)
		return (DepthModelType)model_type;
	else
		return DMT_UNDEFINED;
}

DepthModelType DepthBridge::getModelType(const std::string model_type_str)
{
	for (auto it : detail_model_type_vector)
	{
		if (model_type_str == it.second)
		{
			return it.first;
		}
	}

	return DMT_UNDEFINED;
}

std::string DepthBridge::getModelTypeText(DepthModelType model_type)
{
	for (auto it : detail_model_type_vector)
	{
		if ((DepthModelType)model_type == it.first)
			return it.second;
	}

	return "";
}

bool DepthBridge::updateDefinedFeature(const std::string feature_group_name)
{
	if (feature_group_ptr_map_.find(feature_group_name) == feature_group_ptr_map_.end()) {
		// Need to load reference image before restoring all the defined feature
		return false;
	}

	std::string training_name = workspace_ptr_->feature_group().at(feature_group_name).training_name();
	const auto training = workspace_ptr_->training().at(training_name);
	std::string pose_calculator_name = training.pose_calculator_name();

	SPoseDataQuat reference_pose = ProtobufConvertor::protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).reference_pose());
	if (!feature_group_ptr_map_.at(feature_group_name)->updateDefinedFeature(reference_pose)) {
		return false;
	}

	feature_group_ptr_map_.at(feature_group_name)->displayOneContext("");
	return saveFeatureContext(feature_group_name, "");
}

bool DepthBridge::displaySearchRange(const std::string feature_group_name, const std::string feature_name)
{
	if (feature_group_ptr_map_.find(feature_group_name) == feature_group_ptr_map_.end()) {
		// Need to load reference image before restoring all the defined feature
		return false;
	}

	feature_group_ptr_map_.at(feature_group_name)->resetDisplay();
	return Bridge::displaySearchRange(feature_group_name, feature_name);
}

bool DepthBridge::setCurrentModelType(const std::string feature_group_name,
	DepthModelType model_type) 
{
	if (workspace_ptr_->feature_group().find(feature_group_name)
		== workspace_ptr_->feature_group().end()) {
		// print something
		return false;
	}

	workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_feature_group()->set_model_type(model_type);
	return true;
}

bool DepthBridge::createNewPatternWindows(const std::string feature_group_name, DepthModelType model_type)
{
	if (workspace_ptr_->feature_group().find(feature_group_name)
		== workspace_ptr_->feature_group().end()) {
		// print something
		return false;
	}

	switch (model_type)
	{
	case DepthFeatureDefine::DMT_SHAPE:
	case DepthFeatureDefine::DMT_SHAPE_RECT:
	case DepthFeatureDefine::DMT_SHAPE_CIRCLE:
	{
		static const double search_scale_min = 0.8;
		static const double search_scale_max = 1.5;
		static const double aspect_ratio_min = 0.5;
		static const double aspect_ratio_max = 2.0;

		// set feature group
		workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_shape_feature_group()->set_search_scale_min(search_scale_min);
		workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_shape_feature_group()->set_search_scale_max(search_scale_max);
		workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_shape_feature_group()->set_aspect_ratio_min(aspect_ratio_min);
		workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_shape_feature_group()->set_aspect_ratio_max(aspect_ratio_max);

		// create 1 2d pattern windows
		std::string feature_name_2d = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().feature_name_2d() + std::to_string(1);
		(*workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[feature_name_2d] = config::Workspace::FeatureGroup::PatternWindow();
		auto pattern_ptr = &workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows()->at(feature_name_2d);
		pattern_ptr->set_search_scale_min(search_scale_min);
		pattern_ptr->set_search_scale_max(search_scale_max);
		pattern_ptr->set_m_detail_level(Workspace_MLevel_M_VERY_HIGH_PROTO);
		pattern_ptr->mutable_depth_feature()->set_aspect_ratio_min(aspect_ratio_min);
		pattern_ptr->mutable_depth_feature()->set_aspect_ratio_max(aspect_ratio_max);

		// create 1 depth pattern window
		std::string feature_name_depth = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().feature_name_depth() + std::to_string(1);
		(*workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[feature_name_depth] = config::Workspace::FeatureGroup::PatternWindow();
		pattern_ptr = &workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows()->at(feature_name_depth);
		pattern_ptr->set_search_scale_min(search_scale_min);
		pattern_ptr->set_search_scale_max(search_scale_max);
		pattern_ptr->mutable_depth_feature()->set_aspect_ratio_min(aspect_ratio_min);
		pattern_ptr->mutable_depth_feature()->set_aspect_ratio_max(aspect_ratio_max);
		pattern_ptr->set_m_detail_level(Workspace_MLevel_M_MEDIUM_PROTO);

		break;
	}
	case DepthFeatureDefine::DMT_GEOMETRIC:
	{
		// create 1 2d pattern windows
		std::string feature_name_2d = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().feature_name_2d() + std::to_string(1);
		(*workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[feature_name_2d] = config::Workspace::FeatureGroup::PatternWindow();
		auto pattern_ptr = &workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows()->at(feature_name_2d);
		pattern_ptr->set_search_delta_angle(180);
		pattern_ptr->set_enable_search_scale(true);
		pattern_ptr->set_search_scale_min(0.5);
		//pattern_ptr->set_search_scale_max(1.1);
		pattern_ptr->set_m_detail_level(Workspace_MLevel_M_HIGH_PROTO);

		// create 1 depth pattern window
		std::string feature_name_depth = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().feature_name_depth() + std::to_string(1);
		(*workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[feature_name_depth] = config::Workspace::FeatureGroup::PatternWindow();
		pattern_ptr = &workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows()->at(feature_name_depth);
		pattern_ptr->set_search_delta_angle(30);

		pattern_ptr->set_enable_search_scale(true);
		pattern_ptr->set_search_scale_min(0.9);
		//pattern_ptr->set_search_scale_max(1.1);
		pattern_ptr->set_m_detail_level(Workspace_MLevel_M_HIGH_PROTO);

		break;
	}
	case DepthFeatureDefine::DMT_SEGMENT:
	{
		// create 1 segment pattern windows
		std::string feature_name_seg = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().feature_name_seg() + std::to_string(1);
		(*workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[feature_name_seg] = config::Workspace::FeatureGroup::PatternWindow();
		auto pattern_ptr = &workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows()->at(feature_name_seg);
		pattern_ptr->set_search_delta_angle(180);
		pattern_ptr->set_search_scale_min(0.2);
		pattern_ptr->set_search_scale_max(5.0);

		pattern_ptr->mutable_depth_segment_feature()->set_search_angle(15.0);
		pattern_ptr->mutable_depth_segment_feature()->set_search_length(60.0);
		pattern_ptr->mutable_depth_segment_feature()->set_segment_area_min(0);
		pattern_ptr->mutable_depth_segment_feature()->set_segment_area_max(0);
		pattern_ptr->set_m_detail_level(Workspace_MLevel_M_VERY_HIGH_PROTO);

		break;
	}
	default:
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Undefined model type !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	}

	return true;
}

bool DepthBridge::displayModelSizes(const std::string feature_group_name)
{
	if (feature_group_ptr_map_.find(feature_group_name) == feature_group_ptr_map_.end())
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::laser_load_first)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	
	return feature_group_ptr_map_[feature_group_name]->displayModelSizes();
}

bool DepthBridge::createNewPatternWindow(const std::string feature_group_name,
	const std::string pattern_window_name)
{
	if (workspace_ptr_->feature_group().find(feature_group_name)
		== workspace_ptr_->feature_group().end()) {
		// print something
		return false;
	}
	(*workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[pattern_window_name] =
		config::Workspace::FeatureGroup::PatternWindow();
	return true;
}

std::vector<std::string> DepthBridge::getModelTypeStringList() 
{
	std::vector<std::string> model_string_list;

	for (auto model_type : model_type_vector)
	{
		for (auto detail_model_type : detail_model_type_vector)
		{
			if (model_type == detail_model_type.first)
			{
				model_string_list.push_back(detail_model_type.second);
				break;
			}
		}
	}

	return model_string_list;
}

std::vector<std::string> DepthBridge::getShapeTypeStringList() 
{
	std::vector<std::string> shape_string_list;

	for (auto shape_type : shape_type_vector)
	{
		for (auto detail_model_type : detail_model_type_vector)
		{
			if (shape_type == detail_model_type.first)
			{
				shape_string_list.push_back(detail_model_type.second);
				break;
			}
		}
	}

	return shape_string_list;
}

bool DepthBridge::updateReferenceImage(const std::string feature_group_name)
{
	// Camera Fetch Current Image and Robot Fetch Current Pose, save to prototxt
	std::string training_name = workspace_ptr_->feature_group().at(feature_group_name).training_name();
	const auto training = workspace_ptr_->training().at(training_name);
	std::string calibration_name = training.calibration_name();
	std::string camera_name = training.camera_name();
	std::string robot_name = training.robot_name();

	if (camera_name == "")
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::failed_camara_connect)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	if (robot_name == "")
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::connect_robot)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	std::string pose_calculator_name = training.pose_calculator_name();
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name;

	int mounting_type = workspace_ptr_->camera().at(camera_name).mounting_type();
	if (mounting_type == MOUNTED_ON_ROBOT_ARM && ROBOT_MODE)
	{
		if (!isRobotConnected(robot_name))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::connect_robot)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false; // No Robot Connection
		}
	}

	if (!utils::createFolder(base_folder)) // path is invalide
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_invalid_path), base_folder, '\n'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	displayReset();

	// Take a shot, save image
	// when grabbing failed (e.g., camera connection lost), break
	// set camera to capture 3d data not only texture
	if (!isCameraConnected(camera_name)) 
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,translation::messageOut(translation::message::failed_update_reference)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}
	else
	{
		camera_ptr_map_[camera_name]->setCameraOutput(CameraOutputMode::LOCALIZATON);
	}

	if (!singleGrab(camera_name)) {
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,translation::messageOut(translation::message::failed_update_reference)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	std::string img_path = base_folder + "/Reference.wef";
	if (!camera_ptr_map_[camera_name]->saveImage(img_path)) // Save failed
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::invalid_image_path)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	if (camera_ptr_map_[camera_name]->getRangeImageID() != M_NULL)
	{
		std::string range_img_path = base_folder + "/RefRangeImg.wef";
		if (!camera_ptr_map_[camera_name]->saveRangeImage(range_img_path, camera_ptr_map_[camera_name]->getRangeImageID())) // Save failed
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::invalid_range_image_path)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_feature_group()->set_reference_range_image_path("RefRangeImg.wef");
	}

	SPoseDataQuat reference_pose;
	if (mounting_type == MOUNTED_ON_ROBOT_ARM && ROBOT_MODE)
	{
		if (robotGetCurPosition(robot_name, camera_name, reference_pose))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_no_robot_position)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}
	}
	else if (mounting_type == MOUNTED_ON_FIXED)
		reference_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->calibration().at(calibration_name).calibration_center_pose());
	else
		reference_pose = SPoseDataQuat();

	*workspace_ptr_->mutable_pose_calculator()->at(pose_calculator_name).mutable_reference_pose() = ProtobufConvertor::protobufSPoseDataQuat2QuaternionPose(reference_pose);
	workspace_ptr_->mutable_feature_group()->at(feature_group_name).set_reference_image_path("Reference.wef");

	// copy the current depth camera setting
	workspace_ptr_->mutable_feature_group()->at(feature_group_name).mutable_depth_feature_group()->mutable_depth_camera()->
		CopyFrom(workspace_ptr_->camera().at(camera_name).depth_camera());

	return loadReferenceImage(feature_group_name, false);
}

bool DepthBridge::loadReferenceImage(const std::string feature_group_name, bool show_feature)
{
	displayReset();
	if (workspace_ptr_->feature_group().find(feature_group_name) != workspace_ptr_->feature_group().end()) // feature group exists
	{
		if (workspace_ptr_->feature_group().at(feature_group_name).has_reference_image_path() &&
			workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().has_model_type())
		{
			if (feature_group_ptr_map_.find(feature_group_name) != feature_group_ptr_map_.end())
			{
				// Clear Feature Map every time when update or load new
				feature_group_ptr_map_[feature_group_name].reset();
			}

			auto model_type = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().model_type();
			switch (model_type)
			{
			case DMT_SHAPE_RECT:
			{
				feature_group_ptr_map_[feature_group_name] = std::make_shared<DepthRectFeature>(mil_application_,
					mil_system_,
					mil_display_,
					mil_window_id_,
					workspace_ptr_,
					feature_group_name);

				break;
			}
			case DMT_SHAPE_CIRCLE:
			{
				feature_group_ptr_map_[feature_group_name] = std::make_shared<DepthCircleFeature>(mil_application_,
					mil_system_,
					mil_display_,
					mil_window_id_,
					workspace_ptr_,
					feature_group_name);

				break;
			}
			case DMT_GEOMETRIC:
			{
				feature_group_ptr_map_[feature_group_name] = std::make_shared<DepthGeometryFeature>(mil_application_,
					mil_system_,
					mil_display_,
					mil_window_id_,
					workspace_ptr_,
					feature_group_name);

				break;
			}
			case DMT_SEGMENT:
			{
				feature_group_ptr_map_[feature_group_name] = std::make_shared<DepthSegmentFeature>(mil_application_,
					mil_system_,
					mil_display_,
					mil_window_id_,
					workspace_ptr_,
					feature_group_name);

				break;
			}
			default:
				return false;
			}

			if (!feature_group_ptr_map_.at(feature_group_name)->loadReferenceImage()) {
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_failed_load_refernece)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false;
			}
			else
			{
				std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_finish_load_refernece)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
			}

			if (show_feature)
			{
				return loadAllSavedFeatures(feature_group_name);
			}
			else
				return true;
		}
		else
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,translation::messageOut(translation::message::feature_failed_image_path)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
	}
	return false;
}

bool DepthBridge::createNewFeature(const std::string feature_group_name, const std::string feature_name) {
	if (feature_group_ptr_map_.find(feature_group_name) == feature_group_ptr_map_.end()) {
		// Need to load reference image before restoring all the defined feature
		return false;
	}

	std::string training_name = workspace_ptr_->feature_group().at(feature_group_name).training_name();
	const auto training = workspace_ptr_->training().at(training_name);
	std::string pose_calculator_name = training.pose_calculator_name();

	SPoseDataQuat reference_pose = ProtobufConvertor::protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).reference_pose());
	if (feature_group_ptr_map_.at(feature_group_name)->createNewFeature(reference_pose))
	{
		if (!feature_group_ptr_map_.at(feature_group_name)->defineFeature()) return false;
		feature_group_ptr_map_.at(feature_group_name)->displayOneContext("");
		return saveFeatureContext(feature_group_name, feature_name);
	}
	else
		return false;
}

bool DepthBridge::createNewGeo(const std::string feature_group_name, const std::string feature_name)
{
	// create 2d pattern windows
	if (workspace_ptr_->feature_group().find(feature_group_name) ==
		workspace_ptr_->feature_group().end())
		return false;

	return DepthFeatureGroup::duplicateFeature(workspace_ptr_, feature_group_name, feature_name);
}

std::shared_ptr<DepthLocalization> DepthBridge::createModelLocalization(const std::string feature_group_name)
{
	std::shared_ptr<DepthLocalization> localization_ptr = nullptr;
	auto model_type = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().model_type();

	switch (model_type)
	{
	case M_RECTANGLE:
	{
		localization_ptr = std::make_shared<DepthRectLocalization>(mil_application_,
			mil_gige_system_,
			mil_gige_display_,
			mil_window_id_,
			workspace_ptr_,
			feature_group_name);

		break;
	}
	case M_CIRCLE:
	{
		localization_ptr = std::make_shared<DepthCircleLocalization>(mil_application_,
			mil_gige_system_,
			mil_gige_display_,
			mil_window_id_,
			workspace_ptr_,
			feature_group_name);

		break;
	}
	case M_GEOMETRIC:
	{
		localization_ptr = std::make_shared<DepthGeometryLocalization>(mil_application_,
			mil_gige_system_,
			mil_gige_display_,
			mil_window_id_,
			workspace_ptr_,
			feature_group_name);

		break;
	}
	case M_SEGMENT:
	{
		localization_ptr = std::make_shared<DepthSegmentLocalization>(mil_application_,
			mil_gige_system_,
			mil_gige_display_,
			mil_window_id_,
			workspace_ptr_,
			feature_group_name);

		break;
	}
	default:
		break;
	}

	return localization_ptr;
}

bool DepthBridge::accuTestInit(const std::string accu_test_name,
	std::string& robot_name,
	std::string& camera_name,
	std::string& feature_group_name,
	std::string& pose_calculator_name,
	SPoseDataQuat& reference_pose,
	SPoseDataQuat& model_reference_pose,
	std::vector<SPoseDataQuat>& pose_plan_quat_vec,
	const bool& using_images)
{
	if (workspace_ptr_->accu_test().find(accu_test_name) == workspace_ptr_->accu_test().end()) // accuracy test not exists in the proto buf
	{
		return false;
	}

	displayReset();

	std::string training_name = workspace_ptr_->accu_test().at(accu_test_name).training_name();
	const auto training = workspace_ptr_->training().at(training_name);
	feature_group_name = workspace_ptr_->training().at(training_name).feature_group_name();
	pose_calculator_name = workspace_ptr_->training().at(training_name).pose_calculator_name();
	std::string calibration_name = training.calibration_name();
	robot_name = training.robot_name();
	camera_name = training.camera_name();

	if (!using_images)
	{
		if (robot_name == "" || camera_name == "")
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::makesure_lock)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		if (!isRobotConnected(robot_name))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::makesure_robot)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false; // No Robot Connection
		}

		if (!isCameraConnected(camera_name))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::accu_camera_not_connencted), camera_name); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}
		else
		{
			camera_ptr_map_[camera_name]->setCameraOutput(CameraOutputMode::LOCALIZATON);
		}

		// Clean Calibration Image filed
		if (workspace_ptr_->accu_test().at(accu_test_name).images().size() > 0)
		{
			workspace_ptr_->mutable_accu_test()->at(accu_test_name).clear_images();
		}
	}

	if (!localizationPtrInit(feature_group_name))
	{
		return false;
	}

	reference_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).reference_pose());
	model_reference_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).model_reference_pose());

	if (robot_move_ptr_map_.find(accu_test_name) == robot_move_ptr_map_.end()) // ptr does not exist
	{
		robot_move_ptr_map_[accu_test_name] = std::make_shared<RobotMove>(mil_application_, mil_system_, workspace_ptr_, accu_test_name);
	}

	robot_move_ptr_map_[accu_test_name]->accuTestPlan(reference_pose, pose_plan_quat_vec);

	return true;
}

bool DepthBridge::accuTest1(const std::string accu_test_name)
{
	end_accu_test_ = false;
	std::vector<SPoseDataQuat> pose_plan_quat_vec;
	SPoseDataQuat reference_pose;
	SPoseDataQuat model_reference_pose;
	std::string robot_name;
	std::string camera_name;
	std::string feature_group_name;
	std::string pose_calculator_name;
	std::string base_folder = workspace_ptr_->workspace_folder() + "/accu_test/" + accu_test_name;

	if (!accuTestInit(accu_test_name, 
		robot_name, 
		camera_name, 
		feature_group_name, 
		pose_calculator_name, 
		reference_pose, 
		model_reference_pose, 
		pose_plan_quat_vec))
	{
		return false;
	}

	SPoseDataQuat std_dev = SPoseDataQuat();
	double success_poses = 0; // successfully localized positions, that will fall within the movement constraint
	SPoseDataQuat movement_contraint = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->accu_test().at(accu_test_name).movement_constraint());

	// Move Robot Using Robot communication
	for (int i = 0; i < pose_plan_quat_vec.size(); i++)
	{
		if (end_accu_test_)
		{
			break;
		}
		accu_test_progress_ = float(i) / pose_plan_quat_vec.size();

		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::accu_at), i, "/", pose_plan_quat_vec.size(), '\n'); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;

		Sleep(workspace_ptr_->system_config().robot_response_time());
		if (robotMoveToPosition(robot_name, camera_name, pose_plan_quat_vec[i])) {
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::failed_move)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			end_accu_test_ = true;
			break;
		}
		else {
			std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::console_success)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
			Sleep(workspace_ptr_->system_config().robot_response_time());
			// move the robot to a position, and then take  a picture, and then get the localization error
			// Take a shot, save image
			bool can_stop = false;
			bool has_result = false;
			localization_ptr_map_[feature_group_name]->resetlocateState();

			while (!can_stop)
			{
				if (!singleGrab(camera_name, false)) {
					std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::accu_failed_test_image)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
					end_accu_test_ = true;
					break;
				}

				//// TODO(yt) later maybe enable offline accuTest
				//std::string img_path = base_folder + "/AccuImg" + std::to_string(i) + ".wef";
				//if (!camera_ptr_map_[camera_name]->saveImage(img_path)) // Save failed
				//{
				//	std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Image Path is Invalid, Please check access to the folder"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				//	break;
				//}
				//// Save Image and Pose Data
				//auto new_image = workspace_ptr_->mutable_accu_test()->at(accu_test_name).add_images();
				//new_image->set_image_path("AccuImg" + std::to_string(i) + ".wef");
				//*new_image->mutable_pose() = protobufSPoseDataQuat2QuaternionPose(pose_plan_quat_vec[i]);

				localization_ptr_map_[feature_group_name]->pushFrame(
					camera_ptr_map_[camera_name]->getImageID(),
					camera_ptr_map_[camera_name]->getRangeImageID(),
					pose_plan_quat_vec[i],
					can_stop,
					has_result);
			}

			if (end_accu_test_)
				break;

			if (has_result) // check if the error is way bigger than acceptable result
			{
				success_poses += 1;
				SPoseDataQuat model_pose = SPoseDataQuat();
				localization_ptr_map_[feature_group_name]->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);
				std_dev = std_dev + (model_pose.dis(model_reference_pose)).abs();

				/*std::string o_msg = printCoutMsg(LEVEL_BASIC_INFO, "T: [", model_pose.x, ", ", model_pose.y, ", ", model_pose.z, "]"
					" R: [",
					model_pose.q1, ", ", model_pose.q2, ", ", model_pose.q3, ", ", model_pose.q4, "]\n"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;*/
			}
		}
	}

	// Save to protobuf
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_x(std_dev.x / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_y(std_dev.y / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_z(std_dev.z / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q1(std_dev.q1 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q2(std_dev.q2 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q3(std_dev.q3 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q4(std_dev.q4 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).set_success_ratio(success_poses / float(pose_plan_quat_vec.size()));

	// Clean this for another accuracy test
	robot_move_ptr_map_[accu_test_name].reset();
	robot_move_ptr_map_.erase(accu_test_name);

	localization_ptr_map_[feature_group_name].reset();
	localization_ptr_map_.erase(feature_group_name);

	if (end_accu_test_)
	{
		// accu test end with exception
		return false;
	}
	return true;
}

bool DepthBridge::accuTest2(const std::string accu_test_name)
{
	end_accu_test_ = false;
	std::vector<SPoseDataQuat> pose_plan_quat_vec;
	SPoseDataQuat reference_pose;
	SPoseDataQuat model_reference_pose;
	std::string robot_name;
	std::string camera_name;
	std::string feature_group_name;
	std::string pose_calculator_name;
	std::string base_folder = workspace_ptr_->workspace_folder() + "/accu_test/" + accu_test_name;

	if (!accuTestInit(accu_test_name,
		robot_name,
		camera_name,
		feature_group_name,
		pose_calculator_name,
		reference_pose,
		model_reference_pose,
		pose_plan_quat_vec))
	{
		return false;
	}

	// For record accutest accuracy
	SPoseDataQuat std_dev = SPoseDataQuat();
	// IMPORTANT, NEEDS TO MAKE SURE THE TRANSFORMED POSE FALLS INTO THE DISPALCEMENT RANGE
	// OTHERWISE, THE ROBOT MAY MOVE TO DANGEROUS PLACE
	SPoseDataQuat movement_contraint = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->accu_test().at(accu_test_name).movement_constraint());
	double success_poses = 0; // successfully localized positions, that will fall within the movement constraint

	// Move Robot Using Robot communication
	for (int i = 0; i < pose_plan_quat_vec.size(); i++)
	{
		if (end_accu_test_)
		{
			break;
		}
		accu_test_progress_ = float(i) / pose_plan_quat_vec.size();

		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::accu_at), i, "/", pose_plan_quat_vec.size(), '\n'); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;

		Sleep(workspace_ptr_->system_config().robot_response_time());
		if (robotMoveToPosition(robot_name, camera_name, pose_plan_quat_vec[i]))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::move_1_failed)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			end_accu_test_ = true;
			break;
		}
		else {
			std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::move_1_succeed)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
			Sleep(workspace_ptr_->system_config().robot_response_time());
			// move the robot to a position, and then take  a picture, and then get the localization error
			bool can_stop = false;
			bool has_result = false;
			localization_ptr_map_[feature_group_name]->resetlocateState();

			while (!can_stop)
			{
				if (end_accu_test_)
				{
					break;
				}

				if (!singleGrab(camera_name, false)) {
					std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,translation::messageOut(translation::message::accu_failed_test_image)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
					end_accu_test_ = true;
					break;
				}

				//// TODO(yt): uncomment to allow offline mode 
				//std::string img_path = base_folder + "/AccuImg" + std::to_string(i) + ".wef";
				//if (!camera_ptr_map_[camera_name]->saveImage(img_path)) // Save failed
				//{
				//	std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Image Path is Invalid, Please check access to the folder"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				//	break;
				//}
				//// Save Image and Pose Data
				//auto new_image = workspace_ptr_->mutable_accu_test()->at(accu_test_name).add_images();
				//new_image->set_image_path("AccuImg" + std::to_string(i) + ".wef");
				//*new_image->mutable_pose() = protobufSPoseDataQuat2QuaternionPose(pose_plan_quat_vec[i]);

				localization_ptr_map_[feature_group_name]->pushFrame(
					camera_ptr_map_[camera_name]->getImageID(),
					camera_ptr_map_[camera_name]->getRangeImageID(),
					pose_plan_quat_vec[i],
					can_stop,
					has_result);
			}

			if (end_accu_test_)
			{
				break;
			}

			SPoseDataQuat model_pose = SPoseDataQuat();
			if (has_result) // check if the error is way bigger than acceptable result
				localization_ptr_map_[feature_group_name]->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);
			else
				continue;

			// Get the transformed reference pose, move back again, then get new localization error
			SPoseDataQuat transformed_reference_pose;
			localization_ptr_map_[feature_group_name]->getRobotMoveToReference(
				reference_pose,
				model_reference_pose,
				model_pose,
				transformed_reference_pose);

			// the difference is beyond constains
			// so stop current accu test and resume to other pose for accutest
			if (reference_pose.dis(transformed_reference_pose).abs() > movement_contraint)
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::accu_move)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				continue;
			}

			// Move the robot to the transformed pose
			Sleep(workspace_ptr_->system_config().robot_response_time());
			if (robotMoveToPosition(robot_name, camera_name, transformed_reference_pose))
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::move_2_failed)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				end_accu_test_ = true;
				break;
			}
			else {
				std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::move_2_succeed)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
				Sleep(workspace_ptr_->system_config().robot_response_time());

				has_result = false;
				can_stop = false;
				localization_ptr_map_[feature_group_name]->resetlocateState();

				while (!can_stop)
				{
					if (end_accu_test_)
					{
						break;
					}

					if (!singleGrab(camera_name, false)) {
						std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,translation::messageOut(translation::message::accu_failed_test_image)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
						end_accu_test_ = true;
						break;
					}

					//// TODO(yt): uncomment to allow offline mode
					//// Save image for debuging
					//std::string img_path = base_folder + "/AccuImg" + std::to_string(i) + ".wef";
					//if (!camera_ptr_map_[camera_name]->saveImage(img_path)) // Save failed
					//{
					//	std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Image Path is Invalid, Please check access to the folder"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
					//	break;
					//}
					//// Save Image and Pose Data
					//auto new_image = workspace_ptr_->mutable_accu_test()->at(accu_test_name).add_images();
					//new_image->set_image_path("AccuImg" + std::to_string(i) + ".wef");
					//*new_image->mutable_pose() = protobufSPoseDataQuat2QuaternionPose(pose_plan_quat_vec[i]);

					localization_ptr_map_[feature_group_name]->pushFrame(
						camera_ptr_map_[camera_name]->getImageID(),
						camera_ptr_map_[camera_name]->getRangeImageID(),
						transformed_reference_pose,
						can_stop,
						has_result);
				}

				if (end_accu_test_)
				{
					break;
				}

				if (has_result) // check if the error is way bigger than acceptable result
				{
					success_poses += 1;
					localization_ptr_map_[feature_group_name]->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);
					std_dev = std_dev + (model_pose.dis(model_reference_pose)).abs();

					/*std::string o_msg = printCoutMsg(LEVEL_BASIC_INFO, "T: [", model_pose.x, ", ", model_pose.y, ", ", model_pose.z, "]"
						" R: [",
						model_pose.q1, ", ", model_pose.q2, ", ", model_pose.q3, ", ", model_pose.q4, "]\n"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;*/
				}
			}
		}
	}

	// Save to protobuf
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_x(std_dev.x / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_y(std_dev.y / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_z(std_dev.z / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q1(std_dev.q1 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q2(std_dev.q2 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q3(std_dev.q3 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_std_dev()->set_q4(std_dev.q4 / float(success_poses));
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).set_success_ratio(float(success_poses) / float(pose_plan_quat_vec.size()));

	// Clean this for another accuracy test
	robot_move_ptr_map_[accu_test_name].reset();
	robot_move_ptr_map_.erase(accu_test_name);

	localization_ptr_map_[feature_group_name].reset();
	localization_ptr_map_.erase(feature_group_name);

	if (end_accu_test_)
	{
		// Exception meet
		return false;
	}
	else
	{
		return true;
	}
}

bool DepthBridge::jobInit()
{
	bool status = false;

	for (auto job : workspace_ptr_->job())
	{
		std::string job_name = job.first;
		if (!job.second.activated())
			continue;

		if (job.second.job_type() != 0) /*Check if the job is a localization job*/
		{
			continue;
		}

		int search_order = workspace_ptr_->job().at(job_name).search_order();

		std::string training_name = workspace_ptr_->job().at(job_name).training_name();
		if (training_name.empty()) {
			continue;
		}

		const auto training = workspace_ptr_->training().at(training_name);

		if (training.localization_mode() == DEPTH)
		{
			std::string feature_group_name = training.feature_group_name();
			std::string calibration_name = training.calibration_name();
			std::string pose_calculator_name = training.pose_calculator_name();
			std::string robot_name = training.robot_name();
			std::string camera_name = training.camera_name();

			// Connect Resources
			if (!isRobotConnected(robot_name)) {
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::makesure_robot), robot_name); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false; // No Robot Connection
			}

			if (!isCameraConnected(camera_name)) {
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_no_carema), camera_name); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false;
			}
			else
			{
				camera_ptr_map_[camera_name]->setCameraOutput(CameraOutputMode::LOCALIZATON);
			}

			if (!localizationPtrInit(feature_group_name, search_order))
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_stop)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false;
			}

			// check camera uniformity
			auto current_camera_model = workspace_ptr_->camera().at(camera_name).depth_camera().camera_model_name();
			auto feature_camera_model = workspace_ptr_->feature_group().at(feature_group_name).depth_feature_group().depth_camera().camera_model_name();
			if (current_camera_model != "" && 
				current_camera_model != feature_camera_model)
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Current camera is different to feature group camera !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false;
			}

			status = true;
		}
	}
	return status;
}

bool DepthBridge::jobLocateObject(const std::string& job_name, const SPoseDataQuat & capture_pose)
{
	const std::string training_name = workspace_ptr_->job().at(job_name).training_name();
	const std::string camera_name = workspace_ptr_->training().at(training_name).camera_name();
	//const std::string robot_name = workspace_ptr_->training().at(training_name).robot_name();
	//const std::string base_folder = workspace_ptr_->workspace_folder() + "/job/" + job_name;
	const std::string calibration_name = workspace_ptr_->training().at(training_name).calibration_name();
	const std::string feature_group_name = workspace_ptr_->training().at(training_name).feature_group_name();
	MIL_ID mil_image = camera_ptr_map_[camera_name]->getImageID();
	MIL_ID mil_range_image = camera_ptr_map_[camera_name]->getRangeImageID();
	bool locate_status = false;

	for (int i = workspace_ptr_->job().at(job_name).retry_times() - 1; i >= 0; --i)
	{
		if (end_job_)
		{
			// Need to free resource
			break;
		}

		bool can_stop = false;
		localization_ptr_map_[feature_group_name]->resetlocateState();

		SPoseDataQuat pose_base_to_tool;
		if (workspace_ptr_->camera().at(camera_name).mounting_type() == MOUNTED_ON_FIXED)
		{
			pose_base_to_tool = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->calibration().at(calibration_name).calibration_center_pose());
		}
		else if (workspace_ptr_->camera().at(camera_name).mounting_type() == MOUNTED_ON_ROBOT_ARM)
		{
			pose_base_to_tool = capture_pose;
		}

		while (!can_stop)
		{
			if (!singleGrab(camera_name, false)) {
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_error_image)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				end_accu_test_ = true;
				break;
			}

			jobSaveDebugImage(camera_name, job_name, capture_pose, job_image_idx_);

			localization_ptr_map_[feature_group_name]->pushFrame(
				mil_image,
				mil_range_image,
				/*capture_pose*/pose_base_to_tool,
				can_stop,
				locate_status);
		}

		if (locate_status) // check if the error is way bigger than acceptable result
		{
			SPoseDataQuat model_pose = SPoseDataQuat();
			localization_ptr_map_[feature_group_name]->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);

			/*std::string o_msg = printCoutMsg(LEVEL_BASIC_INFO, "T: [", model_pose.x, ", ", model_pose.y, ", ", model_pose.z, "]"
				" R: [",
				model_pose.q1, ", ", model_pose.q2, ", ", model_pose.q3, ", ", model_pose.q4, "]\n"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;*/
		}

		if (locate_status) // Locate succeed
		{
			return true;
		}
		// Wait sleep time before relocalize
		Sleep(workspace_ptr_->job().at(job_name).retry_sleep());
	}

	return locate_status;
}

bool DepthBridge::jobLocateObjectMethod2(const std::string & job_name, SPoseDataQuat& capture_pose)
{
	const std::string training_name = workspace_ptr_->job().at(job_name).training_name();
	const std::string camera_name = workspace_ptr_->training().at(training_name).camera_name();
	const std::string robot_name = workspace_ptr_->training().at(training_name).robot_name();
	//const std::string base_folder = workspace_ptr_->workspace_folder() + "/job/" + job_name;
	//const std::string calibration_name = workspace_ptr_->training().at(training_name).calibration_name();
	const std::string feature_group_name = workspace_ptr_->training().at(training_name).feature_group_name();
	const std::string pose_calculator_name = workspace_ptr_->training().at(training_name).pose_calculator_name();
	MIL_ID mil_image = camera_ptr_map_[camera_name]->getImageID();
	bool locate_status = false;
	bool reposition_locate_status = false;
	SPoseDataQuat reference_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).reference_pose());
	SPoseDataQuat model_reference_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).model_reference_pose());
	SPoseDataQuat movement_contraint = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->job().at(job_name).movement_constraint());
	SPoseDataQuat transformed_reference_pose;

	if (workspace_ptr_->camera().at(camera_name).mounting_type() != MOUNTED_ON_ROBOT_ARM)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_method_1)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	/*Move to Reference Pose, and then relocate*/
	for (int i = workspace_ptr_->job().at(job_name).retry_times() - 1; i >= 0; --i)
	{
		if (end_job_)
		{
			break;
		}

		/*First locate*/
		if (!jobLocateObject(job_name, capture_pose))
		{
			/*Failed to locate at the initial position*/
			break;
		}

		/*Moved to transformed reference pose*/
		SPoseDataQuat model_pose;
		localization_ptr_map_.at(feature_group_name)->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);
		localization_ptr_map_.at(feature_group_name)->getRobotMoveToReference(reference_pose,
			model_reference_pose,
			model_pose,
			transformed_reference_pose);

		// the difference is beyond constains
		// so recapture image, hope the product was moved by people
		if (reference_pose.dis(transformed_reference_pose).abs() > movement_contraint)
		{
			std::cout << "pose: " 
				<< transformed_reference_pose.x << ", " 
				<< transformed_reference_pose.y << ", "
				<< transformed_reference_pose.z << ", "
				<< transformed_reference_pose.q1 << ", "
				<< transformed_reference_pose.q2 << ", "
				<< transformed_reference_pose.q3 << ", "
				<< transformed_reference_pose.q4
				<< std::endl;

			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,translation::messageOut(translation::message::job_exceed_constraint)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			Sleep(workspace_ptr_->job().at(job_name).retry_sleep());
			continue;
		}

		// Move to Reference Pose
		Sleep(workspace_ptr_->system_config().robot_response_time());
		if (robotMoveToPosition(robot_name, camera_name, transformed_reference_pose, "01",false)) {  // 01 denote reference pose
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::failed_move)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			Sleep(workspace_ptr_->job().at(job_name).retry_sleep());
		}
		else {
			std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::success_move)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
			Sleep(workspace_ptr_->system_config().robot_response_time());
			locate_status = true;
			break;
		}
	}

	/*Job Locate again after moved to the reference pose*/
	if (locate_status)
	{
		capture_pose = transformed_reference_pose; // The capture pose was updated
		return jobLocateObject(job_name, capture_pose);
	}
	else
	{
		return false;
	}
}

int DepthBridge::jobMoveToPose(const std::string& job_name, const SPoseDataQuat& taught_pose)
{
	const std::string training_name = workspace_ptr_->job().at(job_name).training_name();
	const std::string camera_name = workspace_ptr_->training().at(training_name).camera_name();
	const std::string robot_name = workspace_ptr_->training().at(training_name).robot_name();
	const std::string base_folder = workspace_ptr_->workspace_folder() + "/job/" + job_name;
	const std::string calibration_name = workspace_ptr_->training().at(training_name).calibration_name();
	const std::string feature_group_name = workspace_ptr_->training().at(training_name).feature_group_name();
	const std::string pose_calculator_name = workspace_ptr_->training().at(training_name).pose_calculator_name();
	//MIL_ID mil_image = camera_ptr_map_[camera_name]->getImageID();
	//bool locate_status = false;
	//bool reposition_locate_status = false;
	SPoseDataQuat internal_taught_pose; // Real taught pose, when mounted, the value will be inversed
	SPoseDataQuat transformed_pose;
	SPoseDataQuat movement_contraint = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->job().at(job_name).movement_constraint());
	SPoseDataQuat model_reference_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->pose_calculator().at(pose_calculator_name).model_reference_pose());

	// Received taught pose at the last element of the vector
	// If the camera was mounted fixed, thus it will transform the received tcp to the calibration robot base (which is a virtual one)
	if (workspace_ptr_->camera().at(camera_name).mounting_type() == MOUNTED_ON_FIXED)
	{
		SPoseDataQuat calibration_center_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->calibration().at(calibration_name).calibration_center_pose());
		derivateCoordWhenMounted(taught_pose, internal_taught_pose, calibration_center_pose);
	}
	else
	{
		internal_taught_pose = taught_pose;
	}

	SPoseDataQuat model_pose = SPoseDataQuat();
	localization_ptr_map_.at(feature_group_name)->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);
	localization_ptr_map_.at(feature_group_name)->getRobotMoveToReference(
		internal_taught_pose,
		model_reference_pose,
		model_pose,
		transformed_pose);

	SPoseDataQuat diff = transformed_pose.dis(internal_taught_pose).abs();
	if (transformed_pose.dis(internal_taught_pose).abs() > movement_contraint)
	{
		std::cout << "pose: "
			<< transformed_pose.x << ", "
			<< transformed_pose.y << ", "
			<< transformed_pose.z << ", "
			<< transformed_pose.q1 << ", "
			<< transformed_pose.q2 << ", "
			<< transformed_pose.q3 << ", "
			<< transformed_pose.q4
			<< std::endl;

		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_exceed_constraint)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		Sleep(workspace_ptr_->job().at(job_name).retry_sleep());
		// When Movement exceed constraint, then need to relocate product
		return ERR_MOVEMENT_EXCEED_CONSTRAINT;
	}
	Sleep(workspace_ptr_->system_config().robot_response_time());

	if (workspace_ptr_->camera().at(camera_name).mounting_type() == MOUNTED_ON_FIXED)
	{
		SPoseDataQuat calibration_center_pose = protobufQuaternionPose2SPoseDataQuat(workspace_ptr_->calibration().at(calibration_name).calibration_center_pose());
		SPoseDataQuat inverse_calibration_center_pose;
		inverseCoordWhenMounted(camera_name, calibration_center_pose, inverse_calibration_center_pose);
		derivateCoordWhenMounted(transformed_pose, transformed_pose, inverse_calibration_center_pose);
	}

	int state = robotMoveToPosition(robot_name, "", transformed_pose, "03", false);
	if (state != ERR_NULL && state != STATE_YASKAWA_NEXT_POS) {
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::failed_move)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		Sleep(workspace_ptr_->job().at(job_name).retry_sleep());
		return ERR_MOVEMENT_FAILED;
	}
	else if (state == STATE_YASKAWA_NEXT_POS) {
		return ERR_NULL;
	}
	else {
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::success_move)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
		Sleep(workspace_ptr_->system_config().robot_response_time());
		// Only set new job to false with a successful move
		// TODO(xc): may need to consider if the second or third movement failed, e.g. it exceed the movement constrain
		// But not able to re-locate the object again
		return ERR_NULL;
	}
}

bool DepthBridge::runJob1()
{
	return false;
}

bool DepthBridge::runJob2()
{
	return false;
}

bool DepthBridge::jobSaveDebugImage(const std::string& camera_name, const std::string& job_name, const SPoseDataQuat& recv_pose, int& idx)
{
	const std::string base_folder = workspace_ptr_->workspace_folder() + "/job/" + job_name;

	if (workspace_ptr_->job().at(job_name).debug_feature())
	{
		MIL_ID mil_image = camera_ptr_map_[camera_name]->getImageID();
		MIL_ID mil_range_image = camera_ptr_map_[camera_name]->getRangeImageID();

		// at least need range image
		if (!mil_range_image) return false;

		int index = idx;
		int image_size = workspace_ptr_->job().at(job_name).images_size();
		if (index > image_size)
			index = image_size + idx;

		auto new_image = workspace_ptr_->mutable_job()->at(job_name).add_images();
		*new_image->mutable_pose() = protobufSPoseDataQuat2QuaternionPose(recv_pose);

		std::string image_save_path_2d = "JobImg" + std::to_string(index) + ".wef";
		if (mil_image)
		{		
			std::string img_path = base_folder + "/" + image_save_path_2d;
			
			if (!camera_ptr_map_[camera_name]->saveImage(img_path)) // Save failed
			{
				std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::invalid_image_path)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
				return false;
			}
		}
		new_image->set_image_path(image_save_path_2d);

		// save depth image
		std::string image_save_path_depth = DEBUG_DEPTH_IMAGE_PREFIX + "JobImg" + std::to_string(index) + ".wef";
		std::string img_path_depth = base_folder + "/" + image_save_path_depth;

		if (!camera_ptr_map_[camera_name]->saveRangeImage(img_path_depth, camera_ptr_map_[camera_name]->getRangeImageID())) // Save failed
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::invalid_image_path)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		idx += 1;
	}
	return true;
}

bool DepthBridge::debug_feature_job(const std::string job_name, const std::string & image_name, const std::string&, const std::string &) 
{
	std::string training_name = workspace_ptr_->job().at(job_name).training_name();
	std::string pose_calculator_name = workspace_ptr_->training().at(training_name).pose_calculator_name();
	std::string feature_group_name = workspace_ptr_->training().at(training_name).feature_group_name();

	// load 2d image & depth image
	std::string image_path_2d = workspace_ptr_->workspace_folder() + "/job/" + job_name + "/" + image_name;
	std::string image_path_depth = workspace_ptr_->workspace_folder() + "/job/" + job_name + "/" + DEBUG_DEPTH_IMAGE_PREFIX + image_name;

	if (debug_image_) MbufFree(debug_image_);
	if (debug_range_image_) MbufFree(debug_range_image_);

	MbufRestore(utils::string2Wchar_tPtr(image_path_2d).get(), mil_gige_system_, &debug_image_);
	MbufRestore(utils::string2Wchar_tPtr(image_path_depth).get(), mil_gige_system_, &debug_range_image_);

	if (!debug_range_image_)
	{
		if (debug_image_)
			MbufFree(debug_image_);

		return false;
	}

	SPoseDataQuat pose_base_to_tool = SPoseDataQuat();
	auto images = workspace_ptr_->job().at(job_name).images();
	for (auto image : images)
	{
		if (image.image_path() == image_name)
		{
			pose_base_to_tool = protobufQuaternionPose2SPoseDataQuat(image.pose());
			break;
		}
	}

	localizationPtrInit(feature_group_name, workspace_ptr_->job().at(job_name).search_order());

	bool can_stop = false;
	bool has_result = false;
	localization_ptr_map_[feature_group_name]->resetlocateState();
	localization_ptr_map_[feature_group_name]->pushFrame(
		debug_image_,
		debug_range_image_,
		pose_base_to_tool,
		can_stop,
		has_result);

	/*if (debug_image_) MbufFree(debug_image_);
	MbufFree(debug_range_image_);*/

	if (has_result)
	{
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Locate successed !"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	}
	else
	{
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Locate failed !"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	}

	return has_result;
}

bool DepthBridge::singleJobInit(const std::string& job_name)
{
	std::string training_name = workspace_ptr_->job().at(job_name).training_name();
	if (training_name.empty()) {
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_no_training)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	const auto training = workspace_ptr_->training().at(training_name);
	const auto search_order = workspace_ptr_->job().at(job_name).search_order();

	std::string feature_group_name = training.feature_group_name();
	std::string camera_name = training.camera_name();

	if (!isCameraConnected(camera_name)) {
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_no_carema), camera_name); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	if (!localizationPtrInit(feature_group_name, search_order))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_stop)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	// Clean Previouse Job Images
	if (workspace_ptr_->job().at(job_name).images().size() > 0) {
		workspace_ptr_->mutable_job()->at(job_name).clear_images();
	}
	return true;
}

bool DepthBridge::localizationPtrInit(const std::string& feature_group_name, const int search_order)
{
	if (localization_ptr_map_.find(feature_group_name) != localization_ptr_map_.end())
		localization_ptr_map_.at(feature_group_name).reset();// start from fresh everytime

	localization_ptr_map_[feature_group_name] = std::move(createModelLocalization(feature_group_name));
	if (!localization_ptr_map_[feature_group_name]->isAllocationSuccess())
	{
		localization_ptr_map_.at(feature_group_name).reset();
		localization_ptr_map_.erase(feature_group_name);
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::job_stop)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	// set search order
	localization_ptr_map_.at(feature_group_name)->setSearchOrder(search_order);

	return true;
}

// training
bool DepthBridge::createNewTraining(int training_number, LocalizationMode dimension)
{
	std::string training_name = "training_" + std::to_string(training_number);
	std::string calibration_name = "calibration_" + std::to_string(training_number);
	std::string feature_group_name = "feature_group_" + std::to_string(training_number);
	std::string accu_test_name = "accu_test_" + std::to_string(training_number);
	std::string pose_calc_name = "pose_calculator_" + std::to_string(training_number);
	(*workspace_ptr_->mutable_training())[training_name] = config::Workspace::Training();

	(*workspace_ptr_->mutable_calibration())[calibration_name] = config::Workspace::Calibration();
	workspace_ptr_->mutable_training()->at(training_name).set_calibration_name(calibration_name);
	workspace_ptr_->mutable_training()->at(training_name).set_localization_mode(dimension);
	workspace_ptr_->mutable_calibration()->at(calibration_name).set_training_name(training_name);

	(*workspace_ptr_->mutable_feature_group())[feature_group_name] = config::Workspace::FeatureGroup();
	workspace_ptr_->mutable_feature_group()->at(feature_group_name).set_training_name(training_name);
	workspace_ptr_->mutable_training()->at(training_name).set_feature_group_name(feature_group_name);

	(*workspace_ptr_->mutable_pose_calculator())[pose_calc_name] = config::Workspace::PoseCalculator();
	workspace_ptr_->mutable_pose_calculator()->at(pose_calc_name).set_training_name(training_name);
	workspace_ptr_->mutable_training()->at(training_name).set_pose_calculator_name(pose_calc_name);

	(*workspace_ptr_->mutable_accu_test())[accu_test_name] = config::Workspace::AccuTest();
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).set_training_name(training_name);
	workspace_ptr_->mutable_training()->at(training_name).set_accu_test_name(accu_test_name);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_x(25);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_y(25);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_z(10);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_q1(25);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_q2(0);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_q3(0);
	workspace_ptr_->mutable_accu_test()->at(accu_test_name).mutable_movement_constraint()->set_q4(0);

	return true;
}
