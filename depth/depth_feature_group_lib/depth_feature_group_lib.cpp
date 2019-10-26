#include "depth_feature_group_lib.h"
#include "utils.h"
#include "depth_localization_lib.h"
#include "depth_camera.h"

DepthFeatureGroup::DepthFeatureGroup(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	FeatureGroup(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name),
	define_succesed_(false),
	reference_range_image_(M_NULL),
	reference_depth_image_(M_NULL),
	result_image_(M_NULL)
{
	camera_name_ = workspace_ptr->training().at(workspace_ptr->feature_group().at(node_name).training_name()).camera_name();
	
	std::string training_name = workspace_ptr_->feature_group().at(node_name).training_name();
	auto calibration_name_ = workspace_ptr_->training().at(training_name).calibration_name();
	std::string mil_cal_path = workspace_ptr_->workspace_folder() + "/calibration/" + calibration_name_ + "/" + workspace_ptr_->calibration().at(calibration_name_).mca_path();
	// Need to handle when calibration can't be restored
	if (!utils::fileExists(mil_cal_path))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::calibration_file_not_exist), mil_cal_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}
	std::shared_ptr<wchar_t> wc_mil_cal_path_ptr = utils::string2Wchar_tPtr(mil_cal_path);
	mil_calibration_ = McalRestore(wc_mil_cal_path_ptr.get(), mil_system_, M_DEFAULT, M_NULL);
	McalSetCoordinateSystem(mil_calibration_, M_CAMERA_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
}

DepthFeatureGroup::~DepthFeatureGroup()
{
	McalFree(mil_calibration_);

	MbufFree(reference_image_original_);
	if (reference_range_image_) MbufFree(reference_range_image_);
	if (reference_depth_image_) MbufFree(reference_depth_image_);
	if (result_image_) MbufFree(result_image_);

	for (auto const& result : mil_mod_result_map_)
	{
		if (result.second)
			MmodFree(result.second);
	}
}

bool DepthFeatureGroup::createNewFeature(SPoseDataQuat pose_base_to_tool)
{
	// zy Idea: Get the Image buffer ID that related to the host display system
	// If different, could not create feature map directly on the image, should load the reference first.
	MIL_ID check_image_buffer;
	MdispInquire(mil_display_, M_SELECTED, &check_image_buffer);

	if (check_image_buffer == result_image_)
	{
		MdispSelectWindow(mil_display_, reference_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}
	else if (check_image_buffer != reference_image_)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Please Load or Update the reference image in Feature Group first."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, mil_graphic_context_list_);
	MgraControl(mil_graphic_context_, M_FIXTURE, M_USE_SOURCE_FIRST);

	/* Enable the interactive mode. */
	MdispControl(mil_display_, M_GRAPHIC_LIST_INTERACTIVE, M_ENABLE);
	/* Enable the use of action keys */
	MgraControlList(mil_graphic_context_list_, M_LIST, M_DEFAULT, M_ACTION_KEYS, M_ENABLE);

	resetReferenceImage();
	MIL_INT mil_rectangle_label = DefineRectROI(mil_system_, M_NULL, mil_graphic_context_, mil_graphic_context_list_);

	// Insert New Region into Workspace
	MIL_DOUBLE location_x =(MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_CORNER_TOP_LEFT_X, M_NULL);
	MIL_DOUBLE location_y = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_CORNER_TOP_LEFT_Y, M_NULL);
	MIL_DOUBLE size_x = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_RECTANGLE_WIDTH, M_NULL);
	MIL_DOUBLE size_y = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_RECTANGLE_HEIGHT, M_NULL);

	int locate_method = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().locate_method();

	std::string feature_name_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d();
	for (auto const& feature : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (feature.first.find(feature_name_2d) != std::string::npos)
		{
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[feature.first].set_location_x(location_x);
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[feature.first].set_location_y(location_y);
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[feature.first].set_size_x(size_x);
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[feature.first].set_size_y(size_y);
		}
	}

	pose_base_to_tool_ = pose_base_to_tool;
	return true;
}

bool DepthFeatureGroup::createSearchRange(std::string)
{
	MIL_ID check_image_buffer;
	MdispInquire(mil_display_, M_SELECTED, &check_image_buffer);
	if (check_image_buffer != reference_image_ &&
		(result_image_ != M_NULL && check_image_buffer != result_image_))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Please Load or Update the reference image in Feature Group first."); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	resetDisplay();

	/* Enable the interactive mode. */
	MdispControl(mil_display_, M_GRAPHIC_LIST_INTERACTIVE, M_ENABLE);
	/* Enable the use of action keys */
	MgraControlList(mil_graphic_context_list_, M_LIST, M_DEFAULT, M_ACTION_KEYS, M_ENABLE);

	MIL_INT mil_rectangle_label = DefineRectROI(mil_system_, reference_image_, mil_graphic_context_, mil_graphic_context_list_);

	// This is the bounding box information of searching range
	MIL_DOUBLE x = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_CORNER_TOP_LEFT_X, M_NULL);
	MIL_DOUBLE y = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_CORNER_TOP_LEFT_Y, M_NULL);
	MIL_DOUBLE size_x = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_RECTANGLE_WIDTH, M_NULL);
	MIL_DOUBLE size_y = (MIL_DOUBLE)MgraInquireList(mil_graphic_context_list_, M_GRAPHIC_LABEL(mil_rectangle_label), M_DEFAULT, M_RECTANGLE_HEIGHT, M_NULL);

	if (x <= 0 || x <= 0 || size_x <= 0 || size_y <= 0)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Search Range window is not properly defined"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	// Update workspace object
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_x((int)x);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_y((int)y);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_x_size((int)size_x);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_y_size((int)size_y);

	return true;
}

void DepthFeatureGroup::displaySearchRange(std::string)
{
	auto roi_x = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x();
	auto roi_y = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y();
	auto roi_x_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x_size();
	auto roi_y_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y_size();

	if (roi_x > 0 && roi_y > 0 && roi_x_size > 0 && roi_y_size > 0)
	{
		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
		MgraRect(mil_graphic_context_, mil_graphic_context_list_,
			roi_x,
			roi_y,
			min(roi_x + roi_x_size, MbufInquire(reference_image_, M_SIZE_X, M_NULL) - 1),
			min(roi_y + roi_y_size, MbufInquire(reference_image_, M_SIZE_Y, M_NULL) - 1));

		// text "search region"
		MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		std::string text = "[search region]";
		std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
		MgraText(mil_graphic_context_, mil_graphic_context_list_, roi_x + 1, roi_y + 1, wc_text.get());
	}
}

bool DepthFeatureGroup::defineFeature()
{
	auto feature_group_ptr = &workspace_ptr_->feature_group().at(node_name_);

	// set up 2d geo patterns
	for (auto const& pattern_window : feature_group_ptr->pattern_windows())
	{
		if (!isValidPattern(pattern_window))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_depth_window),
				pattern_window.first); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}

		if (mil_mod_context_map_.find(pattern_window.first) == mil_mod_context_map_.end()) // Mod context not exist
		{
			mil_mod_context_map_[pattern_window.first] = M_NULL;
		}
		else // Delete previous context, should
		{
			if (mil_mod_context_map_[pattern_window.first])
				MmodFree(mil_mod_context_map_[pattern_window.first]);
			mil_mod_context_map_[pattern_window.first] = M_NULL;
		}

		if (mil_mod_result_map_.find(pattern_window.first) == mil_mod_result_map_.end()) // Mod context not exist
		{
			mil_mod_result_map_[pattern_window.first] = M_NULL;
		}
		else // Delete previous result, should
		{
			if (mil_mod_result_map_[pattern_window.first])
				MmodFree(mil_mod_result_map_[pattern_window.first]);
			mil_mod_result_map_[pattern_window.first] = M_NULL;
		}
	}

	createModel();
	return define_succesed_;
}

bool DepthFeatureGroup::updateDefinedFeature(SPoseDataQuat pose_base_to_tool)
{
	pose_base_to_tool_ = pose_base_to_tool;
	resetReferenceImage();
	return defineFeature();
}

bool DepthFeatureGroup::loadReferenceImage()
{
	auto feature_group_ptr = &workspace_ptr_->feature_group().at(node_name_);

	// Convert String into wchar_t* 
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
	std::string image_path = base_folder + "/" + feature_group_ptr->reference_image_path();

	if (!utils::fileExists(image_path))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_image_not_load), image_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
	MbufImport(wc_image_path.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &reference_image_);
	MbufImport(wc_image_path.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &reference_image_original_);

	// mil_display_ select this reference image
	MdispSelectWindow(mil_display_, reference_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, mil_graphic_context_list_);
	MgraControl(mil_graphic_context_, M_FIXTURE, M_USE_SOURCE_FIRST);

	// load range images
	std::string range_image_path = base_folder + "/" + feature_group_ptr->depth_feature_group().reference_range_image_path();
	if (!utils::fileExists(range_image_path))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_image_not_load), range_image_path); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	std::shared_ptr<wchar_t> wc_image_path_1 = utils::string2Wchar_tPtr(range_image_path);
	MbufImport(wc_image_path_1.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &reference_range_image_);

	// load reference image if it is existing
	image_path = base_folder + "/" + feature_group_ptr->depth_feature_group().reference_depth_image_path();
	if (utils::fileExists(image_path))
	{
		std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
		if (reference_depth_image_)
			MbufFree(reference_depth_image_);
		MbufImport(wc_image_path.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &reference_depth_image_);
	}

	return true;
}

bool DepthFeatureGroup::saveModelContext(const std::string& feature_name)
{
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
	if (!utils::createFolder(base_folder)) // 
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_wrong_folder), base_folder); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	if (mil_mod_context_map_.size() >= 1)
	{
		MIL_INT32 mod_type_int32 = 0;
		for (auto const& context : mil_mod_context_map_)
		{
			if (context.second == M_NULL)
				continue;

			MIL_INT32 mod_type_int32_temp;
			MmodInquire(context.second, M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT32, &mod_type_int32_temp);

			if (mod_type_int32 == 0)
			{
				mod_type_int32 = mod_type_int32_temp;
			}
			else
			{
				if (mod_type_int32 != mod_type_int32_temp)
					printf("Model context types of an object are different, is it correct ?");
			}

			// Convert String into wchar_t* 
			std::string context_path = base_folder + "/" + context.first + ".we"; // the key is context.first	
			std::shared_ptr<wchar_t> wc_context_path = utils::string2Wchar_tPtr(context_path);
			MmodSave(wc_context_path.get(), context.second, M_WITH_CALIBRATION);
		}
	}

	if (reference_depth_image_)
	{
		// Convert String into wchar_t*
		std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
		std::string image_path = base_folder + "/" + workspace_ptr_->feature_group().at(node_name_).depth_feature_group().reference_depth_image_path();

		// check folder existance
		if (utils::dirExists(utils::dirNameFromPath(image_path)) == -2) // this is a file instead of folder
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::check_path), image_path, '\n'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}
		else // create folder no matter it exist or not
		{
			if (utils::createFolder(utils::dirNameFromPath(image_path)))
			{
				std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
				MbufExport(wc_image_path.get(), M_MIL + M_WITH_CALIBRATION, reference_depth_image_);
				return true;
			}
			else
			{
				return false;
			}
		}
	}

	return true;
}

bool DepthFeatureGroup::loadAllModelContext()
{
	// Convert String into wchar_t*
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
	//std::string image_path = base_folder + "/" + workspace_ptr_->feature_group().at(node_name_).reference_image_path();
	bool status = true;
	std::vector<std::string> not_found_mod_name_vec;

	for (auto pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		/*Don't restore met and meas in this way*/
		if (pattern_window.first.substr(0, 3) == "met" || pattern_window.first.substr(0, 4) == "meas")
		{
			continue;
		}

		std::string mod_context_path = base_folder + "/" + pattern_window.first + ".we";
		if (!utils::fileExists(mod_context_path))
		{
			//std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Pattern: ", mod_context_path, " not exists"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			not_found_mod_name_vec.push_back(pattern_window.first);
			status = false;
			continue;
		}

		if (mil_mod_context_map_.find(pattern_window.first) == mil_mod_context_map_.end()) // Mod context not exist
		{
			mil_mod_context_map_[pattern_window.first] = M_NULL;
			//SHOULDN'T ALLOC THE MOD CONTEXT, SINCE THE RESTORE WILL CREATE A MOD PLACE AUTOMATICALLY
			//MmodAlloc(mil_system_, M_GEOMETRIC, M_DEFAULT, &mil_mod_context_map_[pattern_window.first]);
		}
		// TODO:(xc), need to figure out better way to redefine the feature
		else // Delete previous context, should
		{
			MmodFree(mil_mod_context_map_[pattern_window.first]);
			mil_mod_context_map_[pattern_window.first] = M_NULL;
			//SHOULDN'T ALLOC THE MOD CONTEXT, SINCE THE RESTORE WILL CREATE A MOD PLACE AUTOMATICALLY
			//MmodAlloc(mil_system_, M_GEOMETRIC, M_DEFAULT, &mil_mod_context_map_[pattern_window.first]);
		}

		if (MmodRestore(utils::string2Wchar_tPtr(mod_context_path).get(), mil_system_, M_WITH_CALIBRATION, &mil_mod_context_map_[pattern_window.first]) == M_NULL)
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Model restore failed ", mod_context_path, '\n'); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
			return false;
		}
	}

	if (not_found_mod_name_vec.size() == workspace_ptr_->feature_group().at(node_name_).pattern_windows().size())
	{
		status = true;
		std::string o_msg = printCoutMsg(LEVEL_DETAIL_INFO, "First time loading the image"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	}
	else
	{
		for (auto mod_name : not_found_mod_name_vec)
		{
			//std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Pattern ", mod_name, " haven't been defined yet"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
	}

	return status;
}

void DepthFeatureGroup::displayAllContext()
{
	auto feature_group_ptr = &workspace_ptr_->feature_group().at(node_name_);
	// Clear all
	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	if (mil_mod_context_map_.size() == 0)
	{
		// No mod have been defined, should be the first time loading
		return;
	}

	displayContext("");
}

void DepthFeatureGroup::resetDisplay()
{
	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	MdispSelectWindow(mil_display_, reference_image_, MIL_WINDOW_HANDLE(mil_window_id_));
}

bool DepthFeatureGroup::defineSuccessed()
{
	return define_succesed_;
}

bool DepthFeatureGroup::duplicateFeature(Workspace* const workspace_ptr, const std::string feature_group_name, const std::string pattern_name)
{
	// TODO(yt): have to deal with deleting pattern window

	// check pattern name validation
	std::string feature_name_2d = workspace_ptr->feature_group().at(feature_group_name).depth_feature_group().feature_name_2d();
	std::string feature_name_depth = workspace_ptr->feature_group().at(feature_group_name).depth_feature_group().feature_name_depth();

	std::string feature_name = "";
	if (pattern_name.find(feature_name_2d) != std::string::npos)
		feature_name = feature_name_2d;
	else if (pattern_name.find(feature_name_depth) != std::string::npos)
		feature_name = feature_name_depth;
	else
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_error_name), pattern_name); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	// find first reference object
	std::string src_pattern_name = "";
	for (auto pattern_window : workspace_ptr->feature_group().at(feature_group_name).pattern_windows())
	{
		if (pattern_window.first.find(feature_name) != std::string::npos)
		{
			src_pattern_name = pattern_window.first;
			break;
		}
	}

	if (src_pattern_name == "")
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_error_name), pattern_name); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return false;
	}

	// create new workspace pattern window 
	(*workspace_ptr->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows())[pattern_name] = config::Workspace::FeatureGroup::PatternWindow();
	workspace_ptr->mutable_feature_group()->at(feature_group_name).mutable_pattern_windows()->at(pattern_name)
		.CopyFrom(workspace_ptr->feature_group().at(feature_group_name).pattern_windows().at(src_pattern_name));

	// duplicate model context if exist
	std::string mod_base_folder = workspace_ptr->workspace_folder() + "/feature_group/" + feature_group_name + "/";
	std::string mod_context_path[2];
	mod_context_path[0] = mod_base_folder + src_pattern_name + ".we";
	mod_context_path[1] = mod_base_folder + pattern_name + ".we";

	if (utils::fileExists(mod_context_path[0]))
	{
		int buffer_len = 0;
		LPWSTR wstr_mod_context_path[2];
		for (int i = 0; i < 2; i++)
		{
			int buffer_len = ::MultiByteToWideChar(CP_ACP, 0, mod_context_path[i].c_str(), (int)(mod_context_path[i].size()), NULL, 0);
			wstr_mod_context_path[i] = new WCHAR[buffer_len + 1];
			::MultiByteToWideChar(CP_ACP, 0, mod_context_path[i].c_str(), (int)(mod_context_path[i].size()), wstr_mod_context_path[i], buffer_len);
			wstr_mod_context_path[i][buffer_len] = 0;
		}

		CopyFile(wstr_mod_context_path[0], wstr_mod_context_path[1], /*true*/false);
		for (int i = 0; i < 2; i++)
			delete[] wstr_mod_context_path[i];
	}

	return true;
}

void DepthFeatureGroup::resetReferenceImage()
{
	MbufCopy(reference_image_original_, reference_image_);
}

void DepthFeatureGroup::checkModelSearchRegion(int model_region_x, int model_region_y, int model_region_size_x, int model_region_size_y)
{
	auto roi_x = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x();
	auto roi_y = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y();
	auto roi_x_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x_size();
	auto roi_y_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y_size();

	if (roi_x >= 0 && roi_y >= 0 && roi_x_size > 0 && roi_y_size > 0)
	{
		if (roi_x > model_region_x ||
			roi_y > model_region_y ||
			roi_x + roi_x_size < model_region_x + model_region_size_x ||
			roi_y + roi_y_size < model_region_y + model_region_size_y)
		{
			std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "[Warning] Part of mdoel is out of search region !"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
		}

		if (roi_x > 0 && roi_y > 0 && roi_x_size > 0 && roi_y_size > 0)
		{
			MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
			MgraRect(mil_graphic_context_, mil_graphic_context_list_,
				roi_x,
				roi_y,
				min(roi_x + roi_x_size, MbufInquire(reference_image_, M_SIZE_X, M_NULL)),
				min(roi_y + roi_y_size, MbufInquire(reference_image_, M_SIZE_Y, M_NULL)));

			// text "search region"
			MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
			MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
			std::string text = "[search region]";
			std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
			MgraText(mil_graphic_context_, mil_graphic_context_list_, roi_x + 1, roi_y + 1, wc_text.get());
		}
	}
}

void DepthFeatureGroup::drawDepthContext(MIL_ID src_image, MIL_ID dst_image, MIL_INT start_x, MIL_INT start_y, MIL_INT size_x, MIL_INT size_y)
{
	MIL_INT ref_image_size_x = MbufInquire(dst_image, M_SIZE_X, M_NULL);
	MIL_INT ref_image_size_y = MbufInquire(dst_image, M_SIZE_Y, M_NULL);

	/*if (start_x + size_x > ref_image_size_x ||
		start_y + size_y > ref_image_size_y)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_big_map)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}*/

	if (start_x < 0 || start_y < 0 || size_x <= 0 || size_y <= 0)
	{
		std::string msg = "Error parameters: [" + start_x + ', ' + start_y + ', ' + size_x + ', ' + size_y + ']';
		std::string o_msg = printCerrMsg(LEVEL_DETAIL_INFO, msg); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	MIL_INT src_image_size_x = MbufInquire(src_image, M_SIZE_X, M_NULL);
	MIL_INT src_image_size_y = MbufInquire(src_image, M_SIZE_Y, M_NULL);

	if (start_x > src_image_size_x - 1 || start_y > src_image_size_y - 1)
		return;

	MIL_INT _start_x = start_x;
	MIL_INT _start_y = start_y;
	MIL_INT _size_x = size_x;
	MIL_INT _size_y = size_y;

	if (_start_x + _size_x > src_image_size_x) _size_x = src_image_size_x - _start_x;
	if (_start_y + _size_y > src_image_size_y) _size_y = src_image_size_y - _start_y;

	MIL_ID roi_depth = MbufChild2d(src_image,
		_start_x,
		_start_y,
		_size_x,
		_size_y,
		M_NULL);

	MIL_INT size_x_2d = min(_size_x, ref_image_size_x / 4);
	MIL_INT size_y_2d = min(_size_y, ref_image_size_y / 4);
	MIL_ID roi_2d = MbufChild2d(dst_image, 0, 0,
		size_x_2d,
		size_y_2d, M_NULL);
	MbufClear(roi_2d, 0);
	MimResize(roi_depth, roi_2d, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BILINEAR);
	MimRemap(M_DEFAULT, roi_2d, roi_2d, M_FIT_SRC_DATA);
	MbufFree(roi_2d);
	MbufFree(roi_depth);

	MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_YELLOW);
	MgraRect(mil_graphic_context_, mil_graphic_context_list_,
		0,
		0,
		size_x_2d,
		size_y_2d);
}

DepthModelDefinedFeature::DepthModelDefinedFeature(
	MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	DepthFeatureGroup(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name),
	depth_disp_(M_NULL),
	depth_lut_(M_NULL),
	depth_gra_list_(M_NULL)
{
	SDepthData depth_data = DPCLib::getCameraDepthData(workspace_ptr->camera().at(camera_name_).depth_camera());
	MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, 16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, &depth_map_);

#if DEBUGING_MODE
	// display for depth map
	{
		MdispAlloc(mil_system, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_WINDOWED, &depth_disp_);
		MgraAllocList(mil_system, M_DEFAULT, &depth_gra_list_);
		MdispControl(depth_disp_, M_ASSOCIATED_GRAPHIC_LIST_ID, depth_gra_list_);
		MdispControl(depth_disp_, M_TITLE, M_PTR_TO_DOUBLE(MIL_TEXT("Depth")));
		MdispSelect(depth_disp_, depth_map_);

		MbufAllocColor(mil_system, 3, MIL_UINT16_MAX, 1, 8 + M_UNSIGNED, M_LUT, &depth_lut_);
		MbufClear(depth_lut_, M_COLOR_BLACK);
		MIL_ID MilDispLutChild = MbufChild1d(depth_lut_, 1, MIL_UINT16_MAX - 1, M_NULL);
		MgenLutFunction(MilDispLutChild, M_COLORMAP_JET, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		MbufFree(MilDispLutChild);
		MdispLut(depth_disp_, depth_lut_);
	}
#endif
}

DepthModelDefinedFeature::~DepthModelDefinedFeature()
{
	MbufFree(depth_map_);
	if (depth_gra_list_) MgraFree(depth_gra_list_);
	if (depth_disp_) MdispFree(depth_disp_);
	if (depth_lut_) MbufFree(depth_lut_);
}

void DepthModelDefinedFeature::createModel()
{
	if (reference_image_ == M_NULL && reference_range_image_ == M_NULL)
		return;

	bool allow_locate_method_2d = (reference_image_ != M_NULL);
	bool allow_locate_method_depth = true;

	define_succesed_ = false;
	MgraClear(M_DEFAULT, mil_graphic_context_list_);
	MgraClear(M_DEFAULT, depth_gra_list_);

	std::string feature_name_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d() + std::to_string(DONIMAL_FEATURE_INDEX);
	std::string feature_name_depth = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_depth() + std::to_string(DONIMAL_FEATURE_INDEX);

	// 1. find model in intensity image 
	MIL_DOUBLE pos_x;
	MIL_DOUBLE pos_y;
	MIL_DOUBLE size_x;
	MIL_DOUBLE size_y;
	bool has_feature = false;

	config::Workspace_FeatureGroup::PatternWindow src_pattern;
	for (auto const& pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (pattern_window.first == feature_name_2d)
		{
			has_feature = true;

			pos_x = pattern_window.second.location_x();
			pos_y = pattern_window.second.location_y();
			size_x = pattern_window.second.size_x();
			size_y = pattern_window.second.size_y();
			src_pattern = pattern_window.second;

			break;
		}
	}

	if (!has_feature)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_wrong_2d)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// 2. determine the width & height of the model
	/*MIL_ID masked_ref_image = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);*/
	MIL_ID roi_2d = MbufChild2d(reference_image_, (MIL_INT)pos_x, (MIL_INT)pos_y, (MIL_INT)size_x, (MIL_INT)size_y, M_NULL);

	// create images
	MIL_DOUBLE margin = 2.0;
	MIL_DOUBLE margined_size_x = min(pos_x + size_x + margin, MbufInquire(reference_image_, M_SIZE_X, M_NULL));
	MIL_DOUBLE margined_size_y = min(pos_y + size_y + margin, MbufInquire(reference_image_, M_SIZE_Y, M_NULL));

	if (allow_locate_method_2d)
	{
		allow_locate_method_2d = define2DModelContext(feature_name_2d,
			roi_2d,
			size_x,
			size_y,
			margined_size_x,
			margined_size_y,
			src_pattern,
			workspace_ptr_->feature_group().at(node_name_).depth_feature_group().m_number_2d());
	}
	MbufFree(roi_2d);

	// @ check search region
	checkModelSearchRegion(
		(int)pos_x,
		(int)pos_y,
		(int)size_x,
		(int)size_y);
	displaySearchRange();

	// 3. create model depth map
	has_feature = false;

	for (auto const& pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (pattern_window.first == feature_name_depth)
		{
			has_feature = true;
			src_pattern = pattern_window.second;
			break;
		}
	}

	if (!has_feature)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_wrong_depth)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	double max_depth = 0.0;
	SPoseDataQuat ground_rotation = SPoseDataQuat();
	DPCLib::getGroundEraseParams(workspace_ptr_, node_name_, mil_calibration_, pose_base_to_tool_, max_depth, ground_rotation);

	std::shared_ptr<DepthCameraDataWrapper> depth_camera_data_wrapper =
		std::move(DPCLib::createDepthMapGeneratorPtr(
			mil_system_,
			workspace_ptr_->feature_group().at(node_name_).depth_feature_group().depth_camera(),
			mil_calibration_));

	MIL_ID image_mask = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(image_mask, 0);
	/*MgraColor(M_DEFAULT, M_RGB888(1, 1, 1));*/

	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -pos_x);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -pos_y);
	MgraColor(mil_graphic_context_, 1);
	MmodDraw(mil_graphic_context_, mil_mod_result_map_[feature_name_2d], mil_graphic_context_list_, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
	MgraDraw(mil_graphic_context_list_, image_mask, M_DEFAULT);
	MgraFill(M_DEFAULT, image_mask, (MIL_INT)(pos_x + size_x / 2), (MIL_INT)(pos_y + size_y / 2));
	MimClip(image_mask, image_mask, M_NOT_EQUAL, 0, M_NULL, 1, M_NULL);

	// draw 2d result
	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -pos_x);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -pos_y);
	MgraColor(mil_graphic_context_, M_COLOR_GREEN);
	MmodDraw(mil_graphic_context_, mil_mod_result_map_[feature_name_2d], mil_graphic_context_list_, M_DRAW_POSITION + M_DRAW_EDGES + M_DRAW_BOX, M_DEFAULT, M_DEFAULT);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
	
	if (!depth_camera_data_wrapper->createDepthMap(
		reference_range_image_,
		image_mask/*M_NULL*/,
		depth_map_,
		0,
		max_depth,
		0,
		false,
		false,
		/*false*/true,
		ground_rotation))
	{
		depth_camera_data_wrapper.reset();
		MbufFree(image_mask);
		return;
	}
	MbufFree(image_mask);

	// 4. define model in depth image
	if (!depth_camera_data_wrapper->cropPointCloudToFixBox(/*0*/1))
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_no_point)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	MIL_DOUBLE min_x, min_y, min_z;
	MIL_DOUBLE max_x, max_y, max_z;
	MIL_ID model_point_cloud = depth_camera_data_wrapper->getPointCloudID();
	DPCLib::getPointCloudBoudingBox(model_point_cloud, min_x, min_y, min_z, max_x, max_y, max_z);
	SDepthData depth_data = depth_camera_data_wrapper->getDepthData();
	depth_camera_data_wrapper.reset();

	// Convert x-y bounding corners from world to pixel units in the depth-map.
	MIL_DOUBLE roi_min_x, roi_min_y;
	MIL_DOUBLE roi_max_x, roi_max_y;
	McalTransformCoordinate(depth_map_, M_WORLD_TO_PIXEL, min_x, min_y, &roi_min_x, &roi_min_y);
	McalTransformCoordinate(depth_map_, M_WORLD_TO_PIXEL, max_x, max_y, &roi_max_x, &roi_max_y);

	MIL_ID depth_map_uint8 = MbufAlloc2d(mil_system_,
		depth_data.depth_map_size_x_,
		depth_data.depth_map_size_y_,
		8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
	DPCLib::mapDepthImageTo8Bits(mil_system_, depth_map_, depth_map_uint8);

	MIL_DOUBLE depth_model_roi_x = abs(max_x - min_x);
	MIL_DOUBLE depth_model_roi_y = abs(max_y - min_y);
	allow_locate_method_depth = defineDepthModelContext(
		feature_name_depth,
		depth_map_uint8,
		depth_model_roi_x,
		depth_model_roi_y,
		src_pattern,
		workspace_ptr_->feature_group().at(node_name_).depth_feature_group().m_number_depth());

	if (!allow_locate_method_depth)
	{
		drawDepthContext(
			depth_map_uint8,
			reference_image_,
			(MIL_INT)roi_min_x,
			(MIL_INT)roi_min_y,
			(MIL_INT)depth_model_roi_x,
			(MIL_INT)depth_model_roi_y);
	}

	// set locate method flag 
	int locate_method = 0;
	if (!allow_locate_method_2d && !allow_locate_method_depth)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE,
			"2D & 3D model not found,"
			"please adjust model/robot pose!"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}
	else if (!allow_locate_method_2d && allow_locate_method_depth)
	{
		locate_method = LocateMethod::LM_DEPTH;
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE,
			"2D model not found,"
			"please adjust model/robot pose!"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	}
	else if (allow_locate_method_2d && !allow_locate_method_depth)
	{
		locate_method = LocateMethod::LM_2D;
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE,
			"3D model not found,"
			"please adjust model/robot pose!"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	}
	else
	{
		locate_method = LocateMethod::LM_2D | LocateMethod::LM_DEPTH | LocateMethod::LM_2D_DEPTH;
	}

	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_locate_method_flag(locate_method);
	if (locate_method == 0)
	{
		MbufFree(depth_map_uint8);
		return;
	}

	//5. calculate the model pose
	std::shared_ptr<DepthLocalization> localization_ptr = std::move(createModelLocalizationPtr());
	bool can_stop = false;
	bool has_result = false;

	localization_ptr->resetlocateState();
	localization_ptr->pushFrame(reference_image_,
		reference_range_image_,
		pose_base_to_tool_,
		can_stop,
		has_result);

	if (!has_result)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;

		// draw depth on the left corner
		if (allow_locate_method_2d)
		{
			drawDepthContext(
				depth_map_uint8,
				reference_image_,
				(MIL_INT)roi_min_x,
				(MIL_INT)roi_min_y,
				(MIL_INT)depth_model_roi_x,
				(MIL_INT)depth_model_roi_y);
		}

		MosGetch();
		localization_ptr.reset();
		MbufFree(depth_map_uint8);
		return;
	}

	SPoseDataQuat pose;
	localization_ptr->getModelPose(ModelPoseType::MODEL_TO_WORLD, pose);

	/*std::string o_msg = printCoutMsg(LEVEL_BASIC_INFO, "T: [",
		pose.x, ", ", pose.y, ", ", pose.z, "]"
		" R: [",
		pose.q1, ", ", pose.q2, ", ", pose.q3, ", ", pose.q4, "]"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;*/

	config::Workspace::QuaternionPose data;
	data.set_x(pose.x);
	data.set_y(pose.y);
	data.set_z(pose.z);
	data.set_q1(pose.q1);
	data.set_q2(pose.q2);
	data.set_q3(pose.q3);
	data.set_q4(pose.q4);

	std::string training_name = workspace_ptr_->feature_group().at(node_name_).training_name();
	const auto training = workspace_ptr_->training().at(training_name);
	std::string pose_calculator_name = training.pose_calculator_name();
	*workspace_ptr_->mutable_pose_calculator()->at(pose_calculator_name).mutable_model_reference_pose() = data;

	if (reference_depth_image_) MbufFree(reference_depth_image_);
	MbufClone(depth_map_uint8, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, &reference_depth_image_);
	MbufFree(depth_map_uint8);

	// 6. free memory
	MosGetch();
	localization_ptr.reset();
	std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Model is successfully defined !"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;

	define_succesed_ = true;
}

bool DepthModelDefinedFeature::isValidPattern(
	const google::protobuf::MapPair<std::string, config::Workspace_FeatureGroup_PatternWindow> pattern)
{
	std::string feature_name_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d();
	if (pattern.first.find(feature_name_2d) != std::string::npos)
	{
		if (pattern.second.location_x() <= 0 ||
			pattern.second.location_y() <= 0 ||
			pattern.second.size_x() <= 0 ||
			pattern.second.size_x() >= 1024 ||
			pattern.second.size_y() <= 0 ||
			pattern.second.size_y() >= 1024)
		{
			return false;
		}
	}

	return true;
}

DepthCircleFeature::DepthCircleFeature(MIL_ID mil_application,
	MIL_ID mil_system,
	MIL_ID mil_display,
	MIL_ID mil_window_id,
	Workspace* const workspace_ptr,
	const std::string& node_name) :
	DepthModelDefinedFeature(mil_application,
		mil_system,
		mil_display,
		mil_window_id,
		workspace_ptr,
		node_name)
{
}

bool DepthCircleFeature::define2DModelContext(
	std::string mod_name,
	MIL_ID mod_find_image,
	MIL_DOUBLE model_size_x,
	MIL_DOUBLE model_size_y,
	MIL_DOUBLE max_model_size_x,
	MIL_DOUBLE max_model_size_y,
	config::Workspace_FeatureGroup::PatternWindow src_pattern,
	int mod_number)
{
	MIL_INT nb_model_found;
	MIL_DOUBLE radius = max(model_size_x, model_size_y) / 2;

	MIL_ID mod_context_2d = MmodAlloc(mil_system_, M_SHAPE_CIRCLE, M_DEFAULT, M_NULL);
	MmodDefine(mod_context_2d, M_CIRCLE, M_DEFAULT, radius, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	ModelProcess::setContextFromPattern(mod_context_2d, src_pattern, 1);
	MmodControl(mod_context_2d, M_DEFAULT, M_SCALE_MIN_FACTOR, 0.4);
	MmodControl(mod_context_2d, M_DEFAULT, M_SCALE_MAX_FACTOR, 1.0);
	MmodPreprocess(mod_context_2d, M_DEFAULT);

	MmodAllocResult(mil_system_, M_SHAPE_CIRCLE, &mil_mod_result_map_[mod_name]);
	MmodFind(mod_context_2d, mod_find_image, mil_mod_result_map_[mod_name]);
	MmodGetResult(mil_mod_result_map_[mod_name], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);
	MmodFree(mod_context_2d);

	// filter width & height
	MIL_INT index = -1;
	radius = 0.0;
	if (nb_model_found > 0)
	{
		for (MIL_INT i = 0; i < nb_model_found; i++)
		{
			MIL_DOUBLE r;
			MmodGetResult(mil_mod_result_map_[mod_name], i, M_RADIUS, &r);

			if (r > radius)
			{
				radius = r;
				index = i;
			}
		}
	}

	if (index == -1)
	{
		return false;
	}
	else
	{
		// redifine model
		MmodAlloc(mil_system_, M_SHAPE_CIRCLE, M_DEFAULT, &mil_mod_context_map_[mod_name]);
		MmodDefine(mil_mod_context_map_[mod_name], M_CIRCLE, M_DEFAULT, radius, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		ModelProcess::setContextFromPattern(mil_mod_context_map_[mod_name], src_pattern, mod_number);

		MgraColor(M_DEFAULT, M_COLOR_YELLOW);
		MmodDraw(M_DEFAULT, mil_mod_result_map_[mod_name], mil_graphic_context_list_, M_DRAW_POSITION + M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
		MgraColor(M_DEFAULT, M_COLOR_GREEN);
		MmodDraw(M_DEFAULT, mil_mod_result_map_[mod_name], mil_graphic_context_list_, M_DRAW_POSITION + M_DRAW_EDGES + M_DRAW_BOX, /*M_DEFAULT*/index, M_DEFAULT);

		return true;
	}
}

bool DepthCircleFeature::defineDepthModelContext(
	std::string mod_name,
	MIL_ID mod_find_image,
	MIL_DOUBLE model_size_x,
	MIL_DOUBLE model_size_y,
	config::Workspace_FeatureGroup::PatternWindow src_pattern,
	int mod_number)
{
	MIL_INT nb_model_found;
	MIL_DOUBLE radius = max(model_size_x, model_size_y) / 2.0;
	
	// define model finder
	MIL_ID mod_context_depth = MmodAlloc(mil_system_, M_SHAPE_CIRCLE, M_DEFAULT, M_NULL);
	MmodDefine(mod_context_depth, M_CIRCLE, M_DEFAULT, radius, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	ModelProcess::setContextFromPattern(mod_context_depth, src_pattern, 1);
	MmodControl(mod_context_depth, M_DEFAULT, M_SCALE_MIN_FACTOR, 0.4);
	MmodControl(mod_context_depth, M_DEFAULT, M_SCALE_MAX_FACTOR, 1.0);
	MmodPreprocess(mod_context_depth, M_DEFAULT);

	MmodAllocResult(mil_system_, M_SHAPE_CIRCLE, &mil_mod_result_map_[mod_name]);
	MmodFind(mod_context_depth, mod_find_image, mil_mod_result_map_[mod_name]);
	MmodGetResult(mil_mod_result_map_[mod_name], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);
	MmodFree(mod_context_depth);

	// filter width & height
	MIL_INT index = -1;
	radius = 0.0;
	if (nb_model_found > 0)
	{
		for (MIL_INT i = 0; i < nb_model_found; i++)
		{
			MIL_DOUBLE r;
			MmodGetResult(mil_mod_result_map_[mod_name], i, M_RADIUS, &r);

			if (r > radius)
			{
				radius = r;
				index = i;
			}
		}
	}

	if (index == -1)
	{
		return false;
	}
	else
	{
		MmodAlloc(mil_system_, M_SHAPE_CIRCLE, M_DEFAULT, &mil_mod_context_map_[mod_name]);
		MmodDefine(mil_mod_context_map_[mod_name], M_CIRCLE, M_DEFAULT, radius, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		ModelProcess::setContextFromPattern(mil_mod_context_map_[mod_name], src_pattern, mod_number);

		MgraColor(M_DEFAULT, M_COLOR_YELLOW);
		MmodDraw(M_DEFAULT, mil_mod_result_map_[mod_name], depth_gra_list_, M_DRAW_POSITION + M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
		MgraColor(M_DEFAULT, M_COLOR_GREEN);
		MmodDraw(M_DEFAULT, mil_mod_result_map_[mod_name], depth_gra_list_, M_DRAW_POSITION + M_DRAW_EDGES, /*M_DEFAULT*/index, M_DEFAULT);

		return true;
	}
}

std::shared_ptr<DepthLocalization> DepthCircleFeature::createModelLocalizationPtr()
{
	std::shared_ptr<DepthLocalization> ptr = std::make_shared<DepthCircleLocalization>(
			mil_application_,
			mil_system_,
			mil_display_,
			mil_window_id_,
			workspace_ptr_,
			node_name_,
			mil_mod_context_map_);

	return ptr;
}

void DepthCircleFeature::displayContext(std::string pattern_name)
{
	/* Disable the display update for better performance. */
	MdispControl(mil_display_, M_UPDATE, M_DISABLE);

	std::string feature_name_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d() + std::to_string(DONIMAL_FEATURE_INDEX);
	std::string feature_name_depth = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_depth() + std::to_string(DONIMAL_FEATURE_INDEX);

	for (auto it : mil_mod_context_map_/*mil_mod_result_map_*/)
	{
		if (it.first == feature_name_2d)
		{
			MIL_DOUBLE pos_x = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).location_x();
			MIL_DOUBLE pos_y = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).location_y();
			MIL_DOUBLE size_x = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).size_x();
			MIL_DOUBLE size_y = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).size_y();

			/*MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
			MgraRect(mil_graphic_context_, mil_graphic_context_list_,
				pos_x,
				pos_y,
				pos_x + size_x,
				pos_y + size_y);*/

			MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_RED);
			MgraRectAngle(
				mil_graphic_context_,
				mil_graphic_context_list_,
				pos_x + size_x / 2.0,
				pos_y + size_y / 2.0,
				MmodInquire(it.second, M_DEFAULT, /*M_RADIUS*/M_BOX_SIZE_X, M_NULL),
				MmodInquire(it.second, M_DEFAULT, /*M_RADIUS*/M_BOX_SIZE_Y, M_NULL),
				0,
				M_CENTER_AND_DIMENSION);

			MgraControl(mil_graphic_context_, M_BACKGROUND_MODE, M_TRANSPARENT);
			MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
			std::string text = "[" + it.first + "]";
			std::shared_ptr<wchar_t> wc_text_1 = utils::string2Wchar_tPtr(text);
			MgraText(mil_graphic_context_, mil_graphic_context_list_, pos_x, pos_y + size_y + 1, wc_text_1.get());

			break;
		}
	}

	// draw model depth on the left-top of the image
	double depth_pos_x;
	double depth_pos_y;
	double depth_size_x;
	double depth_size_y;
	double depth_angle;
	bool has_feature = false;
	for (auto const& pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (pattern_window.first == feature_name_depth)
		{
			has_feature = true;
			depth_pos_x = pattern_window.second.location_x();
			depth_pos_y = pattern_window.second.location_y();
			depth_size_x = pattern_window.second.size_x();
			depth_size_y = pattern_window.second.size_y();
			depth_angle = pattern_window.second.depth_feature().model_angle();

			for (auto it : mil_mod_context_map_)
			{
				if (it.first == feature_name_depth)
				{
					MIL_ID temp_depth_image = MbufClone(reference_depth_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);

					drawDepthContext(
						/*reference_depth_image_*/temp_depth_image,
						reference_image_,
						(MIL_INT)depth_pos_x,
						(MIL_INT)depth_pos_y,
						(MIL_INT)depth_size_x,
						(MIL_INT)depth_size_y);
					MbufFree(temp_depth_image);

					/*MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_RED);
					MgraRectAngle(
						mil_graphic_context_,
						mil_graphic_context_list_,
						depth_size_x / 2.0,
						depth_size_y / 2.0,
						MmodInquire(it.second, M_DEFAULT, M_WIDTH, M_NULL),
						MmodInquire(it.second, M_DEFAULT, M_HEIGHT, M_NULL),
						depth_angle,
						M_CENTER_AND_DIMENSION);*/

						// text "depth"
					MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
					MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
					std::string text = "[" + feature_name_depth + "]";
					std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
					MgraText(mil_graphic_context_, mil_graphic_context_list_, 0, (MIL_INT)depth_size_y + 1, wc_text.get());

					break;
				}
			}

			break;
		}
	}

	// draw valid region
	{
		auto roi_x = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x();
		auto roi_y = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y();
		auto roi_x_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x_size();
		auto roi_y_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y_size();

		if (roi_x > 0 && roi_y > 0 && roi_x_size > 0 && roi_y_size > 0)
		{
			MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
			MgraRect(mil_graphic_context_, mil_graphic_context_list_,
				roi_x,
				roi_y,
				min(roi_x + roi_x_size, MbufInquire(reference_image_, M_SIZE_X, M_NULL)),
				min(roi_y + roi_y_size, MbufInquire(reference_image_, M_SIZE_Y, M_NULL)));

			// text "search region"
			MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
			MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
			std::string text = "[search region]";
			std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
			MgraText(mil_graphic_context_, mil_graphic_context_list_, roi_x, roi_y, wc_text.get());
		}
	}

	MdispControl(mil_display_, M_UPDATE, M_ENABLE);
}