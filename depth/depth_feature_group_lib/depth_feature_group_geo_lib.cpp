#include "depth_feature_group_geo_lib.h"
#include "utils.h"
#include "depth_localization_geo_lib.h"
#include "depth_camera.h"

DepthGeometryFeature::DepthGeometryFeature(MIL_ID mil_application,
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

DepthGeometryFeature::~DepthGeometryFeature()
{
}

bool DepthGeometryFeature::editMask(const std::string pattern_name, const MIL_INT mil_window_id)
{
	bool edit_result = false;

	if (pattern_name.find(workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d()) != std::string::npos)
	{
		edit_result = FeatureGroup::editMask(pattern_name, mil_window_id);
	}
	else if (pattern_name.find(workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_depth()) != std::string::npos)
	{
		// TODO(yt): temp fix for depth model context
		MIL_ID reference_image_temp = reference_image_;
		reference_image_ = reference_depth_image_;
		edit_result = FeatureGroup::editMask(pattern_name, mil_window_id);
		reference_image_ = reference_image_temp;
	}

	return edit_result;
}

void DepthGeometryFeature::createModel()
{
	// TODO(yt): modify when figure out how to use LocateMethod::LM_2D
	define_succesed_ = false;

	if (reference_image_ == M_NULL && reference_range_image_ == M_NULL)
		return;

	bool allow_locate_method_2d = (reference_image_ != M_NULL);

	MIL_DOUBLE pos_x;
	MIL_DOUBLE pos_y;
	MIL_DOUBLE size_x;
	MIL_DOUBLE size_y;
	bool has_feature = false;

	// 1. define model in intensity image 
	std::string feature_name_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d();
	std::string first_feature_name = "";
	auto m_number_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().m_number_2d();
	for (auto const& pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (pattern_window.first.find(feature_name_2d) != std::string::npos)
		{
			if (!has_feature)
			{
				has_feature = true;
				first_feature_name = pattern_window.first;
				pos_x = pattern_window.second.location_x();
				pos_y = pattern_window.second.location_y();
				size_x = pattern_window.second.size_x();
				size_y = pattern_window.second.size_y();
			}

			if (allow_locate_method_2d)
			{
				MmodAlloc(mil_system_, M_GEOMETRIC, M_DEFAULT, &mil_mod_context_map_[pattern_window.first]);
				MmodDefine(mil_mod_context_map_[pattern_window.first], M_IMAGE, reference_image_, pos_x, pos_y, size_x, size_y);
				ModelProcess::setContextFromPattern(mil_mod_context_map_[pattern_window.first], pattern_window.second, m_number_2d);
			}
		}
	}

	if (!has_feature)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_wrong_2d)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// @ check search region
	checkModelSearchRegion(
		(int)pos_x,
		(int)pos_y,
		(int)size_x,
		(int)size_y);
	displaySearchRange();

	// 2. create model mask
	MIL_ID image_mask = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(image_mask, 0);
	MgraColor(M_DEFAULT, 1);
	MmodDraw(M_DEFAULT, mil_mod_context_map_[first_feature_name], image_mask, M_DRAW_BOX, M_DEFAULT, M_ORIGINAL);
	MgraFill(M_DEFAULT, image_mask, (MIL_INT)(pos_x + size_x / 2), (MIL_INT)(pos_y + size_y / 2));

	double max_depth = 0.0;
	SPoseDataQuat ground_rotation = SPoseDataQuat();
	DPCLib::getGroundEraseParams(workspace_ptr_, node_name_, mil_calibration_, pose_base_to_tool_, max_depth, ground_rotation);

	// @ create 2d model mask
	MIL_ID image_dst_2d = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MIL_ID range_cond = MbufClone(reference_range_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(image_dst_2d, 0);
	MbufClear(range_cond, 0);
	MimClip(reference_range_image_, range_cond, M_GREATER, max_depth, M_NULL, 65535, M_NULL);

	MIL_ID roi_src_2d = MbufChild2d(reference_image_, (MIL_INT)pos_x, (MIL_INT)pos_y, (MIL_INT)size_x, (MIL_INT)size_y, M_NULL);
	MIL_ID roi_dst_2d = MbufChild2d(image_dst_2d, (MIL_INT)pos_x, (MIL_INT)pos_y, (MIL_INT)size_x, (MIL_INT)size_y, M_NULL);
	MIL_ID roi_cond = MbufChild2d(range_cond, (MIL_INT)pos_x, (MIL_INT)pos_y, (MIL_INT)size_x, (MIL_INT)size_y, M_NULL);
	MbufCopyCond(roi_src_2d, roi_dst_2d, roi_cond, M_NOT_EQUAL, 65535);
	MimClip(roi_dst_2d, roi_dst_2d, M_GREATER, 0, M_NULL, 255, M_NULL);

	std::string model_mask_2d_path = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_ + "/" + MASK_IMAGE_FILE_NAME_2D;
	std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(model_mask_2d_path);
	MbufExport(wc_image_path.get(), M_MIL, image_dst_2d);

	MbufFree(roi_cond);
	MbufFree(roi_src_2d);
	MbufFree(roi_dst_2d);

	MbufFree(range_cond);
	MbufFree(image_dst_2d);

	// 3. create model depth map
	std::shared_ptr<DepthCameraDataWrapper> depth_camera_data_wrapper =
		std::move(DPCLib::createDepthMapGeneratorPtr(
			mil_system_,
			workspace_ptr_->feature_group().at(node_name_).depth_feature_group().depth_camera(),
			mil_calibration_));

	if (!depth_camera_data_wrapper->createDepthMap(
		reference_range_image_,
		image_mask/*M_NULL*/,
		depth_map_,
		0,
		max_depth,
		0,
		false,
		false,
		false,
		ground_rotation))
	{
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

	min_x -= POINT_CLOUD_BOUNDING_OFFSET;
	min_y -= POINT_CLOUD_BOUNDING_OFFSET;
	min_z -= POINT_CLOUD_BOUNDING_OFFSET;
	max_x += POINT_CLOUD_BOUNDING_OFFSET;
	max_y += POINT_CLOUD_BOUNDING_OFFSET;
	max_z += POINT_CLOUD_BOUNDING_OFFSET;

	// Convert x-y bounding corners from world to pixel units in the depth-map.
	MIL_DOUBLE roi_min_x, roi_min_y;
	MIL_DOUBLE roi_max_x, roi_max_y;
	McalTransformCoordinate(depth_map_, M_WORLD_TO_PIXEL, min_x, min_y, &roi_min_x, &roi_min_y);
	McalTransformCoordinate(depth_map_, M_WORLD_TO_PIXEL, max_x, max_y, &roi_max_x, &roi_max_y);

	MIL_INT model_depth_roi_size_x = static_cast<MIL_INT>(roi_max_x - roi_min_x);
	MIL_INT model_depth_roi_size_y = static_cast<MIL_INT>(roi_max_y - roi_min_y);

	/*MimShift(depth_map_, depth_map_uint8_, -8);*/
	SDepthData depth_data = depth_camera_data_wrapper->getDepthData();
	MIL_ID depth_map_uint8 = MbufAlloc2d(mil_system_,
		depth_data.depth_map_size_x_,
		depth_data.depth_map_size_y_,
		8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
	DPCLib::mapDepthImageTo8Bits(mil_system_, depth_map_, depth_map_uint8);

	// for loop all depth model
	std::string feature_name_depth = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_depth();
	auto m_number_depth = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().m_number_depth();
	has_feature = false;
	for (auto const& pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (pattern_window.first.find(feature_name_depth) != std::string::npos)
		{
			if (!has_feature)
			{
				has_feature = true;
				first_feature_name = pattern_window.first;
			}

			MmodAlloc(mil_system_, M_GEOMETRIC, M_DEFAULT, &mil_mod_context_map_[pattern_window.first]);
			MIL_ID mod_context = mil_mod_context_map_[pattern_window.first];

			MmodDefine(mod_context, M_IMAGE, depth_map_uint8,
				roi_min_x,
				roi_min_y,
				(MIL_DOUBLE)model_depth_roi_size_x,
				(MIL_DOUBLE)model_depth_roi_size_y);

			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[pattern_window.first].set_location_x(double(roi_min_x));
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[pattern_window.first].set_location_y(double(roi_min_y));
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[pattern_window.first].set_size_x(double(model_depth_roi_size_x));
			(*workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_pattern_windows())[pattern_window.first].set_size_y(double(model_depth_roi_size_y));

			ModelProcess::setContextFromPattern(mod_context, pattern_window.second, m_number_depth);
		}
	}

	if (!has_feature)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_wrong_depth)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// save model mask for use in 3d alignment
	MIL_ID depth_map_buf =
		MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, 16 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
	MbufCopy(depth_map_, depth_map_buf);
	MIL_ID model_roi_buf = MbufAlloc2d(mil_system_,
		depth_data.depth_map_size_x_,
		depth_data.depth_map_size_y_,
		M_UNSIGNED + 1, M_IMAGE + M_PROC, M_NULL);

	MIL_INT start_x = static_cast<MIL_INT>(roi_min_x);
	MIL_INT start_y = static_cast<MIL_INT>(roi_min_y);

	MgraColor(M_DEFAULT, M_RGB888(1, 1, 1));
	MgraRectFill(M_DEFAULT, model_roi_buf,
		start_x,
		start_y,
		start_x + model_depth_roi_size_x,
		start_y + model_depth_roi_size_y);

	MimArith(model_roi_buf, depth_map_, depth_map_buf, M_MULT);
	MbufFree(model_roi_buf);

	MIL_ID model_mask = MbufAlloc2d(mil_system_,
		depth_data.depth_map_size_x_,
		depth_data.depth_map_size_y_,
		M_UNSIGNED + 1, M_PROC + M_IMAGE, M_NULL);
	MbufClear(model_mask, 0.0);
	MbufCopyCond(depth_map_buf, model_mask, depth_map_buf, M_NOT_EQUAL, 65535);
	MbufFree(depth_map_buf);

	MIL_DOUBLE model_mean_depth;
	M3dmapStat(depth_map_, M_NULL, model_mask, M_NULL, M_DEVIATION_MEAN + M_STAT_ALL, M_DEFAULT, M_DEFAULT, &model_mean_depth);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_model_mean_depth(model_mean_depth);

	// save model mask
	std::string mil_model_mask_path = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_ + "/" + MASK_IMAGE_FILE_NAME_DEPTH;
	std::shared_ptr<wchar_t> wc_mil_model_mask_path_ptr = utils::string2Wchar_tPtr(mil_model_mask_path);
	MbufExport(wc_mil_model_mask_path_ptr.get(), M_MIL + M_WITH_CALIBRATION, model_mask);
	MbufFree(model_mask);

	MgraClear(M_DEFAULT, depth_gra_list_);
	MgraColor(M_DEFAULT, M_COLOR_GREEN);
	MmodDraw(M_DEFAULT, mil_mod_context_map_[first_feature_name], depth_gra_list_, M_DRAW_POSITION + M_DRAW_EDGES + M_DRAW_BOX, M_DEFAULT, M_ORIGINAL);

	// 5. save model point cloud
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
	if (!utils::createFolder(base_folder)) // 
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_wrong_folder), base_folder); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	std::string point_cloud_name = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().model_name_point_cloud();
	std::string point_cloud_path = base_folder + "/" + point_cloud_name + ".we";

	MIL_ID point_cloud = M3dmapAllocResult(mil_system_, M_POINT_CLOUD_CONTAINER, M_DEFAULT, M_NULL);
	M3dmapClear(point_cloud, M_ALL, M_DELETE, M_DEFAULT);

	depth_camera_data_wrapper->resetPCBoundingBox();
	M3dmapCopy(depth_camera_data_wrapper->getPointCloudID(),
		PC_DEFAULT_LABEL,
		point_cloud,
		PC_DEFAULT_LABEL,
		M_COPY + M_EXCLUDE_INVALID_POINTS,
		M_DEFAULT);

	std::shared_ptr<wchar_t> wc_context_path = utils::string2Wchar_tPtr(point_cloud_path);
	M3dmapSave(wc_context_path.get(), model_point_cloud, M_DEFAULT);
	M3dmapFree(point_cloud);

	//6. calculate the model pose
	std::unique_ptr<DepthGeometryLocalization> localization_ptr =
		std::make_unique<DepthGeometryLocalization>(
			mil_application_,
			mil_system_,
			mil_display_,
			mil_window_id_,
			workspace_ptr_,
			node_name_,
			mil_mod_context_map_);

	bool can_stop = false;
	bool has_result = false;
	localization_ptr->resetlocateState();

	MIL_ID masked_ref_image = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MIL_ID mask = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(mask, 0);
	MgraColor(M_DEFAULT, 255);
	MgraRectFill(M_DEFAULT, mask, pos_x, pos_y, pos_x + size_x, pos_y + size_y);
	MbufCopyCond(reference_image_, masked_ref_image, mask, M_NOT_EQUAL, 0);
	MbufFree(mask);

	// TODO(yt): currently, geo dose not support LM_2D method, fix later
	int locate_method = 0;
	if (allow_locate_method_2d)
		locate_method = LocateMethod::LM_2D | LocateMethod::LM_DEPTH | LocateMethod::LM_2D_DEPTH;
	else
		locate_method = LocateMethod::LM_DEPTH;
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_locate_method_flag(locate_method);

	localization_ptr->pushFrame(masked_ref_image,
		reference_range_image_,
		pose_base_to_tool_,
		can_stop,
		has_result);
	MbufFree(masked_ref_image);

	if (!has_result)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;

		// draw depth on the left corner
		if (model_depth_roi_size_x > MbufInquire(reference_image_, M_SIZE_X, M_NULL) ||
			model_depth_roi_size_y > MbufInquire(reference_image_, M_SIZE_Y, M_NULL))
		{
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_big_map)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		else
		{
			MIL_ID roi_depth = MbufChild2d(depth_map_uint8,
				(MIL_INT)roi_min_x,
				(MIL_INT)roi_min_y,
				model_depth_roi_size_x,
				model_depth_roi_size_y,
				M_NULL);

			MIL_ID roi_2d = MbufChild2d(reference_image_, 0, 0, model_depth_roi_size_x, model_depth_roi_size_y, M_NULL);
			MbufClear(roi_2d, 0);
			MbufCopy(roi_depth, roi_2d);
			MimRemap(M_DEFAULT, roi_2d, roi_2d, M_FIT_SRC_DATA);
			MbufFree(roi_2d);
			MbufFree(roi_depth);
		}

		MbufFree(depth_map_uint8);
		localization_ptr.reset();
		return;
	}

	{
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
	}

	if (reference_depth_image_)
	{
		MbufFree(reference_depth_image_);
	}
	MbufClone(depth_map_uint8, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, &reference_depth_image_);
	MbufFree(depth_map_uint8);

	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_locate_method_flag(locate_method);

	MosGetch();
	localization_ptr.reset();
	std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_model_def)); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	define_succesed_ = true;
}

void DepthGeometryFeature::displayContext(std::string pattern_name)
{
	bool show_first = false;
	if (pattern_name == "")
		show_first = true;

	/* Disable the display update for better performance. */
	MdispControl(mil_display_, M_UPDATE, M_DISABLE);

	std::string feature_name_2d = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_2d();
	for (auto it : mil_mod_context_map_)
	{
		if (it.first.find(feature_name_2d) == std::string::npos)
			continue;

		if (!show_first)
		{
			if (it.first != pattern_name)
				continue;
		}

		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
		MmodDraw(mil_graphic_context_, it.second, mil_graphic_context_list_, M_DRAW_BOX, 0, M_ORIGINAL);
		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_MAGENTA);
		MmodDraw(mil_graphic_context_, it.second, mil_graphic_context_list_, M_DRAW_POSITION, 0, M_ORIGINAL);
		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_RED);
		MmodDraw(mil_graphic_context_, it.second, mil_graphic_context_list_, M_DRAW_EDGES, 0, M_ORIGINAL);

		MIL_DOUBLE pos_x = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).location_x();
		MIL_DOUBLE pos_y = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).location_y();
		MIL_DOUBLE size_x = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).size_x();
		MIL_DOUBLE size_y = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).size_y();

		MgraControl(mil_graphic_context_, M_BACKGROUND_MODE, M_TRANSPARENT);

		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		std::string text = "[" + it.first + "]";
		std::shared_ptr<wchar_t> wc_text_1 = utils::string2Wchar_tPtr(text);
		MgraText(mil_graphic_context_, mil_graphic_context_list_, pos_x, pos_y + size_y, wc_text_1.get());

		break;
	}

	// draw model depth on the left-top of the image
	std::string feature_name_depth = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_depth();
	for (auto it : mil_mod_context_map_)
	{
		if (it.first.find(feature_name_depth) == std::string::npos)
			continue;

		if (!show_first)
		{
			if (it.first != pattern_name)
				continue;
		}

		MIL_INT model_box_size_x = MmodInquire(it.second, M_DEFAULT, M_BOX_SIZE_X + M_TYPE_MIL_INT32, M_NULL);
		MIL_INT model_box_size_y = MmodInquire(it.second, M_DEFAULT, M_BOX_SIZE_Y + M_TYPE_MIL_INT32, M_NULL);

		MIL_ID model = MbufAlloc2d(mil_system_, model_box_size_x, model_box_size_y, 8 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
		MmodDraw(M_DEFAULT, it.second, model, M_DRAW_IMAGE, M_DEFAULT, M_DEFAULT);
		MimRemap(M_DEFAULT, model, model, M_FIT_SRC_DATA);

		MIL_ID model_roi = MbufChild2d(reference_image_, 0, 0, model_box_size_x, model_box_size_y, M_NULL);

		MbufClear(model_roi, 0);
		MbufCopy(model, model_roi);
		MbufFree(model);
		MbufFree(model_roi);

		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
		MmodDraw(mil_graphic_context_, it.second, mil_graphic_context_list_, M_DRAW_BOX, 0, M_DEFAULT);
		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_RED);
		MmodDraw(mil_graphic_context_, it.second, mil_graphic_context_list_, M_DRAW_EDGES, 0, M_DEFAULT);
		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_MAGENTA);
		MmodDraw(mil_graphic_context_, it.second, mil_graphic_context_list_, M_DRAW_POSITION, 0, M_DEFAULT);

		// text "depth"
		MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		std::string text = "[" + it.first + "]";
		std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
		MgraText(mil_graphic_context_, mil_graphic_context_list_, 0, model_box_size_y, wc_text.get());

		break;
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