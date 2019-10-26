#include "depth_feature_group_rect_lib.h"
#include "depth_localization_rect_lib.h"
#include "depth_camera.h"
#include "utils.h"

DepthRectFeature::DepthRectFeature(MIL_ID mil_application,
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

void DepthRectFeature::createModel()
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
	if (allow_locate_method_2d)
	{
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
		MgraFill(mil_graphic_context_, image_mask, (MIL_INT)(pos_x + size_x / 2), (MIL_INT)(pos_y + size_y / 2));
		MimClip(image_mask, image_mask, M_NOT_EQUAL, 0, M_NULL, 1, M_NULL);
	}
	else
	{
		MbufClear(image_mask, 1);
	}	

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
		image_mask,
		depth_map_,
		0,
		max_depth,
		0,
		false,
		false,
		true,
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
	// temporary set roi to the selected window
	auto old_roi_x = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x();
	auto old_roi_y = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y();
	auto old_roi_x_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x_size();
	auto old_roi_y_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y_size();
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_x((int)pos_x);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_y((int)pos_y);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_x_size((int)size_x);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_y_size((int)size_y);

	std::shared_ptr<DepthLocalization> localization_ptr = std::move(createModelLocalizationPtr());
	bool can_stop = false;
	bool has_result = false;

	localization_ptr->resetlocateState();
	localization_ptr->pushFrame(reference_image_,
		reference_range_image_,
		pose_base_to_tool_,
		can_stop,
		has_result);

	// recover temporary set roi
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_x((int)old_roi_x);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_y((int)old_roi_y);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_x_size((int)old_roi_x_size);
	workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_feature_group()->set_roi_y_size((int)old_roi_y_size);

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

	/*printCoutMsg(LEVEL_BASIC_INFO, "T: [",
		pose.x, ", ", pose.y, ", ", pose.z, "]"
		" R: [",
		pose.q1, ", ", pose.q2, ", ", pose.q3, ", ", pose.q4, "]");*/

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

bool DepthRectFeature::define2DModelContext(
	std::string mod_name,
	MIL_ID mod_find_image,
	MIL_DOUBLE model_size_x,
	MIL_DOUBLE model_size_y,
	MIL_DOUBLE max_model_size_x,
	MIL_DOUBLE max_model_size_y,
	config::Workspace_FeatureGroup::PatternWindow src_pattern,
	int mod_number)
{
	auto foreground = workspace_ptr_->feature_group().at(node_name_).depth_shape_feature_group().foreground_color();
	MIL_INT FOREGROUND = M_DEFAULT;
	if (foreground == 0)
		FOREGROUND = M_FOREGROUND_BLACK;
	else if(foreground == 1)
		FOREGROUND = M_FOREGROUND_WHITE;

	MIL_INT nb_model_found;
	MIL_DOUBLE width = model_size_x;
	MIL_DOUBLE height = model_size_y;

	if (width < height) std::swap(width, height);

	MIL_ID mod_context_2d = MmodAlloc(mil_system_, M_SHAPE_RECTANGLE, M_DEFAULT, M_NULL);
	// set both sides to width
	MmodDefine(mod_context_2d, M_RECTANGLE, FOREGROUND, width, height, M_DEFAULT, M_DEFAULT);
	ModelProcess::setContextFromPattern(mod_context_2d, src_pattern, 1);

	double aspect_min = 0.2;
	double aspect_max = 5.0;
	double scale_min = 0.4;
	double scale_max = 1.0;
	MmodControl(mod_context_2d, M_DEFAULT, M_MODEL_ASPECT_RATIO_MIN_FACTOR, aspect_min);
	MmodControl(mod_context_2d, M_DEFAULT, M_MODEL_ASPECT_RATIO_MAX_FACTOR, aspect_max);
	MmodControl(mod_context_2d, M_DEFAULT, M_SCALE_MIN_FACTOR, scale_min);
	MmodControl(mod_context_2d, M_DEFAULT, M_SCALE_MAX_FACTOR, scale_max);

	MmodPreprocess(mod_context_2d, M_DEFAULT);

	// set segment
	MIL_ID mil_context_segment;
	MIL_ID mil_result_segment;
	{
		// get rectangle pixel width & height & scale to define segment length
		double width_min = width * scale_min;
		double width_max = width * scale_max;
		double height_min = height / aspect_max * scale_min;
		double height_max = height / aspect_min * scale_max;

		double length_min = min(width_min, height_min);
		double length_max = max(width_max, height_max);
		int length = (int)(length_min + length_max) / 2;

		mil_context_segment = MmodAlloc(mil_system_, M_SHAPE_SEGMENT, M_DEFAULT, M_NULL);
		MmodDefine(mil_context_segment, M_SEGMENT, length, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
		MmodControl(mil_context_segment, M_DEFAULT, M_SCALE_MIN_FACTOR, length_min / length);
		MmodControl(mil_context_segment, M_DEFAULT, M_SCALE_MAX_FACTOR, length_max / length);
		MmodControl(mil_context_segment, M_CONTEXT, M_DETAIL_LEVEL, M_VERY_HIGH);
		MmodControl(mil_context_segment, M_DEFAULT, M_NUMBER, M_ALL);

		MmodPreprocess(mil_context_segment, M_DEFAULT);
		mil_result_segment = MmodAllocResult(mil_system_, M_SHAPE_SEGMENT, M_NULL);
	}

	MIL_ID image = MbufClone(mod_find_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);

	MmodFind(mil_context_segment, image, mil_result_segment);
	MIL_INT nb_segments;
	MmodGetResult(mil_result_segment, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_segments);
	if (nb_segments > 0)
	{
		// consider foreground color later
		if (FOREGROUND == M_FOREGROUND_BLACK || FOREGROUND == M_DEFAULT)
			MgraColor(mil_graphic_context_, 255);
		else if (FOREGROUND == M_FOREGROUND_WHITE)
			MgraColor(mil_graphic_context_, 0);

		MmodDraw(mil_graphic_context_, mil_result_segment, image, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);

		//// save process_image_2d
		//{
		//	std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr("C:/yt_folder/test/" + std::to_string(7) + ".jpg");
		//	MbufExport(wc_image_path.get(), M_MIL + M_WITH_CALIBRATION, image);
		//}
	}

	MmodFree(mil_context_segment);
	MmodFree(mil_result_segment);

	MmodAllocResult(mil_system_, M_SHAPE_RECTANGLE, &mil_mod_result_map_[mod_name]);
	MmodFind(mod_context_2d, image, mil_mod_result_map_[mod_name]);
	MmodGetResult(mil_mod_result_map_[mod_name], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);
	MmodFree(mod_context_2d);
	MbufFree(image);

	// filter width & height
	MIL_INT index = -1;
	if (nb_model_found > 0)
	{
		MIL_DOUBLE area = 0;
		for (MIL_INT i = 0; i < nb_model_found; i++)
		{
			MmodGetResult(mil_mod_result_map_[mod_name], i, M_WIDTH, &width);
			MmodGetResult(mil_mod_result_map_[mod_name], i, M_HEIGHT, &height);

			/*if (width > max_model_size_x - 1.0 &&
				height > max_model_size_y - 1.0)
				continue;*/

			if (area < width * height)
			{
				index = i;
				area = width * height;
			}
		}
	}

	if (index == -1)
	{
		return false;
	}
	else
	{
		// redefine model
		MmodAlloc(mil_system_, M_SHAPE_RECTANGLE, M_DEFAULT, &mil_mod_context_map_[mod_name]);
		MmodDefine(mil_mod_context_map_[mod_name], M_RECTANGLE, FOREGROUND, width, height, M_DEFAULT, M_DEFAULT);
		ModelProcess::setContextFromPattern(mil_mod_context_map_[mod_name], src_pattern, mod_number);

		workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_shape_feature_group()->set_width_pixel(width);
		workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_shape_feature_group()->set_height_pixel(height);

		return true;
	}
}

bool DepthRectFeature::defineDepthModelContext(
	std::string mod_name,
	MIL_ID mod_find_image,
	MIL_DOUBLE model_size_x,
	MIL_DOUBLE model_size_y,
	config::Workspace_FeatureGroup::PatternWindow src_pattern,
	int mod_number)
{
	MIL_INT nb_model_found;
	MIL_DOUBLE width = model_size_x;
	MIL_DOUBLE height = model_size_y;

	if (width < height) std::swap(width, height);
	MIL_ID mod_context_depth = MmodAlloc(mil_system_, M_SHAPE_RECTANGLE, M_DEFAULT, M_NULL);
	MmodDefine(mod_context_depth, M_RECTANGLE, M_DEFAULT, width, height, M_DEFAULT, M_DEFAULT);
	ModelProcess::setContextFromPattern(mod_context_depth, src_pattern, 1);
	MmodControl(mod_context_depth, M_DEFAULT, M_DEVIATION_TOLERANCE, 40.0);
	MmodPreprocess(mod_context_depth, M_DEFAULT);

	MmodAllocResult(mil_system_, M_SHAPE_RECTANGLE, &mil_mod_result_map_[mod_name]);
	MmodFind(mod_context_depth, mod_find_image, mil_mod_result_map_[mod_name]);
	MmodGetResult(mil_mod_result_map_[mod_name], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);
	MmodFree(mod_context_depth);

	// filter width & height
	MIL_INT index = -1;
	if (nb_model_found > 0)
	{
		MIL_DOUBLE area = 0;
		for (MIL_INT i = 0; i < nb_model_found; i++)
		{
			MmodGetResult(mil_mod_result_map_[mod_name], i, M_WIDTH, &width);
			MmodGetResult(mil_mod_result_map_[mod_name], i, M_HEIGHT, &height);

			if (area < width * height)
			{
				index = i;
				area = width * height;
			}
		}
	}

	if (index == -1)
	{
		return false;
	}
	else
	{
		// re-define model
		MmodAlloc(mil_system_, M_SHAPE_RECTANGLE, M_DEFAULT, &mil_mod_context_map_[mod_name]);
		MmodDefine(mil_mod_context_map_[mod_name], M_RECTANGLE, M_DEFAULT, width, height, M_DEFAULT, M_DEFAULT);
		// TODO(yt) move M_DEVIATION_TOLERANCE to gui
		MmodControl(mil_mod_context_map_[mod_name], M_DEFAULT, M_DEVIATION_TOLERANCE, 40.0);
		ModelProcess::setContextFromPattern(mil_mod_context_map_[mod_name], src_pattern, mod_number);

		workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_shape_feature_group()->set_width_world(width);
		workspace_ptr_->mutable_feature_group()->at(node_name_).mutable_depth_shape_feature_group()->set_height_world(height);

		MgraColor(M_DEFAULT, M_COLOR_YELLOW);
		MmodDraw(M_DEFAULT, mil_mod_result_map_[mod_name], depth_gra_list_, M_DRAW_POSITION + M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
		MgraColor(M_DEFAULT, M_COLOR_GREEN);
		MmodDraw(M_DEFAULT, mil_mod_result_map_[mod_name], depth_gra_list_, M_DRAW_POSITION + M_DRAW_EDGES, /*M_DEFAULT*/index, M_DEFAULT);

		return true;
	}
}

std::shared_ptr<DepthLocalization> DepthRectFeature::createModelLocalizationPtr()
{
	std::shared_ptr<DepthLocalization> ptr = std::make_shared<DepthRectLocalization>(
		mil_application_,
		mil_system_,
		mil_display_,
		mil_window_id_,
		workspace_ptr_,
		node_name_,
		mil_mod_context_map_);

	return ptr;
}

bool DepthRectFeature::displayModelSizes()
{
	if (reference_image_ == M_NULL)
		return false;

	MdispControl(mil_display_, M_UPDATE, M_DISABLE);
	MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, mil_graphic_context_list_);

	auto width_pixel = workspace_ptr_->feature_group().at(node_name_).depth_shape_feature_group().width_pixel();
	auto height_pixel = workspace_ptr_->feature_group().at(node_name_).depth_shape_feature_group().height_pixel();
	auto scale_min = workspace_ptr_->feature_group().at(node_name_).depth_shape_feature_group().search_scale_min();
	auto scale_max = workspace_ptr_->feature_group().at(node_name_).depth_shape_feature_group().search_scale_max();

	if (result_image_)
		MbufFree(result_image_);	
	result_image_ = MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(result_image_, 0);
	MdispSelectWindow(mil_display_, result_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	MIL_INT center_x = MbufInquire(result_image_, M_SIZE_X, M_NULL) / 2;
	MIL_INT center_y = MbufInquire(result_image_, M_SIZE_Y, M_NULL) / 2;

	// compute the min rect & draw
	int w_min = (int)(width_pixel * scale_min);
	int h_min = (int)(height_pixel * scale_min);

	MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
	MgraRect(mil_graphic_context_, mil_graphic_context_list_,
		max(0, center_x - w_min / 2),
		max(0, center_y - h_min / 2),
		(center_x + w_min / 2 > center_x * 2 - 1) ? (center_x * 2 - 1) : (center_x + w_min / 2 - 1),
		(center_y + h_min / 2 > center_y * 2 - 1) ? (center_y * 2 - 1) : (center_y + h_min / 2 - 1));

	// compute the max rect size
	int w_max = (int)(width_pixel * scale_max);
	int h_max = (int)(height_pixel * scale_max);

	MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_BLUE);
	MgraRect(mil_graphic_context_, mil_graphic_context_list_,
		max(0, center_x - w_max / 2),
		max(0, center_y - h_max / 2),
		(center_x + w_max / 2 > center_x * 2 - 1) ? (center_x * 2 - 1) : (center_x + w_max / 2 - 1),
		(center_y + h_max / 2 > center_y * 2 - 1) ? (center_y * 2 - 1) : (center_y + h_max / 2 - 1));

	MdispControl(mil_display_, M_UPDATE, M_ENABLE);
	return true;
}

void DepthRectFeature::displayContext(std::string pattern_name)
{
	/* Disable the display update for better performance. */
	MdispControl(mil_display_, M_UPDATE, M_DISABLE);
	MdispControl(mil_display_, M_ASSOCIATED_GRAPHIC_LIST_ID, mil_graphic_context_list_);

	// draw valid region
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

	// import result image
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
	std::string image_path = base_folder + "/result_" + workspace_ptr_->feature_group().at(node_name_).depth_feature_group().reference_depth_image_path();
	std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
	if (result_image_)
		MbufFree(result_image_);
	MbufImport(wc_image_path.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &result_image_);
	MdispSelectWindow(mil_display_, result_image_, (MIL_WINDOW_HANDLE)mil_window_id_);

	MdispControl(mil_display_, M_UPDATE, M_ENABLE);
}
