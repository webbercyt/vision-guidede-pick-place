#include "depth_feature_group_seg_lib.h"
#include "depth_localization_seg_lib.h"
#include "depth_camera.h"
#include "utils.h"

DepthSegmentFeature::DepthSegmentFeature(
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
		node_name)
{
}

DepthSegmentFeature::~DepthSegmentFeature()
{
}

void DepthSegmentFeature::createModel()
{
	if (reference_image_ == M_NULL && reference_range_image_ == M_NULL)
		return;

	MgraClear(mil_graphic_context_, mil_graphic_context_list_);
	define_succesed_ = false;

	// 1. load segment parameters
	std::string feature_name_seg = 
		workspace_ptr_->feature_group().at(node_name_).depth_feature_group().feature_name_seg() + std::to_string(DONIMAL_FEATURE_INDEX);
	bool has_feature = false;
	config::Workspace_FeatureGroup::PatternWindow src_pattern;
	for (auto const& pattern_window : workspace_ptr_->feature_group().at(node_name_).pattern_windows())
	{
		if (pattern_window.first == feature_name_seg)
		{
			has_feature = true;
			src_pattern = pattern_window.second;

			break;
		}
	}

	if (!has_feature)
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Error segment model workspace"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		return;
	}

	// 2. add depth mask
	double max_depth = 0.0;
	SPoseDataQuat ground_rotation = SPoseDataQuat();
	DPCLib::getGroundEraseParams(workspace_ptr_, node_name_, mil_calibration_, pose_base_to_tool_, max_depth, ground_rotation);

	double range_image_offset = workspace_ptr_->camera().at(camera_name_).depth_camera().range_image_offset();
	double range_image_scale = workspace_ptr_->camera().at(camera_name_).depth_camera().range_image_scale();
	int max_value = range_image_scale == 0 ? 
		(int)max_depth : 
		(int)(((double)max_depth - range_image_offset) / range_image_scale);
	
	MIL_ID temp_range_image = MbufClone(reference_range_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(temp_range_image, 0);
	MimClip(reference_range_image_, temp_range_image, M_GREATER, max_value, M_NULL, 0, M_NULL);

	MIL_INT image_size_x = MbufInquire(reference_image_, M_SIZE_X, M_NULL);
	MIL_INT image_size_y = MbufInquire(reference_image_, M_SIZE_Y, M_NULL);
	MIL_ID binary_image = MbufAlloc2d(mil_system_, image_size_x, image_size_y, M_UNSIGNED + 1, M_IMAGE + M_DISP + M_PROC, M_NULL);
	MbufClear(binary_image, 1);
	MimBinarize(temp_range_image, binary_image, M_FIXED + M_NOT_EQUAL, 0, M_NULL);

	// 3. draw & mask search region
	auto roi_x = max(0, workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x());
	auto roi_y = max(0, workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y());
	auto roi_x_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_x_size();
	auto roi_y_size = workspace_ptr_->feature_group().at(node_name_).depth_feature_group().roi_y_size();

	if (roi_x_size <= 0) roi_x_size = (int)image_size_x;
	if (roi_y_size <= 0) roi_y_size = (int)image_size_y;
	if (roi_x + roi_x_size > image_size_x) roi_x_size = (int)image_size_x - roi_x;
	if (roi_y + roi_y_size > image_size_y) roi_y_size = (int)image_size_y - roi_x;

	displaySearchRange();
	MIL_ID search_roi = MbufChild2d(reference_image_, roi_x, roi_y, roi_x_size, roi_y_size, M_NULL);

	// 4. define model
	MmodAlloc(mil_system_, M_SHAPE_SEGMENT, M_DEFAULT, &mil_mod_context_map_[feature_name_seg]);
	auto segment_length = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(feature_name_seg).depth_segment_feature().search_length();
	MmodDefine(mil_mod_context_map_[feature_name_seg], M_SEGMENT, segment_length, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	ModelProcess::setContextFromPattern(mil_mod_context_map_[feature_name_seg], src_pattern, M_ALL);

	MIL_INT nb_model_found;
	MmodAllocResult(mil_system_, M_SHAPE_SEGMENT, &mil_mod_result_map_[feature_name_seg]);
	MmodFind(mil_mod_context_map_[feature_name_seg], search_roi, mil_mod_result_map_[feature_name_seg]);
	MbufFree(search_roi);
	MmodGetResult(mil_mod_result_map_[feature_name_seg], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

	if (nb_model_found > 0)
	{
		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_RED);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -roi_x);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -roi_y);
		MmodDraw(mil_graphic_context_, mil_mod_result_map_[feature_name_seg], mil_graphic_context_list_, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);

		// @ eliminate segment result from image
		MgraControl(mil_graphic_context_, M_COLOR, 0);
		MmodDraw(mil_graphic_context_, mil_mod_result_map_[feature_name_seg], binary_image, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);

		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
	}
	
	// @ prepare blob analysis image: erode -> thick -> add operation
	MIL_ID bianry_temp = MbufClone(binary_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);
	MimErode(bianry_temp, bianry_temp, 7, M_BINARY);
	MimThick(bianry_temp, bianry_temp, 12, M_BINARY);
	MimArith(bianry_temp, binary_image, binary_image, M_AND);
	MbufFree(bianry_temp);
	MimErode(binary_image, binary_image, 1, M_BINARY); // erode 1 for easy blob calculate
	
	// @ do blob analysis
	MIL_ID mil_blob_context = MblobAlloc(mil_system_, M_DEFAULT, M_DEFAULT, M_NULL);
	DepthSegmentLocalization::setBlobContextControl(mil_blob_context);
	MIL_ID mil_blob_result = MblobAllocResult(mil_system_, M_DEFAULT, M_DEFAULT, M_NULL);

	MIL_ID binary_search_roi = MbufChild2d(binary_image, roi_x, roi_y, roi_x_size, roi_y_size, M_NULL);
	MblobCalculate(mil_blob_context, binary_search_roi, M_NULL, mil_blob_result);

	auto depth_segment_feature = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(feature_name_seg).depth_segment_feature();
	auto segment_area_min = depth_segment_feature.segment_area_min();
	auto segment_area_max = depth_segment_feature.segment_area_max();

	if (segment_area_max > segment_area_min ||
		segment_area_min ||
		segment_area_max)
	{
		if (segment_area_min > 0)
			MblobSelect(mil_blob_result, M_EXCLUDE, M_BOX_AREA, M_LESS, segment_area_min, M_NULL);

		if (segment_area_max > 0)
			MblobSelect(mil_blob_result, M_EXCLUDE, M_BOX_AREA, M_GREATER, segment_area_max, M_NULL);
	}

	MblobGetResult(mil_blob_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);
	if (nb_model_found > 0)
	{
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -roi_x);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -roi_y);

		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		MblobDraw(mil_graphic_context_, mil_blob_result, mil_graphic_context_list_, M_DRAW_CENTER_OF_GRAVITY + M_DRAW_BLOBS_CONTOUR, M_INCLUDED_BLOBS, M_DEFAULT);

		MgraColor(mil_graphic_context_, M_COLOR_GREEN);
		MblobDraw(mil_graphic_context_, mil_blob_result, mil_graphic_context_list_, M_DRAW_CENTER_OF_GRAVITY + M_DRAW_BLOBS_CONTOUR, M_BLOB_INDEX(0), M_DEFAULT);

		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
	}
	else
	{
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "No segmented object found !"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}
	MblobFree(mil_blob_context);
	MblobFree(mil_blob_result);

	std::unique_ptr<DepthSegmentLocalization> localization_ptr =
		std::make_unique<DepthSegmentLocalization>(
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
	localization_ptr->pushFrame(reference_image_,
		reference_range_image_,
		pose_base_to_tool_,
		can_stop,
		has_result);

	if (has_result)
	{
		std::string training_name = workspace_ptr_->feature_group().at(node_name_).training_name();
		const auto training = workspace_ptr_->training().at(training_name);
		std::string pose_calculator_name = training.pose_calculator_name();

		SPoseDataQuat pose_2_image;
		localization_ptr->getModelPose(ModelPoseType::MODEL_TO_IMAGE, pose_2_image);
		config::Workspace::QuaternionPose data;
		data.set_x(pose_2_image.x);
		data.set_y(pose_2_image.y);
		data.set_z(pose_2_image.z);
		data.set_q1(pose_2_image.q1);
		data.set_q2(pose_2_image.q2);
		data.set_q3(pose_2_image.q3);
		data.set_q4(pose_2_image.q4);
		*workspace_ptr_->mutable_pose_calculator()->at(pose_calculator_name).mutable_model_reference_image_pose() = data;

		SPoseDataQuat pose;
		localization_ptr->getModelPose(ModelPoseType::MODEL_TO_WORLD, pose);
		data.set_x(pose.x);
		data.set_y(pose.y);
		data.set_z(pose.z);
		data.set_q1(pose.q1);
		data.set_q2(pose.q2);
		data.set_q3(pose.q3);
		data.set_q4(pose.q4);
		*workspace_ptr_->mutable_pose_calculator()->at(pose_calculator_name).mutable_model_reference_pose() = data;
	}
	else
	{
		// draw depth on the top left corner
		MimClip(temp_range_image, temp_range_image, M_NOT_EQUAL, 0, M_NULL, 255, M_NULL);

		MIL_INT thumbnail_size_x = image_size_x / 4;
		MIL_INT thumbnail_size_y = image_size_y / 4;

		MIL_ID thumbnail = MbufChild2d(reference_image_,
			1,
			1,
			thumbnail_size_x,
			thumbnail_size_y,
			M_NULL);

		MbufClear(thumbnail, 0);
		MimResize(temp_range_image, thumbnail, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BILINEAR);
		MbufFree(thumbnail);

		MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
		MgraRect(mil_graphic_context_, mil_graphic_context_list_,
			0,
			0,
			thumbnail_size_x + 2,
			thumbnail_size_y + 2);

		MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		std::string text = "[ground erase result]";
		std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
		MgraText(mil_graphic_context_, mil_graphic_context_list_, 0, thumbnail_size_y + 4, wc_text.get());

		drawPattern(feature_name_seg);

		MdispSelectWindow(mil_display_, reference_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
		std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, "Failed to define feature, please modify parameters and try again"); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
	}

	localization_ptr.reset();

	// 6. free memory
	MbufFree(temp_range_image);
	MbufFree(binary_search_roi);
	MbufFree(binary_image);

	if (!has_result)
		return;

	std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Model is successfully defined !"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
	define_succesed_ = true;
}

void DepthSegmentFeature::displayContext(std::string pattern_name)
{
	drawPattern(pattern_name);

	if (pattern_name == "")
	{
		displaySearchRange();

		// get reference image pose to draw cross
		std::string training_name = workspace_ptr_->feature_group().at(node_name_).training_name();
		const auto training = workspace_ptr_->training().at(training_name);
		std::string pose_calculator_name = training.pose_calculator_name();

		auto quat_pose = workspace_ptr_->pose_calculator().at(pose_calculator_name).model_reference_image_pose();
		MIL_DOUBLE center_x = quat_pose.x();
		MIL_DOUBLE center_y = quat_pose.y();

		MIL_DOUBLE offset = 60.0;
		MgraColor(mil_graphic_context_, M_COLOR_BLUE);
		for (int i = -1; i < 2; i++)
		{
			MgraLine(mil_graphic_context_, mil_graphic_context_list_,
				center_x + i, 0, center_x + i, MbufInquire(reference_image_, M_SIZE_Y, M_NULL));
			MgraLine(mil_graphic_context_, mil_graphic_context_list_,
				0, center_y + i, MbufInquire(reference_image_, M_SIZE_X, M_NULL), center_y + i);
		}

		MgraControl(mil_graphic_context_, M_FONT_SIZE, 18);
		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		std::string text = "[ground erase result]";
		std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
		MgraText(mil_graphic_context_, mil_graphic_context_list_, 1, 1, wc_text.get());

		std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + node_name_;
		std::string image_path = base_folder + "/result_" + workspace_ptr_->feature_group().at(node_name_).depth_feature_group().reference_depth_image_path();

		std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
		if (result_image_)
			MbufFree(result_image_);
		MbufImport(wc_image_path.get(), M_DEFAULT, M_RESTORE + M_NO_GRAB + M_NO_COMPRESS, mil_system_, &result_image_);
	}
	else
	{
		if (!result_image_)
			MbufClone(reference_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, &result_image_);
		MbufClear(result_image_, 0);
	}

	MdispSelectWindow(mil_display_, result_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
}

void DepthSegmentFeature::drawPattern(std::string pattern_name)
{
	bool show_all = pattern_name == "";
	for (auto it : mil_mod_context_map_)
	{
		if (show_all || it.first.find(pattern_name) != std::string::npos)
		{
			auto depth_segment_feature = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).depth_segment_feature();
			auto segment_length = depth_segment_feature.search_length();
			auto scale_min = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).search_scale_min();
			auto scale_max = workspace_ptr_->feature_group().at(node_name_).pattern_windows().at(it.first).search_scale_max();

			auto segment_area_min = depth_segment_feature.segment_area_min();
			auto segment_area_max = depth_segment_feature.segment_area_max();

			MIL_INT x_start = 18;
			MIL_INT y_offset = 18;
			MIL_INT margin = 4;

			MIL_INT image_size_x = MbufInquire(reference_image_, M_SIZE_X, M_NULL);
			MIL_INT image_size_y = MbufInquire(reference_image_, M_SIZE_Y, M_NULL);
			MIL_INT max_size_x = image_size_x / 2;
			MIL_INT max_size_y = image_size_y / 2;

			int font_size = 18;
			MgraControl(mil_graphic_context_, M_FONT_SIZE, font_size);

			// draw blob area on the left bottom corner
			if (segment_area_max > 0)
			{
				MIL_INT max_blob_length = (MIL_INT)sqrt(segment_area_max);
				std::string text = "[max object area]";
				std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);

				if (show_all)
				{
					if (max_blob_length <= max_size_x && max_blob_length <= max_size_y)
					{
						MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_BRIGHT_GRAY);
						MgraRectFill(mil_graphic_context_, mil_graphic_context_list_,
							x_start, image_size_y - y_offset - max_blob_length, x_start + max_blob_length, image_size_y - y_offset);

						MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
						MgraText(mil_graphic_context_, mil_graphic_context_list_,
							x_start, image_size_y - y_offset - max_blob_length - font_size - margin, wc_text.get());

						y_offset += (y_offset + max_blob_length + font_size + margin + margin);
					}
					else
					{
						MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
						std::string text_over = "[max object area: over display limit]";
						std::shared_ptr<wchar_t> wc_text_over = utils::string2Wchar_tPtr(text_over);
						MgraText(mil_graphic_context_, mil_graphic_context_list_,
							x_start, image_size_y - y_offset - font_size - margin, wc_text_over.get());

						y_offset += (y_offset + font_size + margin + margin);
					}
				}		
				else
				{
					MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
					MgraRect(mil_graphic_context_, mil_graphic_context_list_,
						max(margin, x_start),
						max(margin, image_size_y - y_offset - max_blob_length),
						min(image_size_x - margin, x_start + max_blob_length),
						min(image_size_y - margin, image_size_y - y_offset));

					MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
					MgraText(mil_graphic_context_, mil_graphic_context_list_,
						x_start, max(margin, image_size_y - y_offset - max_blob_length - font_size - margin), wc_text.get());
				}
				
			}
			else
			{
				MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
				std::string text = "[max object area: N/A]";
				std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
				MgraText(mil_graphic_context_, mil_graphic_context_list_,
					x_start, image_size_y - y_offset - font_size - margin, wc_text.get());

				y_offset += (y_offset + font_size + margin + margin);
			}

			if (segment_area_min > 0)
			{
				MIL_INT min_blob_length = (MIL_INT)sqrt(segment_area_min);
				std::string text = "[min object area]";
				std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);

				if (show_all)
				{
					if (min_blob_length <= max_size_x && min_blob_length <= max_size_y)
					{
						MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_BRIGHT_GRAY);
						MgraRectFill(mil_graphic_context_, mil_graphic_context_list_,
							x_start, image_size_y - y_offset - min_blob_length, x_start + min_blob_length, image_size_y - y_offset);

						MgraColor(mil_graphic_context_, M_COLOR_YELLOW);

						MgraText(mil_graphic_context_, mil_graphic_context_list_,
							x_start, image_size_y - y_offset - min_blob_length - font_size - margin, wc_text.get());

						y_offset += (y_offset + min_blob_length + font_size + margin + margin);
					}
					else
					{
						MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
						std::string text_over = "[min object area: over display limit]";
						std::shared_ptr<wchar_t> wc_text_over = utils::string2Wchar_tPtr(text_over);
						MgraText(mil_graphic_context_, mil_graphic_context_list_,
							x_start, image_size_y - y_offset - font_size - margin, wc_text_over.get());

						y_offset += (y_offset + font_size + margin + margin);
					}
				}				
				else
				{
					MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
					MgraRect(mil_graphic_context_, mil_graphic_context_list_,
						max(margin, x_start),
						max(margin, image_size_y - y_offset - min_blob_length),
						min(image_size_x - margin, x_start + min_blob_length),
						min(image_size_y - margin, image_size_y - y_offset));

					MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
					MgraText(mil_graphic_context_, mil_graphic_context_list_,
						x_start, max(margin, image_size_y - y_offset - min_blob_length - font_size - margin), wc_text.get());
				}
			}
			else
			{
				MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
				std::string text = "[min object area: N/A]";
				std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr(text);
				MgraText(mil_graphic_context_, mil_graphic_context_list_,
					x_start, image_size_y - y_offset - font_size - margin, wc_text.get());

				y_offset += (y_offset + font_size + margin + margin);
			}

			// draw segment length on the right top corner
			y_offset = 18;

			if (segment_length > 0)
			{
				MgraControl(mil_graphic_context_, M_TEXT_ALIGN_HORIZONTAL, M_RIGHT);

				double min_length = segment_length * scale_min;
				if (min_length <= max_size_x && min_length <= max_size_y)
				{
					std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr("[min segment length]");
					MgraColor(mil_graphic_context_, M_COLOR_RED);
					MgraText(mil_graphic_context_, mil_graphic_context_list_,
						image_size_x - margin, y_offset, wc_text.get());

					MgraLine(mil_graphic_context_, mil_graphic_context_list_,
						image_size_x - (int)min_length - margin, y_offset + font_size + margin * 2, image_size_x - margin, y_offset + font_size + margin * 2);

					y_offset += (18 + font_size + margin * 3);
				}

				if (segment_length <= max_size_x && segment_length <= max_size_y)
				{
					std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr("[segment length]");
					MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
					MgraText(mil_graphic_context_, mil_graphic_context_list_,
						image_size_x - margin, y_offset, wc_text.get());

					MgraLine(mil_graphic_context_, mil_graphic_context_list_,
						image_size_x - (int)segment_length - margin, y_offset + font_size + margin * 2, image_size_x - margin, y_offset + font_size + margin * 2);

					y_offset += (18 + font_size + margin * 3);
				}

				double max_length = segment_length * scale_max;
				if (max_length <= max_size_x && max_length <= max_size_y)
				{
					std::shared_ptr<wchar_t> wc_text = utils::string2Wchar_tPtr("[max segment length]");
					MgraColor(mil_graphic_context_, M_COLOR_BLUE);
					MgraText(mil_graphic_context_, mil_graphic_context_list_,
						image_size_x - margin, y_offset, wc_text.get());

					MgraLine(mil_graphic_context_, mil_graphic_context_list_,
						image_size_x - (int)max_length - margin, y_offset + font_size + margin * 2, image_size_x - margin, y_offset + font_size + margin * 2);

					y_offset += (y_offset + font_size + margin * 3);
				}

				MgraControl(mil_graphic_context_, M_TEXT_ALIGN_HORIZONTAL, M_LEFT);
			}

			break;
		}
	}
}