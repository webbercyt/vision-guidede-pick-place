#include "depth_localization_seg_lib.h"
#include "depth_camera.h"
#include "utils.h"

#define DRAW_INTERMEDIUM_RESULT 1
#define SEGMENT_ITERATION_TIMES 3
#define ERODE_ITERATIONS 7
#define THICK_ITERATIONS ERODE_ITERATIONS + 5

DepthSegmentLocalization::DepthSegmentLocalization(MIL_ID mil_application,
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
	max_depth_value_(0xFFFF),
	display_image_(M_NULL)
{
	if (!setupModContext())
	{
		is_allocation_success_ = false;
		return;
	}

	is_allocation_success_ = true;
}

DepthSegmentLocalization::DepthSegmentLocalization(MIL_ID mil_application,
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
	max_depth_value_(0xFFFF),
	display_image_(M_NULL)
{
	if (!setupModContext())
	{
		is_allocation_success_ = false;
		return;
	}

	std::string feature_name_seg = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_seg();
	for (auto it : mil_mod_context_map)
	{
		if (it.first.find(feature_name_seg) != std::string::npos)
		{
			if (it.second == M_NULL)
				continue;

			seg_context_map_.emplace(it);

			MIL_INT64 mod_type;
			MmodInquire(it.second, M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_type);

			auto const model_type = *ModelProcess::model_context_result_type_map.find(mod_type);
			MIL_INT64 mod_result_type = model_type.second;
			MIL_ID mod_result = MmodAllocResult(mil_system_, mod_result_type, M_NULL);
			seg_result_map_.emplace(it.first, mod_result);
		}
	}

	is_allocation_success_ = true;
}

DepthSegmentLocalization::~DepthSegmentLocalization()
{
	if(!is_defining_model_)
		for (auto it : seg_context_map_) MmodFree(it.second);

	for (auto it : seg_result_map_) MmodFree(it.second);
	for (auto it : blob_context_map_) MblobFree(it.second);
	for (auto it : blob_result_map_) MblobFree(it.second);

	if (display_image_) MbufFree(display_image_);
}

void DepthSegmentLocalization::setContextFromPattern(
	MIL_ID mod_context,
	config::Workspace_FeatureGroup::PatternWindow pattern,
	MIL_INT control_number)
{
	auto detail_level = pattern.m_detail_level();
	MIL_INT32 mil_detail_level = M_DEFAULT;
	switch (detail_level)
	{
	case Workspace_MLevel_M_LOW_PROTO:
	case Workspace_MLevel_M_MEDIUM_PROTO:
		mil_detail_level = M_MEDIUM;
		break;
	case Workspace_MLevel_M_HIGH_PROTO:
		mil_detail_level = M_HIGH;
		break;
	case Workspace_MLevel_M_VERY_HIGH_PROTO:

		mil_detail_level = M_VERY_HIGH;
		break;
	default:
		break;
	}

	MmodControl(mod_context, M_CONTEXT, M_DETAIL_LEVEL, mil_detail_level);
	MmodControl(mod_context, M_DEFAULT, M_CERTAINTY, pattern.m_certainty());
	MmodControl(mod_context, M_DEFAULT, M_ACCEPTANCE, pattern.m_acceptance());
	MmodControl(mod_context, M_CONTEXT, M_SMOOTHNESS, pattern.m_smoothness());
	MmodControl(mod_context, M_CONTEXT, M_TIMEOUT, pattern.m_timeout());

	MmodControl(mod_context, M_DEFAULT, M_SCALE_MIN_FACTOR, pattern.search_scale_min());
	MmodControl(mod_context, M_DEFAULT, M_SCALE_MAX_FACTOR, pattern.search_scale_max());

	MmodControl(mod_context, M_DEFAULT, M_ANGLE, pattern.depth_segment_feature().search_angle());
	MmodControl(mod_context, M_DEFAULT, M_ANGLE_DELTA_NEG, pattern.search_delta_angle());
	MmodControl(mod_context, M_DEFAULT, M_ANGLE_DELTA_POS, pattern.search_delta_angle());

	MmodControl(mod_context, M_DEFAULT, M_NUMBER, control_number);
	MmodControl(mod_context, M_DEFAULT, M_ANGLE_MULTIPLE_RANGE, M_STEP_90);
}

void DepthSegmentLocalization::setBlobContextControl(MIL_ID blob_context)
{
	MblobControl(blob_context, M_IDENTIFIER_TYPE, M_BINARY);
	MblobControl(blob_context, M_SORT1, M_BOX_AREA);
	MblobControl(blob_context, M_SORT1_DIRECTION, M_SORT_DOWN);
	MblobControl(blob_context, M_BOX, M_ENABLE);
	MblobControl(blob_context, M_CONNECTIVITY, M_4_CONNECTED);
	MblobControl(blob_context, M_CENTER_OF_GRAVITY + M_BINARY, M_ENABLE);
}

bool DepthSegmentLocalization::setupModContext()
{
	std::string mod_base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_ + "/";
	std::string feature_name_seg = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_seg();

	for (auto const& pattern_window : workspace_ptr_->feature_group().at(feature_group_name_).pattern_windows())
	{
		if (pattern_window.first.find(feature_name_seg) == std::string::npos)
		{
			continue;
		}

		if (!is_defining_model_)
		{
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
			DepthSegmentLocalization::setContextFromPattern(mod_context, pattern_window.second, M_ALL);
			MmodPreprocess(mod_context, M_DEFAULT);

			seg_context_map_.emplace(pattern_window.first, mod_context);
			seg_result_map_.emplace(pattern_window.first, mod_resutl);
		}

		// @ blob
		MIL_ID mil_blob_context = MblobAlloc(mil_system_, M_DEFAULT, M_DEFAULT, M_NULL);
		DepthSegmentLocalization::setBlobContextControl(mil_blob_context);
		MIL_ID mil_blob_result = MblobAllocResult(mil_system_, M_DEFAULT, M_DEFAULT, M_NULL);

		blob_context_map_.emplace(pattern_window.first, mil_blob_context);
		blob_result_map_.emplace(pattern_window.first, mil_blob_result);

		auto depth_segment_feature = workspace_ptr_->feature_group().at(feature_group_name_).pattern_windows().at(pattern_window.first).depth_segment_feature();
		auto segment_area_min = depth_segment_feature.segment_area_min();
		auto segment_area_max = depth_segment_feature.segment_area_max();
		blob_area_select_.emplace(pattern_window.first, std::pair<double, double>(segment_area_min, segment_area_max));
	}

	return true;
}

void DepthSegmentLocalization::runLocalization()
{
	// @ set up display
	if (!is_defining_model_)
	{
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

		if (!display_image_)
			MbufClone(src_2d_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, &display_image_);
		MbufCopy(src_2d_image_, display_image_);		

		MdispSelectWindow(mil_display_, display_image_, (MIL_WINDOW_HANDLE)mil_window_id_);
	}

	// @ eliminate gound form range image
	MIL_ID no_ground_range_image = MbufClone(src_range_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(no_ground_range_image, 0);

	{
		double max_depth = 0.0;
		SPoseDataQuat ground_rotation = SPoseDataQuat();
		DPCLib::getGroundEraseParams(workspace_ptr_, feature_group_name_, mil_cal_ground_erase_, pose_base_to_tool_, max_depth, ground_rotation);

		std::string camera_name = workspace_ptr_->training().at(workspace_ptr_->feature_group().at(feature_group_name_).training_name()).camera_name();
		double range_image_offset = workspace_ptr_->camera().at(camera_name).depth_camera().range_image_offset();
		double range_image_scale = workspace_ptr_->camera().at(camera_name).depth_camera().range_image_scale();

		max_depth_value_ = range_image_scale == 0 ? (int)max_depth : (int)(((double)max_depth - range_image_offset) / range_image_scale);
	}

	MimClip(src_range_image_, no_ground_range_image, M_GREATER, max_depth_value_, M_NULL, 0, M_NULL);

	// @ compute ideal object using segment & blob analysis
	MIL_ID best_blob_result; 
	MIL_INT best_blob_index;
	MIL_INT x_offset, y_offset;

	bool has_result = computeSegmentBlob(no_ground_range_image, best_blob_result, best_blob_index, x_offset, y_offset);

	// @ compute the pose of the segmented object
	SPoseDataQuat pose;
	if (has_result)
		has_result = computePoseFromBlobResult(pose, no_ground_range_image, best_blob_result, best_blob_index, x_offset, y_offset);

	// draw depth erase result on the top left corner
	MimClip(no_ground_range_image, no_ground_range_image, M_NOT_EQUAL, 0, M_NULL, 255, M_NULL);
	MIL_INT thumbnail_size_x = MbufInquire(no_ground_range_image, M_SIZE_X, M_NULL) / 4;
	MIL_INT thumbnail_size_y = MbufInquire(no_ground_range_image, M_SIZE_Y, M_NULL) / 4;

	MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_GREEN);
	MgraRect(mil_graphic_context_, mil_graphic_context_list_,
		0,
		0,
		thumbnail_size_x + 2,
		thumbnail_size_y + 2);

	if (!is_defining_model_)
	{
		MIL_ID thumbnail = MbufChild2d(display_image_, 1, 1, thumbnail_size_x, thumbnail_size_y, M_NULL);
		MbufClear(thumbnail, 0);
		MimResize(no_ground_range_image, thumbnail, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BILINEAR);
		MbufFree(thumbnail);
	}

	if (has_result)
	{
		if (is_defining_model_)
		{
			MIL_ID resutl_display_image = MbufAllocColor(mil_system_, 3, image_size_x_, image_size_y_, 8 + M_UNSIGNED, M_IMAGE + M_DISP + M_PROC, M_NULL);
			MimConvert(src_2d_image_, resutl_display_image, M_L_TO_RGB);

			// draw depth on the top left corner
			MIL_ID thumbnail = MbufChild2d(resutl_display_image, 1, 1, thumbnail_size_x, thumbnail_size_y, M_NULL);

			MbufClear(thumbnail, 0);
			MimResize(no_ground_range_image, thumbnail, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BILINEAR);
			MbufFree(thumbnail);

			MgraDraw(mil_graphic_context_list_, resutl_display_image, M_DEFAULT);

			std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_;
			std::string image_path = base_folder + "/result_" + workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().reference_depth_image_path();

			std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
			MbufExport(wc_image_path.get(), M_MIL + M_WITH_CALIBRATION, resutl_display_image);

			MbufFree(resutl_display_image);
		}

		MIL_DOUBLE center_x;
		MIL_DOUBLE center_y;
		MblobGetResult(best_blob_result, best_blob_index, M_CENTER_OF_GRAVITY_X + M_BINARY, &center_x);
		MblobGetResult(best_blob_result, best_blob_index, M_CENTER_OF_GRAVITY_Y + M_BINARY, &center_y);
		pose_model_to_image_ = SPoseDataQuat(center_x + x_offset, center_y + y_offset, 0, 0, 0, 0, 0);
		pose_model_to_cam_ = pose;
		pose = transformPoseFromCameraToWorld(pose);

		// set up original state
		model_collect_times_ = 1;
		model_rotate_.clear();

		model_poses_x_ = pose.x;
		model_poses_y_ = pose.y;
		model_poses_z_ = pose.z;
		model_rotate_.emplace(0, pose);

		handleCollectedData();

		state_ = LocalizationState::SUCCESSED;
	}

	MbufFree(no_ground_range_image);
}

bool DepthSegmentLocalization::computeSegmentBlob(MIL_ID range_image, MIL_ID& blob_result, MIL_INT& index, MIL_INT& x_offset, MIL_INT& y_offset)
{
	static const MIL_INT BLOB_MAX_AREA_INDEX = M_BLOB_INDEX(0);

	// @ create b_range_image: binary image without ground information
	MIL_ID b_range_image = MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, M_UNSIGNED + 1, M_IMAGE + M_DISP + M_PROC, M_NULL);
	MbufClear(b_range_image, 1);
	MimBinarize(range_image, b_range_image, M_FIXED + M_NOT_EQUAL, 0, M_NULL);

	// initial search region & clip region of interest to valid
	MIL_DOUBLE roi_x_temp = roi_x_;
	MIL_DOUBLE roi_y_temp = roi_y_;
	MIL_DOUBLE roi_x_size_temp = roi_x_size_;
	MIL_DOUBLE roi_y_size_temp = roi_y_size_;

	roi_x_temp = max(roi_x_temp, 0.0);
	roi_y_temp = max(roi_y_temp, 0.0);

	if (!(roi_x_temp && roi_x_size_temp)) roi_x_size_temp = (MIL_DOUBLE)image_size_x_;
	if (!(roi_y_temp && roi_y_size_temp)) roi_y_size_temp = (MIL_DOUBLE)image_size_y_;
	if (roi_x_temp + roi_x_size_temp > image_size_x_) roi_x_size_temp = (MIL_DOUBLE)image_size_x_ - roi_x_temp;
	if (roi_y_temp + roi_y_size_temp > image_size_y_) roi_y_size_temp = (MIL_DOUBLE)image_size_y_ - roi_y_temp;

	// @ start segment & blob analysis iterations
	MIL_ID best_blob_result = M_NULL;
	MIL_INT best_blob_index = -1;

	MIL_ID segment_search_image = 
		MbufClone(src_2d_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(segment_search_image, 0);
	MbufCopyCond(src_2d_image_, segment_search_image, b_range_image, M_NOT_EQUAL, 0);
	
	for (int i = 0; i < SEGMENT_ITERATION_TIMES; i++)
	{
		// 16*16 is the minimum image size of MmodFind
		if (roi_x_size_temp < 16 || roi_y_size_temp < 16)
			continue;

		// @ do segment model find
		MIL_ID segment_search_roi = MbufChild2d(
			segment_search_image,
			(MIL_INT)roi_x_temp,
			(MIL_INT)roi_y_temp,
			(MIL_INT)roi_x_size_temp,
			(MIL_INT)roi_y_size_temp, M_NULL);

		MIL_ID b_seg_no_ground_image = MbufClone(b_range_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);
		for (auto seg : seg_context_map_)
		{
			MIL_INT nb_model_found;
			MIL_ID seg_result = seg_result_map_[seg.first];
			MmodFind(seg.second, segment_search_roi, seg_result);
			MmodGetResult(seg_result_map_[seg.first], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

			if (nb_model_found > 0)
			{		
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -roi_x_temp);
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -roi_y_temp);
				
				if (DRAW_INTERMEDIUM_RESULT)
				{
					MgraControl(mil_graphic_context_, M_COLOR, M_COLOR_RED);
					MmodDraw(mil_graphic_context_, seg_result, mil_graphic_context_list_, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
				}		

				// @ eliminate segment result from image
				MgraControl(mil_graphic_context_, M_COLOR, 0);
				MmodDraw(mil_graphic_context_, seg_result, b_seg_no_ground_image, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);

				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
			}
		}
		MbufFree(segment_search_roi);

		// @ prepare blob analysis image: erode -> thick -> add operation
		MIL_ID b_process_buf = MbufClone(b_seg_no_ground_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);
		MimErode(b_process_buf, b_process_buf, ERODE_ITERATIONS, M_BINARY);
		MimThick(b_process_buf, b_process_buf, THICK_ITERATIONS, M_BINARY);
		MimArith(b_process_buf, b_seg_no_ground_image, b_seg_no_ground_image, M_AND);
		MbufFree(b_process_buf);
		MimErode(b_seg_no_ground_image, b_seg_no_ground_image, 1, M_BINARY); // erode 1 for easy blob calculate

		// @ do blob analysis
		MIL_ID blob_search_roi = MbufChild2d(
			b_seg_no_ground_image,
			(MIL_INT)roi_x_temp,
			(MIL_INT)roi_y_temp,
			(MIL_INT)roi_x_size_temp,
			(MIL_INT)roi_y_size_temp, M_NULL);

		// @ set up variables to record/filter results
		MIL_DOUBLE max_blob_area = 0;
		best_blob_result = M_NULL;
		best_blob_index = -1;
		// best_blob_results_map --- <string>: patern name; <int>: best result index in blob_result_map_[patern name]
		std::map<std::string, int> best_blob_results_map;	

		for (auto blob : blob_context_map_)
		{
			std::string pattern_name = blob.first;
			MblobCalculate(blob.second, blob_search_roi, M_NULL, blob_result_map_[pattern_name]);

			// @ filter blob area
			double segment_area_min = blob_area_select_[pattern_name].first;
			double segment_area_max = blob_area_select_[pattern_name].second;
			if (segment_area_max > segment_area_min ||
				segment_area_min ||
				segment_area_max)
			{
				if (segment_area_min > 0)
					MblobSelect(blob_result_map_[pattern_name], M_EXCLUDE, M_BOX_AREA, M_LESS, segment_area_min, M_NULL);

				if (segment_area_max > 0)
					MblobSelect(blob_result_map_[pattern_name], M_EXCLUDE, M_BOX_AREA, M_GREATER, segment_area_max, M_NULL);
			}

			// @ check current blob result
			MIL_INT nb_model_found;
			MblobGetResult(blob_result_map_[pattern_name], M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);
			if (nb_model_found > 0)
			{
				if (search_order_ > SearchOrder::BEST_FIRST)
				{
					blob_gravity_center_x_.clear();
					blob_gravity_center_y_.clear();
					MblobGetResult(blob_result_map_[pattern_name], M_DEFAULT, M_CENTER_OF_GRAVITY_X + M_BINARY, blob_gravity_center_x_);
					MblobGetResult(blob_result_map_[pattern_name], M_DEFAULT, M_CENTER_OF_GRAVITY_Y + M_BINARY, blob_gravity_center_y_);

					MIL_INT blob_size_seg_x = image_size_x_ / 16;
					MIL_INT blob_size_seg_y = image_size_y_ / 16;
					double scale = 0.5;

					std::map<int, std::pair<double, double>> sub_blob_poses;
					for (int j = 0; j < nb_model_found; j++)
					{
						MIL_DOUBLE state;
						MblobGetResult(blob_result_map_[pattern_name], M_BLOB_INDEX(j), M_BLOB_INCLUSION_STATE, &state);
						if (state != M_INCLUDED) continue;

						sub_blob_poses.emplace(j,
							std::pair<double, double>(
								blob_gravity_center_x_[j],
								blob_gravity_center_y_[j]));
					}

					if (sub_blob_poses.size() != 0)
					{
						std::vector<int> blob_result_order_vec;
						blob_result_order_vec.reserve(sub_blob_poses.size());

						common_utils::composeSearchOrder(
							(double)image_size_x_,
							(double)image_size_y_,
							(double)blob_size_seg_x,
							(double)blob_size_seg_y,
							scale,
							sub_blob_poses,
							search_order_,
							blob_result_order_vec);

						best_blob_results_map.emplace(pattern_name, blob_result_order_vec[0]);
					}					
				}
				else
				{
					MIL_DOUBLE state;
					MblobGetResult(blob_result_map_[pattern_name], BLOB_MAX_AREA_INDEX, M_BLOB_INCLUSION_STATE, &state);
					if (state != M_INCLUDED) continue;

					MIL_DOUBLE blob_area;
					MblobGetResult(blob_result_map_[pattern_name], BLOB_MAX_AREA_INDEX, M_BOX_AREA, &blob_area);

					if (blob_area > max_blob_area)
					{
						max_blob_area = blob_area;
						best_blob_result = blob_result_map_[pattern_name];						
					}
				}
			}
		}

		// @ set best blob index & result
		if(search_order_ > SearchOrder::BEST_FIRST && best_blob_results_map.size() > 0)
		{
			std::map<int, std::pair<double, double>> best_blob_poses;
			int index = 0;
			for (auto blob_result : best_blob_results_map)
			{
				MIL_DOUBLE blob_gravity_center_x;
				MIL_DOUBLE blob_gravity_center_y;
				MblobGetResult(blob_result_map_[blob_result.first], M_BLOB_INDEX(blob_result.second), M_CENTER_OF_GRAVITY_X + M_BINARY, &blob_gravity_center_x);
				MblobGetResult(blob_result_map_[blob_result.first], M_BLOB_INDEX(blob_result.second), M_CENTER_OF_GRAVITY_Y + M_BINARY, &blob_gravity_center_y);

				best_blob_poses.emplace(index,
					std::pair<double, double>(
						blob_gravity_center_x,
						blob_gravity_center_y));

				index++;
			}

			MIL_INT blob_size_seg_x = image_size_x_ / 16;
			MIL_INT blob_size_seg_y = image_size_y_ / 16;
			double scale = 0.5;

			std::vector<int> best_blob_order_vec;
			best_blob_order_vec.reserve(best_blob_results_map.size());

			common_utils::composeSearchOrder(
				(double)image_size_x_,
				(double)image_size_y_,
				(double)blob_size_seg_x,
				(double)blob_size_seg_y,
				scale,
				best_blob_poses,
				search_order_,
				best_blob_order_vec);

			index = 0;
			for (auto blob_result : best_blob_results_map)
			{
				if (index == best_blob_order_vec[0])
				{
					best_blob_result = blob_result_map_[blob_result.first];
					best_blob_index = M_BLOB_INDEX(best_blob_results_map[blob_result.first]);
					break;
				}
				index++;
			}
		}
		else
		{
			best_blob_index = BLOB_MAX_AREA_INDEX;
		}

		// @ draw sub results
		if (DRAW_INTERMEDIUM_RESULT)
		{
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -roi_x_temp);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -roi_y_temp);

			for (auto blob_result : blob_result_map_)
			{
				MIL_INT nb_model_found;
				MblobGetResult(blob_result.second, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

				for (int k = 0; k < nb_model_found; k++)
				{
					MIL_INT blob_index = M_BLOB_INDEX(k);
					MIL_DOUBLE state;
					MblobGetResult(blob_result.second, blob_index, M_BLOB_INCLUSION_STATE, &state);
					if (state == M_INCLUDED && !(blob_result.second == best_blob_result && blob_index == best_blob_index))
					{
						MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
						MblobDraw(mil_graphic_context_, blob_result.second,
							mil_graphic_context_list_, M_DRAW_CENTER_OF_GRAVITY + M_DRAW_BLOBS_CONTOUR, blob_index, M_DEFAULT);
					}
				}
			}

			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
		}
		

		// @ free memory
		MbufFree(blob_search_roi);
		MbufFree(b_seg_no_ground_image);

		// @ update region of interest or show final result
		if (best_blob_result != M_NULL)
		{
			if (i != SEGMENT_ITERATION_TIMES - 1)
			{
				MIL_DOUBLE old_roi_x_start = roi_x_temp;
				MIL_DOUBLE old_roi_y_start = roi_y_temp;
				MIL_DOUBLE old_roi_x_end = old_roi_x_start + roi_x_size_temp;
				MIL_DOUBLE old_roi_y_end = old_roi_y_start + roi_y_size_temp;

				MIL_DOUBLE box_x_min, box_x_max, box_y_min, box_y_max;
				MblobGetResult(best_blob_result, best_blob_index, M_BOX_X_MIN, &box_x_min);
				MblobGetResult(best_blob_result, best_blob_index, M_BOX_X_MAX, &box_x_max);
				MblobGetResult(best_blob_result, best_blob_index, M_BOX_Y_MIN, &box_y_min);
				MblobGetResult(best_blob_result, best_blob_index, M_BOX_Y_MAX, &box_y_max);

				box_x_min += old_roi_x_start;
				box_x_max += old_roi_x_start;
				box_y_min += old_roi_y_start;
				box_y_max += old_roi_y_start;

				roi_x_temp = box_x_min < old_roi_x_start ? old_roi_x_start : box_x_min;
				roi_y_temp = box_y_min < old_roi_y_start ? old_roi_y_start : box_y_min;
				roi_x_size_temp = box_x_max > old_roi_x_end ? old_roi_x_end - roi_x_temp : box_x_max - roi_x_temp;
				roi_y_size_temp = box_y_max > old_roi_y_end ? old_roi_y_end - roi_y_temp : box_y_max - roi_y_temp;
			}
		}
		else
		{
			break;
		}
	}

	MbufFree(segment_search_image);
	MbufFree(b_range_image);

	// @ handle final result
	if (best_blob_result != M_NULL)
	{
		blob_result = best_blob_result;
		index = best_blob_index;
		x_offset = (MIL_INT)roi_x_temp;
		y_offset = (MIL_INT)roi_y_temp;

		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -x_offset);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -y_offset);
		MgraColor(mil_graphic_context_, M_COLOR_GREEN);
		MblobDraw(mil_graphic_context_, blob_result, mil_graphic_context_list_, M_DRAW_CENTER_OF_GRAVITY + M_DRAW_BLOBS_CONTOUR, index, M_DEFAULT);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -x_offset);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -y_offset);

		return true;
	}
	else
		return false;
}

bool DepthSegmentLocalization::computePoseFromBlobResult(SPoseDataQuat& pose, MIL_ID range_image, MIL_ID blob_result,
	MIL_INT blob_index, MIL_INT x_offset, MIL_INT y_offset)
{
	// @ create blob range image from result
	MIL_ID blob_contour_image = MbufClone(range_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MIL_ID blob_range_image = MbufClone(range_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
	MbufClear(blob_contour_image, 0);
	MbufClear(blob_range_image, 0);

	MIL_DOUBLE blob_gravity_center_x;
	MIL_DOUBLE blob_gravity_center_y;
	MblobGetResult(blob_result, blob_index, M_CENTER_OF_GRAVITY_X + M_BINARY, &blob_gravity_center_x);
	MblobGetResult(blob_result, blob_index, M_CENTER_OF_GRAVITY_Y + M_BINARY, &blob_gravity_center_y);

	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -x_offset);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -y_offset);

	MgraColor(mil_graphic_context_, 1);
	MblobDraw(mil_graphic_context_, blob_result, blob_contour_image, M_DRAW_BLOBS_CONTOUR, blob_index, M_DEFAULT);
	MgraFill(mil_graphic_context_, blob_contour_image, blob_gravity_center_x, blob_gravity_center_y);
	MbufCopyCond(range_image, blob_range_image, blob_contour_image, M_NOT_EQUAL, 0);
	MbufFree(blob_contour_image);

	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
	MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);

	// @ compute transition from neighbor smooth gaussian
	MIL_DOUBLE box_x_min, box_x_max, box_y_min, box_y_max;
	MblobGetResult(blob_result, blob_index, M_BOX_X_MIN, &box_x_min);
	MblobGetResult(blob_result, blob_index, M_BOX_X_MAX, &box_x_max);
	MblobGetResult(blob_result, blob_index, M_BOX_Y_MIN, &box_y_min);
	MblobGetResult(blob_result, blob_index, M_BOX_Y_MAX, &box_y_max);

	MIL_DOUBLE box_size_x = box_x_max - box_x_min;
	MIL_DOUBLE box_size_y = box_y_max - box_y_min;

	static const MIL_DOUBLE max_area = 320 * 180;

	// clip size to speed up computation
	if (box_size_x * box_size_y > max_area)
	{
		MIL_DOUBLE ratio = max_area / (box_size_x * box_size_y);

		MIL_DOUBLE x_offset = box_size_x * (1 - ratio) / 2.0;
		MIL_DOUBLE y_offset = box_size_y * (1 - ratio) / 2.0;

		box_x_min += x_offset;
		box_x_max -= x_offset;
		box_y_min += y_offset;
		box_y_max -= y_offset;
	}

	if (!depth_camera_data_wrapper_->get3DPointUsingNeighborFilter(
		(int)(blob_gravity_center_x + x_offset),
		(int)(blob_gravity_center_y + y_offset),
		pose.x,
		pose.y,
		pose.z,
		blob_range_image,
		(int)(blob_gravity_center_x - box_x_min),
		(int)(box_x_max - blob_gravity_center_x),
		(int)(blob_gravity_center_y - box_y_min),
		(int)(box_y_max - blob_gravity_center_y)))
	{
		MbufFree(blob_range_image);
		return false;
	}

	// compute rotation from 3d plane fit
	MIL_ID pc_label = 1;
	if (!depth_camera_data_wrapper_->createDepthMap(blob_range_image, M_NULL, M_NULL, 
		pc_label, 0, 0, false, false, false))
	{
		return false;
	}

	MIL_ID geometry_model_plane = M3dmapAlloc(mil_system_, M_GEOMETRY, M_DEFAULT, M_NULL);
	M3dmapSetGeometry(geometry_model_plane, M_PLANE, M_FIT_POINT_CLOUD,
		static_cast<MIL_DOUBLE>(depth_camera_data_wrapper_->getPointCloudID()),
		M_EXCLUDE_INVALID_POINTS, M_DEFAULT,
		static_cast<MIL_DOUBLE>(PC_LABEL(pc_label)), M_DEFAULT);
	compute3DPLaneRotation(geometry_model_plane, pose, 0);
	M3dmapFree(geometry_model_plane);

	MbufFree(blob_range_image);
	return true;
}