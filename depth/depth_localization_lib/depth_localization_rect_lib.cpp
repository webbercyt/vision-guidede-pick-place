#include "depth_localization_rect_lib.h"
#include "utils.h"
#include "depth_camera.h"

#define DEBUG_TIME /*1*/0

RectCandinate::RectCandinate()
{
	result_2d = M_NULL;
	result_2d_index = -1;
	result_depth = M_NULL;
	result_depth_index = -1;
	pc_label = -2;
	offset_x_2d = 0;
	offset_y_2d = 0;
}

RectCandinate::RectCandinate(
	MIL_ID _result_2d,
	MIL_INT _result_2d_index,
	MIL_ID _result_depth,
	MIL_INT _result_depth_index,
	MIL_INT _pc_label)
{
	result_2d = _result_2d;
	result_2d_index = _result_2d_index;
	result_depth = _result_depth;
	result_depth_index = _result_depth_index;
	pc_label = _pc_label;

	offset_x_2d = 0;
	offset_y_2d = 0;
}

RectCandinate RectCandinate::operator = (RectCandinate rc)
{
	return rc;
}

DepthRectLocalization::DepthRectLocalization(MIL_ID mil_application,
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
	setupModSegment();
}

DepthRectLocalization::DepthRectLocalization(MIL_ID mil_application,
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
	setupModSegment();
}

DepthRectLocalization::~DepthRectLocalization()
{
	MmodFree(mil_context_segment_);
	MmodFree(mil_result_segment_);
}

void DepthRectLocalization::setContextFromPattern(
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

	MmodControl(mod_context, M_CONTEXT, M_SMOOTHNESS, pattern.m_smoothness());
	MmodControl(mod_context, M_CONTEXT, M_TIMEOUT, pattern.m_timeout());
	MmodControl(mod_context, M_DEFAULT, M_ACCEPTANCE, pattern.m_acceptance());
	MmodControl(mod_context, M_DEFAULT, M_CERTAINTY, pattern.m_certainty());
	MmodControl(mod_context, M_DEFAULT, M_NUMBER, control_number);

	//if (control_number > 1)
	MmodControl(mod_context, M_CONTEXT, M_SHARED_EDGES, M_ENABLE);
	MmodControl(mod_context, M_DEFAULT, M_MODEL_ASPECT_RATIO_MIN_FACTOR, pattern.depth_feature().aspect_ratio_min());
	MmodControl(mod_context, M_DEFAULT, M_MODEL_ASPECT_RATIO_MAX_FACTOR, pattern.depth_feature().aspect_ratio_max());
	MmodControl(mod_context, M_DEFAULT, M_SCALE_MIN_FACTOR, pattern.search_scale_min());
	MmodControl(mod_context, M_DEFAULT, M_SCALE_MAX_FACTOR, pattern.search_scale_max());;
}

void DepthRectLocalization::setupModSegment()
{
	// get rectangle pixel width & height & scale to define segment length
	std::string feature_name_2d = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_2d() + "1";
	MIL_DOUBLE width, height;
	MmodInquire(mod_context_map_2d_[feature_name_2d], M_DEFAULT, M_WIDTH, &width);
	MmodInquire(mod_context_map_2d_[feature_name_2d], M_DEFAULT, M_HEIGHT, &height);

	auto aspect_min = workspace_ptr_->feature_group().at(feature_group_name_).depth_shape_feature_group().aspect_ratio_min();
	auto aspect_max = workspace_ptr_->feature_group().at(feature_group_name_).depth_shape_feature_group().aspect_ratio_max();
	auto scale_min = workspace_ptr_->feature_group().at(feature_group_name_).depth_shape_feature_group().search_scale_min();
	auto scale_max = workspace_ptr_->feature_group().at(feature_group_name_).depth_shape_feature_group().search_scale_max();

	double width_min = width * scale_min;
	double width_max = width * scale_max;
	double height_min = height / aspect_max * scale_min;
	double height_max = height / aspect_min * scale_max;

	double length_min = min(width_min, height_min);
	double length_max = max(width_max, height_max);
	int length = (int)(length_min + length_max) / 2;

	MmodAlloc(mil_system_, M_SHAPE_SEGMENT, M_DEFAULT, &mil_context_segment_);
	MmodDefine(mil_context_segment_, M_SEGMENT, /*100*/length, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	MmodControl(mil_context_segment_, M_DEFAULT, M_SCALE_MIN_FACTOR, length_min / length);
	MmodControl(mil_context_segment_, M_DEFAULT, M_SCALE_MAX_FACTOR, /*5.0*/length_max / length);
	MmodControl(mil_context_segment_, M_DEFAULT, M_NUMBER, M_ALL);

	MIL_DOUBLE detial_lvl;
	MmodInquire(mod_context_map_2d_[feature_name_2d], M_CONTEXT, M_DETAIL_LEVEL, &detial_lvl);
	MmodControl(mil_context_segment_, M_CONTEXT, M_DETAIL_LEVEL, detial_lvl);

	MmodPreprocess(mil_context_segment_, M_DEFAULT);
	MmodAllocResult(mil_system_, M_SHAPE_SEGMENT, &mil_result_segment_);
}

void DepthRectLocalization::getModelSquareROIFromResult(MIL_ID mod_result, MIL_INT mod_index, MIL_ID image,
	MIL_INT& roi_x, MIL_INT& roi_y, MIL_INT& roi_size_x, MIL_INT& roi_size_y)
{
	MIL_DOUBLE height, width;
	MmodGetResult(mod_result, mod_index, M_WIDTH, &width);
	MmodGetResult(mod_result, mod_index, M_HEIGHT, &height);

	MIL_DOUBLE center_pos_x, center_pos_y;
	MIL_DOUBLE z_axis_angle;
	MmodGetResult(mod_result, mod_index, M_POSITION_X, &center_pos_x);
	MmodGetResult(mod_result, mod_index, M_POSITION_Y, &center_pos_y);
	MmodGetResult(mod_result, mod_index, M_ANGLE, &z_axis_angle);

	// set ROI
	MIL_DOUBLE half_side_1;
	MIL_DOUBLE half_side_2;
	MIL_DOUBLE radius = height / 2;
	half_side_1 = abs(radius * cos(z_axis_angle * DEG_TO_RAD));
	half_side_2 = abs(radius * sin(z_axis_angle * DEG_TO_RAD));
	MIL_DOUBLE half_side = max(half_side_1, half_side_2);

	MIL_DOUBLE  pixel_size_x = 1, pixel_size_y = 1;
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

MIL_DOUBLE DepthRectLocalization::computeOccMeanElevation(MIL_ID mod_context, MIL_ID mod_result, MIL_INT index)
{
	MIL_ID pc = depth_camera_data_wrapper_->getPointCloudID();

	SDepthData depth_data = depth_camera_data_wrapper_->getDepthData();
	MIL_ID depth_map_buf =
		MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, M_UNSIGNED + 16, M_IMAGE + M_DISP + M_PROC, M_NULL);
	MbufClear(depth_map_buf, 0);

	// Extract the top-view depth-map of the scene fixtured on the occurrence.
	M3dmapExtract(pc, depth_map_buf, M_NULL, M_CORRECTED_DEPTH_MAP, /*M_ALL*/PC_LABEL(index + 1), M_DEFAULT);
	MbufClearCond(depth_map_buf, 0.0, M_NULL, M_NULL, depth_map_buf, M_EQUAL, 65535);

	// Compute occurrence's mean elevation in the scene only from the pixels
	// enabled by the model's mask.
	MIL_DOUBLE occ_mean_elevation;
	M3dmapStat(depth_map_buf, M_NULL, M_NULL, M_NULL, M_DEVIATION_MEAN + M_STAT_ALL, M_DEFAULT, M_DEFAULT, &occ_mean_elevation);

	MbufFree(depth_map_buf);
	return occ_mean_elevation;
}

MIL_DOUBLE DepthRectLocalization::getModelZAngleFromResult(MIL_ID mod_result, MIL_INT mod_index)
{
	MIL_DOUBLE z_axis_angle;
	MmodGetResult(mod_result, mod_index, M_ANGLE, &z_axis_angle);

	return z_axis_angle;
}

std::set<MIL_ID> DepthRectLocalization::deleteOverlap(MIL_ID mod_result)
{
	static const int ERODE_ITERATION = 2;

	std::set<MIL_ID> ret;
	MIL_INT nb_model_found = 0;
	MmodGetResult(mod_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

	// sort index by area
	std::multimap<MIL_DOUBLE, MIL_ID> area_map;
	for (int i = 0; i < nb_model_found; i++)
	{
		MIL_DOUBLE width, height;
		MmodGetResult(mod_result, i, M_WIDTH, &width);
		MmodGetResult(mod_result, i, M_HEIGHT, &height);
		area_map.emplace(width * height, i);
	}

	MIL_INT image_size_x = MbufInquire(src_2d_image_, M_SIZE_X, M_NULL);
	MIL_INT image_size_y = MbufInquire(src_2d_image_, M_SIZE_Y, M_NULL);

	MIL_ID filter = MbufAlloc2d(mil_system_, image_size_x, image_size_y, M_UNSIGNED + 1, M_IMAGE + M_DISP + M_PROC, M_NULL);
	MbufClear(filter, 0);

	MIL_ID mil_stat_context = MimAlloc(mil_system_, M_STATISTICS_CONTEXT, M_DEFAULT, M_NULL);
	MIL_ID mil_stat_result = MimAllocResult(mil_system_, M_DEFAULT, M_STATISTICS_RESULT, M_NULL);
	MimControl(mil_stat_context, M_STAT_SUM, M_ENABLE);

	MIL_DOUBLE sum_filter = 0;
	for (auto it : area_map)
	{
		MIL_DOUBLE center_x, center_y;
		MmodGetResult(mod_result, it.second, M_CENTER_X, &center_x);
		MmodGetResult(mod_result, it.second, M_CENTER_Y, &center_y);

		// draw to filter image
		MIL_ID temp = MbufAlloc2d(mil_system_, image_size_x, image_size_y, M_UNSIGNED + 1, M_IMAGE + M_DISP + M_PROC, M_NULL);
		MbufClear(temp, 0);
		MgraColor(M_DEFAULT, 1);
		MmodDraw(M_DEFAULT, mod_result, temp, M_DRAW_EDGES, it.second, M_DEFAULT);
		MgraFill(M_DEFAULT, temp, center_x, center_y);
		MimErode(temp, temp, ERODE_ITERATION, M_BINARY);

		MimStatCalculate(mil_stat_context, temp, mil_stat_result, M_DEFAULT);
		MIL_DOUBLE sum_before = 0;
		MimGetResult(mil_stat_result, M_STAT_SUM, &sum_before);

		MimArith(filter, temp, temp, M_OR);
		MimStatCalculate(mil_stat_context, temp, mil_stat_result, M_DEFAULT);
		MIL_DOUBLE sum_after = 0;
		MimGetResult(mil_stat_result, M_STAT_SUM, &sum_after);

		if ((sum_after - sum_filter) >= /*sum_before -*/ sum_before * /*0.001*/0.999)
		{
			MimArith(filter, temp, filter, M_OR);
			sum_filter = sum_after;

			ret.emplace(it.second);
		}

		MbufFree(temp);
	}

	MimFree(mil_stat_context);
	MimFree(mil_stat_result);
	MbufFree(filter);

	return ret;
}

MIL_INT DepthRectLocalization::locateModel2D(MIL_ID mil_2d_image, std::string& found_model_name)
{
	// do segment before model find	
	MmodFind(mil_context_segment_, mil_2d_image, mil_result_segment_);
	MIL_INT nb_segments;
	MmodGetResult(mil_result_segment_, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_segments);
	if (nb_segments > 0)
	{
		std::string feature_name_2d = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().feature_name_2d() + "1";
		MIL_DOUBLE foreground;
		MmodInquire(mod_context_map_2d_[feature_name_2d], M_DEFAULT, M_FOREGROUND_VALUE, &foreground);

		// consider foreground color later
		if(foreground == M_FOREGROUND_BLACK || foreground == M_DEFAULT)
			MgraColor(mil_graphic_context_, 255);
		else if(foreground == M_FOREGROUND_WHITE)
			MgraColor(mil_graphic_context_, 0);

		MmodDraw(mil_graphic_context_, mil_result_segment_, mil_2d_image, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
	}

	if (mod_result_order_vec_.size() > 0)
		mod_result_order_vec_.clear();

	MIL_INT nb_model_found = 0;
	for (auto it : mod_context_map_2d_)
	{
		MIL_ID mod_context = it.second;
		MIL_ID mod_result = mod_result_map_2d_[it.first];

		MmodFind(mod_context, mil_2d_image, mod_result);
		MmodGetResult(mod_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model_found);

		if (nb_model_found >= 1)
		{
			found_model_name = it.first;

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

	return nb_model_found;
}

bool DepthRectLocalization::runLocateMethod2dDepth()
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

#if DEBUG_TIME
	clock_t t_total = clock();
	clock_t t = clock();
#endif

	std::string found_model_name_2d;

	MIL_ID src_2d_image = MbufClone(src_2d_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);
	
	MIL_ID image_buf;
	if (isROIValid())
		image_buf = MbufChild2d(src_2d_image, roi_x_, roi_y_, roi_x_size_, roi_y_size_, M_NULL);
	else
		image_buf = MbufClone(src_2d_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);
	
	MIL_INT nb_model_found = locateModel2D(image_buf, found_model_name_2d);
	if (!nb_model_found)
	{
		if (is_defining_model_) {
			std::string o_msg = printCerrMsg(LEVEL_CUSTOMER_VISIABLE, translation::messageOut(translation::message::feature_bad_2d_model)); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}
		return false;
	}

#if DEBUG_TIME
	t = clock() - t;
	printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "locateModel2D: " + std::to_string(((float)t) / CLOCKS_PER_SEC) + " s");
#endif

	std::set<MIL_ID> valid_2d_index;
	std::map<int, RectCandinate> rect_candidates; // int: new id; MIL_ID: result; MIL_INT: index in result

	std::vector<MIL_ID> result_2d;
	std::vector<MIL_ID> result_depth;

#if DEBUG_TIME
	t = clock();
#endif
	int new_index = 0;
	if (nb_model_found > 0)
	{
		valid_2d_index = deleteOverlap(mod_result_map_2d_[found_model_name_2d]);
	}
#if DEBUG_TIME
	t = clock() - t;
	printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "deleteOverlap: " + std::to_string(((float)t) / CLOCKS_PER_SEC) + " s");
#endif

#if DEBUG_TIME
	t = clock();
#endif
	if (valid_2d_index.size() > 0)
	{
		MIL_INT64 mod_type_depth;
		MmodInquire(mod_context_map_2d_[found_model_name_2d], M_DEFAULT, M_CONTEXT_TYPE + M_TYPE_MIL_INT64, &mod_type_depth);
		auto const model_type = *ModelProcess::model_context_result_type_map.find(mod_type_depth);
		MIL_INT64 mod_result_type = model_type.second;

		for (auto index : valid_2d_index)
		{
			// do segment before model find
			MIL_ID sub_image = MbufClone(src_2d_image, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_COPY_SOURCE_DATA, M_NULL);

			MIL_INT image_size_x = MbufInquire(image_buf, M_SIZE_X, M_NULL);
			MIL_INT image_size_y = MbufInquire(image_buf, M_SIZE_Y, M_NULL);

			MIL_DOUBLE center_x, center_y;
			MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_CENTER_X, &center_x);
			MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_CENTER_Y, &center_y);

			// new roi
			//compute 4 corners of rectangle
			MIL_DOUBLE sub_width, sub_height, sub_angle;
			MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_WIDTH, &sub_width);
			MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_HEIGHT, &sub_height);
			MmodGetResult(mod_result_map_2d_[found_model_name_2d], index, M_ANGLE, &sub_angle);

			MIL_DOUBLE x_old[4];
			MIL_DOUBLE y_old[4];

			x_old[0] = center_x - sub_width / 2;
			y_old[0] = center_y - sub_height / 2;
			x_old[1] = center_x + sub_width / 2;
			y_old[1] = y_old[0];
			x_old[2] = x_old[1];
			y_old[2] = center_y + sub_height / 2;
			x_old[3] = x_old[0];
			y_old[3] = y_old[2];

			MIL_DOUBLE x_new[4];
			MIL_DOUBLE y_new[4];

			for (int i = 0; i < 4; i++)
			{
				x_new[i] = center_x + (x_old[i] - center_x) * cos(sub_angle / PI)
					+ (y_old[i] - center_y) * sin(sub_angle / PI);

				y_new[i] = center_y + (x_old[i] - center_x) * sin(sub_angle / PI)
					+ (y_old[i] - center_y) * cos(sub_angle / PI);
			}

			MIL_DOUBLE x_min = min(min(x_new[0], x_new[1]), min(x_new[2], x_new[3]));
			MIL_DOUBLE x_max = max(max(x_new[0], x_new[1]), max(x_new[2], x_new[3]));
			MIL_DOUBLE y_min = min(min(y_new[0], y_new[1]), min(y_new[2], y_new[3]));
			MIL_DOUBLE y_max = max(max(y_new[0], y_new[1]), max(y_new[2], y_new[3]));

			MIL_INT sub_margin = 16;
			MIL_INT offset_x = (MIL_INT)x_min - sub_margin + roi_x_;
			MIL_INT offset_y = (MIL_INT)y_min - sub_margin + roi_y_;
			MIL_ID roi = MbufChild2d(sub_image,
				offset_x,
				offset_y,
				(MIL_INT)(x_max - x_min) + sub_margin * 2,
				(MIL_INT)(y_max - y_min) + sub_margin * 2,
				M_NULL);

			MmodFind(mil_context_segment_, roi, mil_result_segment_);
			MIL_INT nb_seg_found;
			MmodGetResult(mil_result_segment_, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_seg_found);

			if (nb_seg_found > 0)
			{
				MIL_DOUBLE foreground;
				MmodInquire(mod_context_map_2d_[found_model_name_2d], M_DEFAULT, M_FOREGROUND_VALUE, &foreground);

				// consider foreground color later
				if (foreground == M_FOREGROUND_BLACK || foreground == M_DEFAULT)
					MgraColor(mil_graphic_context_, 255);
				else if (foreground == M_FOREGROUND_WHITE)
					MgraColor(mil_graphic_context_, 0);

				MmodDraw(mil_graphic_context_, mil_result_segment_, roi, M_DRAW_EDGES, M_DEFAULT, M_DEFAULT);
			}

			// 2d model find
			MIL_ID mod_result = MmodAllocResult(mil_system_, mod_result_type, M_NULL);
			MmodFind(mod_context_map_2d_[found_model_name_2d], roi, mod_result);
			MIL_INT nb_model;
			MmodGetResult(mod_result, M_DEFAULT, M_NUMBER + M_TYPE_MIL_INT, &nb_model);

			MbufFree(roi);
			MbufFree(sub_image);

			if (nb_model > 0)
			{
				result_2d.push_back(mod_result);

				std::set<MIL_ID> index_set = deleteOverlap(mod_result);
				for (auto _index : index_set)
				{
					MIL_ID mod_result_depth = MmodAllocResult(mil_system_, mod_result_type, M_NULL);
					result_depth.push_back(mod_result_depth);

					RectCandinate candidate = RectCandinate(mod_result, _index, mod_result_depth, -1, new_index);
					candidate.offset_x_2d = (int)offset_x;
					candidate.offset_y_2d = (int)offset_y;

					rect_candidates.emplace(new_index, candidate);
					new_index++;

					if (!is_defining_model_) 
					{
						if (mil_graphic_context_)
						{
							MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -candidate.offset_x_2d);
							MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -candidate.offset_y_2d);

							MgraColor(mil_graphic_context_, M_COLOR_RED);
							MmodDraw(mil_graphic_context_,
								mod_result,
								mil_graphic_context_list_,
								M_DRAW_EDGES + M_DRAW_POSITION,
								_index,
								M_DEFAULT);

							MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
							MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
						}
					}
				}
			}
			else
			{
				MmodFree(mod_result);
			}
		}
	}
#if DEBUG_TIME
	t = clock() - t;
	printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Pick candidate: " + std::to_string(((float)t) / CLOCKS_PER_SEC) + " s");
#endif

	SPoseDataQuat pose;
	MIL_DOUBLE error;

	// select the needed depth model
	std::multimap<MIL_DOUBLE, int> mod_depth_index_map; // key: depth; value: new_index

#if DEBUG_TIME
	t = clock();
#endif
	depth_camera_data_wrapper_->clearPointCloud();
	for (auto candidate : rect_candidates)
	{
		// create depth map & do depth model find
		int roi_x = roi_x_;
		int roi_y = roi_y_;

		roi_x_ = candidate.second.offset_x_2d;
		roi_y_ = candidate.second.offset_y_2d;

		if (!createModelDepthMap(
			candidate.second.result_2d, 
			candidate.second.result_2d_index, 
			candidate.second.pc_label))
			break;

		roi_x_ = roi_x;
		roi_y_ = roi_y;

		bool has_result = false;
		// try to locate model in depth map 
		for (auto it : mod_context_map_depth_)
		{
			it.second;
			if (locateModelDepth(image_depth_, it.second, candidate.second.result_depth) > 0)
			{
				has_result = true;

				// compute mean depth
				MIL_DOUBLE occ_mean_elevation = computeOccMeanElevation(it.second, candidate.second.result_depth, candidate.second.pc_label);
				mod_depth_index_map.emplace(occ_mean_elevation, candidate.first);

				// display 2d result
				if (!is_defining_model_)
				{
					MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -candidate.second.offset_x_2d);
					MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -candidate.second.offset_y_2d);

					MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
					MmodDraw(mil_graphic_context_,
						candidate.second.result_2d,
						mil_graphic_context_list_,
						M_DRAW_EDGES + M_DRAW_POSITION,
						candidate.second.result_2d_index,
						M_DEFAULT);

					MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
					MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
				}

				break;
			}
		}

		// compute depth for filter
		if (!depth_order_pick_ && has_result)
			break;
	}
#if DEBUG_TIME
	t = clock() - t;
	printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Filter candidate: " + std::to_string(((float)t) / CLOCKS_PER_SEC) + " s");
#endif

#if DEBUG_TIME
	t = clock();
#endif
	//@ compute the model pose
	MIL_DOUBLE rotate_angle = M_INVALID;
	if (mod_depth_index_map.size() > 0)
	{
		int index = mod_depth_index_map.begin()->second;
		auto candidate = rect_candidates[index];

		{
			MIL_ID range_image = MbufClone(src_range_image_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);
			MIL_ID cond = MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, 1 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
			MbufClear(cond, 0);

			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -candidate.offset_x_2d);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -candidate.offset_y_2d);
			MgraColor(mil_graphic_context_, 255);
			MmodDraw(mil_graphic_context_, candidate.result_2d, cond, M_DRAW_EDGES, candidate.result_2d_index, M_DEFAULT);
			MIL_DOUBLE center_x, center_y;
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_CENTER_X, &center_x);
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_CENTER_Y, &center_y);
			MgraFill(mil_graphic_context_, cond, center_x, center_y);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
			MbufCopyCond(src_range_image_, range_image, cond, M_NOT_EQUAL, 0);

			// new roi
			//compute 4 corners of rectangle
			MIL_DOUBLE sub_width, sub_height, sub_angle;
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_WIDTH, &sub_width);
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_HEIGHT, &sub_height);
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_ANGLE, &sub_angle);

			MIL_DOUBLE x_old[4];
			MIL_DOUBLE y_old[4];

			x_old[0] = center_x - sub_width / 2;
			y_old[0] = center_y - sub_height / 2;
			x_old[1] = center_x + sub_width / 2;
			y_old[1] = y_old[0];
			x_old[2] = x_old[1];
			y_old[2] = center_y + sub_height / 2;
			x_old[3] = x_old[0];
			y_old[3] = y_old[2];

			MIL_DOUBLE x_new[4];
			MIL_DOUBLE y_new[4];

			for (int i = 0; i < 4; i++)
			{
				x_new[i] = center_x + (x_old[i] - center_x) * cos(sub_angle / PI)
					+ (y_old[i] - center_y) * sin(sub_angle / PI);

				y_new[i] = center_y + (x_old[i] - center_x) * sin(sub_angle / PI)
					+ (y_old[i] - center_y) * cos(sub_angle / PI);
			}

			MIL_DOUBLE x_min = min(min(x_new[0], x_new[1]), min(x_new[2], x_new[3]));
			MIL_DOUBLE x_max = max(max(x_new[0], x_new[1]), max(x_new[2], x_new[3]));
			MIL_DOUBLE y_min = min(min(y_new[0], y_new[1]), min(y_new[2], y_new[3]));
			MIL_DOUBLE y_max = max(max(y_new[0], y_new[1]), max(y_new[2], y_new[3]));

			if (!depth_camera_data_wrapper_->get3DPointUsingNeighborFilter(
				(int)center_x + candidate.offset_x_2d,
				(int)center_y + candidate.offset_y_2d,
				pose.x,
				pose.y,
				pose.z,
				range_image,
				(int)((x_max - x_min) / 2),
				(int)((x_max - x_min) / 2),
				(int)((y_max - y_min) / 2),
				(int)((y_max - y_min) / 2)))
			{
				return false;
			}
			MbufFree(range_image);

			// do 3d plane fit
			MIL_ID geometry_model_plane = M3dmapAlloc(mil_system_, M_GEOMETRY, M_DEFAULT, M_NULL);
			M3dmapSetGeometry(geometry_model_plane, M_PLANE, M_FIT_POINT_CLOUD,
				static_cast<MIL_DOUBLE>(depth_camera_data_wrapper_->getPointCloudID()),
				M_INCLUDE_POINTS_INSIDE_BOX_ONLY, M_DEFAULT,
				/*M_ALL*/static_cast<MIL_DOUBLE>(PC_LABEL(candidate.pc_label + 1)), M_DEFAULT);
			M3dmapInquire(geometry_model_plane, M_DEFAULT, M_FIT_RMS_ERROR, &error);

			rotate_angle = compute3DPLaneRotation(geometry_model_plane, pose, sub_angle);
			M3dmapFree(geometry_model_plane);
		}

		//printf("@ move: [%f, %f, %f] [%f, %f, %f]\n", tx, ty, tz, rx, ry, rz);
		std::string msg = "Pose: [" 
			+ std::to_string(pose.x) + ", "
			+ std::to_string(pose.y) + ", " 
			+ std::to_string(pose.z) + ", "
			+ std::to_string(pose.q1) + ", "
			+ std::to_string(pose.q2) + ", "
			+ std::to_string(pose.q3) + ", "
			+ std::to_string(pose.q4) + "]";
		std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, msg); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;

		if (rotate_angle != M_INVALID)
		{
			// re-draw depth image & display result
			int roi_x = roi_x_;
			int roi_y = roi_y_;

			roi_x_ = candidate.offset_x_2d;
			roi_y_ = candidate.offset_y_2d;

			createModelDepthMap(candidate.result_2d, candidate.result_2d_index, candidate.pc_label);
			roi_x_ = roi_x;
			roi_y_ = roi_y;

			if (mil_graphic_context_)
			{
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -candidate.offset_x_2d);
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -candidate.offset_y_2d);

				MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
				MmodDraw(mil_graphic_context_, candidate.result_2d, mil_graphic_context_list_, M_DRAW_EDGES, candidate.result_2d_index, M_DEFAULT);
				MgraColor(mil_graphic_context_, M_COLOR_GREEN);
				MmodDraw(mil_graphic_context_, candidate.result_2d, mil_graphic_context_list_, M_DRAW_BOX + M_DRAW_POSITION, candidate.result_2d_index, M_DEFAULT);
				
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
				MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
			}

			display3DResult(candidate.result_depth, M_DEFAULT);

			MIL_DOUBLE center_pos_x, center_pos_y;
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_POSITION_X, &center_pos_x);
			MmodGetResult(candidate.result_2d, candidate.result_2d_index, M_POSITION_Y, &center_pos_y);
			pose_model_to_image_ = SPoseDataQuat(center_pos_x, center_pos_y, 0, 0, 0, 0, 0);

			// draw result
			if (is_defining_model_)
			{
				MIL_ID resutl_display_image = MbufAllocColor(mil_system_, 3, image_size_x_, image_size_y_, 8 + M_UNSIGNED, M_IMAGE + M_DISP + M_PROC, M_NULL);
				MimConvert(src_2d_image_, resutl_display_image, M_L_TO_RGB);

				// draw depth on the top left corner
				MmodControl(candidate.result_depth, M_DEFAULT, M_RESULT_OUTPUT_UNITS, M_PIXEL);
				MIL_DOUBLE center_x, center_y;
				MmodGetResult(candidate.result_depth, 0, M_CENTER_X, &center_x);
				MmodGetResult(candidate.result_depth, 0, M_CENTER_Y, &center_y);
				MIL_DOUBLE width, height;
				MmodGetResult(candidate.result_depth, 0, M_WIDTH, &width);
				MmodGetResult(candidate.result_depth, 0, M_HEIGHT, &height);
				MmodControl(candidate.result_depth, M_DEFAULT, M_RESULT_OUTPUT_UNITS, M_WORLD);

				static const double depth_margin = 8;
				MIL_DOUBLE half_radius = sqrt(width * width + height * height) / 2 + depth_margin;
				MbufClear(image_depth_uint8_, 0);
				DPCLib::mapDepthImageTo8Bits(mil_system_, image_depth_, image_depth_uint8_);
				MimClip(image_depth_uint8_, image_depth_uint8_, M_GREATER, 0, M_NULL, 255, M_NULL);
				MIL_ID depth_thumbnail = MbufChild2d(image_depth_uint8_,
					max(0, (MIL_INT)(center_x - half_radius)), 
					max(0, (MIL_INT)(center_y - half_radius)),
					(MIL_INT)(half_radius * 2),
					(MIL_INT)(half_radius * 2),
					M_NULL);

				MIL_INT thumbnail_size_x = min(MbufInquire(src_2d_image_, M_SIZE_X, M_NULL) / 4, (MIL_INT)(half_radius * 2));
				MIL_INT thumbnail_size_y = min(MbufInquire(src_2d_image_, M_SIZE_Y, M_NULL) / 4, (MIL_INT)(half_radius * 2));

				MIL_ID thumbnail = MbufChild2d(resutl_display_image, 1, 1, thumbnail_size_x, thumbnail_size_y, M_NULL);
				MbufClear(thumbnail, 0);

				MimResize(depth_thumbnail, thumbnail, M_FILL_DESTINATION, M_FILL_DESTINATION, M_BILINEAR);
				MbufFree(depth_thumbnail);
				MbufFree(thumbnail);

				MgraDraw(mil_graphic_context_list_, resutl_display_image, M_DEFAULT);

				std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_;
				std::string image_path = base_folder + "/result_" + workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().reference_depth_image_path();

				std::shared_ptr<wchar_t> wc_image_path = utils::string2Wchar_tPtr(image_path);
				MbufExport(wc_image_path.get(), M_MIL + M_WITH_CALIBRATION, resutl_display_image);

				MbufFree(resutl_display_image);
			}
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

#if DEBUG_TIME
	t = clock() - t;
	printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "Compute pose: " + std::to_string(((float)t) / CLOCKS_PER_SEC) + " s");
#endif

	for (auto it : result_2d)
	{
		MmodFree(it);
	}

	for (auto it : result_depth)
	{
		MmodFree(it);
	}

	MbufFree(image_buf);
	MbufFree(src_2d_image);

#if DEBUG_TIME
	t_total = clock() - t_total;
	printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "It took me " + std::to_string(((float)t_total) / CLOCKS_PER_SEC) + " seconds to run localization.");
#endif

	//@ if the result is good, transform the pose from camera coordinate to world
	if (rotate_angle != M_INVALID)
	{
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

bool DepthRectLocalization::createModelDepthMap(MIL_ID mod_result, MIL_INT model_index, MIL_INT pc_label)
{
	if (mod_result != M_NULL)
	{
		// create model mask image
		MIL_ID image_int_model_mask = MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, 1 + M_UNSIGNED, M_IMAGE + M_PROC + M_DISP, M_NULL);
		MbufClear(image_int_model_mask, 0);

		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -roi_x_);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -roi_y_);
		MgraColor(mil_graphic_context_, 1);
		/*MgraColor(mil_graphic_context_, 255);*/
		MmodDraw(mil_graphic_context_, mod_result, image_int_model_mask, M_DRAW_EDGES, model_index, M_DEFAULT);
		MIL_DOUBLE center_x, center_y;
		MmodGetResult(mod_result, model_index, M_POSITION_X, &center_x);
		MmodGetResult(mod_result, model_index, M_POSITION_Y, &center_y);
		MgraFill(mil_graphic_context_, image_int_model_mask, center_x, center_y);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
		MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);

		if (depth_camera_data_wrapper_->createDepthMap(
			src_range_image_,
			image_int_model_mask,
			image_depth_,
			pc_label + 1,
			0,
			0/*1*/,
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
			pc_label + 1,
			0,
			0,
			!is_defining_model_,
			false,
			false);

		MbufFree(image_int_model_mask);
		return ret;
	}
}

void DepthRectLocalization::display2DResult(MIL_ID mod_reuslt, MIL_INT model_index)
{
	if (mil_graphic_context_)
	{
		if (isROIValid())
		{
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, -roi_x_);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, -roi_y_);
		}

		MgraColor(mil_graphic_context_, M_COLOR_YELLOW);
		MmodDraw(mil_graphic_context_, mod_reuslt, mil_graphic_context_list_, M_DRAW_EDGES + M_DRAW_POSITION, model_index, M_DEFAULT);

		if (isROIValid())
		{
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_X, 0);
			MgraControl(mil_graphic_context_, M_DRAW_OFFSET_Y, 0);
		}
	}
}