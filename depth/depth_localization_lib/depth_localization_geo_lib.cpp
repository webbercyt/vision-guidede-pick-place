#include "depth_localization_geo_lib.h"
#include "utils.h"
#include "depth_camera.h"
#include "convertor.h"

DepthGeometryLocalization::DepthGeometryLocalization(MIL_ID mil_application,
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
	loadModelInfo();
}

DepthGeometryLocalization::DepthGeometryLocalization(MIL_ID mil_application,
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
	loadModelInfo();
}

DepthGeometryLocalization::~DepthGeometryLocalization()
{
	// free memory
	M3dmapFree(mil_align_context_);
	M3dmapFree(mil_align_result_);
	MbufFree(mil_prealign_matrix_);
	MbufFree(model_mask_2d_);
	MbufFree(model_mask_depth_);
	M3dmapFree(model_pc_);

	//#if DEBUGING_MODE
	if (point_cloud_disp_) MdispD3DFree(point_cloud_disp_);
	//#endif
}

void DepthGeometryLocalization::setContextFromPattern(
	MIL_ID mod_context,
	config::Workspace_FeatureGroup::PatternWindow pattern,
	MIL_INT control_number)
{
	MmodControl(mod_context, M_CONTEXT, M_ACCURACY, pattern.m_accuracy());
	MmodControl(mod_context, M_CONTEXT, M_DETAIL_LEVEL, pattern.m_detail_level());
	MmodControl(mod_context, M_CONTEXT, M_SMOOTHNESS, pattern.m_smoothness());
	MmodControl(mod_context, M_CONTEXT, M_TIMEOUT, pattern.m_timeout());
	MmodControl(mod_context, M_DEFAULT, M_ACCEPTANCE, pattern.m_acceptance());
	MmodControl(mod_context, M_DEFAULT, M_CERTAINTY, pattern.m_certainty());
	MmodControl(mod_context, M_DEFAULT, M_NUMBER, control_number);

	//if (control_number > 1)
	{
		MmodControl(mod_context, M_CONTEXT, M_SHARED_EDGES, M_ENABLE);
	}

	MmodControl(mod_context, M_CONTEXT, M_SEARCH_POSITION_RANGE, M_ENABLE);

	if (pattern.search_delta_angle() != 0)
	{
		MmodControl(mod_context, M_CONTEXT, M_SEARCH_ANGLE_RANGE, M_ENABLE);
		MmodControl(mod_context, M_DEFAULT, M_ANGLE, M_DEFAULT);
		MmodControl(mod_context, M_DEFAULT, M_ANGLE_DELTA_NEG, pattern.search_delta_angle());
		MmodControl(mod_context, M_DEFAULT, M_ANGLE_DELTA_POS, pattern.search_delta_angle());
	}
	else
	{
		MmodControl(mod_context, M_CONTEXT, M_SEARCH_ANGLE_RANGE, M_DISABLE);
	}

	if (pattern.enable_search_scale())
	{
		MmodControl(mod_context, M_CONTEXT, M_SEARCH_SCALE_RANGE, M_ENABLE);
		MmodControl(mod_context, M_DEFAULT, M_SCALE_MIN_FACTOR, pattern.search_scale_min());
		MmodControl(mod_context, M_DEFAULT, M_SCALE_MAX_FACTOR, 1 / pattern.search_scale_min());
	}
	else
	{
		MmodControl(mod_context, M_CONTEXT, M_SEARCH_SCALE_RANGE, M_DISABLE);
	}
}

void DepthGeometryLocalization::loadModelInfo()
{
	//#if DEBUGING_MODE
	point_cloud_disp_ = M_NULL;
	//#endif

	// load PC
	std::string base_folder = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_;
	std::string point_cloud_name = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().model_name_point_cloud();
	std::string context_path = base_folder + "/" + point_cloud_name + ".we"/*".ply"*/;
	std::shared_ptr<wchar_t> wc_context_path = utils::string2Wchar_tPtr(context_path);
	M3dmapRestore(wc_context_path.get(), mil_system_, M_DEFAULT, &model_pc_);

	// load model mask
	std::string model_mask_2d_path = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_ + "/" + MASK_IMAGE_FILE_NAME_2D;
	std::shared_ptr<wchar_t> model_mask_2d_path_ptr = utils::string2Wchar_tPtr(model_mask_2d_path);
	model_mask_2d_ = MbufRestore(model_mask_2d_path_ptr.get(), mil_system_, M_NULL);

	std::string model_mask_depth_path = workspace_ptr_->workspace_folder() + "/feature_group/" + feature_group_name_ + "/" + MASK_IMAGE_FILE_NAME_DEPTH;
	std::shared_ptr<wchar_t> model_mask_depth_path_ptr = utils::string2Wchar_tPtr(model_mask_depth_path);
	model_mask_depth_ = MbufRestore(model_mask_depth_path_ptr.get(), mil_system_, M_NULL);

	// load model var.
	model_mean_ = workspace_ptr_->feature_group().at(feature_group_name_).depth_feature_group().model_mean_depth();

	// config 3d alignment
	M3dmapAlloc(mil_system_, M_PAIRWISE_ALIGNMENT_CONTEXT, M_DEFAULT, &mil_align_context_);
	M3dmapAllocResult(mil_system_, M_ALIGNMENT_RESULT, M_DEFAULT, &mil_align_result_);
	MbufAlloc2d(mil_system_, 4, 4, M_FLOAT + 32, M_ARRAY, &mil_prealign_matrix_);

	M3dmapControl(mil_align_context_, M_DEFAULT, M_DECIMATION_STEP_MODEL, 4/*2*/);	// Corresponds to a decimation factor of 4*4=16. 
	M3dmapControl(mil_align_context_, M_DEFAULT, M_DECIMATION_STEP_SCENE, 4/*2*/);	// Corresponds to a decimation factor of 4*4=16. 
	M3dmapControl(mil_align_context_, M_DEFAULT, M_MODEL_OVERLAP, 90.0/*95*/);		// (%)
	M3dmapControl(mil_align_context_, M_DEFAULT, M_MAX_ITERATIONS, 50);
	M3dmapControl(mil_align_context_, M_DEFAULT, M_ERROR_MINIMIZATION_METRIC, M_POINT_TO_POINT);
}

bool DepthGeometryLocalization::createModelDepthMap(MIL_ID mod_context, MIL_ID mod_result, MIL_INT model_index)
{
	if (mod_result != M_NULL)
	{
		// use model find result to create new model mask
		MIL_ID image_int_model_mask = MbufClone(model_mask_2d_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_NULL);;
		MbufClear(image_int_model_mask, 0);

		MIL_DOUBLE a, b, c, d;
		MmodGetResult(mod_result, model_index, M_A_REVERSE, &a);
		MmodGetResult(mod_result, model_index, M_B_REVERSE, &b);
		MmodGetResult(mod_result, model_index, M_C_REVERSE, &c);
		MmodGetResult(mod_result, model_index, M_D_REVERSE, &d);

		// add offset
		MIL_DOUBLE offset_x, offset_y;
		MmodInquire(mod_context, M_DEFAULT, M_ALLOC_OFFSET_X, &offset_x);
		MmodInquire(mod_context, M_DEFAULT, M_ALLOC_OFFSET_Y, &offset_y);
		c += offset_x;
		d += offset_y;

		// compute 4 corners
		float x1, y1, x2, y2, x3, y3, x4, y4;
		x1 = (float)c;
		y1 = (float)d;
		x2 = (float)(a * float(image_size_x_ - 1) + c);
		y2 = (float)(-b * float(image_size_x_ - 1) + d);
		x3 = (float)(a * float(image_size_x_ - 1) + b * float(image_size_y_ - 1) + c);
		y3 = (float)(-b * float(image_size_x_ - 1) + a * float(image_size_y_ - 1) + d);
		x4 = (float)(b * float(image_size_y_ - 1) + c);
		y4 = (float)(a * float(image_size_y_ - 1) + d);

		float four_corner_matrix[12] = { x1, y1, x2, y2, x3, y3, x4, y4, 0.0, 0.0, float(image_size_x_ - 1), float(image_size_y_ - 1) };
		MIL_ID four_corner_array = MbufAlloc2d(mil_system_, 12L, 1L, 32L + M_FLOAT, M_ARRAY, M_NULL);
		MbufPut1d(four_corner_array, 0L, 12L, four_corner_matrix);

		// create lut for image warp
		MIL_ID lut_x = MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, 16L + M_SIGNED, M_LUT, M_NULL);
		MIL_ID lut_y = MbufAlloc2d(mil_system_, image_size_x_, image_size_y_, 16L + M_SIGNED, M_LUT, M_NULL);
		MgenWarpParameter(four_corner_array, lut_x, lut_y, M_WARP_4_CORNER + M_FIXED_POINT + 0L, M_DEFAULT, 0.0, 0.0);

		MimWarp(model_mask_2d_, image_int_model_mask, lut_x, lut_y, M_WARP_LUT + M_FIXED_POINT + 0L, M_NEAREST_NEIGHBOR);
		MimClip(image_int_model_mask, image_int_model_mask, M_GREATER, 0, M_NULL, 1, M_NULL);

		MbufFree(lut_x);
		MbufFree(lut_y);
		MbufFree(four_corner_array);

		if (depth_camera_data_wrapper_->createDepthMap(
			src_range_image_,
			image_int_model_mask,
			image_depth_,
			model_index + 1,
			0,
			0,
			!is_defining_model_,
			false,
			false))
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

void RemoveRelativeFixture(MIL_ID Id)
{
	McalSetCoordinateSystem(Id, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_IDENTITY + M_ASSIGN, M_NULL, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);
}

bool DepthGeometryLocalization::createModelPartialPointCloud(
	MIL_ID mod_context,
	MIL_ID mod_result,
	MIL_INT mod_index,
	MIL_INT src_pc_index,
	MIL_ID dst_point_cloud,
	MIL_INT dst_pc_index)
{
	// compose partial point cloud
	SDepthData depth_data = depth_camera_data_wrapper_->getDepthData();
	MIL_INT x_min = depth_data.depth_map_size_x_, y_min = depth_data.depth_map_size_y_;
	MIL_INT x_max = 0, y_max = 0;
	{
		MIL_DOUBLE scale, angle;
		MmodGetResult(mod_result, mod_index, M_SCALE, &scale);
		MmodGetResult(mod_result, mod_index, M_ANGLE, &angle);

		MIL_DOUBLE pos_x, pos_y;
		MmodGetResult(mod_result, mod_index, M_POSITION_X, &pos_x);
		MmodGetResult(mod_result, mod_index, M_POSITION_Y, &pos_y);

		MIL_DOUBLE x_w[1] = { pos_x };
		MIL_DOUBLE y_w[1] = { pos_y };
		MIL_DOUBLE x_p[1];
		MIL_DOUBLE y_p[1];
		McalTransformCoordinateList(image_depth_, M_WORLD_TO_PIXEL, 1, x_w, y_w, x_p, y_p);

		MIL_INT model_box_size_x_ = MmodInquire(mod_context, M_DEFAULT, M_BOX_SIZE_X + M_TYPE_MIL_INT32, M_NULL);
		MIL_INT model_box_size_y_ = MmodInquire(mod_context, M_DEFAULT, M_BOX_SIZE_Y + M_TYPE_MIL_INT32, M_NULL);
		MIL_DOUBLE box_size_x = (MIL_DOUBLE)model_box_size_x_ * scale;
		MIL_DOUBLE box_size_y = (MIL_DOUBLE)model_box_size_y_ * scale;

		if (angle >= 180.0)
			angle -= 180.0;

		if (angle >= 90.0)
			angle = 180.0 - angle;

		MIL_DOUBLE rad = angle * DEG_TO_RAD;
		MIL_DOUBLE half_size_x = 0.5 * (cos(rad) * box_size_x + sin(rad) * box_size_y);
		MIL_DOUBLE half_size_y = 0.5 * (sin(rad) * box_size_x + cos(rad) * box_size_y);

		MIL_DOUBLE offset = 10;
		x_min = MIL_INT(x_p[0] - half_size_x - offset);
		x_max = MIL_INT(x_p[0] + half_size_x + offset);
		y_min = MIL_INT(y_p[0] - half_size_y - offset);
		y_max = MIL_INT(y_p[0] + half_size_y + offset);
	}

	double mm_per_pixel = depth_camera_data_wrapper_->getDepthMapPixelUint();
	double extract_box_x;
	double extract_box_y;
	double extract_box_z;
	double extract_box_size_x;
	double extract_box_size_y;
	double extract_box_size_z;

	depth_camera_data_wrapper_->getPCBoundingBox(extract_box_x, extract_box_y, extract_box_z,
		extract_box_size_x, extract_box_size_y, extract_box_size_z);

	double image_x_center = double(depth_data.depth_map_size_x_) / 2;
	double image_y_center = double(depth_data.depth_map_size_y_) / 2;

	double view_x_start = (double(x_min) - image_x_center) / double(depth_data.depth_map_size_x_) *
		extract_box_size_x;

	double view_y_start = (double(y_min) - image_y_center) / double(depth_data.depth_map_size_y_) *
		extract_box_size_y;

	double view_x_end = (double(x_max) - image_x_center) / double(depth_data.depth_map_size_x_) *
		extract_box_size_x;

	double view_y_end = (double(y_max) - image_y_center) / double(depth_data.depth_map_size_y_) *
		extract_box_size_y;

	if (x_min > image_x_center)
		view_x_start = (x_min - image_x_center) * mm_per_pixel;

	if (x_max < image_x_center)
		view_x_end = (image_x_center - x_max) * mm_per_pixel;

	if (y_min > image_y_center)
		view_y_start = (y_min - image_y_center) * mm_per_pixel;

	if (y_max < image_y_center)
		view_y_end = (image_y_center - y_max) * mm_per_pixel;

	MIL_ID src_point_cloud = depth_camera_data_wrapper_->getPointCloudID();
	M3dmapSetBox(src_point_cloud, M_EXTRACTION_BOX, M_CORNER_AND_DIMENSION,
		view_x_start, view_y_start, extract_box_z,
		view_x_end - view_x_start, view_y_end - view_y_start, extract_box_size_z);

	MIL_INT max_point_nb = M3dmapGet(src_point_cloud, PC_LABEL(src_pc_index), M_POSITION, M_INCLUDE_POINTS_INSIDE_BOX_ONLY,
		M_FLOAT + 64, M_NULL, M_NULL, M_NULL, M_NULL, M_NULL);

	if (max_point_nb < 1)
		return false;

	std::vector<MIL_DOUBLE> x(max_point_nb);
	std::vector<MIL_DOUBLE> y(max_point_nb);
	std::vector<MIL_DOUBLE> z(max_point_nb);

	// Get the points inside the extraction box.
	MIL_INT actual_point_nb = M3dmapGet(src_point_cloud, PC_LABEL(src_pc_index), M_POSITION, M_INCLUDE_POINTS_INSIDE_BOX_ONLY,
		M_FLOAT + 64, max_point_nb, &x[0], &y[0], &z[0], M_NULL);
	if (actual_point_nb < 1)
		return false;

	M3dmapPut(dst_point_cloud, PC_DEFAULT_LABEL, M_POSITION, M_FLOAT + 64, actual_point_nb, &x[0], &y[0], &z[0], src_point_cloud, M_DEFAULT);
	// reset point cloud bounding box
	M3dmapSetBox(src_point_cloud, M_EXTRACTION_BOX, M_CORNER_AND_DIMENSION,
		extract_box_x, extract_box_y, extract_box_z,
		extract_box_size_x, extract_box_size_y, extract_box_size_z);

	depth_camera_data_wrapper_->resetPCBoundingBox();
	return true;
}

MIL_DOUBLE DepthGeometryLocalization::computeOccMeanElevation(MIL_ID mod_context, MIL_ID mod_result, MIL_INT index)
{
	scene_pc_ = depth_camera_data_wrapper_->getPointCloudID();
	RemoveRelativeFixture(scene_pc_);

	SDepthData depth_data = depth_camera_data_wrapper_->getDepthData();
	MIL_ID depth_map_buf =
		MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, M_UNSIGNED + 16, M_IMAGE + M_DISP + M_PROC, M_NULL);

	MIL_ID mil_finder_fixturing_offset = McalAlloc(mil_system_, M_FIXTURING_OFFSET, M_DEFAULT, M_NULL);
	McalFixture(M_NULL, mil_finder_fixturing_offset, M_LEARN_OFFSET,
		M_MODEL_MOD, mod_context, 0, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	McalFixture(scene_pc_, mil_finder_fixturing_offset, M_MOVE_RELATIVE,
		M_RESULT_MOD, mod_result, index, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	McalFree(mil_finder_fixturing_offset);

	MIL_DOUBLE tx, ty, tz;
	// Get 6 pose elements.
	McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_TRANSLATION, M_NULL, &tx, &ty, &tz, M_NULL);

	// Extract the top-view depth-map of the scene fixtured on the occurrence.
	M3dmapExtract(scene_pc_, depth_map_buf, M_NULL, M_CORRECTED_DEPTH_MAP, M_ALL, M_DEFAULT);
	MbufClearCond(depth_map_buf, 0.0, M_NULL, M_NULL, depth_map_buf, M_EQUAL, 65535);
	RemoveRelativeFixture(scene_pc_);

	// Compute occurrence's mean elevation in the scene only from the pixels
	// enabled by the model's mask.
	MIL_DOUBLE occ_mean_elevation;
	M3dmapStat(depth_map_buf, M_NULL, model_mask_depth_, M_NULL, M_DEVIATION_MEAN + M_STAT_ALL, M_DEFAULT, M_DEFAULT, &occ_mean_elevation);

	MbufFree(depth_map_buf);
	return occ_mean_elevation;
}

void DepthGeometryLocalization::computePoseFrom2dModel(MIL_ID mod_context, MIL_ID mod_result,
	SPoseDataQuat& pose, MIL_DOUBLE& rotate_angle, MIL_DOUBLE& error, MIL_DOUBLE param_1, MIL_DOUBLE param_2)
{
	//MIL_DOUBLE org_x, org_y;
	//MmodInquire(mod_context, M_DEFAULT, M_ORIGINAL_X, &org_x);
	//MmodInquire(mod_context, M_DEFAULT, M_ORIGINAL_Y, &org_y);

	//MIL_DOUBLE pos_x, pos_y;
	//MmodGetResult(mod_result, MIL_INT(param_1), M_POSITION_X, &pos_x);
	//MmodGetResult(mod_result, MIL_INT(param_1), M_POSITION_Y, &pos_y);

	//MIL_DOUBLE offset_x = pos_x - org_x;
	//MIL_DOUBLE offset_y = pos_y - org_y;

	//// scale, offset, rotate
}

void DepthGeometryLocalization::computePoseFromDepthModel(MIL_ID mod_context, MIL_ID mod_result, SPoseDataQuat& pose, MIL_DOUBLE& rotate_angle, MIL_DOUBLE& error,
	MIL_DOUBLE param_1,
	MIL_DOUBLE param_2,
	MIL_DOUBLE param_3,
	MIL_DOUBLE param_4)
{
	rotate_angle = M_INVALID;
	if (MIL_ID(param_4) != M_NULL)
		scene_pc_ = MIL_ID(param_4);
	RemoveRelativeFixture(scene_pc_);

	MIL_ID mil_finder_fixturing_offset = McalAlloc(mil_system_, M_FIXTURING_OFFSET, M_DEFAULT, M_NULL);
	McalFixture(M_NULL, mil_finder_fixturing_offset, M_LEARN_OFFSET,
		M_MODEL_MOD, mod_context, 0, M_DEFAULT, M_DEFAULT, M_DEFAULT);
	McalFixture(scene_pc_, mil_finder_fixturing_offset, M_MOVE_RELATIVE,
		M_RESULT_MOD, mod_result, MIL_ID(param_3), M_DEFAULT, M_DEFAULT, M_DEFAULT);
	McalFree(mil_finder_fixturing_offset);

	MIL_DOUBLE occ_mean_elevation = param_1;
	MIL_DOUBLE offset_z = occ_mean_elevation - model_mean_;

	McalSetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_RELATIVE_COORDINATE_SYSTEM,
		M_TRANSLATION, M_NULL, 0.0, 0.0, offset_z, M_DEFAULT);

	// display mod find & depth elevation result
	if (/*false*/true)
	{
		MIL_DOUBLE tx, ty, tz;
		// Get 6 pose elements.
		McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
			M_TRANSLATION, M_NULL, &tx, &ty, &tz, M_NULL);

		MIL_DOUBLE rx, ry, rz;
		// Get 6 pose elements.
		McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
			M_TRANSLATION, M_NULL, &tx, &ty, &tz, M_NULL);

		McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
			M_ROTATION_XYZ, M_NULL, &rx, &ry, &rz, M_NULL);

		printf("@ move: [%f, %f, %f] [%f, %f, %f]\n", tx, ty, tz, rx, ry, rz);
	}

	McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
		M_HOMOGENEOUS_MATRIX, mil_prealign_matrix_, M_NULL, M_NULL, M_NULL, M_NULL);

	RemoveRelativeFixture(scene_pc_);
	// Perform 3D alignment of the model and the selected found occurrence in the scene.

	//M3dmapAllocResult(mil_system, M_POINT_CLOUD_CONTAINER, M_DEFAULT, &point_cloud_);
	M3dmapAlign(mil_align_context_,
		model_pc_, M_ALL,
		scene_pc_, PC_LABEL(MIL_INT(param_2) + 1),
		mil_prealign_matrix_, mil_align_result_, M_DEFAULT, M_NULL);

	// Verify the success of the alignment.
	MIL_INT align_completed = 0; // Status of 3D alignment.
	M3dmapGetResult(mil_align_result_, M_DEFAULT, M_ALIGN_COMPLETED + M_TYPE_MIL_INT, &align_completed);
	if (align_completed)
	{
		// Use the 3D alignment result to fixture the model with the found occurrence in the scene.
		McalFixture(scene_pc_, M_NULL, M_MOVE_RELATIVE, M_RESULT_ALIGNMENT_3DMAP,
			mil_align_result_, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

		getRobotPose(scene_pc_, &pose, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM);

		MIL_DOUBLE tx, ty, tz, rx, ry, rz;
		// Get 6 pose elements.
		McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
			M_TRANSLATION, M_NULL, &tx, &ty, &tz, M_NULL);
		McalGetCoordinateSystem(scene_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
			M_ROTATION_XYZ, M_NULL, &rx, &ry, &rz, M_NULL);

		if (is_defining_model_)
		{
			MIL_DOUBLE error_t = sqrt(tx * tx + ty * ty + tz * tz);

			if (rx > 180)
				rx = 360 - rx;

			if (ry > 180)
				ry = 360 - ry;

			if (rz > 180)
				rz = 360 - rz;

			MIL_DOUBLE error_r = sqrt(rx * rx + ry * ry + rz * rz);

			std::string msg = translation::messageOut(translation::message::feature_pose_error) +
				std::to_string(error_t) +
				", " +
				std::to_string(error_r) +
				"]";
			std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, msg); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
		}

		rotate_angle = 0;
		// display alignment result text
		if (/*false*/true)
		{
			// Keep absolute rotation values below 180 degrees for clarity.
			if (fabs(rx) > 180.0) rx = fmod(rx - 360.0, 360.0);
			if (fabs(ry) > 180.0) ry = fmod(ry - 360.0, 360.0);
			if (fabs(rz) > 180.0) rz = fmod(rz - 360.0, 360.0);

			std::string p = "Pose: T[" +
				std::to_string(tx) + ", " +
				std::to_string(ty) + ", " +
				std::to_string(tz) + "]"
				+ " R[" +
				std::to_string(rx) + ", " +
				std::to_string(ry) + ", " +
				std::to_string(rz) + "]";

			std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, p); if (!o_msg.empty()) LOG(INFO) << "[ERROR] " + o_msg;
		}

		//#if DEBUGING_MODE
		if (/*true*/workspace_ptr_->system_config().show_3d_viewer())
		{
			// Get the full 3D pose of the occurrence.
			MIL_ID move_matrix = MbufAlloc2d(mil_system_, 4, 4, M_FLOAT + 32, M_ARRAY, M_NULL);
			McalGetCoordinateSystem(scene_pc_, M_ABSOLUTE_COORDINATE_SYSTEM, M_RELATIVE_COORDINATE_SYSTEM,
				M_HOMOGENEOUS_MATRIX, move_matrix, M_NULL, M_NULL, M_NULL, M_NULL);
			McalSetCoordinateSystem(model_pc_, M_RELATIVE_COORDINATE_SYSTEM, M_ABSOLUTE_COORDINATE_SYSTEM,
				M_HOMOGENEOUS_MATRIX, move_matrix, M_DEFAULT, M_DEFAULT, M_DEFAULT, M_DEFAULT);

			MIL_ID depth_map_buf[2]; // Model and scene depth maps.
			SDepthData depth_data = depth_camera_data_wrapper_->getDepthData();
			MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, M_UNSIGNED + 16, M_IMAGE + M_DISP + M_PROC, &depth_map_buf[0]);
			MbufAlloc2d(mil_system_, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, M_UNSIGNED + 16, M_IMAGE + M_DISP + M_PROC, &depth_map_buf[1]);

			MbufFree(move_matrix);
			M3dmapExtract(model_pc_, depth_map_buf[0], M_NULL, M_CORRECTED_DEPTH_MAP, M_ALL, M_DEFAULT);

			RemoveRelativeFixture(scene_pc_);
			M3dmapExtract(scene_pc_, depth_map_buf[1], M_NULL, M_CORRECTED_DEPTH_MAP, /*M_ALL*/PC_LABEL(MIL_INT(param_2) + 1), M_DEFAULT);

			// Allocate a texture color map for each depth-map.
			MIL_ID texture[2];
			for (MIL_INT i = 0; i < 2; i++)
				MbufAllocColor(mil_system_, 3, depth_data.depth_map_size_x_, depth_data.depth_map_size_y_, 8 + M_UNSIGNED, M_IMAGE + M_PROC, &texture[i]);

			// The depth maps' textures are uniform.
			MbufClearCond(texture[1], 150, 150, 150, depth_map_buf[1], M_NOT_EQUAL, DEPTHMAP_MISSING_DATA);
			MbufClearCond(texture[0], 0, 255, 0, depth_map_buf[0], M_NOT_EQUAL, DEPTHMAP_MISSING_DATA);

			// Associate the calibration so that both maps are expressed in the 
			// same coordinate system.
			McalAssociate(depth_map_buf[1], depth_map_buf[0], M_DEFAULT);

			// Set to invalid pixels of the model's depth-map that are invalid in the 
			// scene's depth-map to not display them.
			MbufClearCond(depth_map_buf[0], DEPTHMAP_MISSING_DATA, 0, 0, depth_map_buf[1], M_EQUAL, DEPTHMAP_MISSING_DATA);

			// If the 3D display is not allocated yet, allocate and initialize it.
			if (point_cloud_disp_ == M_NULL)
			{
				point_cloud_disp_ = MdepthSysD3DAlloc(M_NULL, M_NULL, M_NULL, M_NULL, depth_map_buf, texture,
					2, 1280, 960, 8, M_NULL/*(MIL_WINDOW_HANDLE)mil_window_id_*/);

				// Set rendering parameters.
				MdispD3DControl(point_cloud_disp_, MD3D_ROTATE, MD3D_FALSE);
				MdispD3DControl(point_cloud_disp_, MD3D_POINT, MD3D_ENABLE);

				// Set camera pose.
				MdispD3DControl(point_cloud_disp_, MD3D_LOOK_AT_X, 0.0);
				MdispD3DControl(point_cloud_disp_, MD3D_LOOK_AT_Y, 0.0);
				MdispD3DControl(point_cloud_disp_, MD3D_LOOK_AT_Z, 0.0);
				MdispD3DControl(point_cloud_disp_, MD3D_EYE_THETA, 45.0);
				MdispD3DControl(point_cloud_disp_, MD3D_EYE_PHI, 45.0);
				MdispD3DControl(point_cloud_disp_, MD3D_EYE_DIST, 50);
			}
			else // If already allocated, only update its data to display
				MdepthSysD3DSetSystems(point_cloud_disp_, M_NULL, M_NULL, M_NULL, M_NULL, depth_map_buf, texture, 2, 8);

			// Show the 3D display.
			MdispD3DShow(point_cloud_disp_);

			// Free allocations.
			for (MIL_INT i = 0; i < 2; i++)
			{
				MbufFree(texture[i]);
				MbufFree(depth_map_buf[i]);
			}
		}
		//#endif
	}
	else
		rotate_angle = M_INVALID;

	// Remove fixturing of the model and scene's point clouds.
	RemoveRelativeFixture(model_pc_);
	RemoveRelativeFixture(scene_pc_);
}