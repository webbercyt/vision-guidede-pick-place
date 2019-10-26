#pragma once
#include "depth_localization_lib.h"

const std::string MASK_IMAGE_FILE_NAME_2D = "model_mask_2d.we";
const std::string MASK_IMAGE_FILE_NAME_DEPTH = "model_mask.we";

/*----------------------------------------
		 DepthGeometryLocalization
----------------------------------------*/
class DepthGeometryLocalization : public DepthModelDefinedLocalization
{
public:
	DepthGeometryLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	DepthGeometryLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

	~DepthGeometryLocalization();

	static void setContextFromPattern(
		MIL_ID mod_context,
		config::Workspace_FeatureGroup::PatternWindow pattern,
		MIL_INT control_number);

private:
	void loadModelInfo();

	virtual bool createModelDepthMap(MIL_ID mod_context, MIL_ID mod_result, MIL_INT model_index) override;
	virtual bool createModelPartialPointCloud(
		MIL_ID mod_context,
		MIL_ID mod_result,
		MIL_INT mod_index,
		MIL_INT src_pc_index,
		MIL_ID dst_point_cloud,
		MIL_INT dst_pc_index) override;
	virtual	MIL_DOUBLE computeOccMeanElevation(MIL_ID mod_context, MIL_ID mod_result, MIL_INT index) override;

	/*
		@ brief
			MIL_DOUBLE param_1: model index
	*/
	virtual void computePoseFrom2dModel(MIL_ID mod_context,
		MIL_ID mod_result,
		SPoseDataQuat& pose,
		MIL_DOUBLE& rotate_angle,
		MIL_DOUBLE& error,
		MIL_DOUBLE param_1 = M_NULL,
		MIL_DOUBLE param_2 = M_NULL) override;

	/*
		@ brief
			MIL_DOUBLE param_1: occurrance mean elevation
			MIL_DOUBLE param_2: point cloud label
			MIL_DOUBLE param_3: model index
			MIL_DOUBLE param_4: scene point cloud for 3d alignment
	*/
	virtual void computePoseFromDepthModel(
		MIL_ID mod_context,
		MIL_ID mod_result,
		SPoseDataQuat& pose,
		MIL_DOUBLE& rotate_angle,
		MIL_DOUBLE& error,
		MIL_DOUBLE param_1 = M_NULL,
		MIL_DOUBLE param_2 = M_NULL,
		MIL_DOUBLE param_3 = M_NULL,
		MIL_DOUBLE param_4 = M_NULL) override;

	MIL_ID mil_align_context_;
	MIL_ID mil_align_result_;
	MIL_ID mil_prealign_matrix_;
	MIL_ID model_mask_2d_;
	MIL_ID model_mask_depth_;
	MIL_ID model_pc_;
	MIL_ID scene_pc_;

	MIL_DOUBLE model_mean_;

	//#if DEBUGING_MODE
	MIL_DISP_D3D_HANDLE point_cloud_disp_;
	//#endif

};