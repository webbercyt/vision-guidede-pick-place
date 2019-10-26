#pragma once
#include "depth_localization_lib.h"

/*----------------------------------------
		 DepthSegmentLocalization
----------------------------------------*/
class DepthSegmentLocalization : public DepthLocalization
{
public:
	DepthSegmentLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	DepthSegmentLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

	~DepthSegmentLocalization();

	static void setContextFromPattern(
		MIL_ID mod_context,
		config::Workspace_FeatureGroup::PatternWindow pattern,
		MIL_INT control_number);

	static void setBlobContextControl(MIL_ID blob_context);

private:
	bool setupModContext();

	virtual void runLocalization() override;
	bool computeSegmentBlob(MIL_ID range_image, MIL_ID& blob_result, MIL_INT& index, MIL_INT& x_offset, MIL_INT& y_offset);
	bool computePoseFromBlobResult(SPoseDataQuat& pose, MIL_ID range_image, MIL_ID blob_result,
		MIL_INT blob_index, MIL_INT x_offset, MIL_INT y_offset);

	// @ Variables
	MIL_ID display_image_;
	int max_depth_value_;

	std::vector<MIL_DOUBLE> blob_gravity_center_x_;
	std::vector<MIL_DOUBLE> blob_gravity_center_y_;
	std::map<std::string, MIL_ID> seg_context_map_;
	std::map<std::string, MIL_ID> seg_result_map_;
	std::map<std::string, MIL_ID> blob_context_map_;
	std::map<std::string, MIL_ID> blob_result_map_;
	std::map<std::string, std::pair<double, double>> blob_area_select_;	// std::pair<double, double>: area min & max
};