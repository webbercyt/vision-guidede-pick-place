#pragma once
#include "depth_localization_lib.h"

/*----------------------------------------
		   DepthRectLocalization
----------------------------------------*/
struct RectCandinate
{
	MIL_ID result_2d;
	MIL_INT result_2d_index;
	MIL_ID result_depth;
	MIL_INT result_depth_index;
	MIL_INT pc_label;
	int offset_x_2d;
	int offset_y_2d;

	RectCandinate();
	RectCandinate(
		MIL_ID result_2d,
		MIL_INT result_2d_index,
		MIL_ID result_depth,
		MIL_INT result_depth_index,
		MIL_INT pc_label);

	RectCandinate operator = (RectCandinate rc);
};

class DepthRectLocalization : public SolidShapeDepthLocalization
{
public:
	DepthRectLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	DepthRectLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

	~DepthRectLocalization();

	static void setContextFromPattern(
		MIL_ID mod_context,
		config::Workspace_FeatureGroup::PatternWindow pattern,
		MIL_INT control_number);

private:

	virtual bool runLocateMethod2dDepth() override;
	virtual bool createModelDepthMap(MIL_ID mod_result, MIL_INT model_index, MIL_INT pc_label) override;
	virtual void display2DResult(MIL_ID mod_reuslt, MIL_INT model_index) override;

	void setupModSegment();

	virtual void getModelSquareROIFromResult(
		MIL_ID mod_result_depth,
		MIL_INT mod_index,
		MIL_ID depth_image,
		MIL_INT& roi_x,
		MIL_INT& roi_y,
		MIL_INT& roi_size_x,
		MIL_INT& roi_size_y) override;

	virtual	MIL_DOUBLE computeOccMeanElevation(MIL_ID mod_context, MIL_ID mod_result, MIL_INT index) override;
	virtual MIL_DOUBLE getModelZAngleFromResult(MIL_ID mod_result, MIL_INT mod_index) override;
	virtual	std::set<MIL_ID> deleteOverlap(MIL_ID mod_result) override;
	virtual MIL_INT locateModel2D(MIL_ID mil_2d_image, std::string& found_model_name) override;

	MIL_ID mil_context_segment_;
	MIL_ID mil_result_segment_;
};