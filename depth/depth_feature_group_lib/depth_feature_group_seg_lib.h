#pragma once
#include "depth_feature_group_lib.h"

/*----------------------------------------
			DepthSegmentFeature
----------------------------------------*/
class DepthSegmentFeature : public DepthFeatureGroup
{
public:
	DepthSegmentFeature(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);
	~DepthSegmentFeature();

private:
	virtual void createModel() override;
	virtual void displayContext(std::string pattern_name = "") override;
	void drawPattern(std::string pattern_name);
};