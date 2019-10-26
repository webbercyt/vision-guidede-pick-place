#pragma once
#include "depth_feature_group_lib.h"

/*----------------------------------------
			DepthGeometryFeature
----------------------------------------*/
class DepthGeometryFeature : public DepthModelDefinedFeature
{
public:
	DepthGeometryFeature(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);
	~DepthGeometryFeature();

	virtual bool editMask(const std::string pattern_name, const MIL_INT mil_window_id = 0) override;

private:
	virtual void createModel() override;
	virtual void displayContext(std::string pattern_name = "") override;
};