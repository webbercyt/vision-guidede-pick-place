#pragma once
#include "depth_feature_group_lib.h"

/*----------------------------------------
			DepthRectFeature
----------------------------------------*/
class DepthRectFeature : public DepthModelDefinedFeature
{
public:
	DepthRectFeature(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	virtual bool displayModelSizes() override;

private:
	virtual void createModel() override;

	virtual bool define2DModelContext(
		std::string mod_name,
		MIL_ID mod_find_image,
		MIL_DOUBLE model_size_x,
		MIL_DOUBLE model_size_y,
		MIL_DOUBLE max_model_size_x,
		MIL_DOUBLE max_model_size_y,
		config::Workspace_FeatureGroup::PatternWindow src_pattern,
		int mod_number) override;

	virtual bool defineDepthModelContext(
		std::string mod_name,
		MIL_ID mod_find_image,
		MIL_DOUBLE model_size_x,
		MIL_DOUBLE model_size_y,
		config::Workspace_FeatureGroup::PatternWindow src_pattern,
		int mod_number) override;

	virtual std::shared_ptr<DepthLocalization> createModelLocalizationPtr() override;

	virtual void displayContext(std::string pattern_name) override;
};
