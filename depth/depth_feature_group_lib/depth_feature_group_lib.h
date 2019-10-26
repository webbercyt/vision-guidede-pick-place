#pragma once
#include "feature_group_lib.h"

static const double POINT_CLOUD_BOUNDING_OFFSET = 20; // in mm
static const int DONIMAL_FEATURE_INDEX = 1;

/*----------------------------------------
			DepthFeatureGroup
----------------------------------------*/
class DepthLocalization;
class DepthFeatureGroup : public FeatureGroup
{
public:
	DepthFeatureGroup(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);
	~DepthFeatureGroup();

	virtual bool createNewFeature(SPoseDataQuat pose_base_to_tool) override;
	virtual bool createSearchRange(std::string = "") override;
	virtual void displaySearchRange(std::string = "") override;
	virtual bool defineFeature() override;
	virtual bool updateDefinedFeature(SPoseDataQuat pose_base_to_tool) override;
	virtual bool loadReferenceImage() override;
	virtual bool saveModelContext(const std::string& feature_name = "") override;
	virtual bool loadAllModelContext() override;
	virtual void displayAllContext() override;
	virtual void resetDisplay() override;

	bool defineSuccessed();
	static bool duplicateFeature(Workspace* const workspace_ptr, const std::string feature_group_name, const std::string pattern_name);

protected:

	virtual void createModel() = 0;
	virtual bool isValidPattern(
		const google::protobuf::MapPair<std::string, config::Workspace_FeatureGroup_PatternWindow>) { return true; }

	/*
		@ brief: use set parameters to define the 2d model context

		@ parameters
			mod_name: e.g. pattern_2d_1
			mod_find_image: image used to define model 
			model_size_x: model size x
			model_size_y: model size y
			max_model_size_x: Useless
			max_model_size_y: Useless
			src_pattern: the protobuf pattern used to get feature parameters
			mod_number: the model number
	*/
	virtual bool define2DModelContext(
		std::string mod_name,
		MIL_ID mod_find_image,
		MIL_DOUBLE model_size_x, 
		MIL_DOUBLE model_size_y,
		MIL_DOUBLE max_model_size_x,
		MIL_DOUBLE max_model_size_y,
		config::Workspace_FeatureGroup::PatternWindow src_pattern,
		int mod_number) { return false; };

	/*
		@ brief: use set parameters to define the depth model context

		@ parameters
			mod_name: e.g. pattern_depth_1
			mod_find_image: image used to define model
			model_size_x: model size x
			model_size_y: model size y
			src_pattern: the protobuf pattern used to get feature parameters
			mod_number: the model number
	*/
	virtual bool defineDepthModelContext(
		std::string mod_name,
		MIL_ID mod_find_image,
		MIL_DOUBLE model_size_x,
		MIL_DOUBLE model_size_y,
		config::Workspace_FeatureGroup::PatternWindow src_pattern,
		int mod_number) { return false; };

	virtual std::shared_ptr<DepthLocalization> createModelLocalizationPtr() { return nullptr; };

	void resetReferenceImage();
	void checkModelSearchRegion(int model_region_x, int model_region_y, int model_region_size_x, int model_region_size_y);
	void drawDepthContext(MIL_ID src_image, MIL_ID dst_image, MIL_INT start_x, MIL_INT start_y, MIL_INT size_x, MIL_INT size_y);

	std::string camera_name_;
	std::map<std::string, MIL_ID> mil_mod_result_map_;

	MIL_ID mil_calibration_;
	MIL_ID reference_image_original_;
	MIL_ID reference_range_image_;
	MIL_ID reference_depth_image_;
	MIL_ID result_image_;
	bool define_succesed_;

	SPoseDataQuat pose_base_to_tool_;
};

/*----------------------------------------
		DepthModelDefinedFeature
----------------------------------------*/
class DepthModelDefinedFeature : public DepthFeatureGroup
{
public:
	DepthModelDefinedFeature(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~DepthModelDefinedFeature();

protected:

	virtual void createModel() override;
	virtual bool isValidPattern(
		const google::protobuf::MapPair<std::string, config::Workspace_FeatureGroup_PatternWindow> pattern) override;

	MIL_ID depth_map_;
	MIL_ID depth_lut_;
	MIL_ID depth_disp_;
	MIL_ID depth_gra_list_;
};

/*----------------------------------------
			DepthCircleFeature
----------------------------------------*/
class DepthCircleFeature : public DepthModelDefinedFeature
{
public:
	DepthCircleFeature(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

private:

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
