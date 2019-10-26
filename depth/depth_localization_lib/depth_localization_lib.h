#pragma once
#include <mil.h>
#include "config.pb.h"
#include "struct_data.h"
#include "localization_lib.h"
//#if DEBUGING_MODE
#include <MdispD3D.h>
//#endif

static MIL_DOUBLE DEPTHMAP_MISSING_DATA = 65535;

class DepthCameraDataWrapper;

/*
	@ brief: the localization state switch as below
		[IDLE]->[IN_PROCESS]->[SUCCESSED/FAILED]->[IDLE]
*/
enum LocalizationState { IDLE, IN_PROCESS, SUCCESSED, FAILED };
enum DisplayMode { DM_NONE, DM_2D, DM_DEPTH };

namespace ModelProcess
{
	static const std::map<MIL_INT64, MIL_INT64> model_context_result_type_map =
	{
	//  {MmodContext type, MmodResult type}
		{M_SEGMENT, M_SHAPE_SEGMENT},
		{M_RECTANGLE, M_SHAPE_RECTANGLE},
		{M_CIRCLE, M_SHAPE_CIRCLE},
		{M_GEOMETRIC, M_DEFAULT},
		{M_IMAGE, M_DEFAULT}
	};

	extern void setContextFromPattern(MIL_ID mod_context, config::Workspace_FeatureGroup::PatternWindow pattern, MIL_INT control_number);
}

/*----------------------------------------
			DepthLocalization
----------------------------------------*/
class DepthLocalization : public Localization
{
public:
	DepthLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	DepthLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

	~DepthLocalization();

	virtual void pushFrame(MIL_ID image, MIL_ID depth_image, SPoseDataQuat pose_base_to_tool, bool& is_stop, bool& has_result) override;
	virtual void resetlocateState() override;
	virtual bool getModelPose(ModelPoseType model_pose_type, SPoseDataQuat& pose) override;
	virtual bool getRobotMoveToReference(
		SPoseDataQuat tar_pose_base_to_tool,
		SPoseDataQuat tar_pose_world_to_model, 
		SPoseDataQuat ref_pose_world_to_model,
		SPoseDataQuat& move) override;
	virtual void setSearchOrder(int search_order) override;

protected:

	// @ Functions
	bool hasLisence();
	bool isROIValid();
	void handleCollectedData();
	MIL_DOUBLE compute3DPLaneRotation(MIL_ID geometric_plane, SPoseDataQuat& pose, MIL_DOUBLE z_axis_angle);
	SPoseDataQuat transformPoseFromCameraToWorld(SPoseDataQuat input);
	virtual void runLocalization() = 0;

	// @ Variables
	MIL_DOUBLE model_poses_x_;
	MIL_DOUBLE model_poses_y_;
	MIL_DOUBLE model_poses_z_;
	std::multimap<MIL_DOUBLE, SPoseDataQuat> model_rotate_;
	std::shared_ptr<DepthCameraDataWrapper> depth_camera_data_wrapper_;

	int model_collect_times_;
	int camera_mounting_type_;
	int roi_x_;			// search region x start
	int roi_y_;			// search region y start
	int roi_x_size_;	// search region x size
	int roi_y_size_;	// search region y size

	bool is_defining_model_; // flag record current localization computation is for model define or not 
	bool depth_order_pick_;

	MIL_ID mil_cal_robot_transformed_;
	MIL_ID mil_cal_ground_erase_;
	MIL_ID src_2d_image_;
	MIL_ID src_range_image_;
	MIL_ID mil_graphic_context_roi_;
	MIL_INT image_size_x_;
	MIL_INT image_size_y_;
	MIL_DOUBLE depth_groud_offset_;

	SearchOrder search_order_;
	DisplayMode display_mode_;
	LocalizationState state_;
	SPoseDataQuat pose_model_to_image_;
	SPoseDataQuat pose_model_to_cam_;
	SPoseDataQuat pose_model_to_world_;
	SPoseDataQuat pose_base_to_tool_;
	
private:
	// set up
	bool setupBuffers();
	void setupDisplays();
};

/*----------------------------------------
	  DepthModelDefinedLocalization
----------------------------------------*/
class DepthModelDefinedLocalization : public DepthLocalization
{
public:
	DepthModelDefinedLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	DepthModelDefinedLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

	~DepthModelDefinedLocalization();

	virtual void resetlocateState() override;

protected:
	virtual void runLocalization() override;
	bool runLocateMethod2d();
	bool runLocateMethodDepth();
	virtual bool runLocateMethod2dDepth();

	/*
		@ brief: do model find in 2d image(mil_2d_image)
	*/
	virtual MIL_INT locateModel2D(MIL_ID mil_2d_image, std::string& found_model_name);

	/*
		@ brief: do model find in depth image(depth_image) with given context & result
	*/
	MIL_INT locateModelDepth(MIL_ID depth_image, MIL_ID mod_context, MIL_ID& mod_result);

	virtual void display2DResult(MIL_ID mod_reuslt, MIL_INT model_index);
	void display3DResult(MIL_ID mod_result, MIL_INT model_index);

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
		MIL_DOUBLE param_2 = M_NULL) {};

	/*
		@ brief
			MIL_DOUBLE param_1: occurrance mean elevation
			MIL_DOUBLE param_2: point cloud label
			MIL_DOUBLE param_3: model index
			MIL_DOUBLE param_4: scene point cloud for 3d alignment
	*/
	virtual void computePoseFromDepthModel(MIL_ID mod_context,
		MIL_ID mod_result,
		SPoseDataQuat& pose,
		MIL_DOUBLE& rotate_angle,
		MIL_DOUBLE& error,
		MIL_DOUBLE param_1 = M_NULL,
		MIL_DOUBLE param_2 = M_NULL,
		MIL_DOUBLE param_3 = M_NULL,
		MIL_DOUBLE param_4 = M_NULL) {};

	virtual bool createModelPartialPointCloud(
		MIL_ID mod_context, 
		MIL_ID mod_result,
		MIL_INT mod_index,
		MIL_INT src_pc_index, 
		MIL_ID dst_point_cloud, 
		MIL_INT dst_pc_index) { return true; };

	virtual bool createModelDepthMap(MIL_ID mod_context, MIL_ID mod_result, MIL_INT model_index) { return false; };
	virtual	MIL_DOUBLE computeOccMeanElevation(MIL_ID mod_constext, MIL_ID mod_result, MIL_INT index) { return 0.0; };
	virtual	std::set<MIL_ID> deleteOverlap(MIL_ID mod_result);

	MIL_ID image_depth_;
	MIL_ID image_depth_uint8_;

	// display control
	MIL_ID depth_lut_debug_;
	MIL_ID depth_disp_debug_;
	MIL_ID depth_gra_list_debug_;
	MIL_ID disp_depth_lut_;
	MIL_ID disp_depth_gra_list_;

	std::vector<MIL_DOUBLE> model_2d_center_x_;
	std::vector<MIL_DOUBLE> model_2d_center_y_;
	std::map<std::string, MIL_ID> mod_context_map_2d_;
	std::map<std::string, MIL_ID> mod_result_map_2d_;
	std::map<std::string, MIL_ID> mod_context_map_depth_;
	std::map<std::string, MIL_ID> mod_result_map_depth_;
	std::vector<int> mod_result_order_vec_;

private:
	bool setupBuffers();
	void setupDisplays();
	bool setupModContext();
};

/*----------------------------------------
		SolidShapeDepthLocalization
----------------------------------------*/
class SolidShapeDepthLocalization : public DepthModelDefinedLocalization
{
public:
	SolidShapeDepthLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	SolidShapeDepthLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

protected:
	virtual bool createModelDepthMap(MIL_ID mod_context, MIL_ID mod_result, MIL_INT model_index) override;

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
		MIL_DOUBLE param_2 = M_NULL);

	/*
		@ brief
			MIL_DOUBLE param_1: occurrance mean elevation
			MIL_DOUBLE param_2: point cloud label
			MIL_DOUBLE param_3: model index
			MIL_DOUBLE param_4: scene point cloud for 3d alignment
	*/
	virtual void computePoseFromDepthModel(MIL_ID mod_context,
		MIL_ID mod_result,
		SPoseDataQuat& pose,
		MIL_DOUBLE& rotate_angle,
		MIL_DOUBLE& error,
		MIL_DOUBLE param_1 = M_NULL,
		MIL_DOUBLE param_2 = M_NULL,
		MIL_DOUBLE param_3 = M_NULL,
		MIL_DOUBLE param_4 = M_NULL) override;

	virtual void getModelSquareROIFromResult(
		MIL_ID mod_result, 
		MIL_INT mod_index, 
		MIL_ID depth_image,
		MIL_INT& roi_x, 
		MIL_INT& roi_y, 
		MIL_INT& roi_size_x, 
		MIL_INT& roi_size_y) = 0;

	virtual MIL_DOUBLE getModelZAngleFromResult(MIL_ID mod_result, MIL_INT mod_index) { return 0.0; };
};

/*----------------------------------------
		  DepthCircleLocalization
----------------------------------------*/
class DepthCircleLocalization : public SolidShapeDepthLocalization
{
public:
	DepthCircleLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name);

	DepthCircleLocalization(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		config::Workspace* const workspace_ptr,
		const std::string& feature_group_name,
		std::map<std::string, MIL_ID> mil_mod_context_map);

	static void setContextFromPattern(
		MIL_ID mod_context,
		config::Workspace_FeatureGroup::PatternWindow pattern,
		MIL_INT control_number);

private:
	virtual void getModelSquareROIFromResult(
		MIL_ID mod_result,
		MIL_INT mod_index,
		MIL_ID depth_image,
		MIL_INT& roi_x,
		MIL_INT& roi_y,
		MIL_INT& roi_size_x,
		MIL_INT& roi_size_y) override;
};
