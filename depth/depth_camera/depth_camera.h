#pragma once
#include "camera.h"

#define PC_DEFAULT_LABEL M_POINT_CLOUD_LABEL(1)	// 0 is invalid
#define PC_LABEL(x) x > 0 ? M_POINT_CLOUD_LABEL(x) : M_POINT_CLOUD_LABEL(1)	// 0 is invalid

static const MIL_DOUBLE PI = 3.14159265358989;
static const MIL_DOUBLE RAD_TO_DEG = 180.0 / PI;
static const MIL_DOUBLE DEG_TO_RAD = PI / 180.0;

/*
	@ brief: SDepthData restore the necessary data 
		for create real scale depth map
*/
struct SDepthData
{
	double depth_view_x_pose_;	// the x start pose of 3d bounding box 
	double depth_view_x_size_;	// the x size of 3d bounding box 
	double depth_view_y_pose_;	// the y start pose of 3d bounding box 
	double depth_view_y_size_;	// the y size of 3d bounding box 
	double depth_view_z_pose_;	// the z start pose of 3d bounding box 
	double depth_view_z_size_;	// the z size of 3d bounding box 

	double depth_map_mm_per_pixel_;	// scale of depth map: mm/pixel

	int depth_map_size_x_;	// x size of depth map
	int depth_map_size_y_;	// y size of depth map

	SDepthData();
};

/*----------------------------------------
			DepthCameraManager
----------------------------------------*/
namespace DepthCameraManager
{
	/*
		@ brief: create & return depth camera pointer
			depending on camera model name
	*/
	extern std::shared_ptr<Camera> createDepthCamPtr(
		MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	/*
		@ brief: return all support camera models
	*/
	extern std::vector<std::string> getCameraModelNames();
}

class DepthCameraDataWrapper;
namespace DPCLib	// depth and point cloud tool library
{
	/*
		@ brief: compute a new bounding box only contain the point cloud
	*/
	extern bool cropPointCloudToFixBox(MIL_ID point_cloud, MIL_ID pc_label, MIL_ID calibration);

	extern void getPointCloudBoudingBox(
		MIL_ID point_clond,
		MIL_DOUBLE& min_x,
		MIL_DOUBLE& min_y,
		MIL_DOUBLE& min_z,
		MIL_DOUBLE& max_x,
		MIL_DOUBLE& max_y,
		MIL_DOUBLE& max_z);

	/*
		@ brief: map 16-bit depth map to 8 bit for MIL model find
	*/
	extern void mapDepthImageTo8Bits(MIL_ID mil_system, MIL_ID src, MIL_ID dst);

	/*
		@ brief: create & return depth camera data wrapper 
			pointer depending on camera model name
	*/
	extern std::shared_ptr<DepthCameraDataWrapper> createDepthMapGeneratorPtr(
		MIL_ID mil_system,
		const Workspace_DepthCamera workspace_depth_camera,
		MIL_ID mil_calibration);

	extern SDepthData getCameraDepthData(config::Workspace_DepthCamera workspace_depth_camera);

	/*
		@ brief: compute the current ground distance & rotation
			depending on camera model name, calibration data (calibration) 
			& current robot pose (pose_base_to_tool)
	*/
	extern void getGroundEraseParams(
		config::Workspace* const workspace_ptr,
		const std::string camera_name,
		MIL_ID calibration,
		SPoseDataQuat pose_base_to_tool,
		double& max_depth,
		SPoseDataQuat& ground_rotation);
}

/*----------------------------------------
			    DepthCamera
----------------------------------------*/
class DepthCamera : public Camera
{
public:
	DepthCamera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~DepthCamera();

	virtual bool isConnected() const override;

	// Camera settings
	virtual int setExposureTime(MIL_DOUBLE exposure_time) override;
	virtual int setGain(MIL_DOUBLE gain) override;

	virtual bool saveRangeImage(std::string file_name, MIL_ID key = 0) override;
	virtual MIL_ID getRangeImageID() override;

	/*
		@ brief: change the displayed image
	*/
	virtual int setDisplayImage(std::string component_name) override;
	virtual std::vector<std::string> getDispImageComponents() override;

protected:

	virtual void setFov() = 0;
	virtual bool savePauseImage(std::string file_name) override;
	void updateDepthRange();

	MIL_ID mil_range_image_;
	MIL_ID mil_display_image_;
	MIL_ID mil_display_lut_;

	int view_depth_min_;
	int view_depth_max_;
};

/*----------------------------------------
			VirtualDepthCamera
----------------------------------------*/
class VirtualDepthCamera : public DepthCamera
{
public:
	VirtualDepthCamera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~VirtualDepthCamera();

	virtual bool singleGrab(bool display_image = true) override;
	virtual void continuousGrabHelper() override;

private:
	virtual void setFov() override;
};

class DepthCameraDataWrapper
{
public:
	DepthCameraDataWrapper(MIL_ID mil_system,
		const Workspace_DepthCamera workspace_depth_camera,
		MIL_ID mil_calibration);
	~DepthCameraDataWrapper();

	/*
		@ brief: create depth map from (masked) range image
			MIL_ID pc_label: if use multiple labels, must make sure this is greater than 0 (0 is invalid)
	*/
	virtual bool createDepthMap(MIL_ID range_image,
		MIL_ID mask_image,
		MIL_ID depth_map,
		MIL_ID pc_label,
		double max_depth = 0.0,
		double size_depth_opening = 1,
		bool inquire_camera = true,
		bool enabel_range_denoising = true,
		bool enable_depth_smoothing = true,
		SPoseDataQuat transform = SPoseDataQuat());

	MIL_ID getPointCloudID();
	bool cropPointCloudToFixBox(MIL_ID pc_label);
	void resetPCBoundingBox();
	void clearPointCloud();
	void getPCBoundingBox(
		double& extract_box_x,
		double& extract_box_y,
		double& extract_box_z,
		double& extract_box_size_x,
		double& extract_box_size_y,
		double& extract_box_size_z);
	MIL_DOUBLE getDepthMapPixelUint() { return depth_data_.depth_map_mm_per_pixel_; };
	SDepthData getDepthData() { return depth_data_; };

	/*
		@ brief: compute the 3d pose of the pixel(input_x, input_y) 
			with the consideration of its neighbors

		@ parameters
			output_x: output 3d pose x value
			output_y: output 3d pose y value
			output_z: output 3d pose z value
			cond_range_image: the image used to do 3d points projection
			neighbor_x_offset_neg: neighbor range -x
			neighbor_x_offset_pos: neighbor range +x
			neighbor_y_offset_neg: neighbor range -y
			neighbor_y_offset_pos: neighbor range +y
	*/
	virtual bool get3DPointUsingNeighborFilter(
		int input_x,
		int input_y,
		double& output_x,
		double& output_y,
		double& output_z,
		MIL_ID cond_range_image,
		int neighbor_x_offset_neg = 0,
		int neighbor_x_offset_pos = 0,
		int neighbor_y_offset_neg = 0,
		int neighbor_y_offset_pos = 0);

	/*
		@ brief: compose the data used to create depth map from point cloud
	*/
	static SDepthData setupDepthData(config::Workspace_DepthCamera workspace_depth_camera);

protected:

	void createDepthMapFromPointCloud(
		MIL_ID depth_map,
		double size_depth_opening,
		bool enabel_range_denoising,
		bool enable_depth_smoothing,
		MIL_ID pc_label = M_ALL);

	/*
		@ brief: create 3d point cloud from range image

		@ parameters:
			max_depth: the maximum depth of the point. The point 
			with depth greater than max_depth will be ignored.
	*/
	virtual void composePointCloudPoint(MIL_ID range_image, double max_depth = 0) {};

	MIL_ID mil_system_;
	MIL_ID mil_calibration_;
	MIL_ID point_cloud_;

	MIL_DOUBLE range_offset_;
	MIL_DOUBLE range_scale_;

	// range image to point cloud buffers
	std::vector<MIL_DOUBLE> x_world_;
	std::vector<MIL_DOUBLE> y_world_;
	std::vector<MIL_DOUBLE> z_world_;

	SDepthData depth_data_;
};