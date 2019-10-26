#pragma once

#include "depth_camera.h"
#include <librealsense2/rs.hpp>

/*----------------------------------------
			RealSenseD400Series
----------------------------------------*/
class RealSenseD400Series : public DepthCamera
{
public:
	RealSenseD400Series(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~RealSenseD400Series();

	virtual bool singleGrab(bool display_image = true) override;
	virtual void continuousGrabHelper() override;
	virtual bool setCameraOutput(CameraOutputMode mode) override;

protected:
	void config();
	bool fetchFrame();
	virtual int getImageSizeX() = 0;
	virtual int getImageSizeY() = 0;
	virtual void setFov() {};

	void startLaserEmitting();
	void stopLaserEmitting();

	std::unique_ptr<rs2::pipeline> pipeline_;
	std::unique_ptr<rs2::align> align_to_color_;
	rs2::config config_;
	rs2::pipeline_profile profile_;

	bool is_laser_running_;
	bool do_align_;
	float depth_scale_;
};

/*----------------------------------------
			RealSenseD415Camera
----------------------------------------*/
class RealSenseD415Camera : public RealSenseD400Series
{
public:
	RealSenseD415Camera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~RealSenseD415Camera();

private:
	virtual void setFov() override;
	virtual int getImageSizeX() override;
	virtual int getImageSizeY() override;

	static const int D415_IMAGE_SIZE_X = 1280;
	static const int D415_IMAGE_SIZE_Y = 720;
};

/*----------------------------------------
		   RealSenseDataWrapper
----------------------------------------*/
class RealSenseDataWrapper : public DepthCameraDataWrapper
{
public:
	RealSenseDataWrapper(MIL_ID mil_system,
		const Workspace_DepthCamera workspace_depth_camera,
		MIL_ID mil_calibration);

	~RealSenseDataWrapper();

private:
	virtual void composePointCloudPoint(MIL_ID range_image, double max_depth) override;

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
		int neighbor_y_offset_pos = 0) override;

	rs2_intrinsics* intrinsics;
};