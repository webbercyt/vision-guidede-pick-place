#pragma once
#include "depth_camera.h"
#include "common.hpp"

/*----------------------------------------
			  CallbackData

	@ brief: CallbackData restore the necessary 
			 data for TYCamera data processing

----------------------------------------*/
struct TY_CAMERA_CALIB_INFO;
struct CallbackData 
{
	int     index;
	void*   h_device_;
	bool    needUndistort;

	TY_CAMERA_CALIB_INFO* depth_calib;
	TY_CAMERA_CALIB_INFO* color_calib;

	CallbackData();
	~CallbackData();
};

/*----------------------------------------
				TYCamera
----------------------------------------*/
struct TY_FRAME_DATA;
struct TY_DEVICE_BASE_INFO;
class TYCamera : public DepthCamera
{
public:
	TYCamera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~TYCamera();

	virtual int setExposureTime(MIL_DOUBLE exposure_time) override;
	virtual int setGain(MIL_DOUBLE gain) override;

	virtual bool singleGrab(bool display_image = true) override;
	virtual void continuousGrabHelper() override;

protected:

	virtual void setFov() {};
	virtual bool openDevice();
	virtual bool isGrabPreCheckSucceed() override;
	virtual void handleFrame(TY_FRAME_DATA* frame, void* userdata);
	void config();

	void* h_iface_ = NULL;
	void* h_device_ = NULL;

	char* frame_buffer_[2];
	CallbackData cb_data_;
	TY_ISP_HANDLE hColorIspHandle_ = NULL;
};

/*----------------------------------------
			  FM810IXCamera
----------------------------------------*/
class FM810IXCamera : public TYCamera
{
public:
	FM810IXCamera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~FM810IXCamera();

private:
	virtual void setFov() override;
	virtual bool openDevice() override;
};

/*----------------------------------------
			FM810GIXE1Camera
----------------------------------------*/
class FM810GIXE1Camera : public TYCamera
{
public:
	FM810GIXE1Camera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~FM810GIXE1Camera();

private:
	virtual void setFov() override;
	virtual bool openDevice() override;
	virtual void handleFrame(TY_FRAME_DATA* frame, void* userdata) override;

	MIL_ID bayer_color_buf_;
};

/*----------------------------------------
			  TYDataWrapper
----------------------------------------*/
class TYDataWrapper : public DepthCameraDataWrapper
{
public:
	TYDataWrapper(MIL_ID mil_system, const Workspace_DepthCamera workspace_depth_camera, MIL_ID mil_calibration);

	~TYDataWrapper();
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

private:
	virtual void composePointCloudPoint(MIL_ID range_image, double max_depth) override;

	TY_CAMERA_CALIB_INFO* pc_project_calib_;	// restore the parameters used to project range image to PC
};