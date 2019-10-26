#pragma once
#include "depth_camera.h"

/*----------------------------------------
	         BaslerToFCamera
----------------------------------------*/
class BaslerToFCamera : public DepthCamera
{
public:
	BaslerToFCamera(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);
	~BaslerToFCamera();

	virtual bool singleGrab(bool display_image = true) override;
	virtual void continuousGrabHelper() override;

private:

	virtual void setFov() override;
	virtual bool isGrabPreCheckSucceed() override;
	void config();

	static MIL_INT MFTYPE continuousGrabHook(MIL_INT hook_type, MIL_ID mil_hook_id, void* hook_data_ptr);
	MIL_INT grabContinuous(MIL_ID mil_hook_id);

	/*
	@brief camera grad buffer & control :
		gradded images will be buffered in mil_grabbed_images_buffer_
		when require_images_ is configured to true, the buffered images
		will be copied to mil_image_ & mil_range_image_			
	*/
	std::vector<MIL_ID> mil_grabbed_images_buffer_;
	bool require_images_;
	std::mutex require_images_lock_;
	MIL_INT require_images_wait_time_;	// in ms
};

/*----------------------------------------
	      BaslerToFDataWrapper
----------------------------------------*/
class BaslerToFDataWrapper : public DepthCameraDataWrapper
{
public:
	BaslerToFDataWrapper(MIL_ID mil_system,
		const Workspace_DepthCamera workspace_depth_camera,
		MIL_ID mil_calibration);

private:
	virtual void composePointCloudPoint(MIL_ID range_image, double max_depth) override;

	std::vector<MIL_DOUBLE> x_unit_;
	std::vector<MIL_DOUBLE> y_unit_;
	std::vector<MIL_DOUBLE> z_unit_;
};