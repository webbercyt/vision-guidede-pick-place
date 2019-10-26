#pragma once
#pragma warning(disable : 4251)

#include "depth_camera.h"
#include "PhoXi.h"
#include <queue>

using namespace pho::api;

/*----------------------------------------
		     PhotoXiScanner
----------------------------------------*/
class PhotoXiScanner : public DepthCamera
{
public:
	PhotoXiScanner(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);

	~PhotoXiScanner();

	virtual bool singleGrab(bool display_image = true) override;
	virtual void continuousGrabHelper() override;
	virtual bool stopContinuousGrab() override;
	virtual int setExposureTime(MIL_DOUBLE exposure_time) override;

	virtual MIL_ID getRangeImageID() override;
	virtual bool saveRangeImage(std::string file_name, MIL_ID key) override;
	virtual bool setCameraOutput(CameraOutputMode mode) override;

	static PointCloud32f getPCLut(MIL_ID key);

protected:
	MIL_INT image_size_x_;
	MIL_INT image_size_y_;

private:

	virtual void setFov() {};
	void connect();
	void disconnect();
	void conifg();
	bool softwareTrigger();

	PPhoXi phoXi_device_;
	std::mutex image_grab_lock_;

	MIL_ID current_pc_lut_id_;

	static std::mutex point_cloud_map_lock_;
	static int pc_buffer_id_;
	static std::deque<std::pair<MIL_ID, PointCloud32f>> point_cloud_deque_;
};

/*----------------------------------------
			 PhotoXiScannerM
----------------------------------------*/
class PhotoXiScannerM : public PhotoXiScanner
{
public:
	PhotoXiScannerM(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);
};

/*----------------------------------------
			 PhotoXiScannerL
----------------------------------------*/
class PhotoXiScannerL : public PhotoXiScanner
{
public:
	PhotoXiScannerL(MIL_ID mil_application,
		MIL_ID mil_system,
		MIL_ID mil_display,
		MIL_ID mil_window_id,
		Workspace* const workspace_ptr,
		const std::string& node_name);
};

/*----------------------------------------
			 PhoXiDataWrapper
----------------------------------------*/
class PhoXiDataWrapper : public DepthCameraDataWrapper
{
public:
	PhoXiDataWrapper(MIL_ID mil_system,
		const Workspace_DepthCamera workspace_depth_camera,
		MIL_ID mil_calibration);

	virtual bool createDepthMap(MIL_ID pc_lut_id,
		MIL_ID mask_image,
		MIL_ID depth_map,
		MIL_ID pc_label,
		double max_depth,
		double,
		bool inquire_camera,
		bool,
		bool,
		SPoseDataQuat transform = SPoseDataQuat()) override;

	static SDepthData setupDepthData(config::Workspace_DepthCamera workspace_depth_camera);

private:

};