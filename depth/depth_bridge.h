#ifndef _DEPTH_BRIDE_H__
#define _DEPTH_BRIDE_H__

#include "mil.h"
#include "depth_3d.h"
#include "config.pb.h"
#include "utils.h"
#include "daoai.h"
#include "bridge.h"

namespace DepthFeatureDefine 
{
	enum DepthModelType { 
		DMT_SHAPE = -1,
		DMT_SHAPE_RECT = M_RECTANGLE,
		DMT_SHAPE_CIRCLE = M_CIRCLE,
		DMT_GEOMETRIC = M_GEOMETRIC,
		DMT_SEGMENT = M_SEGMENT,
		DMT_UNDEFINED = M_NULL};
}

using namespace DepthFeatureDefine;
class DepthBridge : public Bridge
{
public:
	DepthBridge(const std::string workspace_path = "", const MIL_INT mil_window_id = 0);
	~DepthBridge();

	//Camera
	virtual bool continuousGrab(const std::string camera_name);
	virtual bool singleGrab(const std::string camera_name, bool display_image = true);
	virtual bool connectCamera(const std::string& camera_name) override;
	virtual bool endCameraAcquisition(const std::string camera_name) override;
	virtual int setDisplayImage(const std::string& camera_name, std::string image_component_name) override;
	virtual std::vector<std::string> getDispImageComponents(const std::string& camera_name) override;
	virtual std::vector<std::string> getCameraModelNames() override;
	
	// Calibration
	virtual bool calibrationWithRobotMovement(const std::string calibration_name) override;

	// Feature Group
	virtual bool loadReferenceImage(const std::string feature_group_name, bool show_feature = true) override;
	virtual bool updateReferenceImage(const std::string feature_group_name) override;
	virtual bool createNewFeature(const std::string feature_group_name, const std::string feature_name = "") override;
	virtual bool createNewGeo(const std::string feature_group_name, const std::string feature_name) override;
	bool createNewPatternWindow(const std::string feature_group_name, const std::string pattern_window_name);
	bool updateDefinedFeature(const std::string feature_group_name);
	virtual bool displaySearchRange(const std::string feature_group_name, const std::string feature_name = "") override;

	DepthModelType getCurrentModelType(const std::string feature_group_name, bool is_shape_check = false);
	DepthModelType getModelType(const std::string model_type_str);
	std::string getModelTypeText(DepthModelType model_type);
	bool setCurrentModelType(const std::string feature_group_name, DepthModelType model_type);
	bool createNewPatternWindows(const std::string feature_group_name, DepthModelType model_type);
	bool displayModelSizes(const std::string feature_group_name);
	
	std::vector<std::string> getModelTypeStringList();
	std::vector<std::string> getShapeTypeStringList();

	// Training
	virtual bool createNewTraining(int training_number, LocalizationMode dimension = DEPTH) override;
	
	// Accracy Test
	virtual bool accuTestInit(const std::string accu_test_name,
		std::string& robot_name, 
		std::string& camera_name, 
		std::string& feature_group_name,
		std::string& pose_calculator_name, 
		SPoseDataQuat& reference_pose,
		SPoseDataQuat& model_reference_pose,
		std::vector<SPoseDataQuat>& pose_plan_quat_vec, 
		const bool& using_images = false);
	
	virtual bool accuTest1(const std::string accu_test_name) override;
	virtual bool accuTest2(const std::string accu_test_name) override;

	// Job
	virtual bool jobInit() override;
	virtual bool jobLocateObject(const std::string& job_name, const SPoseDataQuat& capture_pose) override;
	virtual bool jobLocateObjectMethod2(const std::string& job_name, SPoseDataQuat& capture_pose) override;
	virtual int jobMoveToPose(const std::string& job_name, const SPoseDataQuat& taught_pose) override;
	virtual bool runJob1() override;
	virtual bool runJob2() override;
	virtual bool jobSaveDebugImage(const std::string& camera_name, const std::string& job_name, const SPoseDataQuat& recv_pose, int& idx) override;
	virtual bool debug_feature_job(const std::string job_name, const std::string & image_name, const std::string&, const std::string&) override;
	virtual bool singleJobInit(const std::string& job_name) override;

	// General
	bool localizationPtrInit(const std::string& feature_group_name, const int search_order = -1);

private:
	std::shared_ptr<DepthLocalization> createModelLocalization(const std::string feature_group_name);
	std::map<std::string, std::vector<std::string>> camera_display_image_component_map_;

	MIL_ID debug_image_ = M_NULL;
	MIL_ID debug_range_image_ = M_NULL;
};

#endif
