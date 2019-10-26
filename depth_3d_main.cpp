#include "depth_3d.h"

#include <google/protobuf/text_format.h>
#include <google/protobuf/map.h>

#include "camera_calib_trial.h"
#include "depth_camera.h"

int main()
{
#if 0 
	double image_size_x_ = 960, image_size_y_ = 640;
	double model_size_x = 50/*MmodInquire(mod_depth_context_, M_CONTEXT, M_BOX_SIZE_X, M_NULL)*/;
	double model_size_y = 40/*MmodInquire(mod_depth_context_, M_CONTEXT, M_BOX_SIZE_X, M_NULL)*/;
	double scale = 0.5;

	std::map<int, std::pair<double, double>> model_pose; // model pose
	model_pose.emplace(0, std::make_pair<double, double>(1.4, 1.5));
	model_pose.emplace(1, std::make_pair<double, double>(58, 128));
	model_pose.emplace(2, std::make_pair<double, double>(57.5, 129));
	model_pose.emplace(3, std::make_pair<double, double>(57.5, 128));
	model_pose.emplace(4, std::make_pair<double, double>(300.7, 345.7));

	std::vector<int> out_vector;
	out_vector.reserve(model_pose.size());
	common_utils::composeSearchOrder(image_size_x_, image_size_y_, model_size_x, model_size_y, scale,
		model_pose, SearchOrder::/*LEFT_RIGHT_TOP_DOWN*/TOP_DOWN_LEFT_RIGHT, out_vector);

	return 1;
#endif

#if 0
	std::string out_path = "C:\\yt_data\\19.05.13.18.48.28 - Copy.log";
	utils::translateBinary("C:\\yt_data\\19.05.13.18.48.28 [UTC] - Copy.log", out_path);
	return 0;
#endif

	/*std::string prototxt_file_name = "C:/yt_data/fm810gix/fm810gix.workspace";*/
	std::string prototxt_file_name = "C:/yt_data/real_sense/real_sense.workspace";
	config::Workspace workspace_config;
	std::string content = utils::readTxtFromFile(prototxt_file_name);
	google::protobuf::TextFormat::ParseFromString(content, &workspace_config);

	MIL_ID MilApplication = MappAlloc(M_NULL, M_DEFAULT, M_NULL);
	MIL_ID MilSystemGenTL = MsysAlloc(M_DEFAULT, /*M_SYSTEM_GENTL*/M_SYSTEM_HOST, M_DEFAULT, M_DEFAULT, M_NULL);
	MIL_ID MilDisplay = M_NULL;
	MdispAlloc(MilSystemGenTL, M_DEFAULT, MIL_TEXT("M_DEFAULT"), M_DEFAULT, &MilDisplay);
	MIL_INT MilWindowID = 0;

	// rectangle model
	
#if 1
	std::string model_name = "feature_group_5";
	//std::string model_name = /*"feature_group_4"*//*"feature_group_3"*//*"feature_group_2"*//*"feature_group_1"*/;
	SearchOrder search_order = SearchOrder::RIGHT_LEFT_TOP_DOWN/*LEFT_RIGHT_DOWN_TOP*//*RIGHT_LEFT_TOP_DOWN*//*RIGHT_LEFT_DOWN_TOP*/;

#endif

#if 0
	std::shared_ptr<Camera> camera_ptr = std::make_shared<FM810IXCamera>(
		MilApplication,
		MilSystemGenTL,
		MilDisplay,
		MilWindowID,
		&workspace_config,
		"camera_1");

	if (camera_ptr->isConnected())	// connect
	{
		//MosGetch();

		std::cout << "\n[singleGrab] 1" << std::endl;
		camera_ptr->singleGrab();		// software trigger (single grab)
		MosGetch();

		std::cout << "\n[singleGrab] 2" << std::endl;
		camera_ptr->singleGrab();		// software trigger (single grab)
		MosGetch();

		std::cout << "\n[disconnet]" << std::endl;
		//camera_ptr->disconnect();
	}
	
	camera_ptr.reset();
#endif

#if 0
	std::shared_ptr<Camera> camera_ptr =
		std::move(DepthCameraManager::createDepthCamPtr(
			MilApplication,
			MilSystemGenTL,
			MilDisplay,
			MilWindowID,
			&workspace_config,
			"camera_1"));
	camera_ptr->setCameraOutput(CameraOutputMode::SHOW_ONLY);

	bool can_stop = false;
	if (camera_ptr->isConnected())
	{
		std::thread t_thread([&]()
		{
			while (!can_stop)
			{
				camera_ptr->singleGrab();
			}

			camera_ptr->stopCamera();
			camera_ptr.reset();
		});

		MosPrintf(MIL_TEXT("Press <Enter> to exit."));
		MosGetch();
		can_stop = true;

		if (t_thread.joinable())
			t_thread.join();
	}
#endif

#if /*1*/ 0
	std::shared_ptr<Camera> camera_ptr =
		std::move(DepthCameraManager::createDepthCamPtr(
			MilApplication,
			MilSystemGenTL,
			MilDisplay,
			MilWindowID,
			&workspace_config,
			"camera_1"));

	camera_ptr->setCameraOutput(CameraOutputMode::SHOW_ONLY);

	bool can_stop = false;
	if (camera_ptr->isConnected())
	{
		std::thread t_thread([&]()
		{
			while (!can_stop)
			{
				camera_ptr->singleGrab();
			}
			camera_ptr.reset();
		});


		MosPrintf(MIL_TEXT("Press <Enter> to exit."));
		MosGetch();
		can_stop = true;

		if (t_thread.joinable())
			t_thread.join();
	}
#endif

#if 0
	//@ camera allocate, config & capture

	std::shared_ptr<Camera> camera_ptr = std::make_shared<BaslerToFCamera>(
		MilApplication, 
		MilSystemGenTL, 
		MilDisplay, 
		MilWindowID, 
		&workspace_config, 
		"camera_1");

	// continuous grab
	{
		camera_ptr->continuousGrab();
		MosPrintf(MIL_TEXT("Start continuous grab."));
		MosPrintf(MIL_TEXT("Press <Enter> to stop."));
		MosGetch();
		camera_ptr->stopContinuousGrab();
	}

	//// single grab
	//{
	//	camera_ptr->singleGrab();
	//	camera_ptr->saveImage("C:/daoai/test.wef");

	//	MosPrintf(MIL_TEXT("Start single grab."));
	//	MosPrintf(MIL_TEXT("Press <Enter> to exit."));
	//	MosGetch();
	//}

	camera_ptr.reset();

#endif

#if 0
	//@ feature group - model define test

	std::shared_ptr<BaslerToFCamera> camera_ptr = std::make_shared<BaslerToFCamera>(
		MilApplication,
		MilSystemGenTL,
		MilDisplay,
		MilWindowID,
		&workspace_config,
		"camera_1");

	if (camera_ptr->isConnected())
	{
		MIL_ID image_id = camera_ptr->getImageID();

		/*std::string mil_cal_path = "C:/daoai/src/protobuf/prototxt/calibration/calibration_1/MilDepthTrialCalibration.mca";*/
		wchar_t* wc_mil_cal_path_ptr = utils::string2Wchar_tPtr(mil_cal_path);
		MIL_ID MilCal = McalRestore(wc_mil_cal_path_ptr, MilSystemGenTL, M_DEFAULT, M_NULL);
		delete wc_mil_cal_path_ptr;

		SPoseDataQuat pose_world_to_cam;
		McalGetCoordinateSystem(MilCal,
			M_CAMERA_COORDINATE_SYSTEM,
			M_ABSOLUTE_COORDINATE_SYSTEM,
			M_TRANSLATION, M_NULL,
			&pose_world_to_cam.x,
			&pose_world_to_cam.y,
			&pose_world_to_cam.z,
			M_NULL);

		McalGetCoordinateSystem(MilCal,
			M_CAMERA_COORDINATE_SYSTEM,
			M_ABSOLUTE_COORDINATE_SYSTEM,
			M_ROTATION_QUATERNION,
			M_NULL,
			&pose_world_to_cam.q1,
			&pose_world_to_cam.q2,
			&pose_world_to_cam.q3,
			&pose_world_to_cam.q4);

		std::unique_ptr<DepthFeatureGroup> feature_group_ptr;
		if(model_type = M_RECTANGLE)
			feature_group_ptr = std::make_unique<DepthRectFeature>(MilApplication,
				MilSystemGenTL,
				MilDisplay,
				MilWindowID,
				&workspace_config,
				model_name);
		else if(model_type == M_CIRCLE)
			feature_group_ptr = std::make_unique<DepthCircleFeature>(MilApplication,
				MilSystemGenTL,
				MilDisplay,
				MilWindowID,
				&workspace_config,
				model_name);

		if (camera_ptr->singleGrab())
		{
			MIL_INT keyPressed = 0;
			//while (keyPressed != 'q')
			{
				//replace pose_world_to_cam by real-time world to camera pose 
				SPoseDataQuat pose_world_to_cam;

				McalGetCoordinateSystem(MilCal,
					M_CAMERA_COORDINATE_SYSTEM,
					M_ABSOLUTE_COORDINATE_SYSTEM,
					M_TRANSLATION, M_NULL,
					&pose_world_to_cam.x,
					&pose_world_to_cam.y,
					&pose_world_to_cam.z,
					M_NULL);

				McalGetCoordinateSystem(MilCal,
					M_CAMERA_COORDINATE_SYSTEM,
					M_ABSOLUTE_COORDINATE_SYSTEM,
					M_ROTATION_QUATERNION,
					M_NULL,
					&pose_world_to_cam.q1,
					&pose_world_to_cam.q2,
					&pose_world_to_cam.q3,
					&pose_world_to_cam.q4);

				feature_group_ptr->createNewFeatureFromDisplay(camera_ptr->getImageID(), camera_ptr->getRangeImageID(), pose_world_to_cam);
				feature_group_ptr->defineFeature();

				/*MosPrintf(MIL_TEXT("Press <Q> to finish define."));
				MosPrintf(MIL_TEXT("Press any to redefine."));
				keyPressed = MosGetch();*/
			}

			if (feature_group_ptr->defineSuccessed())
			{
				feature_group_ptr->displayOneContext("");
				feature_group_ptr->saveModelContext();
			}
		}

		MosPrintf(MIL_TEXT("Press <Enter> to exit."));
		MosGetch();

		McalFree(MilCal);
		feature_group_ptr.reset();
	}

	camera_ptr.reset();

#endif

#if 0
	//@ camera calibration

	std::shared_ptr</*FM810IXCamera*/FM810GIXE1Camera> camera_ptr = std::make_shared</*FM810IXCamera*/FM810GIXE1Camera>(
		MilApplication,
		MilSystemGenTL,
		MilDisplay,
		MilWindowID,
		&workspace_config,
		"camera_1");

	if (camera_ptr->isConnected())
	{
		MIL_ID image_id = camera_ptr->getImageID();
		std::shared_ptr<CameraCalibTrial> calib = std::make_shared<CameraCalibTrial>(MilSystemGenTL,
			MilDisplay,
			MilWindowID,
			&workspace_config,
			"calibration_1",
			image_id);

		do {
			if (!camera_ptr->singleGrab()) break;
			calib->start();
		} while (!calib->isSuccessed());
		MIL_ID MilCal = calib->getCalibration();

		MosPrintf(MIL_TEXT("Press <Enter> to continue.\n"));
		MosGetch();

		calib.reset();
	}

	camera_ptr.reset();

#endif

#if 0

	// @ Geo-pattern localization

	std::shared_ptr<Camera> camera_ptr =
		std::move(DepthCameraManager::createDepthCamPtr(
			MilApplication,
			MilSystemGenTL,
			MilDisplay,
			MilWindowID,
			&workspace_config,
			"camera_1"));

	camera_ptr->setCameraOutput(CameraOutputMode::LOCALIZATON);

	if (camera_ptr->isConnected())
	{
		MIL_ID image_id = camera_ptr->getImageID();

		std::shared_ptr<Localization> localizer = std::make_shared<DepthSegmentLocalization>(MilApplication,
			MilSystemGenTL, MilDisplay, MilWindowID, &workspace_config, model_name);

		SPoseDataQuat pose_base_to_tool = SPoseDataQuat();

		bool can_stop = false;
		bool is_localization_stop = false;
		bool has_result = false;
		std::thread t_thread([&]()
		{
			while (!can_stop)
			{
				if (camera_ptr->singleGrab(false))
				{
					if (is_localization_stop)
						localizer->resetlocateState();

					localizer->pushFrame(camera_ptr->getImageID(), camera_ptr->getRangeImageID(), pose_base_to_tool,
						is_localization_stop, has_result);

					if (has_result)
					{
						SPoseDataQuat model_pose = SPoseDataQuat();
						localizer->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);

						std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "T: [", model_pose.x, ", ", model_pose.y, ", ", model_pose.z, "]"
							" R: [",
							model_pose.q1, ", ", model_pose.q2, ", ", model_pose.q3, ", ", model_pose.q4, "]\n"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
					}
				}
			}

			localizer.reset();
			camera_ptr.reset();
		});


		MosPrintf(MIL_TEXT("Press <Enter> to exit.\n"));
		MosGetch();
		can_stop = true;

		if (t_thread.joinable())
			t_thread.join();
	}

#endif

#if 1
	// @ segment-pick localization

	std::shared_ptr<Camera> camera_ptr =
		std::move(DepthCameraManager::createDepthCamPtr(
			MilApplication,
			MilSystemGenTL,
			MilDisplay,
			MilWindowID,
			&workspace_config,
			"camera_1"));

	camera_ptr->setCameraOutput(CameraOutputMode::LOCALIZATON);

	if (camera_ptr->isConnected())
	{
		MIL_ID image_id = camera_ptr->getImageID();

		std::shared_ptr<Localization> localizer = std::make_shared<DepthSegmentLocalization>(MilApplication,
			MilSystemGenTL, MilDisplay, MilWindowID, &workspace_config, model_name);

		SPoseDataQuat pose_base_to_tool = SPoseDataQuat();

		bool can_stop = false;
		bool is_localization_stop = false;
		bool has_result = false;
		std::thread t_thread([&]()
		{
			while (!can_stop)
			{
				if (camera_ptr->singleGrab(false))
				{
					if (is_localization_stop)
						localizer->resetlocateState();

					localizer->pushFrame(camera_ptr->getImageID(), camera_ptr->getRangeImageID(), pose_base_to_tool,
						is_localization_stop, has_result);

					if (has_result)
					{
						SPoseDataQuat model_pose = SPoseDataQuat();
						localizer->getModelPose(ModelPoseType::MODEL_TO_WORLD, model_pose);

						std::string o_msg = printCoutMsg(LEVEL_CUSTOMER_VISIABLE, "T: [", model_pose.x, ", ", model_pose.y, ", ", model_pose.z, "]"
							" R: [",
							model_pose.q1, ", ", model_pose.q2, ", ", model_pose.q3, ", ", model_pose.q4, "]\n"); if (!o_msg.empty()) LOG(INFO) << "[INFO] " + o_msg;
					}
				}
			}

			localizer.reset();
			camera_ptr.reset();
		});


		MosPrintf(MIL_TEXT("Press <Enter> to exit.\n"));
		MosGetch();
		can_stop = true;

		if (t_thread.joinable())
			t_thread.join();
	}

#endif

	/*std::string msg_string;
	google::protobuf::TextFormat::PrintToString(workspace_config, &msg_string);
	utils::writeTxtToFile(prototxt_file_name, msg_string);*/

	MosGetch();

	MdispFree(MilDisplay);
	MsysFree(MilSystemGenTL);
	MappFree(MilApplication);
}
