#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>

#include "depthai_ros_msgs/TrackedFeatures.h"
#include "ros/ros.h"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/FeatureTracker.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkIn.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include <depthai_bridge/ImuConverter.hpp>
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/TrackedFeaturesConverter.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "feature_tracker_node");
    ros::NodeHandle pnh("~");

    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);
    // Define sources and outputs
    auto monoCamB = pipeline.create<dai::node::MonoCamera>();
    auto monoCamD = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerCamB = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerCamD = pipeline.create<dai::node::FeatureTracker>();
    auto imu = pipeline.create<dai::node::IMU>();

    auto xoutTrackedFeaturesCamB = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesCamD = pipeline.create<dai::node::XLinkOut>();
    auto xoutImageCamB = pipeline.create<dai::node::XLinkOut>();
    auto xoutImageCamD = pipeline.create<dai::node::XLinkOut>();
    auto xoutImu = pipeline.create<dai::node::XLinkOut>();

    xoutTrackedFeaturesCamB->setStreamName("trackedFeaturesLeft");
    xoutTrackedFeaturesCamD->setStreamName("trackedFeaturesRight");
    xoutImageCamB->setStreamName("camB") ; 
    xoutImageCamD->setStreamName("camD") ; 
    xoutImu->setStreamName("imu") ; 

    // Properties
    monoCamB->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    monoCamB->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoCamB->setFps(20) ; 
    monoCamD->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    monoCamD->setBoardSocket(dai::CameraBoardSocket::CAM_D);
    monoCamD->setFps(20) ; 
    // Imu
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
    imu->setBatchReportThreshold(5);
    imu->setMaxBatchReports(20);  // Get one message only for now.

    // Linking
    monoCamB->out.link(featureTrackerCamB->inputImage);
    featureTrackerCamB->outputFeatures.link(xoutTrackedFeaturesCamB->input);
    featureTrackerCamB->passthroughInputImage.link(xoutImageCamB->input) ; 

    monoCamD->out.link(featureTrackerCamD->inputImage);
    featureTrackerCamD->outputFeatures.link(xoutTrackedFeaturesCamD->input);
    featureTrackerCamD->passthroughInputImage.link(xoutImageCamD->input) ; 
    imu->out.link(xoutImu->input) ;

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerCamB->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerCamD->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerCamD->initialConfig.get();
    featureTrackerConfig.cornerDetector.type = dai::RawFeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI ; 
    featureTrackerConfig.featureMaintainer.enable = true ; 
    featureTrackerConfig.featureMaintainer.minimumDistanceBetweenFeatures = 15 ; 
    // featureTrackerConfig.featureMaintainer.trackedFeatureThreshold = 15 ; 
    featureTrackerConfig.cornerDetector.numTargetFeatures = 300 ; 
    featureTrackerConfig.cornerDetector.cellGridDimension = 4 ; 
    // featureTrackerConfig.cornerDetector.thresholds.min = 20 ; 
    featureTrackerConfig.motionEstimator.enable = true ; 
    featureTrackerConfig.motionEstimator.type = dai::RawFeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW ; 
    featureTrackerCamB->initialConfig.set(featureTrackerConfig) ; 
    featureTrackerCamD->initialConfig.set(featureTrackerConfig) ; 

    std::shared_ptr<dai::Device> device;
    std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

    std::cout << "Listing available devices..." << std::endl;
    device = std::make_shared<dai::Device>(pipeline);
    device->setTimesync(true) ; 
    auto outputFeaturesLeftQueue = device->getOutputQueue("trackedFeaturesLeft", 2, false);
    auto outputFeaturesRightQueue = device->getOutputQueue("trackedFeaturesRight", 2, false);
    auto camBQueue = device->getOutputQueue("camB", 2, false);
    auto camDQueue = device->getOutputQueue("camD", 2, false);
    auto imuQueue = device->getOutputQueue("imu", 10, false);
    std::string tfPrefix = "som";
    const std::string camBPubName = std::string("camB");
    const std::string camDPubName = std::string("camD");
    
    dai::rosBridge::TrackedFeaturesConverter camBTrackerConverter(tfPrefix + "camB_optical_frame", true);

    dai::rosBridge::TrackedFeaturesConverter camDTrackerConverter(tfPrefix + "camD_optical_frame", true);
    
    dai::rosBridge::ImuConverter imuConverter("imu_frame", dai::rosBridge::ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL ,0, 0,0,0,false,false,false);

    dai::rosBridge::ImageConverter camBconverter(tfPrefix + "camB_camera_optical_frame", false,false);

    dai::rosBridge::ImageConverter camDconverter(tfPrefix + "camD_camera_optical_frame", false,false);

    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures> featuresPubL(
        outputFeaturesLeftQueue,
        pnh,
        std::string("features_camB"),
        std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, &camBTrackerConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "features_camB");

    featuresPubL.addPublisherCallback();

    dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures> featuresPubR(
        outputFeaturesRightQueue,
        pnh,
        std::string("features_camD"),
        std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, &camDTrackerConverter, std::placeholders::_1, std::placeholders::_2),
        30,
        "",
        "features_camD");

    featuresPubR.addPublisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
       imuQueue,
       pnh,
       std::string("imu"),
       std::bind(&dai::rosBridge::ImuConverter::toRosMsg, &imuConverter, std::placeholders::_1, std::placeholders::_2),
       10,
       "",
       "imu");

    imuPublish.addPublisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> camBPublish(
        camBQueue,
        pnh,
        camBPubName,
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &camBconverter, std::placeholders::_1, std::placeholders::_2),
        10);
    
    camBPublish.addPublisherCallback();

    dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> camDPublish(
        camDQueue,
        pnh,
        camDPubName,
        std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &camDconverter, std::placeholders::_1, std::placeholders::_2),
        10);
    
    camDPublish.addPublisherCallback();    

    std::cout << "Ready." << std::endl;
    ros::spin();

    return 0;
}
