/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/





#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "util/settings.h"
#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include "IOWrapper/ImageDisplay.h"
#include "ROSOutputWrapper.h"


#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "cv_bridge/cv_bridge.h"
#include <opencv/cv.h>
#include <opencv2/core/eigen.hpp>


std::string calib = "";
std::string vignetteFile = "";
std::string gammaFile = "";
bool useSampleOutput=false;

using namespace dso;

void parseArgument(char* arg)
{
	int option;
	char buf[1000];

	if(1==sscanf(arg,"sampleoutput=%d",&option))
	{
		if(option==1)
		{
			useSampleOutput = true;
			printf("USING SAMPLE OUTPUT WRAPPER!\n");
		}
		return;
	}

	if(1==sscanf(arg,"quiet=%d",&option))
	{
		if(option==1)
		{
			setting_debugout_runquiet = true;
			printf("QUIET MODE, I'll shut up!\n");
		}
		return;
	}


	if(1==sscanf(arg,"nolog=%d",&option))
	{
		if(option==1)
		{
			setting_logStuff = false;
			printf("DISABLE LOGGING!\n");
		}
		return;
	}

	if(1==sscanf(arg,"nogui=%d",&option))
	{
		if(option==1)
		{
			disableAllDisplay = true;
			printf("NO GUI!\n");
		}
		return;
	}
	if(1==sscanf(arg,"nomt=%d",&option))
	{
		if(option==1)
		{
			multiThreading = false;
			printf("NO MultiThreading!\n");
		}
		return;
	}
	if(1==sscanf(arg,"calib=%s",buf))
	{
		calib = buf;
		printf("loading calibration from %s!\n", calib.c_str());
		return;
	}
	if(1==sscanf(arg,"vignette=%s",buf))
	{
		vignetteFile = buf;
		printf("loading vignette from %s!\n", vignetteFile.c_str());
		return;
	}

	if(1==sscanf(arg,"gamma=%s",buf))
	{
		gammaFile = buf;
		printf("loading gammaCalib from %s!\n", gammaFile.c_str());
		return;
	}

	if(1==sscanf(arg,"num_active_pts=%d",&option))
	{
		setting_desiredPointDensity = option;
		printf("Setting number of active points to %f!\n", setting_desiredPointDensity);
		return;
	}

	if(1==sscanf(arg,"num_immature_pts=%d",&option))
	{
		setting_desiredImmatureDensity = option;
		printf("Setting number of immature points to %f!\n", setting_desiredImmatureDensity);
		return;
	}

	printf("could not parse argument \"%s\"!!\n", arg);
}




FullSystem* fullSystem = 0;
Undistort* undistorter = 0;
int frameID = 0;


void vidCb(const sensor_msgs::ImageConstPtr img_left,
		   const sensor_msgs::ImageConstPtr img_right,
		   const sensor_msgs::ImageConstPtr img_semantics,
		   int image_width,
		   int image_height)
{
	std::string red = "\033[0;31;1m";
	std::string reset = "\033[0m";

	cv_bridge::CvImagePtr cv_ptr_left = cv_bridge::toCvCopy(img_left, sensor_msgs::image_encodings::MONO8);
	cv_bridge::CvImagePtr cv_ptr_right = cv_bridge::toCvCopy(img_right, sensor_msgs::image_encodings::MONO8);
	// cv::Rect ROI(0, 0, 1242, 375);  // TODO: remove if unnecessary
	// cv_ptr_left->image = cv_ptr_left->image(ROI);
	// cv_ptr_right->image = cv_ptr_right->image(ROI);
	assert(cv_ptr_left->image.type() == CV_8U);
	assert(cv_ptr_left->image.channels() == 1);
	assert(cv_ptr_right->image.type() == CV_8U);
	assert(cv_ptr_right->image.channels() == 1);

	// printf("setting_fullResetRequested: %d\n", setting_fullResetRequested);
	if(setting_fullResetRequested)
	{
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		delete fullSystem;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		fullSystem = new FullSystem();
		fullSystem->linearizeOperation=false;
		fullSystem->outputWrapper = wraps;
	    if(undistorter->photometricUndist != 0)
	    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		setting_fullResetRequested=false;
	}

	MinimalImageB minImg_left((int)cv_ptr_left->image.cols, (int)cv_ptr_left->image.rows,(unsigned char*)cv_ptr_left->image.data);
	MinimalImageB minImg_right((int)cv_ptr_right->image.cols, (int)cv_ptr_right->image.rows,(unsigned char*)cv_ptr_right->image.data);
	ImageAndExposure* undistImg_left = undistorter->undistort<unsigned char>(&minImg_left/* , 1,0, 1.0f */);
	undistImg_left->timestamp = img_left->header.stamp.toSec();
	ImageAndExposure* undistImg_right = undistorter->undistort<unsigned char>(&minImg_right/* , 1,0, 1.0f */);
	undistImg_right->timestamp = img_left->header.stamp.toSec();

	// color image
	cv_bridge::CvImagePtr cv_ptr_left_color = cv_bridge::toCvCopy(img_left, sensor_msgs::image_encodings::RGB8);
	assert(cv_ptr_left_color->image.channels() == 3);
	assert(cv_ptr_left_color->image.type() == CV_8UC3);
	cv::Rect crop_region(0, 0, image_width, image_height);
	cv_ptr_left_color->image = cv_ptr_left_color->image(crop_region).clone();  // need clone() to get continuous cv::Mat memory layout

	std::vector<uint8_t> image_color;
	std::shared_ptr<std::vector<uint8_t>> image_color_ptr = nullptr;
	if (cv_ptr_left_color->image.isContinuous()) {
		// cv::rectangle(cv_ptr_left_color->image, cv::Point(0,0), cv::Point(300, 300), cv::Scalar(0, 0, 255), cv::FILLED);
		image_color.assign(cv_ptr_left_color->image.datastart, cv_ptr_left_color->image.dataend);
		// cv::imwrite("red_image.png", cv_ptr_left_color->image);
		image_color_ptr = std::make_shared<std::vector<uint8_t>>(image_color);
	} else {
		std::cout << red << "OpenCV image not continuous! Cannot pass color image to DSO..." << reset << std::endl;
	}

	// semantic segmentation
	std::vector<uint8_t> image_semantics;
	std::shared_ptr<std::vector<uint8_t>> image_semantics_ptr = nullptr;

	if (img_semantics != nullptr) {
		cv_bridge::CvImagePtr cv_ptr_semantics = cv_bridge::toCvCopy(img_semantics, sensor_msgs::image_encodings::RGB8);
		assert(cv_ptr_semantics->image.channels() == 3);
		assert(cv_ptr_semantics->image.type() == CV_8UC3);
		cv_ptr_semantics->image = cv_ptr_semantics->image(crop_region).clone();  // need clone() to get continuous cv::Mat memory layout
		if (cv_ptr_semantics->image.isContinuous()) {
			// cv::rectangle(cv_ptr_semantics->image, cv::Point(0,0), cv::Point(300, 300), cv::Scalar(0, 0, 255), cv::FILLED);
			image_semantics.assign(cv_ptr_semantics->image.datastart, cv_ptr_semantics->image.dataend);
			// cv::imwrite("red_image.png", cv_ptr_semantics->image);
			image_semantics_ptr = std::make_shared<std::vector<uint8_t>>(image_semantics);
		} else {
			std::cout << red << "OpenCV image not continuous! Cannot pass semantics image to DSO..." << reset << std::endl;
		}
	}
	else
	{
		// set semantics image to black
		// TODO: this won't work if black is not a recognized semantic class in SegMap
        LOG(INFO) << "No semantics image available!";
		image_semantics.assign(image_width*image_height, 0);
	}


	fullSystem->addActiveFrame(undistImg_left, undistImg_right, frameID, image_color_ptr, image_semantics_ptr);
	std::cout << "Added active frame!" << std::endl;
	frameID++;
	delete undistImg_left;
	delete undistImg_right;

}





int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_live");


	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 2000;

	for(int i=1; i<argc;i++) {
		printf("reading arg %s\n", argv[i]);
		parseArgument(argv[i]);
	}

	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;

	setting_photometricCalibration = 0;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;
	bool use_semantics = true;

    undistorter = Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

	printf("global calib size: %d, %d\n",(int)undistorter->getSize()[0], (int)undistorter->getSize()[1]);
    setGlobalCalib(
            (int)undistorter->getSize()[0],
            (int)undistorter->getSize()[1],
            undistorter->getK().cast<float>());
	baseline = undistorter->getBl();

	fullSystem = new FullSystem();
    fullSystem->linearizeOperation=false;


    if(!disableAllDisplay)
	    fullSystem->outputWrapper.push_back(new IOWrap::PangolinDSOViewer(
	    		 (int)undistorter->getSize()[0],
	    		 (int)undistorter->getSize()[1]));

    if(undistorter->photometricUndist != 0) {
		printf("Using setGammaFunction!!\n");
    	fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
	} else {
		printf("Not using setGammaFunction!!\n");
	}
    ros::NodeHandle nh;
	image_transport::SubscriberFilter left_sub, right_sub, semantics_sub;
	image_transport::ImageTransport it(nh);
	left_sub.subscribe(it, "cam02/image_raw", 1/* , "raw" */);
	right_sub.subscribe(it, "cam03/image_raw", 1/* , "raw" */);
	semantics_sub.subscribe(it, "semantics", 1/* , "raw" */);

	typedef message_filters::sync_policies::
		ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>
		ApproximatePolicy;
	typedef message_filters::sync_policies::
		ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image>
		ApproximatePolicySemantics;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	typedef message_filters::Synchronizer<ApproximatePolicySemantics> ApproximateSyncSemantics;
	// typedef message_filters::SimpleFilter<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ApproximateSync> approximate_sync;
	boost::shared_ptr<ApproximateSyncSemantics> approximate_sync_semantics;

	if (use_semantics) {
		std::cout << "Using semantic segmentation!" << std::endl;
		approximate_sync_semantics.reset(new ApproximateSyncSemantics(ApproximatePolicySemantics(10 /* queue_size */), left_sub, right_sub, semantics_sub));
		approximate_sync_semantics->registerCallback(boost::bind(vidCb, _1, _2, _3, undistorter->getSize()[0], undistorter->getSize()[1]));
	}
	else
	{
		std::cout << "Not using semantic segmentation!" << std::endl;
		approximate_sync.reset(new ApproximateSync(ApproximatePolicy(10 /* queue_size */), left_sub, right_sub));
		approximate_sync->registerCallback(boost::bind(vidCb, _1, _2, nullptr, undistorter->getSize()[0], undistorter->getSize()[1]));
	}
    if(useSampleOutput)
        fullSystem->outputWrapper.push_back(new IOWrap::ROSOutputWrapper(nh));
	// ros::Subscriber imgSub = nh.subscribe("image", 1, &vidCb);

	ros::spin();

    for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
    {
        ow->join();
        delete ow;
    }

    delete undistorter;
    delete fullSystem;

	return 0;
}

