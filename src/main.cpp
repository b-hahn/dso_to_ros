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

void vidCb(const sensor_msgs::ImageConstPtr img_left, const sensor_msgs::ImageConstPtr img_right)
{
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
	// int w = undistImg_left->w;
	// int h = undistImg_left->h;
	// MinimalImageB3* internalVideoImg = new MinimalImageB3(w, h);
	// for (int i = 0; i < w; ++i) {
	// 	for (int j = 0; j < h; ++j) {
	// 	// internalVideoImg->data[i * h + j][0] = internalVideoImg->data[i * h + j][1] =
	// 	//     internalVideoImg->data[i * h + j][2] =
	// 	//         undistImg_left->image[i * h + j] * 0.8 > 255.0f ? 255.0 : undistImg_left->image[i * h + j] * 0.8;
	// 	internalVideoImg->data[i * h + j][0] = internalVideoImg->data[i * h + j][1] =
	// 		internalVideoImg->data[i * h + j][2] =
	// 		undistImg_right->image[i * h + j] * 0.8 > 255.0f ? 255.0 : undistImg_right->image[i * h + j] * 0.8;
	// 	}
	// }
	// IOWrap::displayImage("right cam", internalVideoImg);
	// IOWrap::waitKey(10);

        // float* left = undistImg_left->image;
	// cv::Mat cvimg_left((int)cv_ptr_left->image.rows, (int)cv_ptr_left->image.cols, CV_8U, (unsigned char*)left);
	// cv::imshow("left image", cvimg_left);
	// cv::imwrite("left_cam.png", cvimg_left);
	// cv::waitkey(0);

	fullSystem->addActiveFrame(undistImg_left, undistImg_right, frameID);
	frameID++;
	delete undistImg_left;
	delete undistImg_right;

}





int main( int argc, char** argv )
{
	ros::init(argc, argv, "dso_live");



	for(int i=1; i<argc;i++) {
		printf("reading arg %s\n", argv[i]);
		parseArgument(argv[i]);
	}


	setting_desiredImmatureDensity = 1000;
	setting_desiredPointDensity = 2000;
	setting_minFrames = 5;
	setting_maxFrames = 7;
	setting_maxOptIterations=4;
	setting_minOptIterations=1;
	setting_logStuff = false;
	setting_kfGlobalWeight = 1.3;


	// printf("MODE WITH CALIBRATION, but without exposure times!\n");
	setting_photometricCalibration = 0;
	setting_affineOptModeA = 0;
	setting_affineOptModeB = 0;



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
	image_transport::SubscriberFilter left_sub, right_sub;
	image_transport::ImageTransport it(nh);
	left_sub.subscribe(it, "cam02/image_raw", 1/* , "raw" */);
	right_sub.subscribe(it, "cam03/image_raw", 1/* , "raw" */);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
	// typedef message_filters::SimpleFilter<ApproximatePolicy> ApproximateSync;
	boost::shared_ptr<ApproximateSync> approximate_sync;

	approximate_sync.reset(new ApproximateSync(ApproximatePolicy(10 /* queue_size */), left_sub, right_sub));
	approximate_sync->registerCallback(boost::bind(vidCb, _1, _2));

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

