/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>

#include <ros/ros.h>
#include <signal.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"Utils.h"

using namespace std;

class ImageGrabber
{
public:
    float scale;
    unsigned long nImages{0};
    double totalRescaleTime{0.0};

public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, float _scale=1.0):scale(_scale), mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

ORB_SLAM2::System *SLAM;
ImageGrabber *igb;

void SlamShutdown(int sig)
{

    // Stop all threads
    SLAM->Shutdown();

    // Save camera trajectory
    SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    // Delete SLAM system
    delete SLAM;

    // Measure time and delete ImageGrabber object
    if(igb->scale != 1.0) {
        cout << "Total time spent rescaling " << nanoToMilli(igb->totalRescaleTime) << " milliseconds" << endl;
        cout << "Given " << igb->nImages << " images, time per image was "
             << nanoToMilli(igb->totalRescaleTime / igb->nImages) << " milliseconds" << endl;
    }

    delete igb;

    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc < 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings [scale]" << endl;
        ros::shutdown();
        return 1;
    }

    float scale = 1.0;
    if(argc >= 4){
        scale = (float) atof(argv[3]);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    SLAM = new ORB_SLAM2::System(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true,scale);

    igb = new ImageGrabber(SLAM, scale);

    ros::NodeHandle nodeHandler;
    ros::NodeHandle pnh("~");

    // Get camera topic
    string video{"/camera/image_raw"};
    pnh.getParam("cameraTopic", video);

    ros::Subscriber sub = nodeHandler.subscribe(video, 1, &ImageGrabber::GrabImage,igb);

    signal(SIGINT, SlamShutdown);

    ros::spin();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(scale != 1.0) {
        cv::Size new_dimensions( int(scale * msg->width), int(scale * msg->height));
        cv::Mat resized;

        totalRescaleTime += funcTime(cv::resize, cv_ptr->image, resized, new_dimensions, 0, 0, cv::INTER_LINEAR);

        mpSLAM->TrackMonocular(resized,cv_ptr->header.stamp.toSec());
    }
    else
        mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    ++nImages;
}


