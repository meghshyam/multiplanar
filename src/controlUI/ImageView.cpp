/**
Created on 21st February 2015
Author: Anirudh Vemula
*/

#include "ros/ros.h"
#include "ImageView.h"
#include <cvd/gl_helpers.h>
//#include <gvars3/instances.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "GLWindow2.h"
#include <iostream>
#include <fstream>
#include <string>
#include "std_msgs/String.h"
#include "../HelperFunctions.h"
#include "ControlUINode.h"

ImageView::ImageView(ControlUINode *cnode) {
	frameWidth = frameHeight = 0;

	video_channel = nh_.resolveName("ardrone/image_raw");
	command_channel = nh_.resolveName("tum_ardrone/com");

	vid_sub = nh_.subscribe(video_channel, 10, &ImageView::vidCb, this);
	tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ImageView::comCb, this);

	tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);

	node = cnode;
}

ImageView::~ImageView() {

}

void ImageView::vidCb (const sensor_msgs::ImageConstPtr img) {
	newImage(img);
}

void ImageView::comCb (const std_msgs::StringConstPtr str) {

}

void ImageView::startSystem() {
	keepRunning = true;
	changeSizeNextRender = false;
	start();
}

void ImageView::stopSystem() {
	keepRunning = false;
	new_frame_signal.notify_all();
	join();
}

void ImageView::ResetInternal() {
	mimFrameBW.resize(CVD::ImageRef(frameWidth, frameHeight));
	mimFrameBW_workingCopy.resize(CVD::ImageRef(frameWidth, frameHeight));
}


void ImageView::newImage(sensor_msgs::ImageConstPtr img) {
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);

	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	//copy to internal image, convert to bw, set flag
	if (ros::Time::now() - img->header.stamp > ros::Duration(30.0)) {
		mimFrameTimeRos = (ros::Time::now()  - ros::Duration(0.001));
	}
	else {
		mimFrameTimeRos = img->header.stamp;
	}

	mimFrameTime = getMS(mimFrameTimeRos);

	mimFrameSEQ = img->header.seq;

	//copy to mimFrame
	if (mimFrameBW.size().x != img->width || mimFrameBW.size().y != img->height)
		mimFrameBW.resize(CVD::ImageRef(img->width, img->height));

	memcpy(mimFrameBW.data(), cv_ptr->image.data, img->width * img->height);
	newImageAvailable = true;

	lock.unlock();
	new_frame_signal.notify_all();
}


void ImageView::run() {
	while(!newImageAvailable)
		usleep(100000);
	newImageAvailable = false;
	while(!newImageAvailable)
		usleep(100000);

	// read image height and width
	frameHeight = mimFrameBW.size().y;
	frameWidth = mimFrameBW.size().x;

	ResetInternal();

	//create window
	myGLWindow = new GLWindow2(CVD::ImageRef(frameWidth, frameHeight), "DRONE CAMERA FEED", this);
	myGLWindow->set_title("DRONE CAMERA FEED");

	changeSizeNextRender = true;
	if (frameWidth < 640) {
		desiredWindowSize = CVD::ImageRef(frameWidth*2, frameHeight*2);
	}
	else {
		desiredWindowSize = CVD::ImageRef(frameWidth, frameHeight);
	}

	boost::unique_lock<boost::mutex> lock(new_frame_signal_mutex);

	while(keepRunning) {
		if(newImageAvailable) {
			newImageAvailable = false;

			//copy to working copy
			mimFrameBW_workingCopy.copy_from(mimFrameBW);
			mimFrameTime_workingCopy = mimFrameTime;
			mimFrameSEQ_workingCopy = mimFrameSEQ;
			mimFrameTimeRos_workingCopy = mimFrameTimeRos;

			//release lock
			lock.unlock();

			renderFrame();

			if (changeSizeNextRender) {
				myGLWindow->set_size(desiredWindowSize);
				changeSizeNextRender = false;
			}

			lock.lock();
		}
		else
			new_frame_signal.wait(lock);
	}

	lock.unlock();
	delete myGLWindow;

}

void ImageView::renderFrame() {

	//setup
	myGLWindow->SetupViewport();
	myGLWindow->SetupVideoOrtho();
	myGLWindow->SetupVideoRasterPosAndZoom();

	glDrawPixels(mimFrameBW_workingCopy);

	myGLWindow->swap_buffers();
	myGLWindow->HandlePendingEvents();
}

// keyboard input
void ImageView::on_key_down(int key) {

}

void ImageView::on_mouse_down(CVD::ImageRef where, int state, int button) {
	//double x = 4*(where.x/(double)this->myGLWindow->size().x - 0.5);
	//double y = -4*(where.y/(double)this->myGLWindow->size().y - 0.5);

	double x;
	double y;

	if(this->myGLWindow->size().x==640) {
		x = where.x;
	}
	else if(this->myGLWindow->size().x!=0){
		x = (640.0*where.x)/(this->myGLWindow->size().x);
	}

	if(this->myGLWindow->size().y==360) {
		y = where.y;
	}
	else if(this->myGLWindow->size().y!=0){
		y = (360.0*where.y)/(this->myGLWindow->size().y);
	}

	printf("X and Y of point clicked : (%.3f, %.3f)\n", x, y);
}