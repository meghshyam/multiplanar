/*****************************************************************************************
 * CapturePlane.cpp
 *
 *       Created on: 21-Feb-2015
 *    Last Modified: 23-Sep-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description: 
 *
 * Date             Author                          Modification
 * 12-Sep-2016  Sona Praneeth Akula         * Added comments to the code
 * 19-Sep-2016  Sona Praneeth Akula         * Added code for on_key_down(); key c, g
 * 21-Sep-2016  Sona Praneeth Akula         * Added threading to TopView
 * 22-Sep-2016  Sona Praneeth Akula         * Added code to destroy TopView once its job is done
 * 23-Sep-2016  Sona Praneeth Akula         * Added code for testing path planning
 *****************************************************************************************/

#include "CapturePlane.hpp"
#include <cvd/gl_helpers.h>
#include <cv_bridge/cv_bridge.h>

CapturePlane::CapturePlane(ControlUINode *cnode)
{
    node = cnode;
}

CapturePlane::~CapturePlane()
{

}

void
CapturePlane::startSystem()
{
    cout << "CapturePlane Started\n";
    start();
}

void
CapturePlane::stopSystem()
{
    join();
}

void
CapturePlane::run()
{
    node->captureTheCurrentPlane();
    node->image_gui->clearInputVectors();
    node->image_gui->numPointsClicked = 0;
    cout << "Stopping CapturePlane\n";
    stopSystem();
    cout << "Stopped CapturePlane\n";
}
