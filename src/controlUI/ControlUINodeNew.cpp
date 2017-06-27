/*****************************************************************************************
 * ControlUINode.cpp
 *
 *       Created on: 19-Feb-2015
 *    Last Modified: 13-Oct-2016
 *  Original Author: Anirudh Vemula
 *   Current Author: Meghshyam Govind Prasad
 *   Current Author: Sona Praneeth Akula
 *          Project: Quadcopter
 *      Description:
 *
 * Date             Author                          Modification
 * 12-Sep-2016  Sona Praneeth Akula         * Added comments to the code
 * 21-Sep-2016  Sona Praneeth Akula         * Added code to land the quadcopter
 * 12-Oct-2016  Sona Praneeth Akula         * 
 *****************************************************************************************/

#include "ControlUINodeNew.h"
#include "ros/ros.h"
#include "tum_ardrone/keypoint_coord.h"
#include "tum_ardrone/filter_state.h"
#include "ransacPlaneFit.h"
#include "ImageView.h"
#include "allHeaders.hpp"


// OpenCV related stuff
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "ardrone_autonomy/RecordEnable.h"
#include "Multiple-Plane-JLinkage/conversion.hpp"
#include "Multiple-Plane-JLinkage/utilities.hpp"
#include "Multiple-Plane-JLinkage/makeBoundingRects.hpp"
#include "Multiple-Plane-JLinkage/multiplePlanes.hpp"

#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <string>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <list>


using namespace std;
using namespace cv;

pthread_mutex_t ControlUINode::keyPoint_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::pose_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::command_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::navdata_CS = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ControlUINode::changeyaw_CS = PTHREAD_MUTEX_INITIALIZER;


ControlUINode::ControlUINode()
{
    // Setting up the debug and log utilties
    if(testing_code)
    {
        SET_DEBUG_LEVEL(debug_level);
        SET_LOG_LEVEL(log_level);
    }

    PRINT_LOG(1, "Initiating ControlUINode ...\n");
    PRINT_LOG(1, "Initiating various publishers and\n");
    PRINT_LOG(1, "subscribers for channels using tum_ardrone and ardrone.\n");

    // Command channel for sending/receiving commands (goto)
    command_channel = nh_.resolveName("tum_ardrone/com");
    // Channel for receiving the keypoints at various levels (1 to 4)
    keypoint_channel = nh_.resolveName("/keypoint_coord");
    // Channel for getting the current quadcopter position (x, y, z, roll, pitch, yaw)
    pose_channel = nh_.resolveName("ardrone/predictedPose");
    // Channel for landing the quadcopter
    land_channel = nh_.resolveName("ardrone/land");
    // Channel for ardrone navdata (to access battery information)
    // Reference: http://ardrone-autonomy.readthedocs.io/en/latest/reading.html?highlight=battery
    navdata_channel = nh_.resolveName("ardrone/navdata");
    // Subscribing for key point channel
    keypoint_coord_sub = nh_.subscribe(keypoint_channel, 10, &ControlUINode::keyPointDataCb, this);
    // Subscribing for pose channel. Currently using newPoseCb
    // pose_sub = nh_.subscribe(pose_channel, 10, &ControlUINode::poseCb, this);
    // Subscribing for pose channel
    new_pose_sub = nh_.subscribe(pose_channel, 10, &ControlUINode::newPoseCb, this);
    tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);
    tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ControlUINode::comCb, this);
    // For recording video
    video = nh_.serviceClient<ardrone_autonomy::RecordEnable>("ardrone/setrecord");
    timer_checkPos = nh_.createTimer(ros::Duration(pollingTime), &ControlUINode::checkPos, this);
    // timer_record = nh_.createTimer(ros::Duration(recordTime), &ControlUINode::recordVideo);
    // Channel for controlling landing commands
    land_pub = nh_.advertise<std_msgs::Empty>(land_channel, 1);
    // For battery
    navdata_sub = nh_.subscribe(navdata_channel, 10, &ControlUINode::navDataCb, this);

    // New CVD thread for running captureTheCurrenntPlaneCode
    // capture_plane = new CapturePlane(this);
    // New CVD thread for running alignQuadcopterToCurrentPlane code
    align_drone = new AlignDrone(this);
    // Initiating image view class which displays the "drone_controlUI" window
    image_gui = new ImageView(this);
    // Some variables
    ransacVerbose = true;
    useScaleFactor = true;
    threshold = 0.1;
    error_threshold = 0.2;
    recordTime = 3.5; // a second
    pollingTime = 0.5;
    record = true;
    targetSet = false;

    currentCommand = false;
    recordNow = false;
    notRecording = true;
    planeIndex = 0;

    // More variables
    // Whether it's seeing the plane for the first time
    _stage_of_plane_observation = true;
    // Is the plane big? requiring multiple attempts to cover ti
    _is_big_plane = false;
    // Check if the plane in consideration is  covered completely
    _is_plane_covered = false;
    // Number of planes covered completelt till now
    _node_completed_number_of_planes = 0;
    //
    _node_number_of_planes = 0;
    // Are you using moveDroneViaSetOfPoints. Indicating the drone is currently moving
    justNavigation = false;
    // Has the drone completed executing the command sent?
    traverseComplete = false;
    // Current command navigation number
    just_navigation_command_number = -1;
    // Total number of commands to be sent to the drone
    just_navigation_total_commands = -1;
    // Is it just changing the yaw? or also travelling
    linearTraversal = true;
    // Has my changeyaw_CS lock been released?
    changeyawLockReleased = 0;
    _next_plane_dir = CLOCKWISE;
    _next_plane_angle = 0.0;
    _plane_d = 0.0;
    _fixed_distance = _node_max_distance;
    _fixed_height = 0.7;
    _fixed_height_set = false;
    _is_adjusted = false;
    /* working heuristics */
    _move_heuristic = 0.5;
    _angle_heuristic = 4.0;
    /*_move_heuristic = 0.75;
    _angle_heuristic = 5.0;*/
    _jlinkage_calls = 0;
    _sig_plane_index = 0;
    _actual_plane_index = 0;
    _capture_mode_time = 0.0;
    _traversal_mode_time = 0.0;

    PRINT_LOG(1, "Initiated ControlUINode.\n");
}

ControlUINode::~ControlUINode()
{

}

/**
 * @brief Ardrone Autonomy Navadata Callback
 * @details Currently used for capturing battery information
 * @param navPtr
 */
void
ControlUINode::navDataCb(const ardrone_autonomy::Navdata navPtr)
{
    pthread_mutex_lock(&navdata_CS);
    if(!testing_code)
    {
        if (navPtr.batteryPercent < 15)
        {
            PRINT_LOG(1, "Current battery percentage: " << navPtr.batteryPercent << "\n");
            sendLand();
        }
    }
    pthread_mutex_unlock(&navdata_CS);
}

void
ControlUINode::keyPointDataCb (const tum_ardrone::keypoint_coordConstPtr coordPtr)
{
    pthread_mutex_lock(&keyPoint_CS);
    numPoints = coordPtr->num;
    load2dPoints(coordPtr->x_img, coordPtr->y_img);
    load3dPoints(coordPtr->x_w, coordPtr->y_w, coordPtr->z_w);
    loadLevels(coordPtr->levels);
    assert(_2d_points.size()==_3d_points.size() && _3d_points.size()==_levels.size());
    pthread_mutex_unlock(&keyPoint_CS);
}

void
ControlUINode::getCurrentPositionOfDrone()
{
    _node_current_pos_of_drone.clear();
    pthread_mutex_lock(&pose_CS);
        _node_current_pos_of_drone.push_back((double)x_drone);
        _node_current_pos_of_drone.push_back((double)y_drone);
        _node_current_pos_of_drone.push_back((double)z_drone);
        _node_current_pos_of_drone.push_back((double)yaw);
    pthread_mutex_unlock(&pose_CS);
}

void
ControlUINode::getCurrentPositionOfDrone(vector<double> &current_drone_pos)
{
    current_drone_pos.clear();
    pthread_mutex_lock(&pose_CS);
        current_drone_pos.push_back((double)x_drone);
        current_drone_pos.push_back((double)y_drone);
        current_drone_pos.push_back((double)z_drone);
        current_drone_pos.push_back((double)yaw);
    pthread_mutex_unlock(&pose_CS);
}

float
ControlUINode::distance (vector<int> p1, vector<float> p2)
{
    return sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) + (p2[1]-p1[1])*(p2[1]-p1[1]));
}

float
ControlUINode::distance3D (vector<float> p1, vector<float> p2)
{
    return sqrt((p2[0]-p1[0])*(p2[0]-p1[0]) +
                (p2[1]-p1[1])*(p2[1]-p1[1]) +
                (p2[2]-p1[2])*(p2[2]-p1[2]));
}

/**
 * @brief Equality function for 3d keypoints.
 * @details Does it need to be exact equality? Or some heuristic based distance threshold
 * @param
 * @return
 */
bool
ControlUINode::equal(vector<float> p1, vector<float> p2)
{
    if(distance3D(p1, p2) < 0.001)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief Fits 3d points for a single plane
 * @details Assumption: Only one plane is visible  at the current instant
 */
vector<float>
ControlUINode::fitPlane3dForTheCurrentPlane ()
{
    vector<vector<float> > _in_points;
    pthread_mutex_lock(&keyPoint_CS);
    for(unsigned int i = 0; i < _3d_points.size(); i++)
    {
        _in_points.push_back(_3d_points[i]);
    }
    pthread_mutex_unlock(&keyPoint_CS);
    _3d_plane = ransacPlaneFit(_in_points, ransacVerbose);
    _in_points.clear();
    return _3d_plane;
}

void
ControlUINode::sendLand()
{
    land_pub.publish(std_msgs::Empty());
}

void
ControlUINode::setMainAngles(const vector<double> &main_angles)
{
    _node_main_angles.clear();
    for (unsigned int i = 0; i < main_angles.size(); ++i)
    {
        _node_main_angles.push_back(main_angles[i]);
    }
    return ;
}

void
ControlUINode::setMainDirections(const vector<RotateDirection> &main_directions)
{
    _node_main_directions.clear();
    for (unsigned int i = 0; i < main_directions.size(); ++i)
    {
        _node_main_directions.push_back(main_directions[i]);
    }
    return ;
}

void
ControlUINode::setValues(int number_of_planes, float min_height_of_plane, 
                         float min_distance, float max_height_of_plane, float max_distance)
{
    _node_min_distance = min_distance;
    _node_max_distance = max_distance;
    _node_min_height_of_plane = min_height_of_plane;
    _node_max_height_of_plane = max_height_of_plane;
    _node_number_of_planes = number_of_planes;
}

void
ControlUINode::load2dPoints (vector<float> x_img, vector<float> y_img)
{
    _2d_points.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        vector<float> p;
        p.push_back(x_img[i]);
        p.push_back(y_img[i]);
        _2d_points.push_back(p);
    }
}

void
ControlUINode::load3dPoints (vector<float> x_w, vector<float> y_w, vector<float> z_w)
{
    pthread_mutex_lock(&pose_CS);
    _3d_points.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        vector<float> p;
        if(!useScaleFactor)
        {
            p.push_back(x_w[i]);
            p.push_back(y_w[i]);
            p.push_back(z_w[i]);
        }
        else
        {
            p.push_back(x_w[i]*scale + x_offset);
            p.push_back(y_w[i]*scale + y_offset);
            p.push_back(z_w[i]*scale_z + z_offset);
        }
        _3d_points.push_back(p);
    }
    pthread_mutex_unlock(&pose_CS);
}

void
ControlUINode::loadLevels (vector<int> levels)
{
    _levels.clear();
    for (int i = 0; i < numPoints; ++i)
    {
        _levels.push_back(levels[i]);
    }
}

vector<float>
ControlUINode::fitPlane3d (vector<int> ccPoints, vector<vector<int> > pointsClicked)
{
    vector<vector<float> > _in_points;
    vector<vector<int> > points;
    for(unsigned int i=0; i<ccPoints.size(); i++)
    {
        points.push_back(pointsClicked[ccPoints[i]]);
    }
    pthread_mutex_lock(&keyPoint_CS);
    for(unsigned int i=0; i<_2d_points.size(); i++)
    {
        if(liesInside(points, _2d_points[i]))
        {
            //printf("%f, %f, %f\n", _3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
            _in_points.push_back(_3d_points[i]);
        }
    }
    pthread_mutex_unlock(&keyPoint_CS);
    //ROS_INFO("Number of keypoints inside %d", _in_points.size());
    //ROS_INFO("Total number of keypoints %d", _3d_points.size());
    _3d_plane = ransacPlaneFit(_in_points, ransacVerbose);
    return _3d_plane;
}

void
ControlUINode::fitMultiplePlanes3d (vector<int> &ccPoints, vector<vector<int> > &pointsClicked,
                                    vector<vector<float> >&planeParameters,
                                    vector< vector<Point3f> > & continuousBoundingBoxPoints)
{
    vector<Point3f> _in_points;
    vector< vector<int> > points;
    for(unsigned int i = 0; i < ccPoints.size(); i++)
    {
        points.push_back(pointsClicked[ccPoints[i]]);
    }
    pthread_mutex_lock(&keyPoint_CS);
    for(unsigned int i = 0; i < _2d_points.size(); i++)
    {
        if(liesInside(points, _2d_points[i]))
        {
            //printf("%f, %f, %f\n", _3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
            Point3f featurePt;
            featurePt.x = _3d_points[i][0];
            featurePt.y = _3d_points[i][1];
            featurePt.z = _3d_points[i][2];
            _in_points.push_back(featurePt);
        }
    }
    pthread_mutex_unlock(&keyPoint_CS);
    findMultiplePlanes(_in_points, planeParameters, continuousBoundingBoxPoints);
}

void
ControlUINode::Loop ()
{
    while(nh_.ok())
    {
        ros::spinOnce();
    }
}

void
ControlUINode::comCb (const std_msgs::StringConstPtr str)
{
}

vector<float>
ControlUINode::searchNearest (vector<int> pt, bool considerAllLevels)
{

    pthread_mutex_lock(&keyPoint_CS);

    float min = -1;
    vector<float> minPt;

    if(!considerAllLevels)
    {
        for (unsigned int i=0; i<_2d_points.size(); i++)
        {
            if(_levels[i]==0)
            {
                if(min==-1)
                {
                    min = distance(pt, _2d_points[i]);
                    minPt = _3d_points[i];
                }
                else
                {
                    float s = distance(pt, _2d_points[i]);
                    if(s<min)
                    {
                        min = s;
                        minPt = _3d_points[i];
                    }
                }
            }
        }
    }
    else
    {
        for (unsigned int i=0; i<_2d_points.size(); i++)
        {
            if(min==-1)
            {
                min = distance(pt, _2d_points[i]);
                minPt = _3d_points[i];
            }
            else
            {
                float s = distance(pt, _2d_points[i]);
                if(s<min)
                {
                    min = s;
                    minPt = _3d_points[i];
                }
            }
        }
    }

    pthread_mutex_unlock(&keyPoint_CS);


    return minPt;
}

bool
ControlUINode::get2DPoint (vector<float> pt, vector<int> &p,
                            bool considerAllLevels)
{
    pthread_mutex_lock(&keyPoint_CS);
    // ROS_INFO("Total num %d\n", numPoints);
    bool found = false;
    float minDist = 10000000.0;
    int min = -1;
    if(!considerAllLevels)
    {
        for (unsigned int i = 0; i < _3d_points.size(); ++i)
        {
            if(_levels[i]==0 && distance3D(pt, _3d_points[i]) < threshold)
            {
                float s = distance3D(pt, _3d_points[i]);
                if(s<minDist)
                {
                    minDist = s;
                    min = i;
                }
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < _3d_points.size(); ++i)
        {
            if(distance3D(pt, _3d_points[i]) < threshold)
            {
                float s = distance3D(pt, _3d_points[i]);
                if(s<minDist)
                {
                    minDist = s;
                    min = i;
                }
            }
        }
    }
    if(min!=-1)
    {
        found = true;
        p.push_back((int)_2d_points[min][0]);
        p.push_back((int)_2d_points[min][1]);
        //ROS_INFO("The minimum distance is %f", minDist);
    }
    pthread_mutex_unlock(&keyPoint_CS);
    return found;
}

bool
ControlUINode::get2DPointNearest (vector<float> pt, vector<int> &p,
                                    bool considerAllLevels)
{
    pthread_mutex_lock(&keyPoint_CS);
    // ROS_INFO("Total num %d\n", numPoints);
    bool found = false;
    float minDist = 10000000.0;
    int min = -1;
    if(!considerAllLevels)
    {
        for (unsigned int i = 0; i < _3d_points.size(); ++i)
        {
            if(_levels[i]==0)
            {
                float s = distance3D(pt, _3d_points[i]);
                if(s<minDist)
                {
                    minDist = s;
                    min = i;
                }
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < _3d_points.size(); ++i)
        {
            //if(distance3D(pt, _3d_points[i]) < 0.05) {
                float s = distance3D(pt, _3d_points[i]);
                if(s<minDist)
                {
                    minDist = s;
                    min = i;
                }
            //}
        }
    }
    if(min!=-1)
    {
        found = true;
        p.push_back((int)_2d_points[min][0]);
        p.push_back((int)_2d_points[min][1]);
        //ROS_INFO("The minimum distance is %f", minDist);
    }
    pthread_mutex_unlock(&keyPoint_CS);
    return found;
}

int
ControlUINode::getNumKP(bool considerAllLevels)
{
    int c = 0;
    for (int i = 0; i < numPoints; ++i)
    {
        if(_levels[i]==0 && !considerAllLevels)
            c++;
        else if(considerAllLevels)
            c++;
    }
    return c;
}

void
ControlUINode::saveKeyPointInformation (int numFile)
{
    pthread_mutex_lock(&keyPoint_CS);
    //char * name = itoa(numFile);
    stringstream ss;
    ss << numFile;
    string s = ss.str();
    ofstream fp(s.c_str());
    fp << numPoints << endl;
    fp << endl;
    for(unsigned int i=0; i<_3d_points.size(); i++)
    {
        fp << _3d_points[i][0] << ", " << _3d_points[i][1]
                << ", " << _3d_points[i][2] << ", " << _levels[i] << endl;
    }
    fp.close();
    pthread_mutex_unlock(&keyPoint_CS);
}

vector< vector<float> >
ControlUINode::projectPoints (vector<int> ccPoints, vector<vector<float> > keyPoints)
{
    vector<vector<float> > pPoints;
    for(unsigned int i=0; i<ccPoints.size(); i++)
    {
        vector<float> v = projectPoint(_3d_plane, keyPoints[ccPoints[i]]);
        pPoints.push_back(v);
    }
    return pPoints;
}

void
ControlUINode::moveQuadcopter(const vector< vector<float> > &planeParameters,
                              const vector< vector<Point3f> > &continuousBoundingBoxPoints)
{
    PRINT_LOG(1, "Started.\n");
    double drone_length = 0.6;
    // Get the number of planes
    numberOfPlanes = planeParameters.size();
    // UV co-ordinates of bounding box points of the planes
    vector<Point2f> uvCoordinates;
    // UV axes for transformation between XYZ and UV co-ordinates
    vector<Point3f> uvAxes, xyzGridCoord;
    vector<float> uCoord, vCoord, uVector, vVector;
    // Vector containing the number of commands of recording videos for drone
    // = No. of rows * No. of columns in the grid
    // Cumulative counter
    startTargetPtIndex.resize(numberOfPlanes, 0);
    startTargetPtIndex[0] = 0;
    moveDronePathPoints.resize(numberOfPlanes, 0);
    moveDronePathPoints[0] = 0;
    // The position from where drone is hovering in order to move to plane numbered plane_index
    vector<double> prevPosition(3), previousPosition(4);
    double prevYaw = 0;
    // Apply a lock on commands
    pthread_mutex_lock(&command_CS);
    for (int plane_index = 0; plane_index < numberOfPlanes; ++plane_index)
    {
        PRINT_LOG(1, "Drone ready to capture for plane: " << plane_index+1 << ".\n");
        // Make parameters for making the grid
        // ax + by + cz + d = 0 Parameters for the plane numbered by plane_index
        // float a = planeParameters[plane_index][0];
        // float b = planeParameters[plane_index][1];
        // float c = planeParameters[plane_index][2];
        // float d = planeParameters[plane_index][3];
        // Clear the uv co-ordinates and axis vectors
        uvCoordinates.clear();
        uvAxes.clear();
        // Convert XYZ bounding points to UV coordinates
        PRINT_LOG(4, "Generating the UV axes for the plane: " << plane_index+1 << ".\n");
        PRINT_DEBUG(3, print1dVector(continuousBoundingBoxPoints[plane_index], "Bounding Box points for plane:\n", ""));
        PRINT_DEBUG(3, print1dVector(planeParameters[plane_index], "Plane Parameters for plane:\n", ""));
        vector<Point3f> shifted_cbb;
        shifted_cbb = continuousBoundingBoxPoints[plane_index];
        PRINT_DEBUG(4, print1dVector(shifted_cbb, "Initial shifted_cbb:\n", ""));
        float x_centroid = 0.0;
        float y_centroid = 0.0;
        float z_centroid = 0.0;
        for (unsigned int i = 0; i < shifted_cbb.size(); ++i)
        {
            x_centroid += shifted_cbb[i].x;
            y_centroid += shifted_cbb[i].y;
            z_centroid += shifted_cbb[i].z;
        }
        x_centroid /= shifted_cbb.size();
        y_centroid /= shifted_cbb.size();
        z_centroid /= shifted_cbb.size();
        for (unsigned int i = 0; i < shifted_cbb.size(); ++i)
        {
            shifted_cbb[i].x -= x_centroid;
            shifted_cbb[i].y -= y_centroid;
            shifted_cbb[i].z -= z_centroid;
        }
        PRINT_DEBUG(4, print1dVector(shifted_cbb, "After shifted_cbb:\n", ""));
        AllXYZToUVCoordinates(shifted_cbb, planeParameters[plane_index],
                              uvCoordinates, uvAxes);
        // Push the generated UV axis to the required vectors
        uVector.clear();
        uVector.push_back(uvAxes[0].x);
        uVector.push_back(uvAxes[0].y);
        uVector.push_back(uvAxes[0].z);
        vVector.clear();
        vVector.push_back(uvAxes[1].x);
        vVector.push_back(uvAxes[1].y);
        vVector.push_back(uvAxes[1].z);
        uCoord.clear();
        // Build the Grid by dividing the plane into cells
        PRINT_LOG(1, "Building the grid for the plane: " << plane_index+1 << ".\n");
        pGrid grid = buildPGrid(uvCoordinates);
        int num_rows = grid.rowSquares.size()-1, num_cols = 0;
        if(num_rows > 0)
        {
            num_cols = grid.rowSquares[0].size();
        }
        PRINT_LOG(1, "Number of rows in the grid: " << num_rows << ".\n");
        PRINT_LOG(1, "Number of cols in the grid: " << num_cols << ".\n");
        // Print the Grid co-ordinates
        // printGrid(grid, uvAxes, planeParameters[i]);
        // Vector containing target points from where the video recording is supposed to be done
        vector< vector<double> > pTargetPoints;
        // Calculate the angle to rotate to align the drone'e yaw to the plane's normal
        double desiredYaw = 0;
        Point3f projectedNormal(planeParameters[plane_index][0], planeParameters[plane_index][1], 0);
        Point3f yAxis(0, 1, 0);
        desiredYaw = findAngle(projectedNormal, yAxis);
        desiredYaw = desiredYaw*180/M_PI;
        PRINT_DEBUG(4, "Projected Normal: " << projectedNormal << "\n");
        PRINT_DEBUG(4, "Y axis: " << yAxis << "\n");
        PRINT_DEBUG(4, "prevYaw: " << prevYaw << "\n");
        PRINT_DEBUG(4, "desiredYaw: " << desiredYaw << "\n");
        // Call to function for generating the pTargetPoints
        PRINT_LOG(1, "Generating target points for capturing videos of the plane: " << plane_index+1 << ".\n");
        getPTargetPoints(grid, planeParameters[plane_index], plane_index, uvAxes, pTargetPoints);
        PRINT_DEBUG(4, print2dVector(pTargetPoints, "Before shifting pTargetPoints:\n", "matlab"));
        for (unsigned int i = 0; i < pTargetPoints.size(); ++i)
        {
            pTargetPoints[i][0] += (double)x_centroid;
            pTargetPoints[i][1] += (double)y_centroid;
            pTargetPoints[i][2] += (double)z_centroid;
        }
        PRINT_DEBUG(4, print2dVector(pTargetPoints, "After shifting pTargetPoints:\n", "matlab"));
        for (unsigned int i = 0; i < pTargetPoints.size(); ++i)
        {
            pTargetPoints[i][0] += (drone_length*(-1.0)*planeParameters[plane_index][0]);
            pTargetPoints[i][1] += (drone_length*(-1.0)*planeParameters[plane_index][1]);
            pTargetPoints[i][2] += (drone_length*(-1.0)*planeParameters[plane_index][2]);
        }
        PRINT_DEBUG(4, print2dVector(pTargetPoints, "After shifting pTargetPoints and projecting by 0.6:\n", "matlab"));
        string filename = "/home/sonapraneeth/plane"+to_string(plane_index+1)+"_map";
        PRINT_LOG(1, "Writing " << pTargetPoints.size() << " points to file: " << filename << "\n");
        write3DPointsToCSV(pTargetPoints, filename, " ");
        // Having known the prevPosition and pTargetPoints
        // move the drone from prevPosition to pTargetPoints[0]
        // and completely navigate around the plane as specified by the pTargetPoints
        // For plane_index == 0, prevPosition is the current position of the drone
        if(plane_index == 0)
        {
            previousPosition[0] = x_drone;
            previousPosition[1] = y_drone;
            previousPosition[2] = z_drone;
            previousPosition[3] = yaw;
        }
        if(plane_index == 0)
        {
            vector<Point3f> curr_coord_box_points;
            vector<float> curr_plane_parameters;
            moveDroneBetweenPlanes(previousPosition,
                                    curr_coord_box_points,
                                    continuousBoundingBoxPoints[plane_index], 
                                    curr_plane_parameters, 
                                    planeParameters[plane_index], plane_index);
        }
        else
        {
            moveDroneBetweenPlanes(previousPosition,
                                    continuousBoundingBoxPoints[plane_index-1], 
                                    continuousBoundingBoxPoints[plane_index], 
                                    planeParameters[plane_index-1],
                                    planeParameters[plane_index], plane_index);
        }
        PRINT_LOG(3, "Moving the drone for the plane: " << plane_index+1 << ".\n");
        if(plane_index >= 0)
        {
            prevPosition[0] = targetPoints.back()[0];
            prevPosition[1] = targetPoints.back()[1];
            prevPosition[2] = targetPoints.back()[2];
            prevYaw = targetPoints.back()[3];
        }
        PRINT_DEBUG(4, print1dVector(prevPosition, "prevPosition for moveDrone: ", ""));
        PRINT_DEBUG(4, "Previous Yaw: " << prevYaw << "\n");
        PRINT_DEBUG(4, "Desired Yaw: " << desiredYaw << "\n");
        moveDrone(prevPosition, pTargetPoints, prevYaw, desiredYaw);
        int numTargetPoints = pTargetPoints.size();
        // Now the previous position of plane numbered plane_index+1 is the last position where 
        // the drone has captured video for plane numbered plane_index
        prevPosition[0] = pTargetPoints[numTargetPoints-1][0];
        prevPosition[1] = pTargetPoints[numTargetPoints-1][1] - drone_length;
        prevPosition[2] = pTargetPoints[numTargetPoints-1][2];
        prevYaw = desiredYaw;
        previousPosition[0] = pTargetPoints[numTargetPoints-1][0];
        previousPosition[1] = pTargetPoints[numTargetPoints-1][1] - drone_length;
        previousPosition[2] = pTargetPoints[numTargetPoints-1][2];
        previousPosition[3] = desiredYaw;
        // Update the plane index
        planeIndex++;
        PRINT_DEBUG(4, print2dVector(pTargetPoints, "pTargetPoints for plane:\n", "matlab"));
    }
    // Once all the movement of drone is completed unlock the acquired lock
    pthread_mutex_unlock(&command_CS);
    PRINT_LOG(1, "Completed.\n");
    return ;
}

int
ControlUINode::getOrientation(float currentYaw, float destYaw)
{
    PRINT_LOG(1, "Started\n");
    PRINT_DEBUG(4, "Before: currentYaw: " << currentYaw << ", destYaw: " << destYaw << "\n");
    int answer = 0;
    if (destYaw > 0 && currentYaw < 0 && fabs(180.0-destYaw) < 30.0)
    {
        destYaw = -179.0 - (180.0 - destYaw);
    }
    else if (destYaw < 0 && currentYaw > 0 && fabs(-180.0-destYaw) < 30.0)
    {
        destYaw = 179.0 + (180.0 + destYaw);
    }
    PRINT_DEBUG(4, "After: currentYaw: " << currentYaw << ", destYaw: " << destYaw << "\n");
    if(destYaw < 0 && currentYaw < 0)
    {
        if(currentYaw > destYaw)
            answer = 1; // anti-clockwise
        else
            answer = -1; // clockwise
    }
    else if(destYaw > 0 && currentYaw > 0)
    {
        if(currentYaw > destYaw)
            answer = 1;
        else
            answer = -1;
    }
    // assuming the case for -1 curryaw and 34 destyaw
    else if(currentYaw <= 0 && destYaw > 0)
    {
        answer = -1;
    }
    // assuming the case for 1 curryaw and -34 destyaw
    else if(currentYaw >= 0 && destYaw < 0)
    {
        answer = 1;
    }
    PRINT_DEBUG(4, "Orientation: " << answer << "\n");
    PRINT_LOG(1, "Completed\n");
    return answer;
}

void
ControlUINode::getBackTheDrone(vector< vector<double> > &pathPoints)
{
    PRINT_LOG(1, "Started\n");
    string filename = "Plane_Info.txt";
    vector< vector<float> > sortedPlaneParameters;
    vector< vector<Point3f> > boundingBoxPoints;
    image_gui->readPlaneInfo(filename, sortedPlaneParameters, boundingBoxPoints);
    PRINT_LOG(3, print2dVector(sortedPlaneParameters, "Plane Parameters:\n", ""));
    PRINT_LOG(3, print2dVector(boundingBoxPoints, "Bounding Box points:\n", ""));
    
    clear2dVector(pathPoints);
    vector<double> startPosition(4), endPosition(4);
    float distance = -1.5;
    for(unsigned int plane_no = sortedPlaneParameters.size()-1; plane_no > 0; plane_no--)
    {
        Point3f curr_plane_normal, next_plane_normal;
        curr_plane_normal.x = sortedPlaneParameters[plane_no][0];
        curr_plane_normal.y = sortedPlaneParameters[plane_no][1];
        curr_plane_normal.z = sortedPlaneParameters[plane_no][2];
        next_plane_normal.x = sortedPlaneParameters[plane_no-1][0];
        next_plane_normal.y = sortedPlaneParameters[plane_no-1][1];
        next_plane_normal.z = sortedPlaneParameters[plane_no-1][2];
        Point3f yAxis(0.0f, 1.0f, 0.0f);
        Point3f currProjectedNormal(curr_plane_normal.x, curr_plane_normal.y, 0);
        Point3f nextProjectedNormal(next_plane_normal.x, next_plane_normal.y, 0);
        double currYaw = findAngle(currProjectedNormal, yAxis);
        double nextYaw = findAngle(nextProjectedNormal, yAxis);
        currYaw = currYaw*180/M_PI;
        nextYaw = nextYaw*180/M_PI;
        float angle_between_planes = findAngle(next_plane_normal, curr_plane_normal);
        PRINT_DEBUG(4, "Angle to turn in radians: " << angle_between_planes << "\n");
        angle_between_planes = angle_between_planes*180.0/M_PI;
        PRINT_DEBUG(4, "Angle to turn in degrees: " << angle_between_planes << "\n");
        PRINT_DEBUG(4, "Angle to turn in degrees for current plane: " << currYaw << "\n");
        PRINT_DEBUG(4, "Angle to turn in degrees for next plane: " << nextYaw << "\n");
        Point3f next_plane_midpoint(0.0f, 0.0f, 0.0f), next_plane_right_edge_midpoint(0.0f, 0.0f, 0.0f);
        Point3f curr_plane_midpoint(0.0f, 0.0f, 0.0f), curr_plane_left_edge_midpoint(0.0f, 0.0f, 0.0f);
        // Get the current plane's mid point
        curr_plane_midpoint.x += (boundingBoxPoints[plane_no][0].x+boundingBoxPoints[plane_no][1].x);
        curr_plane_midpoint.x += (boundingBoxPoints[plane_no][2].x+boundingBoxPoints[plane_no][3].x);
        curr_plane_midpoint.x /= (4.0);
        curr_plane_midpoint.y += (boundingBoxPoints[plane_no][0].y+boundingBoxPoints[plane_no][1].y);
        curr_plane_midpoint.y += (boundingBoxPoints[plane_no][2].y+boundingBoxPoints[plane_no][3].y);
        curr_plane_midpoint.y /= (4.0);
        curr_plane_midpoint.z += (boundingBoxPoints[plane_no][0].z+boundingBoxPoints[plane_no-1][1].z);
        curr_plane_midpoint.z += (boundingBoxPoints[plane_no][2].z+boundingBoxPoints[plane_no-1][3].z);
        curr_plane_midpoint.z /= (4.0);
        //
        next_plane_midpoint.x += (boundingBoxPoints[plane_no-1][0].x+boundingBoxPoints[plane_no-1][1].x);
        next_plane_midpoint.x += (boundingBoxPoints[plane_no-1][2].x+boundingBoxPoints[plane_no-1][3].x);
        next_plane_midpoint.x /= (4.0);
        next_plane_midpoint.y += (boundingBoxPoints[plane_no-1][0].y+boundingBoxPoints[plane_no-1][1].y);
        next_plane_midpoint.y += (boundingBoxPoints[plane_no-1][2].y+boundingBoxPoints[plane_no-1][3].y);
        next_plane_midpoint.y /= (4.0);
        next_plane_midpoint.z += (boundingBoxPoints[plane_no-1][0].z+boundingBoxPoints[plane_no-1][1].z);
        next_plane_midpoint.z += (boundingBoxPoints[plane_no-1][2].z+boundingBoxPoints[plane_no-1][3].z);
        next_plane_midpoint.z /= (4.0);
        PRINT_DEBUG(4, "Current plane midpoint: " << curr_plane_midpoint << "\n");
        PRINT_DEBUG(4, "Next plane midpoint: " << next_plane_midpoint << "\n");
        next_plane_right_edge_midpoint.x += (boundingBoxPoints[plane_no-1][1].x+boundingBoxPoints[plane_no-1][2].x);
        next_plane_right_edge_midpoint.x /= (2.0);
        next_plane_right_edge_midpoint.y += (boundingBoxPoints[plane_no-1][1].y+boundingBoxPoints[plane_no-1][2].y);
        next_plane_right_edge_midpoint.y /= (2.0);
        next_plane_right_edge_midpoint.z += (boundingBoxPoints[plane_no-1][1].z+boundingBoxPoints[plane_no-1][2].z);
        next_plane_right_edge_midpoint.z /= (2.0);
        //
        curr_plane_left_edge_midpoint.x += (boundingBoxPoints[plane_no][0].x+boundingBoxPoints[plane_no][3].x);
        curr_plane_left_edge_midpoint.x /= (2.0);
        curr_plane_left_edge_midpoint.y += (boundingBoxPoints[plane_no][0].y+boundingBoxPoints[plane_no][3].y);
        curr_plane_left_edge_midpoint.y /= (2.0);
        curr_plane_left_edge_midpoint.z += (boundingBoxPoints[plane_no][0].z+boundingBoxPoints[plane_no][3].z);
        curr_plane_left_edge_midpoint.z /= (2.0);
        Point3f next_plane_right_edge_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f next_plane_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f curr_plane_left_edge_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f curr_plane_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f midway_projection(0.0f, 0.0f, 0.0f);
        next_plane_right_edge_midpoint_projection.x = next_plane_right_edge_midpoint.x + distance*next_plane_normal.x;
        next_plane_right_edge_midpoint_projection.y = next_plane_right_edge_midpoint.y + distance*next_plane_normal.y;
        next_plane_right_edge_midpoint_projection.z = next_plane_right_edge_midpoint.z + distance*next_plane_normal.z;
        next_plane_midpoint_projection.x = next_plane_midpoint.x + distance*next_plane_normal.x;
        next_plane_midpoint_projection.y = next_plane_midpoint.y + distance*next_plane_normal.y;
        next_plane_midpoint_projection.z = next_plane_midpoint.z + distance*next_plane_normal.z;
        curr_plane_left_edge_midpoint_projection.x = curr_plane_left_edge_midpoint.x + distance*curr_plane_normal.x;
        curr_plane_left_edge_midpoint_projection.y = curr_plane_left_edge_midpoint.y + distance*curr_plane_normal.y;
        curr_plane_left_edge_midpoint_projection.z = curr_plane_left_edge_midpoint.z + distance*curr_plane_normal.z;
        curr_plane_midpoint_projection.x = curr_plane_midpoint.x + distance*curr_plane_normal.x;
        curr_plane_midpoint_projection.y = curr_plane_midpoint.y + distance*curr_plane_normal.y;
        curr_plane_midpoint_projection.z = curr_plane_midpoint.z + distance*curr_plane_normal.z;
        float lambda = (next_plane_midpoint.x - curr_plane_midpoint.x)*curr_plane_normal.y - 
                            (next_plane_midpoint.y - curr_plane_midpoint.y)*curr_plane_normal.x;
        float denominator = (curr_plane_normal.x * next_plane_normal.y) - 
                            (curr_plane_normal.y * next_plane_normal.x);
        lambda = lambda / denominator;
        float t = ((next_plane_normal.x * lambda) + next_plane_midpoint.x - curr_plane_midpoint.x) / curr_plane_normal.x;
        Point3f intersection_point(0.0f, 0.0f, 0.0f), mid_plane_normal(0.0f, 0.0f, 0.0f);
        intersection_point.x = (curr_plane_normal.x * t) + curr_plane_midpoint.x;
        intersection_point.y = (curr_plane_normal.y * t) + curr_plane_midpoint.y;
        intersection_point.z = (curr_plane_normal.z * t) + curr_plane_midpoint.z;
        mid_plane_normal.x = intersection_point.x - curr_plane_left_edge_midpoint.x;
        mid_plane_normal.y = intersection_point.y - curr_plane_left_edge_midpoint.y;
        mid_plane_normal.z = intersection_point.z - curr_plane_left_edge_midpoint.z;
        PRINT_DEBUG(4, "Intersection point: " << intersection_point << "\n");
        PRINT_DEBUG(4, "Current plane normal: " << curr_plane_normal << "\n");
        PRINT_DEBUG(4, "Mid plane normal: " << mid_plane_normal << "\n");
        PRINT_DEBUG(4, "Next plane Normal: " << next_plane_normal << "\n");
        midway_projection.x = curr_plane_left_edge_midpoint.x + distance*mid_plane_normal.x;
        midway_projection.y = curr_plane_left_edge_midpoint.y + distance*mid_plane_normal.y;
        midway_projection.z = curr_plane_left_edge_midpoint.z + distance*mid_plane_normal.z;
        if (plane_no == sortedPlaneParameters.size()-1)
        {
            if(testing_code)
            {
                getCurrentPositionOfDrone();
                startPosition[0] = _node_current_pos_of_drone[0];
                startPosition[1] = _node_current_pos_of_drone[1];
                startPosition[2] = _node_current_pos_of_drone[2];
                startPosition[3] = _node_current_pos_of_drone[3];
                endPosition[0] = curr_plane_midpoint_projection.x;
                endPosition[1] = curr_plane_midpoint_projection.y;
                endPosition[2] = curr_plane_midpoint_projection.z;
                endPosition[3] = currYaw;
                PRINT_DEBUG(4, print1dVector(startPosition, "0.1 -> Starting position", ""));
                PRINT_DEBUG(4, print1dVector(endPosition, "0.1 -> Ending position", ""));
                generatePathPoints(startPosition, endPosition, pathPoints, false);
                PRINT_DEBUG(4, "0.1 -> pathPoints size: " << pathPoints.size() << "\n");
            }
        }
        startPosition[0] = curr_plane_midpoint_projection.x;
        startPosition[1] = curr_plane_midpoint_projection.y;
        startPosition[2] = curr_plane_midpoint_projection.z;
        startPosition[3] = currYaw;
        endPosition[0] = curr_plane_left_edge_midpoint_projection.x;
        endPosition[1] = curr_plane_left_edge_midpoint_projection.y;
        endPosition[2] = curr_plane_left_edge_midpoint_projection.z;
        endPosition[3] = currYaw;
        PRINT_DEBUG(4, print1dVector(startPosition, "0.2 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "0.2 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, false);
        PRINT_DEBUG(4, "0.2 -> pathPoints size: " << pathPoints.size() << "\n");
        startPosition[0] = endPosition[0];
        startPosition[1] = endPosition[1];
        startPosition[2] = endPosition[2];
        startPosition[3] = endPosition[3];
        endPosition[0] = midway_projection.x;
        endPosition[1] = midway_projection.y;
        endPosition[2] = midway_projection.z;
        endPosition[3] = currYaw;
        PRINT_DEBUG(4, print1dVector(startPosition, "0.3 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "0.3 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, false);
        PRINT_DEBUG(4, "0.3 -> pathPoints size: " << pathPoints.size() << "\n");
        startPosition[0] = endPosition[0];
        startPosition[1] = endPosition[1];
        startPosition[2] = endPosition[2];
        startPosition[3] = endPosition[3];
        endPosition[0] = next_plane_right_edge_midpoint_projection.x;
        endPosition[1] = next_plane_right_edge_midpoint_projection.y;
        endPosition[2] = next_plane_right_edge_midpoint_projection.z;
        endPosition[3] = nextYaw;
        PRINT_DEBUG(4, print1dVector(startPosition, "0.4 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "0.4 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, false);
        PRINT_DEBUG(4, "0.4 -> pathPoints size: " << pathPoints.size() << "\n");
        startPosition[0] = endPosition[0];
        startPosition[1] = endPosition[1];
        startPosition[2] = endPosition[2];
        startPosition[3] = endPosition[3];
        endPosition[0] = next_plane_midpoint_projection.x;
        endPosition[1] = next_plane_midpoint_projection.y;
        endPosition[2] = next_plane_midpoint_projection.z;
        endPosition[3] = nextYaw;
        PRINT_DEBUG(4, print1dVector(startPosition, "0.5 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "0.5 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, false);
        PRINT_DEBUG(4, "0.5 -> pathPoints size: " << pathPoints.size() << "\n");
    }
    /*pushCommands(pathPoints);
    clear2dVector(pathPoints);*/
    PRINT_LOG(1, "Completed\n");
}

void
ControlUINode::moveDroneBetweenPlanes(const vector<double> &previousPosition,
                                      const vector<Point3f> &curr_coord_box_points,
                                      const vector<Point3f> &next_coord_box_points,
                                      const vector<float> &curr_plane_parameters,
                                      const vector<float> &next_plane_parameters, int plane_index)
{
    PRINT_LOG(1, "Started.\n");
    PRINT_LOG(1, "Designing path for plane " << plane_index+1 << "\n");
    vector< vector<double> > pathPoints;
    clear2dVector(pathPoints);
    vector<double> startPosition(4), endPosition(4);
    float distance = -1.75;
    if (curr_coord_box_points.size() == 0 && curr_plane_parameters.size() == 0 && plane_index == 0)
    {
        PRINT_LOG(2, "You are currently adjusting for plane " << plane_index+1 << "\n");
        PRINT_DEBUG(4, print1dVector(next_plane_parameters, "Current plane parameters: ", ""));
        PRINT_DEBUG(4, print1dVector(next_coord_box_points, "Current plane bounding box points:\n", ""));
        Point3f prevPositionPoint, next_plane_normal;
        prevPositionPoint.x = (float)previousPosition[0];
        prevPositionPoint.y = (float)previousPosition[1];
        prevPositionPoint.z = (float)previousPosition[2];
        PRINT_DEBUG(4, "Previous position: " << prevPositionPoint << "\n");
        // Plane i normal. Normal of next plane
        next_plane_normal.x = next_plane_parameters[0];
        next_plane_normal.y = next_plane_parameters[1];
        next_plane_normal.z = next_plane_parameters[2];
        PRINT_DEBUG(4, "Current plane parameters: " << next_plane_normal << "\n");
        Point3f next_plane_midpoint(0.0f, 0.0f, 0.0f), next_plane_right_edge_midpoint(0.0f, 0.0f, 0.0f);
        // Get the next plane's mid point
        next_plane_midpoint.x += (next_coord_box_points[0].x+next_coord_box_points[1].x);
        next_plane_midpoint.x += (next_coord_box_points[2].x+next_coord_box_points[3].x);
        next_plane_midpoint.x /= (4.0);
        next_plane_midpoint.y += (next_coord_box_points[0].y+next_coord_box_points[1].y);
        next_plane_midpoint.y += (next_coord_box_points[2].y+next_coord_box_points[3].y);
        next_plane_midpoint.y /= (4.0);
        next_plane_midpoint.z += (next_coord_box_points[0].z+next_coord_box_points[1].z);
        next_plane_midpoint.z += (next_coord_box_points[2].z+next_coord_box_points[3].z);
        next_plane_midpoint.z /= (4.0);
        PRINT_DEBUG(4, "Current plane midpoint: " << next_plane_midpoint << "\n");
        // Get the next plane's right edge midpoint
        next_plane_right_edge_midpoint.x = 0.0;
        next_plane_right_edge_midpoint.x += (next_coord_box_points[1].x+next_coord_box_points[2].x);
        next_plane_right_edge_midpoint.x /= (2.0);
        next_plane_right_edge_midpoint.y = 0.0;
        next_plane_right_edge_midpoint.y += (next_coord_box_points[1].y+next_coord_box_points[2].y);
        next_plane_right_edge_midpoint.y /= (2.0);
        next_plane_right_edge_midpoint.z = 0.0;
        next_plane_right_edge_midpoint.z += (next_coord_box_points[1].z+next_coord_box_points[2].z);
        next_plane_right_edge_midpoint.z /= (2.0);
        PRINT_DEBUG(4, "Current plane right edge midpoint: " << next_plane_right_edge_midpoint << "\n");
        Point3f next_plane_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f next_plane_right_edge_midpoint_projection(0.0f, 0.0f, 0.0f);
        // Position in front of plane i-1 where the drone might have to go
        next_plane_midpoint_projection.x = next_plane_midpoint.x + distance*next_plane_normal.x;
        next_plane_midpoint_projection.y = next_plane_midpoint.y + distance*next_plane_normal.y;
        next_plane_midpoint_projection.z = next_plane_midpoint.z + distance*next_plane_normal.z;
        PRINT_DEBUG(4, "Current plane midpoint projection: " << next_plane_midpoint_projection << "\n");
        // 
        next_plane_right_edge_midpoint_projection.x = next_plane_right_edge_midpoint.x + distance*next_plane_normal.x;
        next_plane_right_edge_midpoint_projection.y = next_plane_right_edge_midpoint.y + distance*next_plane_normal.y;
        next_plane_right_edge_midpoint_projection.z = next_plane_right_edge_midpoint.z + distance*next_plane_normal.z;
        PRINT_DEBUG(4, "Current plane right edge midpoint projection: " << next_plane_right_edge_midpoint_projection << "\n");
        PRINT_DEBUG(4, "Start position point: " << prevPositionPoint << "\n");
        float distance13 = distanceBetweenPoints(prevPositionPoint, next_plane_midpoint_projection);
        float distance14 = distanceBetweenPoints(prevPositionPoint, next_plane_right_edge_midpoint_projection);
        PRINT_DEBUG(4, "Distance between last targetPoint and next plane midpoint projection: " << distance13 << "\n");
        PRINT_DEBUG(4, "Distance between last targetPoint and next plane right edge midpoint projection: " << distance14 << "\n");
        Point3f projectedNormal(next_plane_parameters[0], next_plane_parameters[1], 0);
        Point3f yAxis(0, 1, 0);
        double desiredYaw = findAngle(projectedNormal, yAxis);
        desiredYaw = desiredYaw*180/M_PI;
        PRINT_DEBUG(4, "prevYaw: " << previousPosition[3] << "\n");
        PRINT_DEBUG(4, "desiredYaw: " << desiredYaw << "\n");
        startPosition[0] = previousPosition[0];
        startPosition[1] = previousPosition[1];
        startPosition[2] = previousPosition[2];
        startPosition[3] = previousPosition[3];
        if(distance13 < distance14)
        {
            endPosition[0] = next_plane_midpoint_projection.x;
            endPosition[1] = next_plane_midpoint_projection.y;
            endPosition[2] = next_plane_midpoint_projection.z;
            endPosition[3] = desiredYaw;
            generatePathPoints(startPosition, endPosition, pathPoints, false);
            PRINT_DEBUG(4, "1.1 -> pathPoints size: " << pathPoints.size() << "\n");
        }
        else
        {
            endPosition[0] = next_plane_right_edge_midpoint_projection.x;
            endPosition[1] = next_plane_right_edge_midpoint_projection.y;
            endPosition[2] = next_plane_right_edge_midpoint_projection.z;
            endPosition[3] = desiredYaw;
            generatePathPoints(startPosition, endPosition, pathPoints, false);
            PRINT_DEBUG(4, "1.2 -> pathPoints size: " << pathPoints.size() << "\n");
        }
    }
    else
    {
        PRINT_LOG(1, "You are currently adjusting for plane " << plane_index+1 << "\n");
        PRINT_DEBUG(4, print1dVector(curr_plane_parameters, "Current plane parameters: ", ""));
        PRINT_DEBUG(4, print1dVector(curr_coord_box_points, "Current plane bounding box points:\n", ""));
        PRINT_DEBUG(4, print1dVector(next_plane_parameters, "Next plane parameters: ", ""));
        PRINT_DEBUG(4, print1dVector(next_coord_box_points, "Next plane bounding box points:\n", ""));
        Point3f prevPositionPoint, curr_plane_normal, next_plane_normal;
        prevPositionPoint.x = (float)previousPosition[0];
        prevPositionPoint.y = (float)previousPosition[1];
        prevPositionPoint.z = (float)previousPosition[2];
        PRINT_DEBUG(4, "Previous position: " << prevPositionPoint << "\n");
        // Plane i-1 normal. Normal of current plane
        curr_plane_normal.x = curr_plane_parameters[0];
        curr_plane_normal.y = curr_plane_parameters[1];
        curr_plane_normal.z = curr_plane_parameters[2];
        // Plane i normal. Normal of next plane
        next_plane_normal.x = next_plane_parameters[0];
        next_plane_normal.y = next_plane_parameters[1];
        next_plane_normal.z = next_plane_parameters[2];
        PRINT_DEBUG(4, "Current plane parameters: " << curr_plane_normal << "\n");
        PRINT_DEBUG(4, "Next plane parameters: " << next_plane_normal << "\n");
        Point3f curr_plane_midpoint(0.0f, 0.0f, 0.0f), curr_plane_right_edge_midpoint(0.0f, 0.0f, 0.0f);
        // Get the curr plane's mid point
        curr_plane_midpoint.x += (curr_coord_box_points[0].x+curr_coord_box_points[1].x);
        curr_plane_midpoint.x += (curr_coord_box_points[2].x+curr_coord_box_points[3].x);
        curr_plane_midpoint.x /= (4.0);
        curr_plane_midpoint.y += (curr_coord_box_points[0].y+curr_coord_box_points[1].y);
        curr_plane_midpoint.y += (curr_coord_box_points[2].y+curr_coord_box_points[3].y);
        curr_plane_midpoint.y /= (4.0);
        curr_plane_midpoint.z += (curr_coord_box_points[0].z+curr_coord_box_points[1].z);
        curr_plane_midpoint.z += (curr_coord_box_points[2].z+curr_coord_box_points[3].z);
        curr_plane_midpoint.z /= (4.0);
        PRINT_DEBUG(4, "Current plane midpoint: " << curr_plane_midpoint << "\n");
        // Get the curr plane's right edge midpoint
        curr_plane_right_edge_midpoint.x = 0.0;
        curr_plane_right_edge_midpoint.x += (curr_coord_box_points[1].x+curr_coord_box_points[2].x);
        curr_plane_right_edge_midpoint.x /= (2.0);
        curr_plane_right_edge_midpoint.y = 0.0;
        curr_plane_right_edge_midpoint.y += (curr_coord_box_points[1].y+curr_coord_box_points[2].y);
        curr_plane_right_edge_midpoint.y /= (2.0);
        curr_plane_right_edge_midpoint.z = 0.0;
        curr_plane_right_edge_midpoint.z += (curr_coord_box_points[1].z+curr_coord_box_points[2].z);
        curr_plane_right_edge_midpoint.z /= (2.0);
        PRINT_DEBUG(4, "Current plane right edge midpoint: " << curr_plane_right_edge_midpoint << "\n");
        Point3f curr_plane_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f curr_plane_right_edge_midpoint_projection(0.0f, 0.0f, 0.0f);
        // Position in front of plane i-1 where the drone might have to go
        curr_plane_midpoint_projection.x = curr_plane_midpoint.x + distance*curr_plane_normal.x;
        curr_plane_midpoint_projection.y = curr_plane_midpoint.y + distance*curr_plane_normal.y;
        curr_plane_midpoint_projection.z = curr_plane_midpoint.z + distance*curr_plane_normal.z;
        PRINT_DEBUG(4, "Current plane midpoint projection: " << curr_plane_midpoint_projection << "\n");
        // 
        curr_plane_right_edge_midpoint_projection.x = curr_plane_right_edge_midpoint.x + distance*curr_plane_normal.x;
        curr_plane_right_edge_midpoint_projection.y = curr_plane_right_edge_midpoint.y + distance*curr_plane_normal.y;
        curr_plane_right_edge_midpoint_projection.z = curr_plane_right_edge_midpoint.z + distance*curr_plane_normal.z;
        PRINT_DEBUG(4, "Current plane right edge midpoint projection: " << curr_plane_right_edge_midpoint_projection << "\n");
        PRINT_DEBUG(4, "Start position point: " << prevPositionPoint << "\n");
        float distance13 = distanceBetweenPoints(prevPositionPoint, curr_plane_midpoint_projection);
        float distance14 = distanceBetweenPoints(prevPositionPoint, curr_plane_right_edge_midpoint_projection);
        PRINT_DEBUG(4, "Distance between last targetPoint and current plane midpoint projection: " << distance13 << "\n");
        PRINT_DEBUG(4, "Distance between last targetPoint and current plane right edge midpoint projection: " << distance14 << "\n");
        startPosition[0] = previousPosition[0];
        startPosition[1] = previousPosition[1];
        startPosition[2] = previousPosition[2];
        startPosition[3] = previousPosition[3];
        if(distance13 < distance14)
        {
            endPosition[0] = curr_plane_midpoint_projection.x;
            endPosition[1] = curr_plane_midpoint_projection.y;
            endPosition[2] = curr_plane_midpoint_projection.z;
            endPosition[3] = previousPosition[3];
            PRINT_DEBUG(4, print1dVector(startPosition, "2.1 -> Starting position", ""));
            PRINT_DEBUG(4, print1dVector(endPosition, "2.1 -> Ending position", ""));
            generatePathPoints(startPosition, endPosition, pathPoints, false);
            PRINT_DEBUG(4, "2.1 -> pathPoints size: " << pathPoints.size() << "\n");
            startPosition[0] = endPosition[0];
            startPosition[1] = endPosition[1];
            startPosition[2] = endPosition[2];
            startPosition[3] = endPosition[3];
            endPosition[0] = curr_plane_right_edge_midpoint_projection.x;
            endPosition[1] = curr_plane_right_edge_midpoint_projection.y;
            endPosition[2] = curr_plane_right_edge_midpoint_projection.z;
            endPosition[3] = previousPosition[3];
            PRINT_DEBUG(4, print1dVector(startPosition, "2.2 -> Starting position", ""));
            PRINT_DEBUG(4, print1dVector(endPosition, "2.2 -> Ending position", ""));
            generatePathPoints(startPosition, endPosition, pathPoints, false);
            PRINT_DEBUG(4, "2.2 -> pathPoints size: " << pathPoints.size() << "\n");
        }
        else
        {
            endPosition[0] = curr_plane_right_edge_midpoint_projection.x;
            endPosition[1] = curr_plane_right_edge_midpoint_projection.y;
            endPosition[2] = curr_plane_right_edge_midpoint_projection.z;
            endPosition[3] = previousPosition[3];
            PRINT_DEBUG(4, print1dVector(startPosition, "2.3 -> Starting position", ""));
            PRINT_DEBUG(4, print1dVector(endPosition, "2.3 -> Ending position", ""));
            generatePathPoints(startPosition, endPosition, pathPoints, false);
            PRINT_DEBUG(4, "2.3 -> pathPoints size: " << pathPoints.size() << "\n");
        }
        Point3f intersection_point(0.0f, 0.0f, 0.0f), mid_plane_normal(0.0f, 0.0f, 0.0f), path_mid_point(0.0f, 0.0f, 0.0f);
        Point3f next_plane_midpoint(0.0f, 0.0f, 0.0f), next_plane_left_edge_midpoint(0.0f, 0.0f, 0.0f);
        Point3f next_plane_left_edge_midpoint_projection(0.0f, 0.0f, 0.0f);
        Point3f next_plane_midpoint_projection(0.0f, 0.0f, 0.0f);
        // Get the next plane's mid point
        next_plane_midpoint.x += (next_coord_box_points[0].x+next_coord_box_points[1].x);
        next_plane_midpoint.x += (next_coord_box_points[2].x+next_coord_box_points[3].x);
        next_plane_midpoint.x /= (4.0);
        next_plane_midpoint.y += (next_coord_box_points[0].y+next_coord_box_points[1].y);
        next_plane_midpoint.y += (next_coord_box_points[2].y+next_coord_box_points[3].y);
        next_plane_midpoint.y /= (4.0);
        next_plane_midpoint.z += (next_coord_box_points[0].z+next_coord_box_points[1].z);
        next_plane_midpoint.z += (next_coord_box_points[2].z+next_coord_box_points[3].z);
        next_plane_midpoint.z /= (4.0);
        //
        // Get the next plane's left edge mid point
        next_plane_left_edge_midpoint.x += (next_coord_box_points[0].x+next_coord_box_points[3].x);
        next_plane_left_edge_midpoint.x /= (2.0);
        next_plane_left_edge_midpoint.y += (next_coord_box_points[0].y+next_coord_box_points[3].y);
        next_plane_left_edge_midpoint.y /= (2.0);
        next_plane_left_edge_midpoint.z += (next_coord_box_points[0].z+next_coord_box_points[3].z);
        next_plane_left_edge_midpoint.z /= (2.0);
        PRINT_DEBUG(4, "Next plane midpoint: " << next_plane_midpoint << "\n");
        PRINT_DEBUG(4, "Next plane left edge midpoint: " << next_plane_left_edge_midpoint << "\n");
        float lambda = (next_plane_midpoint.x - curr_plane_midpoint.x)*curr_plane_normal.y - 
                            (next_plane_midpoint.y - curr_plane_midpoint.y)*curr_plane_normal.x;
        float denominator = (curr_plane_normal.x * next_plane_normal.y) - 
                            (curr_plane_normal.y * next_plane_normal.x);
        lambda = lambda / denominator;
        float t = ((next_plane_normal.x * lambda) + next_plane_midpoint.x - curr_plane_midpoint.x) / curr_plane_normal.x;
        intersection_point.x = (curr_plane_normal.x * t) + curr_plane_midpoint.x;
        intersection_point.y = (curr_plane_normal.y * t) + curr_plane_midpoint.y;
        intersection_point.z = (curr_plane_normal.z * t) + curr_plane_midpoint.z;
        float angleBetweenPlanes = findAngle(next_plane_normal, curr_plane_normal);
        float angle = 0.0f;
        PRINT_DEBUG(4, "Projected Normal: " << next_plane_normal << "\n");
        PRINT_DEBUG(4, "Current plane normal: " << curr_plane_normal << "\n");
        angle = angleBetweenPlanes/2;
        PRINT_DEBUG(4, "Halfway angle in radians: " << angle << "\n");
        float new_distance = distance/cos(angle);
        PRINT_DEBUG(4, "New distance diagonally: " << new_distance << "\n");
        angle = angle*180.0/M_PI;
        PRINT_DEBUG(4, "Halfway angle in degrees: " << angle << "\n");
        PRINT_DEBUG(4, "Angle between planes in radians: " << angleBetweenPlanes << "\n");
        angleBetweenPlanes = angleBetweenPlanes*180.0/M_PI;
        PRINT_DEBUG(4, "Angle between planes in degrees: " << angleBetweenPlanes << "\n");
        Point3f yAxis(0.0f, 1.0f, 0.0f);
        float finalAngle = findAngle(next_plane_normal, yAxis);
        PRINT_DEBUG(4, "Angle to turn in radians: " << finalAngle << "\n");
        finalAngle = finalAngle*180.0/M_PI;
        PRINT_DEBUG(4, "Angle to turn in degrees: " << finalAngle << "\n");
        float currAngle = findAngle(curr_plane_normal, yAxis);
        PRINT_DEBUG(4, "Angle with current plane in radians: " << currAngle << "\n");
        currAngle = currAngle*180.0/M_PI;
        PRINT_DEBUG(4, "Angle with current plane in degrees: " << currAngle << "\n");
        int rotation_dir = getOrientation(currAngle, finalAngle);
        PRINT_DEBUG(4, "Rotation Direction: " << rotation_dir << "\n");
        mid_plane_normal.x = rotation_dir*(intersection_point.x - curr_plane_right_edge_midpoint.x);
        mid_plane_normal.y = rotation_dir*(intersection_point.y - curr_plane_right_edge_midpoint.y);
        mid_plane_normal.z = rotation_dir*(intersection_point.z - curr_plane_right_edge_midpoint.z);
        float mid_plane_normal_mag = \
                        sqrt((mid_plane_normal.x * mid_plane_normal.x)+ \
                        (mid_plane_normal.y * mid_plane_normal.y)+ \
                        (mid_plane_normal.z * mid_plane_normal.z));
        mid_plane_normal.x /= mid_plane_normal_mag;
        mid_plane_normal.y /= mid_plane_normal_mag;
        mid_plane_normal.z /= mid_plane_normal_mag;
        PRINT_DEBUG(4, "Intersection point: " << intersection_point << "\n");
        PRINT_DEBUG(4, "Mid plane normal: " << mid_plane_normal << "\n");
        // Point3f yAxis(0.0f, 1.0f, 0.0f);
        path_mid_point.x = curr_plane_right_edge_midpoint.x + new_distance*mid_plane_normal.x;
        path_mid_point.y = curr_plane_right_edge_midpoint.y + new_distance*mid_plane_normal.y;
        path_mid_point.z = curr_plane_right_edge_midpoint.z + new_distance*mid_plane_normal.z;
        startPosition[0] = endPosition[0];
        startPosition[1] = endPosition[1];
        startPosition[2] = endPosition[2];
        startPosition[3] = endPosition[3];
        endPosition[0] = path_mid_point.x;
        endPosition[1] = path_mid_point.y;
        endPosition[2] = path_mid_point.z;
        endPosition[3] = previousPosition[3];
        PRINT_DEBUG(4, print1dVector(startPosition, "2.4 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "2.4 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, false);
        PRINT_DEBUG(4, "2.4 -> pathPoints size: " << pathPoints.size() << "\n");
        next_plane_left_edge_midpoint_projection.x = next_plane_left_edge_midpoint.x + distance*next_plane_normal.x;
        next_plane_left_edge_midpoint_projection.y = next_plane_left_edge_midpoint.y + distance*next_plane_normal.y;
        next_plane_left_edge_midpoint_projection.z = next_plane_left_edge_midpoint.z + distance*next_plane_normal.z;
        next_plane_midpoint_projection.x = next_plane_midpoint.x + distance*next_plane_normal.x;
        next_plane_midpoint_projection.y = next_plane_midpoint.y + distance*next_plane_normal.y;
        next_plane_midpoint_projection.z = next_plane_midpoint.z + distance*next_plane_normal.z;
        startPosition[0] = endPosition[0];
        startPosition[1] = endPosition[1];
        startPosition[2] = endPosition[2];
        startPosition[3] = endPosition[3];
        endPosition[0] = next_plane_left_edge_midpoint_projection.x;
        endPosition[1] = next_plane_left_edge_midpoint_projection.y;
        endPosition[2] = next_plane_left_edge_midpoint_projection.z;
        endPosition[3] = finalAngle;
        PRINT_DEBUG(4, print1dVector(startPosition, "2.5 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "2.5 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, true);
        PRINT_DEBUG(4, "2.5 -> pathPoints size: " << pathPoints.size() << "\n");
        startPosition[0] = endPosition[0];
        startPosition[1] = endPosition[1];
        startPosition[2] = endPosition[2];
        startPosition[3] = endPosition[3];
        endPosition[0] = next_plane_midpoint_projection.x;
        endPosition[1] = next_plane_midpoint_projection.y;
        endPosition[2] = next_plane_midpoint_projection.z;
        endPosition[3] = finalAngle;
        PRINT_DEBUG(4, print1dVector(startPosition, "2.6 -> Starting position", ""));
        PRINT_DEBUG(4, print1dVector(endPosition, "2.6 -> Ending position", ""));
        generatePathPoints(startPosition, endPosition, pathPoints, false);
        PRINT_DEBUG(4, "2.6 -> pathPoints size: " << pathPoints.size() << "\n");
    }
    moveDronePathPoints[plane_index] = (int)pathPoints.size() + 8;
    PRINT_LOG(4, print2dVector(pathPoints, "Points needed by drone to move from one plane to another:\n", "matlab"));
    pushCommands(pathPoints);
    string filename = "path_points_" + to_string(plane_index+1);
    write3DPointsToCSV(pathPoints, filename, " ");
    clear2dVector(pathPoints);
    PRINT_LOG(1, "Completed.\n");
    return ;
}

void
ControlUINode::generatePathPoints(const vector<double> &startPosition, 
                                  const vector<double> &endPosition,
                                  vector< vector<double> > &pathPoints,
                                  bool rotation)
{
    PRINT_LOG(1, "Started.\n");
    PRINT_DEBUG(2, print1dVector(startPosition, "Starting position", ""));
    PRINT_DEBUG(2, print1dVector(endPosition, "Ending position", ""));
    vector<double> interm_point(4);
    interm_point[0] = startPosition[0];
    interm_point[1] = startPosition[1];
    interm_point[2] = startPosition[2];
    interm_point[3] = startPosition[3];
    double prevYaw = startPosition[3];
    double desiredYaw = endPosition[3];
    PRINT_DEBUG(4, "Previous yaw: " << prevYaw << "\n");
    PRINT_DEBUG(4, "Desired yaw: " << desiredYaw << "\n");
    if (desiredYaw > 0 && prevYaw < 0 && fabs(180.0-desiredYaw) < 30.0)
    {
        desiredYaw = -179.0 - (180.0 - desiredYaw);
    }
    if (desiredYaw < 0 && prevYaw > 0 && fabs(-180.0-desiredYaw) < 30.0)
    {
        desiredYaw = 179.0 + (180.0 + desiredYaw);
    }
    PRINT_DEBUG(4, "Desired yaw: " << desiredYaw << "\n");
    PRINT_DEBUG(4, "Initial pathPoints size: " << pathPoints.size() << "\n");
    double angle;
    if(rotation)
    {
        PRINT_DEBUG(3, "With rotation\n");
        double angle_to_rotate = 4.0;
        double angle_diff = fabs(desiredYaw-prevYaw);
        int num_steps = ceil(angle_diff/angle_to_rotate);
        PRINT_DEBUG(4, "Angle difference: " << angle_diff << ", Number of steps: " << num_steps << "\n");
        for(int i = 0; i < num_steps; i++)
        {
            interm_point[0] = ((i+1)*endPosition[0] + (num_steps-i-1)*startPosition[0])/num_steps;
            interm_point[1] = ((i+1)*endPosition[1] + (num_steps-i-1)*startPosition[1])/num_steps;
            interm_point[2] = ((i+1)*endPosition[2] + (num_steps-i-1)*startPosition[2])/num_steps;
            angle = ((i+1)*desiredYaw + (num_steps-i-1)*prevYaw)/num_steps;
            PRINT_DEBUG(4, "Previous angle: " << angle << "\n");
            PRINT_DEBUG(4, "Status: " << (angle < -180.0) << ", " << (angle >= 180.0) << "\n");
            if(angle < -180.0)
            {
                interm_point[3] = 360.0+angle;
            }
            else if(angle >= 180.0)
            {
                interm_point[3] = -360.0+angle;
            }
            else
            {
                interm_point[3] = angle;
            }
            PRINT_DEBUG(4, "Correct angle: " << interm_point[3] << "\n");
            PRINT_DEBUG(4, print1dVector(interm_point, "Interim point: ", ""));
            pathPoints.push_back(interm_point);
        }
    }
    else
    {
        PRINT_DEBUG(3, "Without Rotation\n");
        for(int i = 0; i < 6; i++)
        {
            int m = i/2 +1;
            int n = 2 - i/2;
            if(i%2 == 0)
            {
                interm_point[0] = (m*endPosition[0] + n*startPosition[0])/3;
            }
            else
            {
                interm_point[1] = (m*endPosition[1] + n*startPosition[1])/3;
            }
            interm_point[3] = prevYaw*(5-i)/5 + desiredYaw*i/5;
            PRINT_DEBUG(4, print1dVector(interm_point, "Interim point: ", ""));
            pathPoints.push_back(interm_point);
        }
        interm_point[2] = (startPosition[2] + endPosition[2])/2.0;
        pathPoints.push_back(interm_point);
        interm_point[2] = endPosition[2];
        pathPoints.push_back(interm_point);
    }
    PRINT_DEBUG(4, "Final pathPoints size: " << pathPoints.size() << "\n");
    PRINT_DEBUG(4, print2dVector(pathPoints, "From function:\n", "matlab"));
    PRINT_LOG(1, "Completed.\n");
    return ;
}

void
ControlUINode::pushCommands(const vector< vector<double > > &pathPoints)
{
    PRINT_LOG(1, "Started\n");
    PRINT_LOG(3, "Pushing " << pathPoints.size() << " commands\n");
    char buf[100];
    for(unsigned int step = 0; step < pathPoints.size(); step++)
    {
        snprintf(buf, 100, "c goto %lf %lf %lf %lf",
            pathPoints[step][0], pathPoints[step][1], pathPoints[step][2], pathPoints[step][3]);
        std_msgs::String s;
        s.data = buf;
        commands.push_back(s);
        targetPoints.push_back(pathPoints[step]);
    }
    PRINT_LOG(1, "Completed\n");
    return ;
}
void
ControlUINode::printGrid(const pGrid &g, const vector<Point3f> &uvAxes, const vector<float> &plane)
{
    PRINT_DEBUG(5, "Started.\n");
    vector<Point2f> uvCoordinates;
    vector<Point3f> xyzCorners;
    PRINT_DEBUG(5, "UV Grid\n");
    for(unsigned int i = 0; i < g.rowSquares.size(); i++)
    {
        for(unsigned int j = 0; j < g.rowSquares[i].size(); j++)
        {
            float u = g.rowSquares[i][j].u;
            float v = g.rowSquares[i][j].v;
            Point2f uv(u, v);
            uvCoordinates.push_back(uv);
        }
    }
    // 
    AllUVToXYZCoordinates(uvCoordinates, uvAxes, plane[3], xyzCorners);
    PRINT_DEBUG(5, uvCoordinates << "\n");
    PRINT_DEBUG(5, xyzCorners << "\n");
    PRINT_DEBUG(5, "Completed.\n");
}

grid
ControlUINode::buildGrid (vector<vector<float> > pPoints)
{
    vector<float> lu;
    float width, height;
    float squareWidth = 0.8, squareHeight = 0.45, overlap = 0.334;
    // Assuming that the plane is always parallel to XZ plane - ? Gotta change this
    getDimensions(pPoints, lu, width, height);
    grid g(lu[0], lu[1]-height, lu[0]+width, lu[1], squareWidth, squareHeight, overlap);
    gridSquare gs(lu, squareWidth, squareHeight);
    g.add(gs);
    //gs.debugPrint();
    while(g.translate(gs))
    {
        gs = g.getLatest();
        //gs.debugPrint();
    }
    //ROS_INFO("Number of rows in grid : %d", g.row+1);
    //g.print();
    return g;
}

pGrid
ControlUINode::buildPGrid(const vector<Point2f> &uvCoordinates)
{
    PRINT_LOG(1, "Started.\n");
    // Vectors for u and v co-ordinates in UV co-ordinate system
    vector<float> uCoord, vCoord;
    PRINT_LOG(4, print1dVector(uvCoordinates, "UV Co-ordinates:\n", ""));
    vector<Point2f> sortedUVCoordinates;
    // Sort the uv co-ordinates
    sortUVCorners(uvCoordinates, sortedUVCoordinates);
    PRINT_LOG(4, print1dVector(sortedUVCoordinates, "Sorted UV Co-ordinates:\n", ""));
    // Make the X Co-ordinates of plane bounding box points
    // There are 4 corners for a bounded plane
    for (int j = 0; j < 4; ++j)
    {
        uCoord.push_back(sortedUVCoordinates[j].x);
    }
    // Make the Y Co-ordinates of plane bouding box points
    for (int j = 0; j < 4; ++j)
    {
        vCoord.push_back(sortedUVCoordinates[j].y);
    }
    PRINT_LOG(4, print1dVector(uCoord, "U Co-ord: ", ""));
    PRINT_LOG(4, print1dVector(vCoord, "V Co-ord: ", ""));
    vector<float> uVector(2), vVector(2);
    uVector[0] = uCoord[1]-uCoord[0];
    uVector[1] = vCoord[1]-vCoord[0];
    vVector[0] = uCoord[0]-uCoord[3];
    vVector[1] = vCoord[0]-vCoord[3];
    PRINT_LOG(4, print1dVector(uVector, "U vector: ", ""));
    PRINT_LOG(4, print1dVector(vVector, "V vector: ", ""));
    // Get the height and width of the plane?
    float horizDist1 = sqrt(pow(uCoord[0]-uCoord[1],2)+pow(vCoord[0]-vCoord[1],2));
    float horizDist2 = sqrt(pow(uCoord[2]-uCoord[3],2)+pow(vCoord[2]-vCoord[3],2));
    float vertDist1 = sqrt(pow(uCoord[2]-uCoord[1],2)+pow(vCoord[2]-vCoord[1],2));
    float vertDist2 = sqrt(pow(uCoord[0]-uCoord[3],2)+pow(vCoord[0]-vCoord[3],2));
    PRINT_LOG(4, horizDist1 << ", " << horizDist2 << ", " << vertDist1 << ", " << vertDist2 << "\n");
    // Dimensions of the cells of the grid
    float squareWidth = 0.8;
    float squareHeight = 0.45;
    // Percentage of overlap between cells in the grid
    float overlap = 0.2; // 0.334;
    float maxR = max(horizDist1, horizDist2);
    float maxD = max(vertDist1, vertDist2) - squareHeight;
    // Normalizing vectors?
    uVector[0] /= horizDist1;
    uVector[1] /= horizDist1;
    vVector[0] /= vertDist1;
    vVector[1] /= vertDist1;
    // pGridSquare width and height
    PRINT_LOG(4, "Making pGrid\n");
    pGrid grid(uCoord[0], vCoord[0], uVector, vVector, squareWidth, squareHeight, overlap, maxR, maxD);
    PRINT_LOG(4, "Making pGridSquare\n");
    pGridSquare gridSquare = pGridSquare(uCoord[0], vCoord[0], squareWidth, squareHeight, uVector, vVector);
    grid.add(gridSquare);
    PRINT_LOG(4, "Adding pGridSquare\n");
    PRINT_LOG(4, "In while loop\n");
    while (grid.translate(gridSquare))
    {
        gridSquare = grid.getLatest();
    }
    PRINT_LOG(4, "Out of while loop\n");
    PRINT_LOG(1, "Completed.\n");
    return grid;
}

void
ControlUINode::getPTargetPoints(const pGrid &g, const vector<float> & plane,
                                int plane_no, const vector<Point3f> &uvAxes,
                                vector< vector<double> > &finalSortTPoints)
{
    PRINT_LOG(1, "Started.\n");
    PRINT_DEBUG(4, "Inputs\n");
    PRINT_DEBUG(4, print1dVector(plane, "Plane: ", ""));
    PRINT_DEBUG(4, print1dVector(uvAxes, "UV Axes:\n", ""));
    PRINT_DEBUG(4, "Camera calibration: " << calibrated << "\n");
    if(!calibrated)
        calibrate();
    vector< vector<double> > tPoints;
    vector< vector<double> > tPoints_z;
    // Points in the image. 4 corners + mid points of lines formed by 4 corners
    // + centeral point of the image
    vector<Point2d> imgPoints;
    imgPoints.push_back(Point2d(0, 0));
    imgPoints.push_back(Point2d(320, 0));
    imgPoints.push_back(Point2d(640, 0));
    imgPoints.push_back(Point2d(640, 180));
    imgPoints.push_back(Point2d(640, 360));
    imgPoints.push_back(Point2d(320, 360));
    imgPoints.push_back(Point2d(0, 360));
    imgPoints.push_back(Point2d(0, 180));
    imgPoints.push_back(Point2d(320, 180));
    // Creating an image matrix
    Mat imgPoints_mat(9, 1, CV_64FC2);
    for(int i = 0; i < 9; i++)
        imgPoints_mat.at<Point2d>(i,0) = imgPoints[i];
    PRINT_DEBUG(4, "Image Points:\n" << imgPoints_mat << "\n");
    vector< vector<double> > sortedTPoints;
    // Camera Matrix
    Mat cameraMatrix(3, 3, DataType<double>::type);
    // Setting camera matrix for vga quality
    // From calibration done on our drone
    cameraMatrix.at<double>(0, 0) = 565.710890694431;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 329.70046366652;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 565.110297594854;
    cameraMatrix.at<double>(1, 2) = 169.873085097623;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;
    // Distortion Matrix
    Mat distCoeffs(5,1,DataType<double>::type);
    // Setting distortion coefficients
    // From calibration done on our drone
    distCoeffs.at<double>(0) = -0.516089772391501;
    distCoeffs.at<double>(1) = 0.285181914111246;
    distCoeffs.at<double>(2) = -0.000466469917823537;
    distCoeffs.at<double>(3) = 0.000864792975814983;
    distCoeffs.at<double>(4) = 0;
    // Matrix hilders for rotation and translation
    Mat rvec(3, 1, DataType<double>::type);
    Mat tvec(3, 1, DataType<double>::type);
    Mat final_result(3, 1, DataType<double>::type);
    PRINT_DEBUG(4, "Camera Matrix:\n" << cameraMatrix << "\n");
    PRINT_DEBUG(4, "Distortion co-efficients:\n" << distCoeffs << "\n");
    //
    vector<Point3d> objPoints;
    vector< vector<Point3d> > completeObjPoints;
    clear2dVector(completeObjPoints);
    Point3f projectedNormal(plane[0], plane[1], 0);
    Point3f yAxis(0, 1, 0);
    // Calculate the yaw of the rotation of the plane
    double yaw = findAngle(projectedNormal, yAxis);
    yaw = -yaw;
    PRINT_DEBUG(4, "yaw in radians: " << yaw << "\n");
    PRINT_DEBUG(4, "yaw in degrees: " << ((yaw*180)/M_PI) << "\n");
    Mat rotation_cam = Mat::eye(3, 3, CV_64F);
    rotation_cam.at<double>(0, 0) = cos(yaw);
    rotation_cam.at<double>(0, 2) = -sin(yaw);
    rotation_cam.at<double>(2, 0) = sin(yaw);
    rotation_cam.at<double>(2, 2) = cos(yaw);
    PRINT_DEBUG(4, "Rotation matrix to bring normal:\n" << rotation_cam << "\n");
    Mat rotation = Mat::eye(3, 3, CV_64F);
    rotation.at<double>(0, 0) = cos(yaw);
    rotation.at<double>(0, 1) = -sin(yaw);
    rotation.at<double>(1, 0) = sin(yaw);
    rotation.at<double>(1, 1) = cos(yaw);
    // Whether the drone is moving forward or backward (left to right or right to left)
    bool forward = true; // Need to iterate forward or backward
    vector<Point2f> uvCorners;
    vector<Point3f> xyzCorners, rotatedXYZCorners, sortedXYZCorners;
    for(unsigned int i = 0; i < g.rowSquares.size()-1; i++)
    {
        if(forward)
        {
            PRINT_DEBUG(4, "Forward\n");
            for(unsigned int j=0; j < g.rowSquares[i].size(); j++)
            {
                PRINT_DEBUG(4, "i: " << i << ", j: " << j << "\n");
                objPoints.clear();
                pGridSquare gs = g.rowSquares[i][j];
                vector<Point2f> uvCorners;
                vector<Point3f> xyzCorners, rotatedXYZCorners, sortedXYZCorners;
                uvCorners.clear(); xyzCorners.clear();
                rotatedXYZCorners.clear(); sortedXYZCorners.clear();
                getGridSquareUVCorners(gs, uvCorners);
                AllUVToXYZCoordinates(uvCorners, uvAxes, 0.0, xyzCorners);
                PRINT_DEBUG(4, print1dVector(xyzCorners, "XYZ Corners:\n", ""));
                rotate3fPoints(xyzCorners, rotation, rotatedXYZCorners);
                PRINT_DEBUG(4, print1dVector(rotatedXYZCorners, "Rotated XYZ Corners:\n", ""));
                sortXYZCorners(rotatedXYZCorners, sortedXYZCorners);
                PRINT_DEBUG(4, print1dVector(sortedXYZCorners, "Sorted XYZ Corners:\n", ""));
                Point3d corner1 = Point3d(sortedXYZCorners[0].x, -sortedXYZCorners[0].z, sortedXYZCorners[0].y);
                Point3d corner2 = Point3d(sortedXYZCorners[1].x, -sortedXYZCorners[1].z, sortedXYZCorners[1].y);
                Point3d corner3 = Point3d(sortedXYZCorners[2].x, -sortedXYZCorners[2].z, sortedXYZCorners[2].y);
                Point3d corner4 = Point3d(sortedXYZCorners[3].x, -sortedXYZCorners[3].z, sortedXYZCorners[3].y);
                Point3d mid1 = (corner1 + corner2)*0.5;
                // mid1.z = getY(mid1.x, -mid1.y, plane);
                mid1.z = corner1.z;
                Point3d mid2 = (corner2 + corner3)*0.5;
                // mid2.z = getY(mid2.x, -mid2.y, plane);
                mid2.z = corner1.z;
                Point3d mid3 = (corner3 + corner4)*0.5;
                // mid3.z = getY(mid3.x, -mid3.y, plane);
                mid3.z = corner1.z;
                Point3d mid4 = (corner4 + corner1)*0.5;
                // mid4.z = getY(mid4.x, -mid4.y, plane);
                mid4.z = corner1.z;
                Point3d center = (mid1 + mid3)*0.5;
                // center.z = getY(center.x, -center.y,plane);
                center.z = corner1.z;
                PRINT_DEBUG(4, "Center: " << center << "\n");
                PRINT_DEBUG(4, print1dVector(plane, "Plane normal: ", ""));
                // 
                objPoints.push_back(corner1);
                objPoints.push_back(mid1);
                objPoints.push_back(corner2);
                objPoints.push_back(mid2);
                objPoints.push_back(corner3);
                objPoints.push_back(mid3);
                objPoints.push_back(corner4);
                objPoints.push_back(mid4);
                objPoints.push_back(center);
                completeObjPoints.push_back(objPoints);
                // 
                Mat objPoints_mat(9, 1, CV_64FC3);
                for(int i = 0; i < 9; i++)
                {
                    objPoints_mat.at<Point3d>(i,0) = objPoints[i];
                }
                PRINT_DEBUG(4, "Object Points:\n" << objPoints << "\n");
                // [MGP]Dont know but we have to call undistortPoints as a dummy call
                // Something to do with older version of opencv which gets linked by mrpt
                Mat dummy;
                undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
                //
                Mat rot_guess = Mat::eye(3,3, CV_64F);
                PRINT_DEBUG(4, "Rotation guess:\n" << rot_guess << "\n");
                Rodrigues(rot_guess, rvec);
                /*tvec.at<double>(0)  = -(center.x-0.6*plane[0]);
                tvec.at<double>(1)  = -(center.y+0.6*plane[2]);
                tvec.at<double>(2)  = -(center.z-0.6*plane[1]);*/
                tvec.at<double>(0)  = -(center.x);
                tvec.at<double>(1)  = -(center.y);
                tvec.at<double>(2)  = -(center.z-0.6);
                PRINT_DEBUG(4, "Translation:\n" << tvec << "\n");
                // 
                // solvePnPRansac(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec);//, true, CV_ITERATIVE);
                // solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);
                solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);
                // 
                Mat rot(3, 3, DataType<double>::type);
                Rodrigues(rvec, rot);
                PRINT_DEBUG(4, "Rotation:\n" << rot << "\n");
                PRINT_DEBUG(4, "Rotation Vector:\n" << rvec << "\n");
                // 
                Mat rotinv;
                transpose(rot, rotinv);
                PRINT_DEBUG(4, "Rotation Inverse:\n" << rotinv << "\n");
                // 
                tvec = -rotinv * tvec;
                PRINT_DEBUG(4, "Translation:\n" << tvec << "\n");
                final_result = rotation_cam*tvec;
                PRINT_DEBUG(4, "Final Result:\n" << final_result << "\n");
                // 
                vector<double> pt;
                pt.push_back(tvec.at<double>(0));
                pt.push_back(tvec.at<double>(2));
                pt.push_back(-tvec.at<double>(1));
                PRINT_DEBUG(4, print1dVector(pt, "Pt one: ", ""));
                tPoints.push_back(pt);
                // 
                /*pt.clear();
                pt.push_back(gs.u + (gs.width/2));
                pt.push_back(tvec.at<double>(2));
                pt.push_back(gs.v - (gs.height/2));
                PRINT_DEBUG(3, print1dVector(pt, "Pt two: ", ""));
                tPoints_z.push_back(pt);*/
            }
        }
        else
        {
            PRINT_DEBUG(4, "Backward\n");
            for(int j = g.rowSquares[i].size()-1; j >= 0 ; j--)
            {
                PRINT_DEBUG(4, "i: " << i << ", j: " << j << "\n");
                //ROS_INFO("Accessing %dth square of %dth row", j, i);
                objPoints.clear();
                pGridSquare gs = g.rowSquares[i][j];
                uvCorners.clear(); xyzCorners.clear();
                rotatedXYZCorners.clear(); sortedXYZCorners.clear();
                getGridSquareUVCorners(gs, uvCorners);
                AllUVToXYZCoordinates(uvCorners, uvAxes, 0.0, xyzCorners);
                PRINT_DEBUG(4, print1dVector(xyzCorners, "XYZ Corners:\n", ""));
                rotate3fPoints(xyzCorners, rotation, rotatedXYZCorners);
                PRINT_DEBUG(4, print1dVector(rotatedXYZCorners, "Rotated XYZ Corners:\n", ""));
                sortXYZCorners(rotatedXYZCorners, sortedXYZCorners);
                PRINT_DEBUG(4, print1dVector(sortedXYZCorners, "Sorted XYZ Corners:\n", ""));
                // 
                Point3d corner1 = Point3d(sortedXYZCorners[0].x, -sortedXYZCorners[0].z, sortedXYZCorners[0].y);
                Point3d corner2 = Point3d(sortedXYZCorners[1].x, -sortedXYZCorners[1].z, sortedXYZCorners[1].y);
                Point3d corner3 = Point3d(sortedXYZCorners[2].x, -sortedXYZCorners[2].z, sortedXYZCorners[2].y);
                Point3d corner4 = Point3d(sortedXYZCorners[3].x, -sortedXYZCorners[3].z, sortedXYZCorners[3].y);
                // 
                Point3d mid1 = (corner1 + corner2)*0.5;
                // mid1.z = getY(mid1.x, -mid1.y, plane);
                mid1.z = corner1.z;
                Point3d mid2 = (corner2 + corner3)*0.5;
                // mid2.z = getY(mid2.x, -mid2.y, plane);
                mid2.z = corner1.z;
                Point3d mid3 = (corner3 + corner4)*0.5;
                // mid3.z = getY(mid3.x, -mid3.y, plane);
                mid3.z = corner1.z;
                Point3d mid4 = (corner4 + corner1)*0.5;
                // mid4.z = getY(mid4.x, -mid4.y, plane);
                mid4.z = corner4.z;
                Point3d center = (mid1 + mid3)*0.5;
                // center.z = getY(center.x, -center.y,plane);
                center.z = corner1.z;
                PRINT_DEBUG(4, "Center: " << center << "\n");
                PRINT_DEBUG(4, print1dVector(plane, "Plane normal: ", ""));
                // 
                objPoints.push_back(corner1);
                objPoints.push_back(mid1);
                objPoints.push_back(corner2);
                objPoints.push_back(mid2);
                objPoints.push_back(corner3);
                objPoints.push_back(mid3);
                objPoints.push_back(corner4);
                objPoints.push_back(mid4);
                objPoints.push_back(center);
                completeObjPoints.push_back(objPoints);
                Mat objPoints_mat(9,1, CV_64FC3);
                for(int i=0; i<9; i++)
                {
                    objPoints_mat.at<Point3d>(i,0) = objPoints[i];
                }
                PRINT_DEBUG(4, "Object Points:\n" << objPoints << "\n");
                // [MGP]Dont know but we have to call undistortPoints as a dummy call
                // Something to do with older version of opencv which gets linked by mrpt
                Mat dummy;
                undistortPoints(imgPoints_mat, dummy, cameraMatrix, distCoeffs);
                //
                Mat rot_guess = Mat::eye(3,3, CV_64F);
                PRINT_DEBUG(4, "Rotation guess:\n" << rot_guess << "\n");
                Rodrigues(rot_guess, rvec);
                /*tvec.at<double>(0)  = -(center.x-0.6*plane[0]);
                tvec.at<double>(1)  = -(center.y+0.6*plane[2]);
                tvec.at<double>(2)  = -(center.z-0.6*plane[1]);*/
                tvec.at<double>(0)  = -(center.x);
                tvec.at<double>(1)  = -(center.y);
                tvec.at<double>(2)  = -(center.z-0.6);
                PRINT_DEBUG(4, "Translation:\n" << tvec << "\n");
                // 
                // solvePnPRansac(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec);//, true, CV_ITERATIVE);
                // solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);
                solvePnP(objPoints_mat, imgPoints_mat, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);
                // 
                Mat rot(3, 3, DataType<double>::type);
                Rodrigues(rvec, rot);
                PRINT_DEBUG(4, "Rotation:\n" << rot << "\n");
                PRINT_DEBUG(4, "Rotation Vector:\n" << rvec << "\n");
                // 
                Mat rotinv;
                transpose(rot, rotinv);
                PRINT_DEBUG(4, "Rotation Inverse:\n" << rotinv << "\n");
                // 
                tvec = -rotinv * tvec;
                PRINT_DEBUG(4, "Translation:\n" << tvec << "\n");
                final_result = rotation_cam*tvec;
                PRINT_DEBUG(4, "Final Result:\n" << final_result << "\n");
                // 
                vector<double> pt;
                pt.push_back(tvec.at<double>(0));
                pt.push_back(tvec.at<double>(2));
                pt.push_back(-tvec.at<double>(1));
                PRINT_DEBUG(4, print1dVector(pt, "Pt one: ", ""));
                tPoints.push_back(pt);
                // 
                /*pt.clear();
                pt.push_back(gs.u + (gs.width/2));
                pt.push_back(tvec.at<double>(2));
                pt.push_back(gs.v - (gs.height/2));
                PRINT_DEBUG(3, print1dVector(pt, "Pt two: ", ""));
                tPoints_z.push_back(pt);*/
            }
        }
        forward = !forward;
    }
    int numRows = g.rowSquares.size()-1;
    vector<int> numColsPerRow;
    for(int i = 0; i < numRows; i++)
    {
        int n  = g.rowSquares[i].size();
        numColsPerRow.push_back(n);
    }
    PRINT_LOG(4, "Printing Object points for plane " << plane_no << "\n");
    PRINT_LOG(4, print2dVector(completeObjPoints, "Complete object points:\n", "matlab"));
    PRINT_LOG(4, print2dVector(tPoints, "LOG Target points:\n", "matlab"));
    PRINT_DEBUG(4, "Numrows: " << numRows << "\n");
    PRINT_DEBUG(4, print1dVector(numColsPerRow, "Number of cols per row:\n", ""));
    sortTargetPoints(numRows, numColsPerRow, tPoints, sortedTPoints);
    PRINT_LOG(4, print2dVector(sortedTPoints, "Sorted LOG Target points:\n", "matlab"));
    PRINT_DEBUG(4, "Number of SortedTPoints: " << sortedTPoints.size() << "\n");
    PRINT_DEBUG(4, "SortedTPoints size: " << sortedTPoints[0].size() << "\n");
    vector<Point3d> sortedTPointsVec, rotSortedTPointsVec;
    for(unsigned int i = 0; i < sortedTPoints.size(); i++)
    {
        sortedTPointsVec.push_back(Point3d(sortedTPoints[i][0], sortedTPoints[i][1], sortedTPoints[i][2]));
    }
    rotate3dPoints(sortedTPointsVec, rotation.inv(), rotSortedTPointsVec);
    // vector< vector<double> > finalSortTPoints;
    clear2dVector(finalSortTPoints);
    vector<double> finalSortTPoint;
    for(unsigned int i = 0; i < rotSortedTPointsVec.size(); i++)
    {
        finalSortTPoint.clear();
        finalSortTPoint.push_back(rotSortedTPointsVec[i].x);
        finalSortTPoint.push_back(rotSortedTPointsVec[i].y);
        finalSortTPoint.push_back(rotSortedTPointsVec[i].z);
        finalSortTPoints.push_back(finalSortTPoint);
    }
    finalSortTPoint.clear();
    PRINT_LOG(1, "Completed.\n");
}

void
ControlUINode::moveDrone (const vector<double> &prevPosition,
                          vector<vector<double> > tPoints,
                          double prevYaw, double desiredYaw)
{
    PRINT_LOG(1, "Started.\n");
    // double drone_length = 0.6;
    for (unsigned int i = 0; i < tPoints.size(); ++i)
    {
        vector<double> p = tPoints[i];
        // p[1] = p[1] - drone_length;
        char buf[100];
        if(i == 0)
        {
            vector< vector <double> > xyz_yaw;
            getInitialPath(prevPosition, p, prevYaw, desiredYaw, xyz_yaw);
            for(unsigned int j = 0; j < xyz_yaw.size(); j++)
            {
                 vector<double> interm_point;
                interm_point = xyz_yaw[j];
                snprintf(buf, 100, "c goto %lf %lf %lf %lf",
                    interm_point[0], interm_point[1], interm_point[2], interm_point[3]);
                std_msgs::String s;
                s.data = buf;
                // ROS_INFO("Message: ");
                // ROS_INFO(buf);
                commands.push_back(s);
                targetPoints.push_back(interm_point);
                PRINT_DEBUG(3, print1dVector(interm_point, "", ""));
            }
        }
        else
        {
            snprintf(buf, 100, "c goto %lf %lf %lf %lf", p[0], p[1], p[2], desiredYaw);
            std_msgs::String s;
            s.data = buf;
            // ROS_INFO("Message: ");
            // ROS_INFO(buf);
            commands.push_back(s);
            targetPoints.push_back(p);
            PRINT_DEBUG(3, print1dVector(p, "p: ", ""));
            PRINT_DEBUG(3, "Desired yaw: " << desiredYaw << "\n");
        }
    }
    if(planeIndex < (numberOfPlanes-1))
    {
        startTargetPtIndex[planeIndex+1] = targetPoints.size();
    }
    PRINT_LOG(1, "Completed.\n");
}

void
ControlUINode::calibrate()
{
    PRINT_LOG(5, "Started.\n");
    cameraMatrix = Mat(3, 3, DataType<float>::type);
    // Camera Matrix
    cameraMatrix.at<float>(0, 0) = 565.710890694431;
    cameraMatrix.at<float>(0, 1) = 0;
    cameraMatrix.at<float>(0, 2) = 329.70046366652;
    cameraMatrix.at<float>(1, 0) = 0;
    cameraMatrix.at<float>(1, 1) = 565.110297594854;
    cameraMatrix.at<float>(1, 2) = 169.873085097623;
    cameraMatrix.at<float>(2, 0) = 0;
    cameraMatrix.at<float>(2, 1) = 0;
    cameraMatrix.at<float>(2, 2) = 1;
    PRINT_DEBUG(5, "Camera Matrix used for calibration.\n");
    PRINT_DEBUG(5, cameraMatrix);
    PRINT_DEBUG(5, "\n");
    // Distortion co-efficients
    distCoeffs = Mat::zeros(5, 1, DataType<float>::type);
    // Vectors for storing object and image points
    vector<Vec3f> object_points;
    vector<Vec2f> image_pts;
    // Acquire lock on keypoints
    pthread_mutex_lock(&keyPoint_CS);
    assert(_3d_points.size() == _2d_points.size());
    int numPts = _3d_points.size();
    // Capturing points in world co-ordinates and points image co-ordinates
    // for estimating the camera matrix
    for(int i = 0; i < numPts; i++)
    {
        Vec3f obj_pt(_3d_points[i][0], _3d_points[i][1], _3d_points[i][2]);
        Vec2f img_pt(_2d_points[i][0], _2d_points[i][1]);
        object_points.push_back(obj_pt);
        image_pts.push_back(img_pt);
    }
    vector< vector<Vec3f> > object;
    vector< vector<Vec2f> > image;
    object.push_back(object_points);
    image.push_back(image_pts);
    PRINT_LOG(5, "Calibrating camera.\n");
    calibrateCamera(object, image, Size(640, 360), cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS);
    pthread_mutex_unlock(&keyPoint_CS);
    // Calibrated is set to true
    calibrated = true;
    PRINT_LOG(5, "Completed.\n");
}

void
ControlUINode::project3DPointsOnImage(const vector<Point3f> &worldPts, vector<Point2f> &imagePts)
{
    PRINT_LOG(5, "Started.\n");
    calibrate();
    cv::projectPoints(worldPts, rvecs[0], tvecs[0], cameraMatrix, distCoeffs, imagePts);
    // int numPoints = imagePts.size();
    PRINT_LOG(5, "Completed.\n");
}

void
ControlUINode::checkPos(const ros::TimerEvent&)
{
    if(targetSet)
    {
        double x = targetPoint[0];
        double y = targetPoint[1];
        double z = targetPoint[2];
        pthread_mutex_lock(&pose_CS);
        double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2));
        pthread_mutex_unlock(&pose_CS);
        if(ea < error_threshold)
            targetSet = false;
        else
            targetSet = true;
    }
}

void
ControlUINode::getGridSquareUVCorners(const pGridSquare &gs, vector<Point2f> &uvCorners)
{
    Point2f corner1 = Point2f(gs.u, gs.v);
    Point2f corner2 = Point2f(gs.u+gs.width, gs.v);
    Point2f corner3 = Point2f(gs.u+gs.width, gs.v - gs.height);
    Point2f corner4 = Point2f(gs.u, gs.v-gs.height);
    uvCorners.push_back(corner1);
    uvCorners.push_back(corner2);
    uvCorners.push_back(corner3);
    uvCorners.push_back(corner4);
}

void
ControlUINode::sortTargetPoints(int numRows, const vector<int> &numColsPerRow,
                                const vector< vector<double> > &tPoints,
                                vector< vector<double> > &sortedTPoints)
{
    vector<float> z_coord(numRows);
    vector<float> sortedZcoord;
    vector<int> rowStartIndex;
    int rowStart = 0;
    for(int i = 0; i < numRows; i++)
    {
        rowStartIndex.push_back(rowStart);
        int index = rowStart;
        z_coord[i] = tPoints[index][2];
        rowStart += numColsPerRow[i];
    }
    vector<int> indices;
    sortData(z_coord, sortedZcoord, indices, false);
    for(int i=0; i<numRows; i++)
    {
        int index= rowStartIndex[indices[i]] ;
        int numCols = numColsPerRow[indices[i]];
        vector< vector<double> > rowTPoints;
        for(int j=0; j < numCols; j++)
        {
            vector<double> tPoint = tPoints[index+j];
            rowTPoints.push_back(tPoint);
        }
        if( i%2 == (indices[i])%2) //if new index and old index have same parity then the order need not be changed
        {
            sortedTPoints.insert(sortedTPoints.end(), rowTPoints.begin(), rowTPoints.end());
        }
        else //otherwise need to reverse the order
        {
            for(int j = numCols-1; j >= 0; j--)
            {
                sortedTPoints.push_back(rowTPoints[j]);
            }
        }
    }
}

void
ControlUINode::getInitialPath(const vector<double> &prevPosition, const vector<double> &tPoint,
                              double prevYaw, double desiredYaw, vector<vector<double> > &xyz_yaw)
{
    PRINT_LOG(1, "Started\n");
    PRINT_DEBUG(4, "prevYaw: " << prevYaw << "\n");
    PRINT_DEBUG(4, "desiredYaw: " << desiredYaw << "\n");
    vector<double> interm_point(4);
    interm_point[0] = prevPosition[0];
    interm_point[1] = prevPosition[1];
    interm_point[2] = prevPosition[2];
    interm_point[3] = 0;
    for(int i = 0; i < 6; i++)
    {
        int m = i/2 +1;
        int n = 2 - i/2;
        if(i%2 == 0)
        {
            interm_point[0] = (m*tPoint[0] + n*prevPosition[0])/3;
        }
        else
        {
            interm_point[1] = (m*tPoint[1] + n*prevPosition[1])/3;
        }
        interm_point[3] = prevYaw*(5-i)/5 + desiredYaw*i/5;
        xyz_yaw.push_back(interm_point);
    }
    interm_point[2] = prevPosition[2]/2+ tPoint[2]/2;
    xyz_yaw.push_back(interm_point);
    interm_point[2] = tPoint[2];
    xyz_yaw.push_back(interm_point);
    PRINT_DEBUG(4, print2dVector(xyz_yaw, "XYZ Yaw:\n", "matlab"));
    PRINT_LOG(1, "Completed\n");
}

void
ControlUINode::newPoseCb (const tum_ardrone::filter_stateConstPtr statePtr)
{
    pthread_mutex_lock(&pose_CS);
    scale = statePtr->scale;
    scale_z = statePtr->scale_z;
    x_offset = statePtr->x_offset;
    y_offset = statePtr->y_offset;
    z_offset = statePtr->z_offset;
    x_drone = statePtr->x;
    y_drone = statePtr->y;
    z_drone = statePtr->z;
    yaw = statePtr->yaw;
    roll = statePtr->roll;
    pitch = statePtr->pitch;
    pthread_mutex_unlock(&pose_CS);
    // Goto commands left to be executed
    pthread_mutex_lock(&command_CS);
    PRINT_DEBUG(10, "Acquired command_CS Lock\n");
    static int numCommands = 0;
    /*cout << "[ DEBUG] [poseCb] Checking for just navigation commands\n";
    cout << "[ DEBUG] [poseCb] Number of commands left: " << just_navigation_commands.size() << "\n";
    cout << "[ DEBUG] [poseCb] justNavigation: " << justNavigation << "\n";
    cout << "[ DEBUG] [poseCb] traverseComplete: " << traverseComplete << "\n";*/
    // PRINT_DEBUG(1, commands.size() << ", " << currentCommand << ", " << recordNow << "\n");
    if(just_navigation_commands.size() == 0 && changeyawLockReleased==-1)
    {
        pthread_mutex_unlock(&changeyaw_CS);
        changeyawLockReleased = 0;
        PRINT_DEBUG(10, "Released first changeyaw_CS Lock\n");
    }
    else if(just_navigation_commands.size() > 0 && !justNavigation)
    {
        justNavigation = true;
        traverseComplete = false;
        just_navigation_command_number++;
        pthread_mutex_lock(&tum_ardrone_CS);
            tum_ardrone_pub.publish(just_navigation_commands.front());
        pthread_mutex_unlock(&tum_ardrone_CS);
        targetPoint.clear();
        targetPoint = targetPoints.front();
        PRINT_DEBUG(3, "Just navigation current target " << just_navigation_command_number
                <<" of " << just_navigation_total_commands << ": (" << targetPoint[0] << ", " 
                << targetPoint[1] << ", " << targetPoint[2] << ", " << targetPoint[3] << ")\n");
    }
    else if(justNavigation && !traverseComplete)
    {
        double x = targetPoint[0];
        double y = targetPoint[1];
        double z = targetPoint[2];
        double ya = targetPoint[3];
        //double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2) + pow(ya - yaw, 2));
        //double ea = fabs(fabs(x-x_drone)+fabs(y-y_drone)+fabs(z-z_drone)+fabs(ya-yaw));
        /*cout << "[ DEBUG] [poseCb] Current Pose: " << x_drone << ", " << y_drone << ", " << z_drone << ", " << yaw << "\n";
        cout << "[ DEBUG] [poseCb] " << fabs(x-x_drone) << ", " << fabs(y-y_drone) << ", "
                    << fabs(z-z_drone) << ", " << fabs(ya-yaw) << "\n";*/
        //printf("[ DEBUG] [poseCb] Error %lf\n", ea);
        getCurrentPositionOfDrone();
        PRINT_DEBUG(10, "Current position of drone: (" << x_drone << ", " << y_drone << ", " << z_drone << ")\n");
        /*print1dVector(_node_current_pos_of_drone, "[ DEBUG] [newPoseCb] Current position of drone");
        cout << " [DEBUG] [newPoseCb] x: " << x << ", y: " << y << ", z: " << z << "\n";
        cout << "[ DEBUG] [newPoseCb] Error: " << fabs(x-x_drone) << ", " << fabs(y-y_drone) << ", "
                    << fabs(z-z_drone) << ", " << fabs(ya-yaw) << "\n";*/
        bool var1 = (fabs(x-x_drone) < 0.08);
        bool var2 = (fabs(y-y_drone) < 0.08);
        bool var3 = (fabs(z-z_drone) < 0.08);
        bool var4 = (fabs(ya-yaw) < 0.3);
        PRINT_DEBUG(10, "x: " << var1 << ", y: " << var2 << ", z: " << var3 << ", yaw: " << var4 << "\n");
        if( var1 & var2 & var3 & var4 )
        {
            // cout << "[ DEBUG] [poseCb] Destination reached for command no. " << just_navigation_number << "\n";
            PRINT_DEBUG(1, "Reached targetPoint: (" << x << ", " << y << ", " << z << ", " << ya << ")\n");
            if(just_navigation_command_number <= just_navigation_total_commands)
            {
                ros::Duration(1).sleep();
                justNavigation = false;
                traverseComplete = true;
                just_navigation_commands.pop_front();
                targetPoints.pop_front();
                targetPoint.clear();
                pthread_mutex_unlock(&command_CS);
                int leftCommands = just_navigation_total_commands - just_navigation_command_number;
                PRINT_DEBUG(4, "Need to complete " << leftCommands << " of " << just_navigation_total_commands << "\n");
                PRINT_DEBUG(4, "Released elseif command_CS Lock\n");
                getCurrentPositionOfDrone();
                PRINT_DEBUG(4, print1dVector(_node_current_pos_of_drone, "Current position of drone after command", ""));
                if(just_navigation_commands.size() == 0) {changeyawLockReleased = -1;}
                return;
            }
        }
        else
        {
            justNavigation = true; traverseComplete = false;
        }
    }
    else if(commands.size() > 0 && !currentCommand)
    {
        // Looking for a new position to move.
        // There are more commands to be executed and
        // Currently drone has not been given any command to move
        currentCommand = true;
        // Total number of commands given
        numCommands++;
        pthread_mutex_lock(&tum_ardrone_CS);
        // Publish the command at the front of queue
        tum_ardrone_pub.publish(commands.front());
        pthread_mutex_unlock(&tum_ardrone_CS);
        targetPoint = targetPoints.front();
        PRINT_DEBUG(3, "Current target: (" << targetPoint[0] << ", " << targetPoint[1] << ", "
                        << targetPoint[2] << ", " << targetPoint[3] << ")\n");
    }
    else if(currentCommand && !recordNow)
    {
        // Drone has been given a position to move to but not record a video
        // Current index of plane which the drone has to move
        static int planeIndexCurrent = 0;
        // Change the plane number if total number of commands executed is greater than 
        // ?
        PRINT_DEBUG(5, "numCommands: " << numCommands << "\n");
        PRINT_DEBUG(5, "startTargetPtIndex[" << planeIndexCurrent << "]: " << startTargetPtIndex[planeIndexCurrent] << "\n");
        PRINT_DEBUG(5, "moveDronePathPoints[" << planeIndexCurrent << "]: " << moveDronePathPoints[planeIndexCurrent] << "\n");
        if( planeIndexCurrent < (numberOfPlanes-1) &&
            numCommands > startTargetPtIndex[planeIndexCurrent+1])
        {
            planeIndexCurrent++;
        }
        if(numCommands < startTargetPtIndex[planeIndexCurrent]+moveDronePathPoints[planeIndexCurrent])
        {
            PRINT_LOG(4 , "Sleeping for 1 seconds\n");
            ros::Duration(1).sleep();
            /*if(numCommands == startTargetPtIndex[planeIndexCurrent]+moveDronePathPoints[planeIndexCurrent]-8)
            {
                PRINT_LOG(3, "Sleeping for 15 seconds\n");
                ros::Duration(15).sleep();
            }*/
            /*else if(numCommands > startTargetPtIndex[planeIndexCurrent])
            {
                PRINT_LOG(3, "Sleeping for 3 seconds\n");
                ros::Duration(3).sleep();
            }*/
            currentCommand = false;
            commands.pop_front();
            targetPoints.pop_front();
            pthread_mutex_unlock(&command_CS);
            return;
        }
        double x = targetPoint[0];
        double y = targetPoint[1];
        double z = targetPoint[2];
        double ya = targetPoint[3];
        pthread_mutex_lock(&pose_CS);
        double ea = sqrt(pow(x - x_drone, 2) + pow(y - y_drone, 2) + pow(z - z_drone, 2));
        //printf("Error %lf\n", ea);
        pthread_mutex_unlock(&pose_CS);
        if(ea < error_threshold)
        {
            PRINT_DEBUG(1, "Reached targetPoint: (" << x << ", " << y << ", " << z  << ", " << ya << ")\n");
            recordNow = true;
            ros::Duration(3).sleep();
            last= ros::Time::now();
        }
        else
        {
            recordNow = false;
        }
    }
    else if(recordNow)
    {
        if(ros::Time::now() - last < ros::Duration(recordTime))
        {
            if(record && notRecording)
            {
                getCurrentPositionOfDrone();
                PRINT_LOG(1, "Recording Video at: (" << _node_current_pos_of_drone[0] << ", " 
                            << _node_current_pos_of_drone[1] << ", " << _node_current_pos_of_drone[2]
                            << ", " << _node_current_pos_of_drone[3] << ")\n");
                ardrone_autonomy::RecordEnable srv;
                srv.request.enable = true;
                video.call(srv);
                notRecording = false;
                popen("rosbag record /ardrone/image_raw /ardrone/predictedPose --duration=3", "r");
            }
            else if(!notRecording)
            {

            }
        }
        else
        {
            ardrone_autonomy::RecordEnable srv;
            srv.request.enable = false;
            video.call(srv);
            currentCommand = false;
            notRecording = true;
            recordNow = false;
            commands.pop_front();
            targetPoints.pop_front();
            ros::Duration(3).sleep();
        }
    }
    else
    {
        // do nothing
    }
    pthread_mutex_unlock(&command_CS);
    PRINT_DEBUG(10, "Released command_CS Lock\n");
}

/**
 *
 */
void
ControlUINode::moveDroneByMeasure(double dest, int direction)
{
    // 1 - Left
    // 2 - Right
    // 3 - Front
    // 4 - Back
    // 5 - Top
    // 6 - Back
    PRINT_DEBUG(4, "Started.\n");
    PRINT_DEBUG(4, "Direction: " << direction << ".\n");
    if(direction == MOVE_DIRECTIONS::LEFT) // Left
    {
        moveLeft(dest);
    }
    else if(direction == MOVE_DIRECTIONS::RIGHT) // Right
    {
        moveRight(dest);
    }
    else if(direction == MOVE_DIRECTIONS::FORWARD) // Front
    {
        moveForward(dest);
    }
    else if(direction == MOVE_DIRECTIONS::BACKWARD) // Back
    {
        moveBackward(dest);
    }
    else if(direction == MOVE_DIRECTIONS::UP) // Up
    {
        moveUp(dest);
    }
    else if(direction == MOVE_DIRECTIONS::DOWN) // Down
    {
        moveDown(dest);
    }
    else if(direction == MOVE_DIRECTIONS::CLOCK) // Clockwise
    {
        rotateClockwise(dest);
    }
    else if(direction == MOVE_DIRECTIONS::COUNTERCLOCK) // CounterClockwise
    {
        rotateCounterClockwise(dest);
    }
    else
    {
        PRINT_DEBUG(4, "I don't understand this direction\n");
    }
    PRINT_DEBUG(4, "Completed\n");
    return ;
}

void
ControlUINode::move(double distance, int i)
{
    PRINT_DEBUG(5, "Started\n");
    // Clearing the vector containing information about old path points
    clear2dVector(_interm_path);
    // Get the current position of drone for calculation
    getCurrentPositionOfDrone();
    // This vector contains the destination position of drone 
    // assuming current position of drone as origin
    _node_dest_pos_of_drone.clear();
    _node_dest_pos_of_drone.push_back(0.0);
    _node_dest_pos_of_drone.push_back(0.0);
    _node_dest_pos_of_drone.push_back(0.0);
    _node_dest_pos_of_drone.push_back(0.0);
    PRINT_DEBUG(3, print1dVector(_node_current_pos_of_drone, "Current position of drone: ", ""));
    PRINT_DEBUG(5, "Requested movement: " << distance << ", Direction: " << i << "\n");
    double start = 0.0, move;
    // Determine the direction of motion
    if(signbit(distance))
    {
        move = -1.0;
    }
    else
    {
        move = 1.0;
    }
    PRINT_DEBUG(4, "Move sign: " << move << "\n");
    bool to_move = false;
    // Use the destination point directly if it is at a very short distance from the current position
    // Else generate intermediate points
    if(fabs(start - distance) < _move_heuristic)
    {
        start = distance;
        _node_dest_pos_of_drone[i] = start;
        PRINT_DEBUG(4, print1dVector(_node_dest_pos_of_drone, "Dest. position of drone (relative)", ""));
        // Convert the destination position of drone wrt to world origin
        convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
        _interm_path.push_back(_node_ac_dest_pos_of_drone);
    }
    else
    {
        to_move = true;
    }
    // Generate commands until you've reached the destination point
    while(to_move)
    {
        if(fabs(start - distance) < _move_heuristic)
        {
            start = distance;
            to_move = false;
        }
        else
        {
            start += (move * _move_heuristic);
            to_move = true;
        }
        // Change the position according to the direction index
        _node_dest_pos_of_drone[i] = start;
        PRINT_DEBUG(4, print1dVector(_node_dest_pos_of_drone, "Dest position of drone (relative): ", ""));
        // Convert the generated position wrt world's origin
        convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
        _interm_path.push_back(_node_ac_dest_pos_of_drone);
    }
    if(fabs(start - distance) < _move_heuristic)
    {
        start = distance;
        _node_dest_pos_of_drone[i] = start;
        // Convert the generated position wrt world's origin
        convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
        _interm_path.push_back(_node_ac_dest_pos_of_drone);
    }
    // Move the drone via the generated set of points
    moveDroneViaSetOfPoints(_interm_path);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveUp(double up_distance)
{
    PRINT_DEBUG(5, "Started\n");
    move(up_distance, 2);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveDown(double down_distance)
{
    PRINT_DEBUG(5, "Started\n");
    move(-down_distance, 2);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveLeft(double left_distance)
{
    PRINT_DEBUG(5, "Started\n");
    move(-left_distance, 0);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveRight(double right_distance)
{
    PRINT_DEBUG(5, "Started\n");
    move(right_distance, 0);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveForward(double forward_distance)
{
    PRINT_DEBUG(5, "Started\n");
    move(forward_distance, 1);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveBackward(double backward_distance)
{
    PRINT_DEBUG(5, "Started\n");
    move(-backward_distance, 1);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

// @todo Please look into what this function does
void
ControlUINode::moveInDirection(const vector<float> &dir,
                               const vector<double> &position,
                               const vector<Point3f> &points)
{
    PRINT_DEBUG(5, "Started\n");
    float point_distance = getPointToPlaneDistance(dir, position);
    int move = (_fixed_distance >= point_distance) ? -1: 1;
    float step_distance = fabs(_fixed_distance - point_distance);
    PRINT_DEBUG(5, "Drone Distance: " << point_distance << ", Fixed Distance: " << _fixed_distance << "\n");
    PRINT_DEBUG(5, "Move: " << move << ", Step Distance: " << step_distance << "\n");
    if(move == -1)
    {
        PRINT_DEBUG(5, "Moving backwards\n");
    }
    else if(move == 1)
    {
        PRINT_DEBUG(5, "Moving forwards\n");
    }
    Point3f pp, pos((float)position[0], (float)position[1], (float)position[2]);
    float t, avg_a = 0.0, avg_b = 0.0, avg_c = 0.0;
    float a = dir[0];
    float b = dir[1];
    float c = dir[2];
    PRINT_DEBUG(5, print1dVector(dir, "Plane Parameters for distance adjustment"));
    PRINT_DEBUG(5, print1dVector(position, "Current position of drone"));
    unsigned int numberOfPointsInThisPlane = points.size();
    for (unsigned int j = 0; j < numberOfPointsInThisPlane; ++j)
    {
        avg_a += points[j].x;
        avg_b += points[j].y;
        avg_c += points[j].z;
    }
    avg_a /= (float)points.size();
    avg_b /= (float)points.size();
    avg_c /= (float)points.size();
    Point3f p(avg_a, avg_b, avg_c), qp;
    float mag = ((a*a)+(b*b)+(c*c));
    qp = pos - p;
    t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
    Point3f proj(pos.x - a*t, pos.y - b*t, pos.z - c*t);
    Point3f dest;
    PRINT_DEBUG(5, "t: " << t << "\n");
    if(signbit(t)) {t = -1.0;}
    else {t = 1.0;}
    dest.x = proj.x + a*t*(_fixed_distance);
    dest.y = proj.y + b*t*(_fixed_distance);
    dest.z = proj.z + c*t*(_fixed_distance);
    PRINT_DEBUG(5, "t: " << t*(_fixed_distance) << "\n");
    PRINT_DEBUG(5, "Point on plane: " << proj << ", Point to move: " << dest << "\n");
    vector<double> init_pos, dest_pos;
    init_pos.push_back(position[0]);
    init_pos.push_back(position[1]);
    init_pos.push_back(position[2]);
    dest_pos.push_back((double)dest.x);
    dest_pos.push_back((double)dest.y);
    dest_pos.push_back((double)dest.z);
    clear2dVector(_interm_path);
    getInitialPath(init_pos, dest_pos, position[3], position[3], _interm_path);
    moveDroneViaSetOfPoints(_interm_path);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::moveDroneViaSetOfPoints(const vector< vector<double> > &dest_points)
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started.\n");
    pthread_mutex_lock(&changeyaw_CS);
    changeyawLockReleased = 1;
    PRINT_DEBUG(4, "Acquired first changeyaw_CS Lock\n");
    char buf[100];
    just_navigation_commands.clear();
    targetPoints.clear();
    just_navigation_total_commands = dest_points.size();
    just_navigation_command_number = 0;
    PRINT_DEBUG(5, "Total commands generated: " << just_navigation_total_commands << "\n");
    PRINT_DEBUG(4, print2dVector(dest_points, "Moving points:\n", ""));
    for (unsigned int i = 0; i < dest_points.size(); ++i)
    {
        snprintf(buf, 100, "c goto %lf %lf %lf %lf",
            dest_points[i][0], dest_points[i][1], dest_points[i][2], dest_points[i][3]);
        // Contains the set of points generated by the code and
        // traversed by the drone
        PRINT_DEBUG(4, "Target point: (" << dest_points[i][0] << ", " << dest_points[i][1]
                            << ", " << dest_points[i][2] << ", " << dest_points[i][3] << ")\n");
        visited_motion_points.push_back(dest_points[i]);
        std_msgs::String s;
        s.data = buf;
        just_navigation_commands.push_back(s);
        targetPoints.push_back(dest_points[i]);
    }
    PRINT_DEBUG(4, "Commands to execute: " << just_navigation_commands.size() << "\n");
    // Once the commands are done, give an indication to release the lock
    if(just_navigation_commands.size() == 0)
    {
        changeyawLockReleased = -1;
    }
    pthread_mutex_lock(&changeyaw_CS);
    PRINT_DEBUG(4, "Acquired second changeyaw_CS Lock\n");
    pthread_mutex_unlock(&changeyaw_CS);
    getCurrentPositionOfDrone();
    PRINT_DEBUG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone after movement: ", ""));
    PRINT_DEBUG(4, "Released second changeyaw_CS Lock\n");
    just_navigation_total_commands = -1;
    just_navigation_command_number = -1;
    PRINT_LOG(1, "Completed.\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
}

void
ControlUINode::rotateClockwise(double step_angle)
{
    PRINT_DEBUG(5, "Started.\n");
    getCurrentPositionOfDrone();
    designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+step_angle);
    moveDroneViaSetOfPoints(_interm_path);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

void
ControlUINode::rotateCounterClockwise(double step_angle)
{
    PRINT_DEBUG(5, "Started\n");
    getCurrentPositionOfDrone();
    designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]-step_angle);
    moveDroneViaSetOfPoints(_interm_path);
    PRINT_DEBUG(5, "Completed\n");
    return ;
}

/**
 * @brief Generates the set of points (smoothly distributed) which drone has to follow to change its yaw by large angles
 * @details The current point is represented as (x, y, z, yaw)
 * @param [vector< vector<double> >] dest_points - Points to where quadcopter has to travel
 *                                  Includes yaw in the vector
 * @return
 */
void
ControlUINode::designPathToChangeYaw(const vector<double> &curr_point,
                                     double dest_yaw)
{
    PRINT_DEBUG(5, "Started\n");
    clear2dVector(_interm_path);
    vector<double> interm_point;
    double currentYaw = curr_point[3], desiredYaw = dest_yaw;
    PRINT_DEBUG(5, "Changing yaw by " << _angle_heuristic << " step\n");
    PRINT_DEBUG(5, "Current Yaw: " << currentYaw << "\n");
    PRINT_DEBUG(5, "Destination Yaw: " << desiredYaw << "\n");
    double move; bool to_move = true;
    if(currentYaw > desiredYaw) {move = -1.0;}
    else {move = 1.0;}
    double prog_yaw = currentYaw;
    if(move == 1.0)
    {
        PRINT_DEBUG(5, "Move clockwise\n");
    }
    else
    {
        PRINT_DEBUG(5, "Move counter-clockwise\n");
    }
    while(to_move)
    {
        if(fabs(desiredYaw - prog_yaw) < _angle_heuristic)
        {
            if(move == -1.0)
            {
                if(desiredYaw < -180.0)
                {
                    desiredYaw = 360.0+desiredYaw; prog_yaw = desiredYaw;
                }
            }
            else if(move == 1.0)
            {
                if(desiredYaw >= 180.0)
                {
                    desiredYaw = -360.0+desiredYaw; prog_yaw = desiredYaw;
                }
            }
            else
            { }
            to_move = false;
        }
        else
        {
            prog_yaw += (move * (double)_angle_heuristic);
            if(move == -1.0)
            {
                if(prog_yaw < -180.0)
                {
                    prog_yaw = 360.0+prog_yaw; desiredYaw = 360.0+desiredYaw;
                }
            }
            else if(move == 1.0)
            {
                if(prog_yaw >= 180.0)
                {
                    prog_yaw = -360.0+prog_yaw; desiredYaw = -360.0+desiredYaw;
                }
            }
            else
            { }
            to_move = true;
        }
        interm_point.clear();
        interm_point.push_back(curr_point[0]); interm_point.push_back(curr_point[1]);
        interm_point.push_back(curr_point[2]); interm_point.push_back(prog_yaw);
        _interm_path.push_back(interm_point);
    }
    PRINT_DEBUG(5,  print2dVector(_interm_path, "Final Path"));
    PRINT_DEBUG(5, "Completed\n");
}

void
ControlUINode::copyNecessaryInfo()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    // If the plane was covered in one snap, ie, the plane is smaller in dimension
    if(!_is_big_plane && aug_three_d_points.size() == 0)
    {
        PRINT_LOG(1, "Not a big plane. Covered in one go. Copying the necessary information\n");
        PRINT_LOG(1, "Plane no.: " << _node_completed_number_of_planes << "\n");
        PRINT_LOG(2, print1dVector(this_plane_parameters, "Final Plane parameters:\n", ""));
        PRINT_LOG(2, print1dVector(this_continuous_bounding_box_points, "Final Continuous Bounding Box points:\n", ""));
        // Currently captured plane parameters of the plane
        visited_plane_parameters.push_back(this_plane_parameters);
        // Currently captured bounding box points of the plane
        visited_continuous_bounding_box_points.push_back(this_continuous_bounding_box_points);
    }
    // If the plane was big enough to fit in the drone's view in onwe shot
    else if((!_is_big_plane && aug_three_d_points.size() > 0) || _is_big_plane)
    {
        PRINT_LOG(1, "A big plane. Could not cover in one go. Estimating the best plane\n");
        if(!_is_big_plane)
        {
            PRINT_LOG(1, "Using information from augmentInfo()\n");
        }
        PRINT_LOG(1, "Plane no.: " << _node_completed_number_of_planes << "\n");
        vector<float> new_plane_params;
        new_plane_params.clear();
        // Fit the plane to the augmented 3D points
        fitPlane3D(aug_three_d_points, new_plane_params);
        PRINT_DEBUG(3, print1dVector(new_plane_params, "New plane parameters:\n", ""));
        getCurrentPositionOfDrone();
        // Fix the plane parameters sign based on its position
        checkPlaneParametersSign(_node_current_pos_of_drone, aug_three_d_points, new_plane_params);
        //new_plane_params[3] = _plane_d/(float)_plane_d_num;
        vector<Point3f> new_bounding_box_points;
        new_bounding_box_points.clear();
        PRINT_DEBUG(3, print1dVector(aug_plane_bounding_box_points, "Aug. Continuous Bounding Box Points:\n", ""));
        PRINT_DEBUG(3, "Calculating the bounding box points\n");
        projectPointsOnPlane(aug_plane_bounding_box_points, new_plane_params, new_bounding_box_points);
        PRINT_DEBUG(3, print1dVector(new_bounding_box_points, "New Continuous Bounding Box Points:\n", ""));
        PRINT_LOG(3, "Copying the necessary information\n");
        visited_plane_parameters.push_back(new_plane_params);
        visited_continuous_bounding_box_points.push_back(new_bounding_box_points);
        aug_three_d_points.clear();
        aug_plane_bounding_box_points.clear();
        new_bounding_box_points.clear();
    }
    vector< vector<Point3f> > cbb;
    vector< vector<Point3f> > last_visited_plane;
    PRINT_LOG(2, "Rendering last visited plane\n");
    image_gui->setRender(false, false, false, true);
    image_gui->renderFrame();
    getContinuousBoundingBox (visited_continuous_bounding_box_points, 
                                visited_plane_parameters, cbb);
    last_visited_plane.push_back(cbb.back());
    image_gui->setVisitedBoundingBoxPoints(last_visited_plane);
    clear2dVector(cbb);
    clear2dVector(last_visited_plane);
    /*this_plane_parameters.clear();
    this_continuous_bounding_box_points.clear();*/
    PRINT_LOG(3, print2dVector(visited_plane_parameters, "Visited Plane Parameters:\n", ""));
    PRINT_LOG(3, print2dVector(visited_continuous_bounding_box_points, "Visited Continuous Bounding Box Points:\n", ""));
    string filename = "Plane_Info.txt";
    _is_big_plane = false;
    _stage_of_plane_observation = true;
    _node_completed_number_of_planes++;
    _plane_d = 0.0;
    _plane_d_num = 0;
    image_gui->WriteInfoToFile(visited_continuous_bounding_box_points.back(), 
                                visited_plane_parameters.back(), _node_completed_number_of_planes, filename);
    PRINT_LOG(1, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

void
ControlUINode::augmentInfo()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    PRINT_LOG(1, "Adding the 3d points to be used later for best fit\n");
    int index = _actual_plane_index;
    PRINT_DEBUG(4, "Index: " << index << ", Actual Plane Index: " << _actual_plane_index 
                    << ", SigPlaneIndex: " << _sig_plane_index << "\n");
    //if(index == -1) {index  = 0;}
    PRINT_DEBUG(2, "Stage of plane observation: " << _stage_of_plane_observation << "\n");
    if(_stage_of_plane_observation)
    {
        aug_plane_bounding_box_points.clear();
        for (unsigned int i = 0; i < this_continuous_bounding_box_points.size(); ++i)
        {
            aug_plane_bounding_box_points.push_back(this_continuous_bounding_box_points[i]);
        }
        aug_three_d_points.clear();
        for (unsigned int i = 0; i < jlink_three_d_points[index].size(); ++i)
        {
            aug_three_d_points.push_back(jlink_three_d_points[index][i]);
        }
        _stage_of_plane_observation = false;
    }
    else
    {
        for (unsigned int i = 0; i < jlink_three_d_points[index].size(); ++i)
        {
            aug_three_d_points.push_back(jlink_three_d_points[index][i]);
        }
        string filename = "plane_points_" + to_string(_node_completed_number_of_planes+1);
        write3DPointsToCSV(aug_three_d_points, filename, " ", "", 6);
        _capture_mode = false;
        Point3f a0, a1, a2, a3;
        a0 = aug_plane_bounding_box_points[0];
        a1 = this_continuous_bounding_box_points[1];
        a2 = this_continuous_bounding_box_points[2];
        a3 = aug_plane_bounding_box_points[3];
        aug_plane_bounding_box_points.clear();
        aug_plane_bounding_box_points.push_back(a0);
        aug_plane_bounding_box_points.push_back(a1);
        aug_plane_bounding_box_points.push_back(a2);
        aug_plane_bounding_box_points.push_back(a3);
        aug_plane_bounding_box_points.push_back(a0);
    }
    PRINT_DEBUG(4, print1dVector(aug_plane_bounding_box_points, "Aug. Plane BB:\n"));
    PRINT_LOG(1, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

/**
 * @brief Get all the 3d planes present within the boundary of the clicked points on he image
 * @details
 */
void
ControlUINode::getMultiplePlanes3d (const vector<int> &ccPoints, const vector< vector<int> > &pointsClicked,
                                    vector< vector<float> > &planeParameters,
                                    vector< vector<Point3f> > &continuousBoundingBoxPoints,
                                    vector< vector<Point3f> > &sorted_3d_points,
                                    vector<float> &percentageOfEachPlane)
{
    clock_t beginTime, endTime, jlBeginTime, jlEndTime;
    double elapsedTime, jlElapsedTime;
    beginTime = clock();
    PRINT_DEBUG(1, "Started\n");
    vector<Point3f> _in_points;
    vector< vector<int> > points;
    _in_points.clear();
    for(unsigned int i = 0; i < ccPoints.size(); i++)
    {
        points.push_back(pointsClicked[ccPoints[i]]);
    }
    pthread_mutex_lock(&keyPoint_CS);
    for(unsigned int i = 0; i < _2d_points.size(); i++)
    {
        if(liesInside(points, _2d_points[i]))
        {
            Point3f featurePt;
            featurePt.x = _3d_points[i][0];
            featurePt.y = _3d_points[i][1];
            featurePt.z = _3d_points[i][2];
            _in_points.push_back(featurePt);
        }
    }
    string filename = "plane_points_" + to_string(_node_completed_number_of_planes+1);
    write3DPointsToCSV(_in_points, filename, " ", "", 6);
    _capture_mode = false;
    pthread_mutex_unlock(&keyPoint_CS);
    PRINT_DEBUG(2, "Captured the 3d points within the clicked points\n");
    // See multiplePlanes.cpp
    vector< vector<Point3f> > in_points;
    vector< vector<float> > in_pp;
    vector< vector<Point3f> > in_cbb;
    vector<float> in_p;
    jlBeginTime = clock();
    findPercBoundEachPlane(_in_points, in_pp, in_cbb, in_points, in_p);
    jlEndTime = clock();
    jlElapsedTime = double(jlEndTime - jlBeginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for j-linkage function is " << jlElapsedTime << " ms.\n");
    getCurrentPositionOfDrone();
    PRINT_DEBUG(5, "Fixing plane orientation and CBB\n");
    orderPlanesFromQuadcopterPosition(_node_current_pos_of_drone, in_points, in_pp, in_cbb, in_p,
                                      sorted_3d_points, planeParameters, continuousBoundingBoxPoints, percentageOfEachPlane);
    clear2dVector(in_points);
    clear2dVector(in_pp);
    clear2dVector(in_cbb);
    in_p.clear();
    PRINT_DEBUG(1, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

/**
 * @brief Get all the 3d planes from the visible feature points in the scene
 * @details
 */
void
ControlUINode::getMultiplePlanes3d (vector< vector<float> > &planeParameters,
                                    vector< vector<Point3f> > &continuousBoundingBoxPoints,
                                    vector< vector<Point3f> > &sorted_3d_points,
                                    vector<float> &percentageOfEachPlane)
{
    clock_t beginTime, endTime, jlBeginTime, jlEndTime;
    double elapsedTime, jlElapsedTime;
    beginTime = clock();
    PRINT_DEBUG(1, "Started\n");
    vector< Point3f > _in_points;
    _in_points.clear();
    pthread_mutex_lock(&keyPoint_CS);
    for(unsigned int i = 0; i < _3d_points.size(); i++)
    {
        Point3f featurePt;
        featurePt.x = _3d_points[i][0];
        featurePt.y = _3d_points[i][1];
        featurePt.z = _3d_points[i][2];
        _in_points.push_back(featurePt);
    }
    pthread_mutex_unlock(&keyPoint_CS);
    PRINT_DEBUG(10, print1dVector(_in_points, "Points Clicked by the user in 3D:\n", ""));
    PRINT_DEBUG(2, "Captured all the 3d points available\n");
    // See multiplePlanes.cpp
    vector< vector<Point3f> > in_points;
    vector< vector<float> > in_pp;
    vector< vector<Point3f> > in_cbb;
    vector<float> in_p;
    jlBeginTime = clock();
    findPercBoundEachPlane(_in_points, in_pp, in_cbb, in_points, in_p);
    jlEndTime = clock();
    jlElapsedTime = double(jlEndTime - jlBeginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function j-linkage is " << jlElapsedTime << " ms.\n");
    getCurrentPositionOfDrone();
    PRINT_DEBUG(5, "Fixing plane orientation and CBB\n");
    orderPlanesFromQuadcopterPosition(_node_current_pos_of_drone, in_points, in_pp, in_cbb, in_p,
                                    sorted_3d_points, planeParameters, continuousBoundingBoxPoints, percentageOfEachPlane);
    clear2dVector(in_points);
    clear2dVector(in_pp);
    clear2dVector(in_cbb);
    in_p.clear();
    PRINT_DEBUG(1, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

int
ControlUINode::checkVisibility(const vector<float> &plane_parameters, 
                               const vector<Point3f> &continuous_bounding_box_points, int which_side)
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_DEBUG(5, "Started\n");
    int move = 0;
    vector<Point2f> image_bounding_box_points;
    PRINT_DEBUG(5, print1dVector(plane_parameters, "Plane Parameters:"));
    PRINT_DEBUG(5, print1dVector(continuous_bounding_box_points, "CBB Points:"));
    if(which_side == 0) // Top and Bottom edge
    {
        image_bounding_box_points.clear();
        project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
        PRINT_DEBUG(5, "IBB Points:");
        PRINT_DEBUG(5, print1dVector(image_bounding_box_points, "Image Bounding Box points:\n", ""));
        Point2f top_mid = (image_bounding_box_points[0]+image_bounding_box_points[1]);
        top_mid.x = top_mid.x/(float)2.0;
        top_mid.y = top_mid.y/(float)2.0;
        Point2f bottom_mid = (image_bounding_box_points[2]+image_bounding_box_points[3]);
        bottom_mid.x = bottom_mid.x/(float)2.0;
        bottom_mid.y = bottom_mid.y/(float)2.0;
        Line2f tb_edge(top_mid, bottom_mid);
        PRINT_DEBUG(5, "Top to Bottom edge: " << tb_edge << "\n");
        // @todo-me Fix this heuristic
        if( //(tb_edge.start.x >= 256.0                                  && tb_edge.start.x <= 384.0) &&
            (tb_edge.start.y >= 72.0 && tb_edge.start.y <= 144.0) ||
            //(tb_edge.end.x >= 256.0 && tb_edge.end.x <= 384.0) &&
            (tb_edge.end.y >= 216.0 && tb_edge.end.y <= 288.0)  )
        {
            move = 0;
        }
        else if( //(tb_edge.start.x >= 256.0 && tb_edge.start.x <= 384.0) &&
                (tb_edge.start.y >= 0.0 && tb_edge.start.y <= 72.0) ||
                //(tb_edge.end.x >= 256.0 && tb_edge.end.x <= 384.0) &&
                (tb_edge.end.y >= 288.0 && tb_edge.end.y <= 360.0) )
        {
            move = -1;
        }
        else
        {
            move = 1;
        }
    }
    else if(which_side == 1) // Left Edge
    {
        PRINT_DEBUG(5, "Checking for left edge\n");
        image_bounding_box_points.clear();
        project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
        PRINT_DEBUG(5, "IBB Points:");
        PRINT_DEBUG(5, print1dVector(image_bounding_box_points, "Image Bounding Box points:\n", ""));
        Point2f start, end;
        if(image_bounding_box_points[0].x >= image_bounding_box_points[1].x)
        {
            start = image_bounding_box_points[1];
            end = image_bounding_box_points[2];
        }
        else
        {
            start = image_bounding_box_points[0];
            end = image_bounding_box_points[3];
        }
        Line2f left_edge(start, end);
        PRINT_DEBUG(5, "Left Edge: " << left_edge << "\n");
        // @todo-me Fix this heuristic
        if( (left_edge.start.x < -40.0) ||
                (left_edge.end.x < -40.0) )
        {
            move = 0;
        }
        else if( (left_edge.start.x >= 128.0 && left_edge.start.x <= 256.0) ||
            //(left_edge.start.y >= 144.0 && left_edge.start.x <= 216.0) &&
            (left_edge.end.x >= 128.0 && left_edge.end.x <= 256.0) ) /*&&
            (left_edge.end.y >= 216.0 && left_edge.end.y <= 288.0)  )*/
        {
            move = 0;
        }
        else if( (left_edge.start.x >= -40.0 && left_edge.start.x <= 128.0) ||
                //(left_edge.start.y >= 0.0 && left_edge.start.x <= 72.0) &&
                (left_edge.end.x >= -40.0 && left_edge.end.x <= 128.0) )/*&&
                (left_edge.end.y >= 288.0 && left_edge.end.y <= 360.0) )*/
        {
            move = -1;
        }
        else
        {
            move = 1;
        }
    }
    else if(which_side == 2) // Right edge
    {
        PRINT_DEBUG(5, "Checking for right edge\n");
        image_bounding_box_points.clear();
        project3DPointsOnImage(continuous_bounding_box_points, image_bounding_box_points);
        PRINT_DEBUG(5, "IBB Points:");
        PRINT_DEBUG(5, print1dVector(image_bounding_box_points, "Image Bounding Box points:\n", ""));
        Point2f start, end;
        if(image_bounding_box_points[0].x >= image_bounding_box_points[1].x)
        {
            start = image_bounding_box_points[0];
            end = image_bounding_box_points[3];
        }
        else
        {
            start = image_bounding_box_points[1];
            end = image_bounding_box_points[2];
        }
        Line2f right_edge(start, end);
        PRINT_DEBUG(5, "Right Edge: " << right_edge << "\n");
        // @todo-me Fix this heuristic
        if( (right_edge.start.x >= 300.0 && right_edge.start.x <= 512.0) ||
            (right_edge.end.x >= 300.0 && right_edge.end.x <= 512.0) )
        {
            move = 0;
        }
        else if( (right_edge.start.x >= 512.0 && right_edge.start.x <= 640.0) ||
                (right_edge.end.x >= 512.0 && right_edge.end.x <= 640.0) )
        {
            move = 1;
        }
        /*if( (right_edge.start.x >= 240.0 && right_edge.start.x <= 580.0) ||
            (right_edge.end.x >= 240.0 && right_edge.end.x <= 580.0) )
        {
            move = 0;
        }
        else if( (right_edge.start.x >= 580.0 && right_edge.start.x <= 640.0) ||
                (right_edge.end.x >= 580.0 && right_edge.end.x <= 640.0) )
        {
            move = 1;
        }*/
        else
        {
            move = -1;
        }
    }
    else
    {
        PRINT_DEBUG(5, "Currently not dealing with it\n");
    }
    PRINT_DEBUG(5, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return move;
}

void
ControlUINode::doJLinkage()
{
    _jlinkage_calls++;
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_DEBUG(5, "Started\n");
    clear2dVector(jlink_all_plane_parameters);
    clear2dVector(jlink_all_continuous_bounding_box_points);
    clear2dVector(jlink_three_d_points);
    jlink_all_percentage_of_each_plane.clear();
    PRINT_DEBUG(5, "Calling JLinkage\n");
    getMultiplePlanes3d (jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points, jlink_three_d_points, jlink_all_percentage_of_each_plane);
    PRINT_DEBUG(5, print2dVector(jlink_all_plane_parameters, "Link Plane Params"));
    PRINT_DEBUG(5, print2dVector(jlink_all_continuous_bounding_box_points, "JLink CBB"));
    PRINT_DEBUG(5, print1dVector(jlink_all_percentage_of_each_plane, "JLink Pecentage"));
    PRINT_DEBUG(5, print2dVector(visited_plane_parameters, "Visited PP"));
    PRINT_DEBUG(5, print2dVector(visited_continuous_bounding_box_points, " Visited CBB"));
    _sig_plane_index = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
    _actual_plane_index = _sig_plane_index;
    if(_actual_plane_index == -2) {_actual_plane_index = 0;}
    if(_actual_plane_index == -1) {_actual_plane_index = (int)jlink_all_plane_parameters.size()-1;}
    PRINT_DEBUG(5, "Sig Plane Index: " << _sig_plane_index << ", Actual Plane Index: " << _actual_plane_index << "\n");
    //assert(_sig_plane_index >= 0);
    getCompleteCurrentPlaneInfo(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
                                jlink_three_d_points, jlink_all_percentage_of_each_plane, _actual_plane_index,
                                this_plane_parameters, this_continuous_bounding_box_points, this_sorted_3d_points);
    PRINT_DEBUG(5, print1dVector(this_plane_parameters, "Sig PP"));
    PRINT_DEBUG(5, print1dVector(this_continuous_bounding_box_points, "Sig CBB"));
    PRINT_DEBUG(5, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

void
ControlUINode::doJLinkage(const vector<int> &ccPoints, const vector< vector<int> > &pointsClicked)
{
    _jlinkage_calls++;
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_DEBUG(5, "Started\n");
    clear2dVector(jlink_all_plane_parameters);
    clear2dVector(jlink_all_continuous_bounding_box_points);
    clear2dVector(jlink_three_d_points);
    jlink_all_percentage_of_each_plane.clear();
    PRINT_DEBUG(5, "Calling JLinkage\n");
    getMultiplePlanes3d (ccPoints, pointsClicked, jlink_all_plane_parameters, 
                        jlink_all_continuous_bounding_box_points, jlink_three_d_points, 
                        jlink_all_percentage_of_each_plane);
    PRINT_DEBUG(5, print2dVector(jlink_all_plane_parameters, "JLink Plane Params:\n", ""));
    PRINT_DEBUG(5, print2dVector(jlink_all_continuous_bounding_box_points, "JLink CBB:\n", ""));
    PRINT_DEBUG(5, print1dVector(jlink_all_percentage_of_each_plane, "JLink Pecentage:\n", ""));
    PRINT_DEBUG(5, print2dVector(visited_plane_parameters, "Visited PP:\n", ""));
    PRINT_DEBUG(5, print2dVector(visited_continuous_bounding_box_points, "Visited CBB:\n", ""));
    _sig_plane_index = getCurrentPlaneIndex(visited_plane_parameters, jlink_all_plane_parameters, jlink_all_percentage_of_each_plane);
    _actual_plane_index = _sig_plane_index;
    if(_actual_plane_index == -2) {_actual_plane_index = 0;}
    if(_actual_plane_index == -1) {_actual_plane_index = (int)jlink_all_plane_parameters.size()-1;}
    PRINT_DEBUG(5, "Sig Plane Index: " << _sig_plane_index << ", Actual Plane Index: " << _actual_plane_index << "\n");
    //assert(_sig_plane_index >= 0);
    getCompleteCurrentPlaneInfo(jlink_all_plane_parameters, jlink_all_continuous_bounding_box_points,
                                jlink_three_d_points,
                                jlink_all_percentage_of_each_plane, _actual_plane_index,
                                this_plane_parameters, this_continuous_bounding_box_points,
                                this_sorted_3d_points);
    PRINT_DEBUG(5, print1dVector(this_plane_parameters, "Sig PP: ", ""));
    PRINT_DEBUG(5, print1dVector(this_continuous_bounding_box_points, "Sig CBB:\n", ""));
    PRINT_DEBUG(5, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}


void
ControlUINode::getCompleteCurrentPlaneInfo(const vector< vector<float> > &plane_parameters,
                                           const vector< vector<Point3f> > &cbb,
                                           const vector< vector<Point3f> > &points,
                                           const vector<float> &percPlane,
                                           int currPlaneIndex,
                                           vector<float> &out_plane_parameters,
                                           vector<Point3f> &out_cbb,
                                           vector<Point3f> &out_3d_points)
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(5, "Started\n");
    if(currPlaneIndex < 0 && currPlaneIndex != -1)
    {
        PRINT_DEBUG(5, "[ERROR] currPlaneIndex is negative: " << currPlaneIndex << "\n");
    }
    else
    {
        out_plane_parameters.clear();
        out_cbb.clear();
        out_3d_points.clear();
        vector<Point3f> three_d_points;
        vector<float> temp_pp;
        vector<Point3f> bbp;
        float temp_plane_d = 0.0;
        unsigned int destPlaneIndex = currPlaneIndex;
        Point3f normal_old;
        getCurrentPositionOfDrone();
        vector<float> plane_old, plane_new;
        plane_old.clear();
        plane_old.push_back(plane_parameters[currPlaneIndex][0]);
        plane_old.push_back(plane_parameters[currPlaneIndex][1]);
        plane_old.push_back(plane_parameters[currPlaneIndex][2]);
        plane_old.push_back(plane_parameters[currPlaneIndex][3]);
        checkPlaneParametersSign(_node_current_pos_of_drone, points[currPlaneIndex], plane_old);
        normal_old.x = plane_old[0];
        normal_old.y = plane_old[1];
        normal_old.z = plane_old[2];
        float plane_heuristic = 0.96592;
        for (unsigned int i = currPlaneIndex+1; i < plane_parameters.size(); ++i)
        {
            Point3f normal_new;
            plane_new.clear();
            plane_new.push_back(plane_parameters[i][0]);
            plane_new.push_back(plane_parameters[i][1]);
            plane_new.push_back(plane_parameters[i][2]);
            plane_new.push_back(plane_parameters[i][3]);
            getCurrentPositionOfDrone();
            checkPlaneParametersSign(_node_current_pos_of_drone, points[i], plane_new);
            normal_new.x = plane_new[0];
            normal_new.y = plane_new[1];
            normal_new.z = plane_new[2];
            float dot_p = ((normal_old.x * normal_new.x)+(normal_old.y * normal_new.y)+(normal_old.z * normal_new.z));
            if(dot_p >= plane_heuristic)
            { destPlaneIndex++; }
            else { break; }
        }
        float avg_a = 0.0, avg_b = 0.0, avg_c = 0.0, avg_d = 0.0;
        PRINT_DEBUG(5, "CurrPlaneIndex: " << currPlaneIndex << ", destPlaneIndex: " << destPlaneIndex << "\n");
        for (unsigned int i = currPlaneIndex; i <= destPlaneIndex; ++i)
        {
            avg_a += plane_parameters[i][0];
            avg_b += plane_parameters[i][1];
            avg_c += plane_parameters[i][2];
            avg_d += plane_parameters[i][3];
            for (unsigned int j = 0; j < points[i].size(); ++j)
            {
                three_d_points.push_back(points[i][j]);
                out_3d_points.push_back(points[i][j]);
            }
            temp_plane_d += plane_parameters[i][3];
        }
        avg_a /= (destPlaneIndex - currPlaneIndex + 1);
        avg_b /= (destPlaneIndex - currPlaneIndex + 1);
        avg_c /= (destPlaneIndex - currPlaneIndex + 1);
        avg_d /= (destPlaneIndex - currPlaneIndex + 1);
        /*out_plane_parameters.push_back(avg_a);
        out_plane_parameters.push_back(avg_b);
        out_plane_parameters.push_back(avg_c);
        out_plane_parameters.push_back(avg_d);*/
        temp_pp.push_back(avg_a);
        temp_pp.push_back(avg_b);
        temp_pp.push_back(avg_c);
        if((int)destPlaneIndex > (int)currPlaneIndex)
        {
            out_plane_parameters.clear();
            fitPlane3D(three_d_points, out_plane_parameters);
        }
        else
        {
            out_plane_parameters.clear();
            out_plane_parameters.push_back(avg_a);
            out_plane_parameters.push_back(avg_b);
            out_plane_parameters.push_back(avg_c);
            out_plane_parameters.push_back(avg_d);
        }
        getCurrentPositionOfDrone();
        checkPlaneParametersSign(_node_current_pos_of_drone, three_d_points, out_plane_parameters);
        if(signbit(avg_a) == signbit(out_plane_parameters[0]))
        {
            PRINT_DEBUG(5, "Plane parameters sign not reversed\n");
            bbp.push_back(cbb[currPlaneIndex][0]);
            bbp.push_back(cbb[destPlaneIndex][1]);
            bbp.push_back(cbb[destPlaneIndex][2]);
            bbp.push_back(cbb[currPlaneIndex][3]);
            bbp.push_back(cbb[currPlaneIndex][0]);
        }
        else
        {
            PRINT_DEBUG(5, "Plane parameters sign reversed\n");
            bbp.push_back(cbb[currPlaneIndex][1]);
            bbp.push_back(cbb[destPlaneIndex][0]);
            bbp.push_back(cbb[destPlaneIndex][3]);
            bbp.push_back(cbb[currPlaneIndex][2]);
            bbp.push_back(cbb[currPlaneIndex][1]);
        }
        PRINT_DEBUG(5, "Calculating the bounding box points\n");
        projectPointsOnPlane(bbp, out_plane_parameters, out_cbb);
        three_d_points.clear();
        bbp.clear();
        PRINT_LOG(5, print1dVector(out_plane_parameters, "Current visible plane parameters: ", ""));
        PRINT_LOG(5, print1dVector(out_cbb, "Current visible cbb:\n", ""));
    }
    PRINT_LOG(5, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

void
ControlUINode::checkPlaneParametersSign(const vector<double> &position, 
                                        const vector<Point3f> &points,
                                        vector<float> &plane_parameters)
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    assert(points.size() >= 3);
    PRINT_LOG(5, "Started\n");
    // Create a matrix out of the vector of points: Dimension: numberOfPoints*3
    float x_c = 0.0, y_c = 0.0, z_c = 0.0;
    for (unsigned int i = 0; i < points.size(); ++i)
    {
        x_c += points[i].x;
        y_c += points[i].y;
        z_c += points[i].z;
    }
    x_c /= points.size();
    y_c /= points.size();
    z_c /= points.size();
    // Calculate the centroid of the points
    float centroidX = x_c;
    float centroidY = y_c;
    float centroidZ = z_c;
    Point3f p(centroidX, centroidY, centroidZ), qp;
    PRINT_DEBUG(5, "Centroid: " << p << "\n");
    PRINT_DEBUG(5, print1dVector(position, "Position of drone: ", ""));
    float a = plane_parameters[0];
    float b = plane_parameters[1];
    float c = plane_parameters[2];
    float mag = ((a*a)+(b*b)+(c*c));
    PRINT_DEBUG(5, print1dVector(plane_parameters, "Old Plane Parameters: ", ""));
    Point3f pos((float)position[0], (float)position[1], (float)position[2]);
    qp = pos - p;
    PRINT_DEBUG(5, "qp: " << qp << "\n");
    float t = (qp.x * (a/mag)) + (qp.y * (b/mag)) + ((qp.z * (c/mag)));
    PRINT_DEBUG(5, "t: " << t << "\n");
    if(!signbit(t))
    {
        PRINT_DEBUG(5, "Sign change required\n");
        plane_parameters[0] = -plane_parameters[0];
        plane_parameters[1] = -plane_parameters[1];
        plane_parameters[2] = -plane_parameters[2];
        plane_parameters[3] = -plane_parameters[3];
    }
    PRINT_DEBUG(5, print1dVector(plane_parameters, "New Plane Parameters: ", ""));
    PRINT_LOG(5, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

/**
 * @brief Align the yaw of the quadcopter to the current plane's (the one which it is seeing) normal
 * @details
 */
void
ControlUINode::alignQuadcopterToCurrentPlane()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    if(_node_number_of_planes == 1)
    {
        _next_plane_dir = CLOCKWISE;
        _next_plane_angle = 0.0;
    }
    else
    {
        _next_plane_dir = _node_main_directions.front();
        _next_plane_angle = _node_main_angles.front();
    }
    PRINT_LOG(1, "Number of planes: " << _node_number_of_planes <<
                ", Completed Number of planes: " << _node_completed_number_of_planes << 
                ", Next plane dir: " << _next_plane_dir << ", Next plane angle: " << _next_plane_angle << "\n");
    adjustYawToCurrentPlane();
    if(_stage_of_plane_observation)
    {
        PRINT_LOG(1, "Observing plane for the first time\n");
        PRINT_DEBUG(5, "Plane Parameters Size: " << this_plane_parameters.size() << "\n");
        adjustTopBottomEdges();
        adjustLeftEdge();
        adjustYawToCurrentPlane();
    }
    PRINT_LOG(3, "Completed plane no.: " << _node_completed_number_of_planes << "\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    PRINT_LOG(1, "Completed\n");
    return ;
}

void
ControlUINode::adjustYawToCurrentPlane()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    getCurrentPositionOfDrone();
    PRINT_LOG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone: ", ""));
    _node_dest_pos_of_drone.clear();
    _node_dest_pos_of_drone.push_back(0.0);
    _node_dest_pos_of_drone.push_back(1.0);
    _node_dest_pos_of_drone.push_back(0.0);
    _node_dest_pos_of_drone.push_back(0.0);
    PRINT_DEBUG(4, "Converting destination position wrt world quadcopter origin\n");
    convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
    Point3f pYAxis(_node_ac_dest_pos_of_drone[0], _node_ac_dest_pos_of_drone[1], _node_ac_dest_pos_of_drone[2]);
    Point3f pOrigin(_node_current_pos_of_drone[0], _node_current_pos_of_drone[1], _node_current_pos_of_drone[2]);
    Point3f projectedNormal(pYAxis-pOrigin);
    PRINT_LOG(4, "Estimating multiple planes -> call to JLinkage\n");
    doJLinkage();
    Point3f plane_params(this_plane_parameters[0], this_plane_parameters[1], 0.0);
    PRINT_DEBUG(4, "pYAxis: " << pYAxis << "\n");
    PRINT_DEBUG(4, "pOrigin: " << pOrigin << "\n");
    PRINT_DEBUG(4, "projectedNormal: " << projectedNormal << "\n");
    PRINT_DEBUG(4, "PP: " << plane_params << "\n");
    float angle = findAngle(projectedNormal, plane_params);
    PRINT_DEBUG(4, "Angle (radians): " << angle << "\n");
    angle = -angle*180/M_PI;
    PRINT_LOG(4, "Change the yaw of quadcopter\n");
    PRINT_LOG(4, "Angle to rotate: " << angle << "\n");
    designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+angle);
    moveDroneViaSetOfPoints(_interm_path);
    PRINT_LOG(1, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    _traversal_mode_time += (elapsedTime/1000.0);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
}

/**
 * @brief Adjust the quadcopter to see the top and bottom edge of the current plane
 * @details
 */
void
ControlUINode::adjustTopBottomEdges()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    float step_distance;
    float point_distance, height;
    /*PRINT_LOG(4, "Estimating multiple planes -> call to JLinkage\n");
    doJLinkage();*/
    PRINT_DEBUG(5, print1dVector(this_plane_parameters, "Sig PP"));
    PRINT_DEBUG(5, print1dVector(this_continuous_bounding_box_points, "Sig CBB"));
    getCurrentPositionOfDrone();
    PRINT_DEBUG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone"));
    point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
    PRINT_DEBUG(2, "Distance from the plane: " << point_distance << "\n");
    step_distance = fabs(_node_max_distance - point_distance);
    int move = (_node_max_distance >= point_distance) ? -1: 1;
    PRINT_DEBUG(3, "Move: " << move << ", Step Distance: " << step_distance << "\n");
    if(move == -1)
    {
        PRINT_DEBUG(4, "Moving backwards\n");
        moveDroneByMeasure(step_distance, MOVE_DIRECTIONS::BACKWARD);
    }
    else if(move == 1)
    {
        PRINT_DEBUG(4, "Moving forwards\n");
        moveDroneByMeasure(step_distance, MOVE_DIRECTIONS::FORWARD);
    }
    getCurrentPositionOfDrone();
    PRINT_DEBUG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone"));
    if(!_fixed_height_set)
    {
        PRINT_DEBUG(3, "Fixed height not set\n");
        height = getHeightFromGround(this_plane_parameters, this_continuous_bounding_box_points, _node_current_pos_of_drone);
        PRINT_DEBUG(2, "Height: " << height << "\n");
        move = (_node_current_pos_of_drone[2] >= height) ? -1: 1;
        step_distance = fabs(_node_current_pos_of_drone[2] - height);
    }
    else
    {
        PRINT_DEBUG(3, "Fixed height set. Fixed Height: " << _fixed_height << "\n");
        move = (_node_current_pos_of_drone[2] >= _fixed_height) ? -1: 1;
        step_distance = fabs(_node_current_pos_of_drone[2] - _fixed_height);
    }
    if(move == -1)
    {
        PRINT_DEBUG(3, "Moving down\n");
        moveDroneByMeasure(step_distance, MOVE_DIRECTIONS::DOWN);
    }
    else if(move == 1)
    {
        PRINT_DEBUG(3, "Moving up\n");
        moveDroneByMeasure(step_distance, MOVE_DIRECTIONS::UP);
    }
    PRINT_DEBUG(3, "Adjusting top and bottom done.\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    _traversal_mode_time += (elapsedTime/1000.0);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

/**
 * @brief Adjust the quadcopter to see the left edge of the current plane
 * @details
 */
void
ControlUINode::adjustLeftEdge()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_DEBUG(1, "Started\n");
    PRINT_DEBUG(4, "Call Jlinkage\n");
    doJLinkage();
    bool planeLeftVisible;
    int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 1);
    if(move==0)
    {
        planeLeftVisible = true;
    }
    else
    {
        planeLeftVisible = false;
    }
    while(!planeLeftVisible)
    {
        getCurrentPositionOfDrone();
        PRINT_DEBUG(4, "Move: " << move << ", Step Distance: " << _move_heuristic << "\n");
        if(move == -1)
        {
            PRINT_DEBUG(4, "Moving left\n");
            moveDroneByMeasure(_move_heuristic, MOVE_DIRECTIONS::LEFT);
        }
        else if(move == 1)
        {
            PRINT_DEBUG(4, "Moving right\n");
            moveDroneByMeasure(_move_heuristic, MOVE_DIRECTIONS::RIGHT);
        }
        doJLinkage();
        move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 1);
        PRINT_DEBUG(4, "Move: " << move << "\n");
        if(move==0) {planeLeftVisible = true;}
        else {planeLeftVisible = false;}
    }
    PRINT_LOG(1, "Completed\n");
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    _traversal_mode_time += (elapsedTime/1000.0);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
}

/**
 * @brief Gets all the data about the points in the clicked region using jlinkage
 * @details To be called after user clicks the 4 points on the DRONE CAMERA FEED Window
 */
void
ControlUINode::captureTheCurrentPlane()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    PRINT_LOG(1, "Total Number of planes: " << _node_number_of_planes 
            << ", Number of planes covered: " << _node_completed_number_of_planes << "\n");
    // 2d image points clicked on the DRONE CAMERA FEED Screen
    vector< vector<int> > points_clicked;
    // the 3d keypoints of control node for nearest keypoints
    vector< vector<float> > key_points_nearest;
    // corners of the convex hull
    vector<int> cc_points;
    // First param: Number of points clicked on the screen
    // Second param: Number of Key points detected
    image_gui->setNumberOfPoints(0, 0);
    // RendeRect: false, RenderPoly: false, RenderSignificantPlane: false
    PRINT_DEBUG(3, "Stopped rendering of frame\n");
    image_gui->setRender(false, false, true, true);
    image_gui->getPointsClicked(points_clicked);
    PRINT_LOG(3, "Extracting Bounding Poly\n");
    image_gui->extractBoundingPoly();
    _sig_plane_index = 0;
    image_gui->getCCPoints(cc_points);
    PRINT_DEBUG(3, print2dVector(points_clicked, "Points Clicked by the user in 2D:\n", ""));
    PRINT_DEBUG(3, print1dVector(cc_points, "CCPoints:\n", ""));
    clear2dVector(key_points_nearest);
    for(unsigned int  i = 0; i < points_clicked.size(); i++)
    {
        key_points_nearest.push_back(searchNearest(points_clicked[i], true));
    }
    PRINT_DEBUG(3, print2dVector(key_points_nearest, "Points Clicked by the user in 3D:\n", ""));
    PRINT_LOG(3, "Get multiple planes from the clicked points using JLinkage\n");
    // Calls JLinkage and finds all planes within the clicked region
    vector< vector<float> > test_plane_parameters;
    // @todo Can it be changed to RANSAC assuming we're clicking on single plane???
    _capture_mode = true;
    doJLinkage(cc_points, points_clicked);
    // doJLinkage();
    // Render significant plane
    image_gui->setContinuousBoundingBoxPoints(jlink_all_continuous_bounding_box_points);
    image_gui->setSigPlaneBoundingBoxPoints(this_continuous_bounding_box_points);
    image_gui->setVisitedBoundingBoxPoints(visited_continuous_bounding_box_points);
    PRINT_LOG(3, "Rendering the frames in the DRONE CAMERA FEED GUI\n");
    image_gui->setRender(false, false, true, true);
    PRINT_DEBUG(4, "SigPlaneIndex: " << _sig_plane_index << "\n");
    PRINT_DEBUG(4, "ActualPlaneIndex: " << _actual_plane_index << "\n");
    PRINT_DEBUG(4, "Rendering significant plane in white and visited planes in black\n");
    image_gui->renderFrame();
    // Check if the plane is completed by finding if a new plane is visible or not other than current one
    clear2dVector(test_plane_parameters);
    for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
    {
        test_plane_parameters.push_back(visited_plane_parameters[i]);
    }
    test_plane_parameters.push_back(this_plane_parameters);
    PRINT_DEBUG(5, print2dVector(visited_plane_parameters, "All planes visited completely"));
    PRINT_DEBUG(5, print2dVector(test_plane_parameters, "All planes visited completely including the current one"));
    if(_node_completed_number_of_planes == 0 && _stage_of_plane_observation)
    {
        PRINT_LOG(3, "Checking if adjustment is required\n");
        getCurrentPositionOfDrone();
        PRINT_DEBUG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone"));
        Point3f top_mid = (this_continuous_bounding_box_points[0]+this_continuous_bounding_box_points[1]);
        top_mid.x = top_mid.x/(float)2.0;
        top_mid.y = top_mid.y/(float)2.0;
        top_mid.z = top_mid.z/(float)2.0;
        float distance = getDistanceToSeePlane((int)ceil(top_mid.z));
        _fixed_distance = distance;
        float point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
        int move = (distance >= point_distance) ? -1: 1;
        float step_distance = fabs(distance - point_distance);
        if(move == -1)
        {
            PRINT_DEBUG(3, "Moving backward\n");
            moveBackward(step_distance);
        }
        else if(move == 1)
        {
            PRINT_DEBUG(3, "Moving forward\n");
            moveForward(step_distance);
        }
        getCurrentPositionOfDrone();
        float height = getHeightFromGround(this_plane_parameters, this_continuous_bounding_box_points, _node_current_pos_of_drone);
        _fixed_height = height;
        _fixed_height_set = true;
        move = (_node_current_pos_of_drone[2] >= height) ? -1: 1;
        step_distance = fabs(_node_current_pos_of_drone[2] - height);
        if(move == -1)
        {
            PRINT_DEBUG(3, "Moving down\n");
            moveDown(step_distance);
        }
        else if(move == 1)
        {
            PRINT_DEBUG(3, "Moving up\n");
            moveUp(step_distance);
        }
        // @todo check if this doJlinkage is necessary?
        doJLinkage();
    }
    PRINT_DEBUG(3, "Observe the plane without rotation\n");
    _is_plane_covered = isNewPlaneVisible(test_plane_parameters,
                                        jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
    PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << "\n");
    if(!_is_plane_covered)
    {
        PRINT_DEBUG(3, "Checking if right edge is within the frame to reach a conclusion\n");
        int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 2);
        if(move==0)
        {
            _is_plane_covered = true;
        }
        else
        {
            _is_plane_covered = false;
        }
        PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << ", Move: " << move << "\n");
    }
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    _capture_mode_time += (elapsedTime/1000.0);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    if(_is_plane_covered)
    {
        copyNecessaryInfo();
        if(_node_completed_number_of_planes != _node_number_of_planes)
        {
            PRINT_LOG(1, "Completed plane no.: " << _node_completed_number_of_planes << "\n");
            PRINT_LOG(1, "Aligning the quadcopter to the next plane\n");
            alignQuadcopterToNextPlane();
        }
        else
        {
            PRINT_LOG(1, "All planes covered\n");
            PRINT_LOG(1, "Total number of jlinkage calls: " << _jlinkage_calls << "\n");
        }
    }
    else
    {
        augmentInfo();
        PRINT_LOG(1, "Adjusting quadcopter for next capture of the same plane\n");
        adjustForNextCapture();
        PRINT_LOG(1, "Completed plane no.: " << _node_completed_number_of_planes << "\n");
        PRINT_LOG(1, "Adjusted for next capture. Please click the 4 points on the DRONE CAMERA FEED\n");
    }
    if(_node_completed_number_of_planes == _node_number_of_planes)
    {
        assert(visited_plane_parameters.size() == visited_continuous_bounding_box_points.size());
        PRINT_LOG(1, "All planes are completed\n");
        string filename = "Complete_Plane_Info.txt";
        PRINT_LOG(1, "Writing info gathered to " << filename << "\n");
        for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
        {
            image_gui->WriteInfoToFile(visited_continuous_bounding_box_points[i], visited_plane_parameters[i], i+1, filename);
        }
        PRINT_LOG(4, print2dVector(visited_plane_parameters, "All Plane Parameters:\n", "matlab"));
        PRINT_LOG(4, print2dVector(visited_continuous_bounding_box_points, "All Plane BBP:\n", "matlab"));
        PRINT_LOG(4, print2dVector(visited_motion_points, "All motion points:\n", "matlab"));
        PRINT_LOG(1, "Writing visited motion points to file: motion_points.txt\n");
        reverse(visited_motion_points.begin(), visited_motion_points.end());
        write3DPointsToCSV(visited_motion_points, "motion_points.txt", " ", "goto ", 4);
        PRINT_LOG(1, "Time spent in total in capture mode is: " << _capture_mode_time << " seconds.\n");
        PRINT_LOG(1, "Time spent in total in traversal mode is: " << _traversal_mode_time << " seconds.\n");
        _capture_mode_time = 0.0;
        _traversal_mode_time = 0.0;
    }
    else
    {
        PRINT_LOG(1, "All planes are not completed\n");
    }
    PRINT_LOG(1, "Completed\n");

}

/**
 * @brief Adjust the quadcopter to capture next part of the same plane
 * @details
 */
void
ControlUINode::adjustForNextCapture()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    vector< vector<float> > test_plane_parameters;
    Point3f top_left = this_continuous_bounding_box_points[0];
    Point3f top_right = this_continuous_bounding_box_points[1];
    double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
                                        (top_right.y - top_left.y)*(top_right.y - top_left.y) +
                                        (top_right.z - top_left.z)*(top_right.z - top_left.z) ));
    PRINT_LOG(1, "Width of the plane is: " << width_of_3d_plane << "\n");
    // Direction of next plane
    if(_next_plane_dir == CLOCKWISE)
    {
        PRINT_LOG(1, "Next plane is CLOCKWISE wrt current plane\n");
        getCurrentPositionOfDrone();
        /*cout << "[ INFO] [adjustForNextCapture] Moving by width of plane by 4\n";
        moveRight(width_of_3d_plane/(double)4.0);*/
        if(_node_completed_number_of_planes != _node_number_of_planes-1)
        {
            PRINT_LOG(3, "Changing the yaw clockwise by " << (3*fabs(_next_plane_angle)/4.0) << "\n");
            rotateClockwise(3*fabs(_next_plane_angle)/4.0);
            PRINT_LOG(3, "Moving backwards by 0.5\n");
            moveBackward(0.5);
            PRINT_LOG(3, "Moving forwards by 0.5\n");
            moveForward(0.5);
        }
        else
        {
            PRINT_DEBUG(3, "Last plane to cover\n");
        }
        PRINT_LOG(3, "Estimating multiple planes -> call to JLinkage\n");
        doJLinkage();
        // Adding the currently seeing plane to find out if a new plane another than the
        // current one is visible by rotating the drone
        clear2dVector(test_plane_parameters);
        for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
        {
            test_plane_parameters.push_back(visited_plane_parameters[i]);
        }
        test_plane_parameters.push_back(this_plane_parameters);
        PRINT_LOG(5, print2dVector(visited_plane_parameters, "All planes visited completely"));
        PRINT_LOG(5, print2dVector(test_plane_parameters, "All planes visited completely including the current one"));
        if(_node_completed_number_of_planes == _node_number_of_planes-1)
        {
            PRINT_LOG(3, "Observe the plane without rotation\n");
            _is_plane_covered = isNewPlaneVisible(test_plane_parameters,
                                            jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
        }
        else
        {
            PRINT_LOG(3, "Observe the plane by rotation: " << _next_plane_dir << "\n");
            _is_plane_covered = isNewPlaneVisible(test_plane_parameters,
                                            jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, true, _next_plane_dir);
        }
        PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << "\n");
        if(_is_plane_covered && 
            (_node_completed_number_of_planes != _node_number_of_planes-1) )
        {
            PRINT_LOG(3, "Checking if the plane is covered\n");
            getCurrentPositionOfDrone();
            if(_actual_plane_index+1 < jlink_all_plane_parameters.size() && _actual_plane_index+1 >= 0)
            {
                float check_dist = 
                        getPointToPlaneDistance(jlink_all_plane_parameters[_actual_plane_index+1], 
                                                _node_current_pos_of_drone);
                PRINT_DEBUG(3, "Check_dist: " << check_dist << ", Max. Dist: " << _node_max_distance << "\n");
                if(check_dist > _node_max_distance+1.0)
                {
                    _is_plane_covered = false;
                }
            }
            else
            {
                PRINT_LOG(3, "Current plane is the right most plane visible\n");
            }
        }
        if(!_is_plane_covered)
        {
            PRINT_DEBUG(3, "Checking if right edge is within the frame to reach a conclusion\n");
            int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 2);
            if(move==0)
            {
                _is_plane_covered = true;
            }
            else
            {
                _is_plane_covered = false;
            }
            PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << ", Move: " << move << "\n");
        }
        PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << "\n");
        if(_is_plane_covered)
        {
            copyNecessaryInfo();
        }
        else
        {
            // _is_plane_covered = false;
            PRINT_LOG(3, "Restoring the yaw back and moving right by width_of_plane by 2\n");
            _is_big_plane = true;
            if(_node_completed_number_of_planes != _node_number_of_planes-1)
            {
                PRINT_DEBUG(3, "Restoring the yaw. Rotating CounterClockwise by " << 3*fabs(_next_plane_angle)/4.0 << "\n");
                rotateCounterClockwise(3*fabs(_next_plane_angle)/4.0);
            }
            else
            {
                PRINT_DEBUG(3, "Last plane to cover. Moving right\n");
            }
            PRINT_DEBUG(3, "Moving right by " << width_of_3d_plane/(double)2.0 << "\n");
            moveRight(width_of_3d_plane/(double)2.0);
        }
    }
    else if(_next_plane_dir == COUNTERCLOCKWISE)
    {
        PRINT_LOG(1, "Next plane is COUNTERCLOCKWISE wrt current plane\n");
        getCurrentPositionOfDrone();
        PRINT_LOG(3, "Moving the drone horizontally by " << width_of_3d_plane << "\n");
        moveRight(width_of_3d_plane);
        PRINT_LOG(3, "Changing the yaw counterclockwise by " << (_next_plane_angle/5.0) << "\n");
        getCurrentPositionOfDrone();
        designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]-(_next_plane_angle/5.0));
        moveDroneViaSetOfPoints(_interm_path);
        PRINT_LOG(3, "Calling Jlinkage\n");
        doJLinkage();
        clear2dVector(test_plane_parameters);
        for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
        {
            test_plane_parameters.push_back(visited_plane_parameters[i]);
        }
        test_plane_parameters.push_back(this_plane_parameters);
        PRINT_LOG(5, print2dVector(visited_plane_parameters, "All planes visited completely"));
        PRINT_LOG(5, print2dVector(test_plane_parameters, "All planes visited completely including the current one"));
        if(_node_completed_number_of_planes == _node_number_of_planes-1)
        {
            PRINT_DEBUG(3, "Observe the plane without rotation\n");
            _is_plane_covered = isNewPlaneVisible(test_plane_parameters,
                                            jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, false);
        }
        else
        {
            PRINT_DEBUG(3, "Observe the plane by rotation: " << _next_plane_dir << "\n");
            _is_plane_covered = isNewPlaneVisible(test_plane_parameters,
                                            jlink_all_plane_parameters, jlink_all_percentage_of_each_plane, true, _next_plane_dir);
        }
        PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << "\n");
        if(!_is_plane_covered)
        {
            PRINT_DEBUG(3, "Checking if the right edge is visible within heuristics\n");
            int move = checkVisibility(this_plane_parameters, this_continuous_bounding_box_points, 2);
            if(move==0)
            {
                _is_plane_covered = true;
            }
            else
            {
                _is_plane_covered = false;
            }
            PRINT_DEBUG(3, "Is plane covered: " << _is_plane_covered << ", Move: " << move << "\n");
        }
        if(_is_plane_covered)
        {
            copyNecessaryInfo();
        }
        else
        {
            // _is_plane_covered = false;
            _is_big_plane = true;
            getCurrentPositionOfDrone();
            PRINT_LOG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone"));
            PRINT_LOG(3, "The plane in inspection is a big one. Restoring the yaw back to complete this plane\n");
            designPathToChangeYaw(_node_current_pos_of_drone, _node_current_pos_of_drone[3]+(_next_plane_angle/5.0));
            moveDroneViaSetOfPoints(_interm_path);
        }
    }
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    _traversal_mode_time += (elapsedTime/1000.0);
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    if(_node_completed_number_of_planes == _node_number_of_planes)
    {
        PRINT_DEBUG(3, "VPP size: " << visited_plane_parameters.size() << "\n");
        assert(visited_plane_parameters.size() == visited_continuous_bounding_box_points.size());
        PRINT_DEBUG(5, print2dVector(visited_plane_parameters, "VPP"));
        PRINT_DEBUG(5, print2dVector(visited_continuous_bounding_box_points, "VCBB"));
        string filename = "Plane_Info.txt";
        PRINT_DEBUG(3, "Writing info gathered to " << filename << "\n");
        for (unsigned int i = 0; i < visited_plane_parameters.size(); ++i)
        {
            image_gui->WriteInfoToFile(visited_continuous_bounding_box_points[i], visited_plane_parameters[i], i+1, filename);
        }
        PRINT_LOG(5, print2dVector(visited_plane_parameters, "All Plane Parameters"));
        PRINT_LOG(5, print2dVector(visited_continuous_bounding_box_points, "All Plane BBP"));
        PRINT_LOG(5, print2dVector(visited_motion_points, "All motion points"));
        PRINT_LOG(1, "All planes are covered!\n");
    }
    if((_is_plane_covered) &&
            (_node_completed_number_of_planes != _node_number_of_planes))
    {
            PRINT_LOG(1, "Current plane is covered.\n");
            PRINT_LOG(1, "Completed no. of planes: " << _node_completed_number_of_planes << "\n");
            PRINT_LOG(1, "Total no. of planes: " << _node_number_of_planes << "\n");
            PRINT_DEBUG(3, print2dVector(visited_plane_parameters, "Visited plane parameters"));
            PRINT_LOG(1, "Aligning quadcopter to next plane\n");
            _is_adjusted = true;
            alignQuadcopterToNextPlane();
    }
    else
    {
        PRINT_LOG(1, "Completed plane no.: " << _node_completed_number_of_planes << "\n");
        PRINT_LOG(1, "Please click 4 points on the DRONE CAMERA FEED window\n");
    }
    PRINT_LOG(1, "Completed\n");
}

/**
 * @brief
 * @details To be implemented if alignQuadcopterToNextPlane() doesnot work as expected
 */
void
ControlUINode::alignQuadcopterToNextPlane()
{
    clock_t beginTime, endTime;
    double elapsedTime;
    beginTime = clock();
    PRINT_LOG(1, "Started\n");
    PRINT_LOG(1, "Completed no. of planes: " << _node_completed_number_of_planes
                    << ", Total number of planes: " << _node_number_of_planes << "\n");
    float angle;
    if(_next_plane_dir == COUNTERCLOCKWISE)
    {
        bool yaw_change = false;
        if(!_is_adjusted)
        {
            yaw_change = true;
            PRINT_LOG(1, "Not adjusted by width of plane\n");
            Point3f top_left = visited_continuous_bounding_box_points.back()[0];
            Point3f top_right = visited_continuous_bounding_box_points.back()[1];
            double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
                                                (top_right.y - top_left.y)*(top_right.y - top_left.y) +
                                                (top_right.z - top_left.z)*(top_right.z - top_left.z) ));
            PRINT_LOG(1, "Width of the plane is: " << width_of_3d_plane << "\n");
            PRINT_LOG(3, "Moving the drone horizontally by " << width_of_3d_plane << "\n");
            moveRight(width_of_3d_plane);
            float m = 1.2;
            if(_next_plane_angle >= 70.0)
            {
                do
                {
                    PRINT_DEBUG(1, "Not adjusted. Can't see a new plane\n");
                    moveRight(m);
                    doJLinkage();
                    m = m-0.2;
                }while(_sig_plane_index == -1);
            }
        }
        else
        {
            float m = 1.2;
            if(_next_plane_angle >= 70.0)
            {
                do
                {
                    PRINT_DEBUG(3, "Not adjusted. Can't see a new plane\n");
                    moveRight(m);
                    doJLinkage();
                    m = m-0.2;
                }while(_sig_plane_index == -1);
            }
        }
        _is_adjusted = false;
        doJLinkage();
        PRINT_LOG(3, "Next plane is counter clockwise to current plane\n");
        image_gui->setContinuousBoundingBoxPoints(jlink_all_continuous_bounding_box_points);
        image_gui->setSigPlaneBoundingBoxPoints(this_continuous_bounding_box_points);
        PRINT_DEBUG(3, "Rendering poly and sig plane\n");
        image_gui->setRender(false, false, true, true);
        image_gui->renderFrame();
        PRINT_DEBUG(3, "_sig_plane_index: " << _sig_plane_index << "\n");
        PRINT_DEBUG(3, "_actual_plane_index: " << _actual_plane_index << "\n");
        bool move_drone = false;
        /*bool new_plane_visible = isNewPlaneVisible(visited_plane_parameters, jlink_all_plane_parameters,
                    jlink_all_percentage_of_each_plane, true, _next_plane_dir);*/
        if(_sig_plane_index == -2)
        {
            PRINT_DEBUG(3, "Seems I can see only new planes\n");
            _actual_plane_index = 0;
            move_drone = true;
        }
        else if(_sig_plane_index == -1)
        {
            PRINT_DEBUG(3, "Seems I can't see a new plane\n");
            PRINT_DEBUG(3, "Moving more right by 1.2\n");
            float m = 1.2;
            do
            {
                PRINT_DEBUG(3, "Not adjusted. Can't see a new plane\n");
                moveRight(m);
                doJLinkage();
                m = m-0.2;
            }while(_sig_plane_index == -1);
        }
        else
        {
            PRINT_DEBUG(3, "Seems I can see both new and old planes\n");
            move_drone = true;
        }
        if(move_drone)
        {
            // bool aligned = (_sig_plane_index == 0)? true: false;
            bool aligned = false;
            double denom = 3.0;
            double initial_move = 0.8;
            PRINT_DEBUG(3, "Next plane angle: " << _next_plane_angle << "\n");
            PRINT_DEBUG(3, "Initial Move: " << initial_move << ", Denominator: " << denom << "\n");
            if(_next_plane_angle <= 50.0)
            {
                initial_move = 0.8;
            }
            else if(_next_plane_angle > 50.0 && _next_plane_angle <= 70.0)
            {
                initial_move = 1.0;
            }
            else if(_next_plane_angle > 70.0 && _next_plane_angle <= 90.0)
            {
                initial_move = 1.3;
                denom = 3.0;
            }
            else
            {
                initial_move = 1.6;
                denom = 2.0;
            }
            int rounds = 0;
            double angle_to_rotate;
            do
            {
                rounds++;
                if(rounds >= 6)
                {
                    angle_to_rotate = (double)fabs(angle);
                }
                else
                {
                    angle_to_rotate = _next_plane_angle/denom;
                }
                PRINT_DEBUG(3, "Round no.: " << rounds << ", Next plane angle: " << _next_plane_angle << "\n");
                PRINT_DEBUG(3, "Angle to rotate: " << angle_to_rotate << "\n");
                if(yaw_change)
                {
                    PRINT_DEBUG(3, "Changing yaw cc by: " << angle_to_rotate << "\n");
                    rotateCounterClockwise(angle_to_rotate);
                }
                yaw_change = true;
                doJLinkage();
                PRINT_DEBUG(3, "Linearly translating along X by " << initial_move << "\n");
                getCurrentPositionOfDrone();
                /*float point_distance = getPointToPlaneDistance(this_plane_parameters, _node_current_pos_of_drone);
                int move = (_fixed_distance >= point_distance) ? -1: 1;
                float step_distance = fabs(_fixed_distance - point_distance);
                cout << "[ DEBUG] [alignQuadcopterToNextPlane] Drone Distance: " << point_distance
                            << ", Fixed Distance: " << _fixed_distance << "\n";
                cout << "[ DEBUG] [alignQuadcopterToNextPlane] Move: " << move << ", Step Distance: " << step_distance << "\n";
                if(move == -1)
                {
                    cout << "[ DEBUG] [alignQuadcopterToNextPlane] Moving backwards\n";
                    moveBackward(step_distance);
                }
                else if(move == 1)
                {
                    cout << "[ DEBUG] [alignQuadcopterToNextPlane] Moving forwards\n";
                    moveForward(step_distance);
                }*/
                moveInDirection(this_plane_parameters, _node_current_pos_of_drone, this_sorted_3d_points);
                getCurrentPositionOfDrone();
                int move = (_node_current_pos_of_drone[2] >= _fixed_height) ? -1:1;
                float step_distance = fabs(_node_current_pos_of_drone[2] - _fixed_height);
                PRINT_DEBUG(3, "Drone Height: " << _node_current_pos_of_drone[2]
                            << ", Fixed Height: " << _fixed_height << "\n");
                PRINT_DEBUG(3, "Move: " << move << ", Step Distance: " << step_distance << "\n");
                if(move == -1)
                {
                    PRINT_DEBUG(3, "Moving down\n");
                    moveDown(step_distance);
                }
                else if(move == 1)
                {
                    PRINT_DEBUG(3, "Moving up\n");
                    moveUp(step_distance);
                }
                moveRight(initial_move);
                doJLinkage();
                Point3f normal_new_plane(this_plane_parameters[0],
                                            this_plane_parameters[1], this_plane_parameters[2]);
                PRINT_DEBUG(3, "Normal new plane: " << normal_new_plane << "\n");
                getCurrentPositionOfDrone();
                PRINT_DEBUG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone"));
                _node_dest_pos_of_drone.clear();
                _node_dest_pos_of_drone.push_back(0.0);
                _node_dest_pos_of_drone.push_back(1.0);
                _node_dest_pos_of_drone.push_back(0.0);
                _node_dest_pos_of_drone.push_back(0.0);
                convertWRTQuadcopterOrigin(_node_current_pos_of_drone, _node_dest_pos_of_drone, _node_ac_dest_pos_of_drone);
                PRINT_LOG(5, print1dVector(_node_current_pos_of_drone, "Current position of drone"));
                PRINT_DEBUG(5, print1dVector(_node_dest_pos_of_drone, "Dest position of drone"));
                PRINT_LOG(5, print1dVector(_node_ac_dest_pos_of_drone, "Actual dest position of drone"));
                Point3f pYAxis(_node_ac_dest_pos_of_drone[0], _node_ac_dest_pos_of_drone[1], _node_ac_dest_pos_of_drone[2]);
                Point3f pOrigin(_node_current_pos_of_drone[0], _node_current_pos_of_drone[1], _node_current_pos_of_drone[2]);
                Point3f projectedNormal(pYAxis-pOrigin);
                Point3f plane_params(normal_new_plane.x, normal_new_plane.y, 0.0);
                projectedNormal.z = 0.0;
                angle = findAngle(projectedNormal, plane_params);
                PRINT_DEBUG(3, "Normal new plane: " << plane_params << "\n");
                PRINT_DEBUG(3, "Normal of quadcopter: " << projectedNormal << "\n");
                PRINT_DEBUG(3, "Opposite Normal of quadcopter: " << (-1.0)*projectedNormal << "\n");
                PRINT_DEBUG(3, "Angle (radians): " << angle << "\n");
                angle = -angle*180/M_PI;
                //float angle_diff = fabs(fabs(angle) - fabs(_node_current_pos_of_drone[3]));
                PRINT_DEBUG(3, "angle: " << angle << "\n");
                /*cout << "[ DEBUG] [alignQuadcopterToNextPlane] drone_angle: " << _node_current_pos_of_drone[3] << "\n";
                cout << "[ DEBUG] [alignQuadcopterToNextPlane] angle_diff: " << angle_diff << "\n";*/
                if(fabs(angle) > 20.0)
                {
                    aligned = false;
                    if(fabs(angle) > 20.0 && fabs(angle) <= 30.0)
                    {
                        initial_move = initial_move - 0.1;
                    }
                    else if(fabs(angle) > 30.0 && fabs(angle) <= 50.0)
                    {
                        //initial_move = initial_move - 0.05;
                        initial_move = 0.7;
                    }
                    else if(fabs(angle) > 50.0 && fabs(angle) <= 70.0)
                    {
                        //initial_move = initial_move - 0.025;
                        initial_move = 0.8;
                    }
                    else
                    {
                        initial_move = 1.0;
                    }
                    PRINT_DEBUG(3, "Next Move distance: " << initial_move << "\n");
                }
                else
                {
                    aligned = true;
                }
                denom += 1.0;
            } while(!aligned);
        }
    }
    else if(_next_plane_dir == CLOCKWISE)
    {
        double denom = 3.0;
        if(!_is_adjusted)
        {
            /*cout << "[ INFO] [alignQuadcopterToNextPlane] Not adjusted by width of plane\n";
            Point3f top_left = visited_continuous_bounding_box_points.back()[0];
            Point3f top_right = visited_continuous_bounding_box_points.back()[1];
            double width_of_3d_plane = (double)fabs(sqrt( (top_right.x - top_left.x)*(top_right.x - top_left.x) +
                                                (top_right.y - top_left.y)*(top_right.y - top_left.y) +
                                                (top_right.z - top_left.z)*(top_right.z - top_left.z) ));
            cout << "[ INFO] [alignQuadcopterToNextPlane] Width of the plane is: " << width_of_3d_plane << "\n";
            cout << "[ INFO] [alignQuadcopterToNextPlane] Moving the drone horizontally by " << width_of_3d_plane/3.0 << "\n";
            moveRight(width_of_3d_plane/(double)4.0);*/
            PRINT_DEBUG(3, "Not adjusted. Can't see a new plane\n");
            PRINT_DEBUG(3, "Therefore, rotating clockwise by: " << 
                    3*fabs(_next_plane_angle)/(double)4.0 << "\n");
            rotateClockwise(3*fabs(_next_plane_angle)/(double)4.0);
            PRINT_LOG(3, "Moving backwards by 0.5\n");
            moveBackward(0.5);
            PRINT_LOG(3, "Moving forwards by 0.5\n");
            moveForward(0.5);
            while(_sig_plane_index == -1)
            {
                rotateClockwise(fabs(_next_plane_angle)/denom);
                doJLinkage();
                denom = denom + 1.0;
            }
        }
        else
        {
            while(_sig_plane_index == -1)
            {
                PRINT_DEBUG(3, "Adjusted. Can't see a new plane\n");
                rotateClockwise(fabs(_next_plane_angle)/denom);
                doJLinkage();
                denom = denom + 1.0;
            }
        }
        _is_adjusted = false;
        rotateClockwise(fabs(_next_plane_angle)/denom);
        denom = denom + 1.0;
        rotateClockwise(fabs(_next_plane_angle)/denom);
        doJLinkage();
        PRINT_LOG(3, "Next plane is clockwise to current plane\n");
        image_gui->setContinuousBoundingBoxPoints(jlink_all_continuous_bounding_box_points);
        image_gui->setSigPlaneBoundingBoxPoints(this_continuous_bounding_box_points);
        PRINT_DEBUG(3, "Rendering poly and sig plane\n");
        image_gui->setRender(false, false, true, true);
        image_gui->renderFrame();
        PRINT_DEBUG(3, "_sig_plane_index: " << _sig_plane_index << "\n");
        PRINT_DEBUG(3, "_actual_plane_index: " << _actual_plane_index << "\n");
        PRINT_DEBUG(3, "Moving backwards by 0.4\n");
        moveBackward(0.4);
        PRINT_DEBUG(3, "Moving forwards by 0.4\n");
        moveForward(0.4);
    }
    endTime = clock();
    elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
    _traversal_mode_time += (elapsedTime/1000.0);
    PRINT_DEBUG(3, "Aligning quadcopter to the new plane\n");
    alignQuadcopterToCurrentPlane();
    adjustLeftEdge();
    _stage_of_plane_observation = true;
    _node_main_directions.pop_front();
    _node_main_angles.pop_front();
    if(_node_main_angles.size() == 0)
    {
        _next_plane_dir = CLOCKWISE;
        _next_plane_angle = 0.0;
    }
    else
    {
        _next_plane_dir = _node_main_directions.front();
        _next_plane_angle = _node_main_angles.front();
    }
    PRINT_DEBUG(1, "Completed\n");
    PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    return ;
}

void
ControlUINode::testUtility(int test_no)
{
    clock_t beginTime, endTime;
    double elapsedTime;
    if(test_no == 0)
    {
        PRINT_DEBUG(1, "Print Current position of drone\n");
        getCurrentPositionOfDrone();
        PRINT_DEBUG(1, print1dVector(_node_current_pos_of_drone, "Current position of drone: ", ""));
    }
    else if(test_no == 1)
    {
        PRINT_DEBUG(1, "Perform Jlinkage\n");
        beginTime = clock();
        doJLinkage();
        endTime = clock();
        elapsedTime = double(endTime - beginTime) / (CLOCKS_PER_SEC/1000);
        PRINT_DEBUG(1, "Time taken for function is " << elapsedTime << " ms.\n");
    }
    else if(test_no == 2)
    {
        vector< vector<double> > path_points;
        copyDoubleVector(visited_motion_points, path_points);
        reverse(path_points.begin(), path_points.end());
        PRINT_LOG(1, "Size of points: " << path_points.size() << "\n");
        moveDroneViaSetOfPoints(path_points);
    }
    else if(test_no == 3)
    {
        _is_adjusted = false;
        alignQuadcopterToNextPlane();
    }
    else if(test_no == 4)
    {
        vector< vector<double> > path_points;
        getBackTheDrone(path_points);
        moveDroneViaSetOfPoints(path_points);
    }
    else if(test_no == 5)
    {}
    else if(test_no == 6)
    {}
    else if(test_no == 7)
    {}
    else if(test_no == 8)
    {}
    else if(test_no == 9)
    {}
    else
    {

    }
    PRINT_DEBUG(1, "Test Completed\n");
}
