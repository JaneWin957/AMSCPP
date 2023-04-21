/***************************************************************************
 *   Copyright (C) 2006 - 2016 by                                          *
 *      Tarek Taha, KURI  <tataha@tarektaha.com>                           *
 *      Randa Almadhoun   <randa.almadhoun@kustar.ac.ae>                   *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
//#define _GLIBCXX_USE_CXX11_ABI 0
#include "sspp/pathplanner.h"
#include "ros/ros.h"
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <cscpp/occlusion_culling.h>

#include "cscpp/coverage_path_planning_heuristic.h"
#include "sspp/rviz_drawing_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
//#include <component_test/mesh_surface.h>
using namespace SSPP;

static Eigen::Vector3d toEulerAngle(const double x,const double y,const double z,const double w)
{
    Eigen::Vector3d euler;
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        euler[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler[1] = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    euler[2] = atan2(siny_cosp, cosy_cosp);
//    cout << "yaw = "<< euler[2] << endl ;
//    cout << "pitch = "<< euler[1] << endl ;
//    cout << "roll = "<< euler[0] << endl << endl;
    return euler;
}

Eigen::Vector3d QuaternionToEuler(const double x,const double y,const double z,const double w) // Z-Y-X Euler angles
{
    Eigen::Vector3d euler;
    const double Epsilon = 0.0009765625f;
    const double Threshold = 0.5f - Epsilon;

    double TEST = w*y - x*z;

    if (TEST < -Threshold || TEST > Threshold) // 奇异姿态,俯仰角为±90°
    {
        int sign = TEST>=0 ? 1 : -1;

        euler[2] = -2 * sign * (double)atan2(x, w); // yaw

        euler[1] = sign * (3.141592654 / 2.0); // pitch

        euler[0] = 0; // roll

    }
    else
    {
        euler[0] = atan2(2 * (y*z + w*x), w*w - x*x - y*y + z*z);
        euler[1] = asin(-2 * (x*z - w*y));
        euler[2] = atan2(2 * (x*y + w*z), w*w + x*x - y*y - z*z);
    }
    cout << "yaw = "<< euler[2] << endl ;
    cout << "pitch = "<< euler[1] << endl ;
    cout << "roll = "<< euler[0] << endl << endl;
    return euler;
}

Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)
{
    Eigen::Quaterniond q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "Quaterniond2Euler result is:" <<endl;
    cout << "x = "<< euler[2] << endl ;
    cout << "y = "<< euler[1] << endl ;
    cout << "z = "<< euler[0] << endl << endl;
    return euler;
}

int main( int argc, char **  argv)
{
    ros::init(argc, argv, "path_planning");
    ros::NodeHandle nh;

    ros::Publisher originalCloudPub  = nh.advertise<sensor_msgs::PointCloud2>("original_point_cloud", 100);
    ros::Publisher visiblePub        = nh.advertise<sensor_msgs::PointCloud2>("occlusion_free_cloud", 100);
    ros::Publisher pathPub           = nh.advertise<visualization_msgs::Marker>("generated_path", 10);
    ros::Publisher searchSpacePub    = nh.advertise<visualization_msgs::Marker>("search_space", 10);
    ros::Publisher connectionsPub    = nh.advertise<visualization_msgs::Marker>("connections", 10);
    ros::Publisher robotPosePub      = nh.advertise<geometry_msgs::PoseArray>("robot_pose", 10);
    ros::Publisher sensorPosePub     = nh.advertise<geometry_msgs::PoseArray>("sensor_pose", 10);
    ros::Publisher robotPoseSSPub    = nh.advertise<geometry_msgs::PoseArray>("SS_robot_pose", 10);
    ros::Publisher octomapPub        = nh.advertise<octomap_msgs::Octomap>("octomap", 1);


    
    std::vector<ros::Publisher> sensorsPoseSSPub;
    std::string modelName = std::string(ros::package::getPath("cscpp")+"/pcd/etihad_nowheels_nointernal_scaled_newdensed.pcd");
//    std::string modelName = std::string(ros::package::getPath("cscpp")+"/pcd/Vertices.pcd");
    //TODO: fix this
    std::string modelBaseName = "etihad_nowheels_nointernal_scaled_newdensed.pcd";
//    std::string modelBaseName = "testYun7.pcd";

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (modelName, *originalCloudPtr);

    pcl::PointCloud<pcl::PointXYZ>::Ptr coveredCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    OcclusionCulling occlusionCulling(nh,modelName);
    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("map","/cscpp_visualisation"));
    visualTools->deleteAllMarkers();
    visualTools->setLifetime(0.2);

    ros::Time timer_start = ros::Time::now();
    geometry_msgs::Pose gridStartPose;
    geometry_msgs::Vector3 gridSize;
    gridStartPose.position.x = -18 ;//-18
    gridStartPose.position.y = -25 ;//-25
    gridStartPose.position.z = -7.5 ;//1
    gridSize.x = 36;//36
    gridSize.y = 50;//50
    gridSize.z = 15;//15
//    gridStartPose.position.x = -36 ;//-18
//    gridStartPose.position.y = -50 ;//-25
//    gridStartPose.position.z = -15 ;//1
//    gridSize.x = 72;//36
//    gridSize.y = 100;//50
//    gridSize.z = 30;//15

    PathPlanner * pathPlanner;
    Pose start(3.0,-34.5,9,DTOR(0.0));
//    Pose start(3.0,-26.5,9,DTOR(0.0));
    Pose   end(1.5,-19.0,4.5,DTOR(0.0));

    double robotH=0.9,robotW=0.5,narrowestPath=0.987;//is not changed
    double distanceToGoal = 1.0,regGridConRad = 2.5;//2.5

    geometry_msgs::Point robotCenter;
    robotCenter.x = -0.3f;
    robotCenter.y = 0.0f;

    Robot *robot= new Robot("Robot",robotH,robotW,narrowestPath,robotCenter);
//    Sensors sensor1(58,45,0.255,0.7,6.0,640,480,Eigen::Vector3f(0,0.022,0.065), Eigen::Vector3f(0,-0.349,0));
    Sensors sensor2(58,45,0.255,0.7,6.0,640,480,Eigen::Vector3f(0,0.022,-0.065), Eigen::Vector3f(0,0.349,0));
//    Sensors sensor1(58,45,0.255,0.7,6.0,640,480,Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0.349,0));
//    Sensors sensor2(58,45,0.255,0.7,6.0,640,480,Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0.698,0));

    std::vector<Sensors> sensors;
//    sensors.push_back(sensor1);
    sensors.push_back(sensor2);

    // Every how many iterations to display the tree
    int progressDisplayFrequency = 1;
    pathPlanner = new PathPlanner(nh,robot,regGridConRad,progressDisplayFrequency,sensors);
    // This causes the planner to pause for the desired amount of time and display the search tree, useful for debugging
    pathPlanner->setDebugDelay(0);

    //choose the coverage heuristic you want and set the params and assign it to the path planner
    //******************************************************************
    double coverageTolerance=1.0, targetCov=92;
    bool gradualVisualization = true;
    std::string collisionCheckModelPath = ros::package::getPath("cscpp") + "/mesh/etihad_nowheels_nointernal_scaled_new.obj"; //bridge_translated, burj_arab_scaled
//    std::string collisionCheckModelPath = ros::package::getPath("cscpp") + "/mesh/Mesh.obj"; //bridge_translated, burj_arab_scaled

    CoveragePathPlanningHeuristic coveragePathPlanningHeuristic(nh,collisionCheckModelPath,modelName,false, gradualVisualization, InfoGainVolumetricH);
    coveragePathPlanningHeuristic.setCoverageTarget(targetCov);
    coveragePathPlanningHeuristic.setCoverageTolerance(coverageTolerance);
    pathPlanner->setHeuristicFucntion(&coveragePathPlanningHeuristic);

    //generate regular grid, filter and connect
    // ( to perform uniform sampling uncomment 1 or 2, to perform dynamic sampling uncomment part 3)
    //****************************************
    // 1- Generate Grid Samples (loading them from files, the generation is done using filtering.cpp node)
    //std::string str1 = ros::package::getPath("cscpp")+"/txt/SearchSpaceUAV_1.5m_1to4_etihad_nowheels_nointernal_scaled_newdensed.txt";//robot
    //std::string str2 = ros::package::getPath("cscpp")+"/txt/SearchSpaceCam_1.5m_1to4_etihad_nowheels_nointernal_scaled_newdensed_0.txt";//sensor1
    //std::string str3 = ros::package::getPath("cscpp")+"/txt/SearchSpaceCam_1.5m_1to4_etihad_nowheels_nointernal_scaled_newdensed_1.txt";//sensor2
    //const char * filename1 = str1.c_str();
    //const char * filename2 = str2.c_str();
    //const char * filename3 = str3.c_str();
    //pathPlanner->loadRegularGrid(filename1,filename2,filename3);
    //pathPlanner->connectNodes();


    // 2- Generate Grid Samples (generate grid and filter it if you want using the cscpp itself)
//    pathPlanner->generateRegularGrid(gridStartPose, gridSize,1.5,true,45,true);
//    pathPlanner->connectNodes();

    std::string searchSpaceFileName = ros::package::getPath("cscpp")+"/txt/"+"generated_search_space_"+modelBaseName+"_dsscpp.txt";
    // 3- Generate Grid Samples (dynamically with different resolution generate grid connect clusters and re-do this until the disc. resolution is 0)
    
    pathPlanner->dynamicNodesGenerationAndConnection(gridStartPose,gridSize,1.5,0.5);
    pathPlanner->connectNodes();
//    pathPlanner->dynamicNodesGenerationAndConnection(gridStartPose,gridSize,4.5,1.5);
//    pathPlanner->saveSearchSpace(searchSpaceFileName.c_str());
//    pathPlanner->loadSearchSpace(searchSpaceFileName.c_str());
    std::cout<<"\nSpace Generation took:"<<double(ros::Time::now().toSec() - timer_start.toSec())<<" secs"<<std::endl;

    //visualize and check the number of connections and search space
    //***************************************************************
    std::cout<<"\nGetting Connections";fflush(stdout);
    std::vector<geometry_msgs::Point> searchSpaceConnections = pathPlanner->getConnections();
    
    std::cout<<"\n Search Space Connections:"<<searchSpaceConnections.size();
    std::cout<<"\nDrawing SearchSpace";fflush(stdout);
    visualization_msgs::Marker connectionsMarker = drawLines(searchSpaceConnections,10000,3,100000000,0.03);
    std::vector<geometry_msgs::Point> searchSpaceNodes = pathPlanner->getSearchSpace();
    std::cout<<"\n\n---->>> Total Nodes in search Space ="<<searchSpaceNodes.size()<<std::endl;
    
    std::vector<geometry_msgs::PoseArray> sensorsPoseSS;
    geometry_msgs::PoseArray robotPoseSS;
    
    std::cout<<"\nGetting Robot Poses";fflush(stdout);
    pathPlanner->getRobotSensorPoses(robotPoseSS,sensorsPoseSS);
    std::cout<<"\nSetting MinMax";fflush(stdout);
    coveragePathPlanningHeuristic.setMaxMinSensorAccuracy(sensorsPoseSS);
    
    std::cout<<"\nVisualizing SearchSpace";fflush(stdout);
    visualization_msgs::Marker searchSpaceMarker = drawPoints(searchSpaceNodes,2,1000000);
    visualTools->publishSpheres(searchSpaceNodes,rviz_visual_tools::PURPLE,0.1,"search_space_nodes");

    //print search space
    //***************************************************************
    ofstream ssRobotFile,ssRobotFile1;


    std::string file_loc2 = ros::package::getPath("cscpp")+"/txt/"+"search_space_lkh_"+modelBaseName+"_dsscpp.txt";
    std::string file_loc1 = ros::package::getPath("cscpp")+"/txt/"+"search_space_robot_"+modelBaseName+"_dsscpp.txt";
    ssRobotFile.open (file_loc1.c_str());
    ssRobotFile1.open (file_loc2.c_str());

    for(int j = 0; j<robotPoseSS.poses.size(); j++)
    {
        double yaw;
        tf::Quaternion qt(robotPoseSS.poses[j].orientation.x, robotPoseSS.poses[j].orientation.y,robotPoseSS.poses[j].orientation.z,robotPoseSS.poses[j].orientation.w);
        yaw = tf::getYaw(qt);
        ssRobotFile << robotPoseSS.poses[j].position.x<<" "<<robotPoseSS.poses[j].position.y<<" "<<robotPoseSS.poses[j].position.z<<" "<<yaw<<"\n";
        ssRobotFile1 << j+1 <<" " <<robotPoseSS.poses[j].position.x<<" "<<robotPoseSS.poses[j].position.y<<" "<<robotPoseSS.poses[j].position.z<<"\n";
    }

    sensor_msgs::PointCloud2 cloud1;
    pcl::toROSMsg(*originalCloudPtr, cloud1); //cloud of original (white) using original cloud
    cloud1.header.stamp = ros::Time::now();
    cloud1.header.frame_id = "map"; //change according to the global frame please!!
    originalCloudPub.publish(cloud1);


    //find the path and print it
    //******************************************************************
    std::cout<<"\nStarting Search";fflush(stdout);
    ros::Time timer_restart = ros::Time::now();
    Node * path = pathPlanner->startSearch(start, end);
    ros::Time timer_end = ros::Time::now();
    std::cout<<"\nPath Finding took:"<<double(timer_end.toSec() - timer_restart.toSec())<<" secs";

    //path print and visualization
    if(path)
    {
        //uncomment if you want to print all the nodes of the generated path (I don't advise if the path is long)
        //pathPlanner->printNodeList();
    }
    else
    {
        std::cout<<"\nNo Path Found";
    }
    std::cout<<"\nPath Finding took:"<<double(timer_end.toSec() - timer_restart.toSec())<<" secs";

    //write to file and visualize the generated path and find the properties of the path
    //******************************************************************
    geometry_msgs::Point linePoint;
    std::vector<geometry_msgs::Point> pathSegments;
    geometry_msgs::PoseArray robotPose,sensorPose;
    double dist=0;
    double yaw, pitch;
    pcl::PointCloud<pcl::PointXYZ> temp_cloud, combined;

    //write to file
    ofstream pathFile;
    std::stringstream ss,cc;
    ss << targetCov;
    cc <<regGridConRad;
//    std::string file_loc = ros::package::getPath("cscpp")+"/txt/"+cc.str()+"_"+ss.str()+"%path_newtests1to4_"+modelBaseName+".csv";
    std::string file_loc = "/home/janewin/SIP_ws/src/StructuralInspectionPlanner-master/koptplanner/data/city240.csv";
    pathFile.open (file_loc.c_str());
    octomap::OcTree* oct;
    std::vector<double> accuracyPerViewpointAvg;
    double accuracySum = 0;
    int index = 0;
    while(path !=NULL)
    {
        index++;
        tf::Quaternion qt(path->pose.p.orientation.x,path->pose.p.orientation.y,path->pose.p.orientation.z,path->pose.p.orientation.w);
        yaw = tf::getYaw(qt);
//        std::cout<<"yaw: "<<yaw<<std::endl;
//        Eigen::Vector3d  ret = QuaternionToEuler(path->senPoses[0].p.orientation.x,path->senPoses[0].p.orientation.y,path->senPoses[0].p.orientation.z,path->senPoses[0].p.orientation.w);
        Eigen::Vector3d  ret = toEulerAngle(path->senPoses[0].p.orientation.x,path->senPoses[0].p.orientation.y,path->senPoses[0].p.orientation.z,path->senPoses[0].p.orientation.w);

//        pathFile << path->pose.p.position.x<<" "<<path->pose.p.position.y<<" "<<path->pose.p.position.z<<" "<<yaw<<"\n";
        pathFile << index<<","<<path->pose.p.position.x<<","<<path->pose.p.position.y<<","<<path->pose.p.position.z<<","<<yaw<<","<<ret[1]<<"\n";

        pcl::PointCloud<pcl::PointXYZ> temp;

        if (path->next !=NULL)
        {
            linePoint.x = path->pose.p.position.x;
            linePoint.y = path->pose.p.position.y;
            linePoint.z = path->pose.p.position.z;
            robotPose.poses.push_back(path->pose.p);
            for(int i =0; i<path->senPoses.size();i++)
            {
                sensorPose.poses.push_back(path->senPoses[i].p);
                temp_cloud=occlusionCulling.extractVisibleSurface(path->senPoses[i].p);
                combined += temp_cloud;
            }
            pathSegments.push_back(linePoint);


            linePoint.x = path->next->pose.p.position.x;
            linePoint.y = path->next->pose.p.position.y;
            linePoint.z = path->next->pose.p.position.z;
            robotPose.poses.push_back(path->next->pose.p);
            for(int i =0; i<path->next->senPoses.size();i++)
            {
                sensorPose.poses.push_back(path->next->senPoses[i].p);
                temp_cloud=occlusionCulling.extractVisibleSurface(path->next->senPoses[i].p);
                combined += temp_cloud;
                temp += temp_cloud;

                if(temp.points.size()!=0)
                {
                    double avgAcc = occlusionCulling.calcAvgAccuracy(temp_cloud,path->next->senPoses[i].p);
                    double a = (occlusionCulling.maxAccuracyError - avgAcc)/occlusionCulling.maxAccuracyError;
                    accuracyPerViewpointAvg.push_back(a);
                    accuracySum += avgAcc;
                    //std::cout<<"accuracy per viewpoints: "<<a<<" "<<avgAcc<<std::endl;
                }
            }


            pathSegments.push_back(linePoint);

            dist=dist+ Dist(path->next->pose.p,path->pose.p);
        }else{
            if(coveragePathPlanningHeuristic.getHeuristicType() == InfoGainVolumetricH)
                oct = new octomap::OcTree(*path->octree);
        }
        path = path->next;
    }
    pathFile.close();
    visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path");
    visualization_msgs::Marker pathMarker = drawLines(pathSegments,20000,1,10000000,0.1);
    coveredCloudPtr->points=combined.points;

    std::cout<<"\nDistance calculated from the path: "<<dist<<"m\n";
    std::cout<<"Covered Cloud % : "<<occlusionCulling.calcCoveragePercent(coveredCloudPtr)<<"%\n";
    std::cout<<"Average Accuracy per viewpoint is "<<accuracySum/accuracyPerViewpointAvg.size()<<std::endl;

    ros::Rate loopRate(10);
    for(int i = 0; i<sensorsPoseSS.size(); i++)
    {
        ros::Publisher  sensorPoseSSPub   = nh.advertise<geometry_msgs::PoseArray>("SS_sensor_pose_"+boost::lexical_cast<std::string>(i), 10);
        sensorsPoseSSPub.push_back(sensorPoseSSPub);
    }
    sensor_msgs::PointCloud2 cloud2;
    while (ros::ok())
    {
        /*
        visualTools->resetMarkerCounts();
        for(int i=0;i<robotPose.poses.size();i++)
        {
            visualTools->publishArrow(robotPose.poses[i],rviz_visual_tools::YELLOW, rviz_visual_tools::LARGE,0.3);
        }
        visualTools->publishSpheres(searchSpaceNodes,rviz_visual_tools::PURPLE,0.1,"search_space_nodes");
        visualTools->publishPath(searchSpaceConnections, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE,"search_space");
        visualTools->publishPath(pathSegments, rviz_visual_tools::RED, rviz_visual_tools::LARGE,"generated_path");
        */
        if( coveragePathPlanningHeuristic.getHeuristicType() == InfoGainVolumetricH)
        {
            octomap_msgs::Octomap octomap ;
            octomap.binary = 1 ;
            octomap.id = 1 ;
            octomap.resolution =0.25;
            octomap.header.frame_id = "map";
            octomap.header.stamp = ros::Time::now();
            bool res = octomap_msgs::fullMapToMsg(*oct, octomap);
            if(res)
            {
                octomapPub.publish(octomap);
            }
            else
            {
                ROS_WARN("OCT Map serialization failed!");
            }
        }

        pcl::toROSMsg(*originalCloudPtr, cloud1); //cloud of original (white) using original cloud
        cloud1.header.stamp = ros::Time::now();
        cloud1.header.frame_id = "map"; //change according to the global frame please!!
        originalCloudPub.publish(cloud1);

        pcl::toROSMsg(*coveredCloudPtr, cloud2); //cloud of original (white) using original cloud
        cloud2.header.stamp = ros::Time::now();
        cloud2.header.frame_id = "map"; //change according to the global frame please!!
        visiblePub.publish(cloud2);

        searchSpacePub.publish(searchSpaceMarker);
        connectionsPub.publish(connectionsMarker);
        pathPub.publish(pathMarker);

        robotPose.header.frame_id= "map";
        robotPose.header.stamp = ros::Time::now();
        robotPosePub.publish(robotPose);

        sensorPose.header.frame_id= "map";
        sensorPose.header.stamp = ros::Time::now();
        sensorPosePub.publish(sensorPose);

        robotPoseSS.header.frame_id= "map";
        robotPoseSS.header.stamp = ros::Time::now();
        robotPoseSSPub.publish(robotPoseSS);

        for(int i = 0 ; i<sensorsPoseSS.size(); i++)
        {
            sensorsPoseSS[i].header.frame_id= "map";
            sensorsPoseSS[i].header.stamp = ros::Time::now();
            sensorsPoseSSPub[i].publish(sensorsPoseSS[i]);
        }

        ros::spinOnce();
        loopRate.sleep();
    }
    delete robot;
    delete pathPlanner;
    return 0;
}
