#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include "graph_searcher.h"
#include "backward.hpp"

using namespace std;
using namespace Eigen;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace backward {
backward::SignalHandling sh;
}

//来自启动文件的模拟参数
// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;    

//有用的全局变量
// useful global variables
bool _has_map   = false;

Vector3d _start_pt;
Vector3d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

// ROS相关
// ros related
ros::Subscriber _map_sub, _pts_sub;
ros::Publisher  _grid_map_vis_pub, _RRTstar_path_vis_pub;

RRTstarPreparatory * _RRTstar_preparatory     = new RRTstarPreparatory();

void rcvWaypointsCallback(const nav_msgs::Path & wp);
void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map);
void pathFinding(const Vector3d start_pt, const Vector3d target_pt);
void visRRTstarPath(vector<Vector3d> nodes );

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
        return;

    Vector3d target_pt;
    target_pt << wp.poses[0].pose.position.x,
                 wp.poses[0].pose.position.y,
                 wp.poses[0].pose.position.z;

    ROS_INFO("[node] receive the planning target");
    pathFinding(_start_pt, target_pt); 
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    if(_has_map ) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    sensor_msgs::PointCloud2 map_vis;

    pcl::fromROSMsg(pointcloud_map, cloud);
    
    if( (int)cloud.points.size() == 0 ) return;

    pcl::PointXYZ pt;
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {    
        pt = cloud.points[idx];        

        //将障碍物设置到网格图中以进行路径规划
        // set obstalces into grid map for path planning
        _RRTstar_preparatory->setObs(pt.x, pt.y, pt.z);

        //仅用于可视化
        // for visualize only
        Vector3d cor_round = _RRTstar_preparatory->coordRounding(Vector3d(pt.x, pt.y, pt.z));
        pt.x = cor_round(0);
        pt.y = cor_round(1);
        pt.z = cor_round(2);
        cloud_vis.points.push_back(pt);
    }

    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "/world";
    _grid_map_vis_pub.publish(map_vis);

    _has_map = true;
}

//我们的碰撞检查器。 对于此演示，我们的机器人的状态空间
// Our collision checker. For this demo, our robot's state space
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}
    //返回给定状态的位置是否与
    //圆形障碍物
    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ob::State* state) const
    {   
        //我们知道我们正在使用RealVectorStateSpace
        //示例，因此我们将状态转换为特定类型。
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ob::RealVectorStateSpace::StateType* state3D =
            state->as<ob::RealVectorStateSpace::StateType>();
        /**
        *
        *
        步骤1：从状态中提取机器人的（x，y，z）位置
        STEP 1: Extract the robot's (x,y,z) position from its state
        *
        *
        */

        //double
        auto x = (*state3D)[0];
        auto y = (*state3D)[1];
        auto z = (*state3D)[2];
        
        return _RRTstar_preparatory->isObsFree(x, y, z);
    }
};

// Returns a structure representing the optimization objective to use
// for optimal motion planning. This method returns an objective which
// attempts to minimize the length in configuration space of computed
// paths.
//返回表示用于优化运动计划的优化目标的结构。
//此方法返回一个目标，该目标试图最小化计算路径的配置空间中的长度。
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(1.51));
    return obj;
}

void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
{
    //构造我们计划中的机器人状态空间。
    // Construct the robot state space in which we're planning. 
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(3));

    // 将空间的边界设置为[0,1]。
    // Set the bounds of space to be in [0,1].
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, - _x_size * 0.5);
    bounds.setLow(1, - _y_size * 0.5);
    bounds.setLow(2, 0.0);

    bounds.setHigh(0, + _x_size * 0.5);
    bounds.setHigh(1, + _y_size * 0.5);
    bounds.setHigh(2, _z_size);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    //为此状态空间构造一个空间信息实例
    // Construct a space information instance for this state space
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    //设置用于检查空间中哪些状态有效的对象
    // Set the object used to check which states in the space are valid
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();

    //设置机器人的启动状态
    // Set our robot's starting state
    ob::ScopedState<> start(space);
    /**
    *
    *
    步骤2：完成启动状态的初始化
    STEP 2: Finish the initialization of start state
    *
    *
    */
    start[0] = (&start_pt)->operator[](0);
    start[1] = (&start_pt)->operator[](1);
    start[2] = (&start_pt)->operator[](2);

    //设置机器人的目标状态
    // Set our robot's goal state
    ob::ScopedState<> goal(space);
    /**
    *
    *
    步骤3：完成目标状态的初始化
    STEP 3: Finish the initialization of goal state
    *
    *
    */
    goal[0] = (&target_pt)->operator[](0);
    goal[1] = (&target_pt)->operator[](1);
    goal[2] = (&target_pt)->operator[](2);



    //创建问题实例
    // Create a problem instance

    /**
    *
    *
    步骤4：创建问题实例，请将变量定义为pdef
    STEP 4: Create a problem instance, 
    please define variable as pdef
    *
    *
    */
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    //设置开始状态和目标状态
    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    //设置优化目标
    // Set the optimization objective
    /**
    *
    *
    步骤5：设置优化目标，您可以选择的选项已在前面定义：
    STEP 5: Set the optimization objective, the options you can choose are defined earlier:
    getPathLengthObjective() and getThresholdPathLengthObj()
    *
    *
    */
    pdef->setOptimizationObjective(getPathLengthObjective(si));

    //使用RRTstar算法构造优化计划器。
    // Construct our optimizing planner using the RRTstar algorithm.
    /**
    *
    *
    步骤6：使用RRT星型算法构建我们的优化规划师，请将变量定义为优化规划师
    STEP 6: Construct our optimizing planner using the RRTstar algorithm, 
    please define varible as optimizingPlanner
    *
    *
    */
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));


    //设置问题实例供我们的计划人员解决
    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    //尝试在计划时间的一秒钟内解决计划问题
    // attempt to solve the planning problem within one second of
    // planning time
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

    if (solved)
    {
        //从问题定义中获取目标表示形式（与目标状态不同），并查询找到的路径
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
        
        vector<Vector3d> path_points;

        for (size_t path_idx = 0; path_idx < path->getStateCount (); path_idx++)
        {
            const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>(); 
            /**
            *
            *
            步骤7：将找到的路径从路径转换为路径点以显示rviz
            STEP 7: Trandform the found path from path to path_points for rviz display
            *
            *
            */ 
            double x = (*state)[0];
            double y = (*state)[1];
            double z = (*state)[2];
            Vector3d tempMat(x,y,z);
            path_points.push_back(tempMat);
        }
        visRRTstarPath(path_points);       
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _map_sub  = nh.subscribe( "map",       1, rcvPointCloudCallBack );
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );

    _grid_map_vis_pub             = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);
    _RRTstar_path_vis_pub         = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis",1);


    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    nh.param("map/resolution",    _resolution,   0.2);
    
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);

    _map_lower << - _x_size/2.0, - _y_size/2.0,     0.0;
    _map_upper << + _x_size/2.0, + _y_size/2.0, _z_size;
    
    _inv_resolution = 1.0 / _resolution;
    
    _max_x_id = (int)(_x_size * _inv_resolution);
    _max_y_id = (int)(_y_size * _inv_resolution);
    _max_z_id = (int)(_z_size * _inv_resolution);

    _RRTstar_preparatory  = new RRTstarPreparatory();
    _RRTstar_preparatory  -> initGridMap(_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id, _max_z_id);
    
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status) 
    {
        ros::spinOnce();      
        status = ros::ok();
        rate.sleep();
    }

    delete _RRTstar_preparatory;
    return 0;
}

void visRRTstarPath(vector<Vector3d> nodes )
{
    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = Line.header.frame_id = "world";
    Points.header.stamp    = Line.header.stamp    = ros::Time::now();
    Points.ns              = Line.ns              = "demo_node/RRTstarPath";
    Points.action          = Line.action          = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id   = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = _resolution/2; 
    Points.scale.y = _resolution/2;
    Line.scale.x   = _resolution/2;

    //点是绿色，线带是蓝色
    //points are green and Line Strip is blue
    Points.color.g = 1.0f;
    Points.color.a = 1.0;
    Line.color.b   = 1.0;
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
}