#ifndef _NODE_H_
#define _NODE_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include "backward.hpp"

#define inf 1>>20
struct GridNode;
typedef GridNode* GridNodePtr;

struct GridNode
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord;  //真实机器人的位置单位（m）
    Eigen::Vector3i dir;   // direction of expanding扩张方向
    Eigen::Vector3i index;  //在图中的下标
	
    double gScore, fScore;  // f(n) = g(n) + h(n)
    GridNodePtr cameFrom;   //指向父节点
    //multimap  主要是用其机制来排序
    std::multimap<double, GridNodePtr>::iterator nodeMapIt;

    //初始化 构造
    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
		id = 0;
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();

		gScore = inf;
		fScore = inf;
		cameFrom = nullptr;
    }

    GridNode(){};
    ~GridNode(){};
};


#endif
