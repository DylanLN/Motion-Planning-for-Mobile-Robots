#include "JPS_searcher.h"

using namespace std;
using namespace Eigen;

inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{
    neighborPtrSets.clear();
    edgeCostSets.clear();
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    int num_neib  = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);

    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        Vector3i neighborIdx;
        Vector3i expandDir;

        if( dev < num_neib ) {
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        else {
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            if( isOccupied(nx, ny, nz) ) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }

        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        nodePtr->dir = expandDir;
        
        neighborPtrSets.push_back(nodePtr);
        edgeCostSets.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}

//跳跃
bool JPSPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    neiIdx = curIdx + expDir;

    if( !isFree(neiIdx) )
        return false;

    if( neiIdx == goalIdx )
        return true;

    if( hasForced(neiIdx, expDir) )
        return true;

    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];

    for( int k = 0; k < num_neib - 1; ++k ){
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }

    return jump(neiIdx, expDir, neiIdx);
}

inline bool JPSPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    //检查三层的邻居
    switch(norm1){
        case 1:
            // 1-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        default:
            return false;
    }
}

inline bool JPSPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool JPSPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

void JPSPathFinder::JPSGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();    
    //起点和终点的索引
    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //起点和终点的位置
    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //初始化表示起始节点和目标节点的结构GridNode的指针
    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    // openSet是通过STL库中的multimap实现的open_list
    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //将开始节点放入开放集中
    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);
    //步骤1：完成AstarPathFinder::getHeu，这是启发式函数
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    步骤2：其他准备工作应在while循环之前完成
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    double tentative_gScore;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    //将起始节点的id设为1 加入openlist  （前面应该已经设置了）
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]->id = 1;
    //创建一个当前的下标用来索引
    Eigen::Vector3i currentIdx;
    //这是主循环
    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        步骤3：从开放集到封闭集删除具有最低成本函数的节点
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */

        //取出openlist中f最小的点，也就是最开始的点,将其加入closelist
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());
        currentIdx = currentPtr->index;

        GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]->id = 1;
        // currentPtr->id = -1;

        //如果当前节点是目标
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[JPS]{sucess} Time in JPS is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );    
            return;
        }
        //获得继承
        //get the succetion
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets); //we have done it for you
        
        /*
        *
        *
        步骤4：对于节点“ n”的所有未展开的邻域“ m”，请完成此循环
        STEP 4:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            判断邻居是否已扩大，请在下面编写您的代码
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : unexpanded
            neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
            *        
            */
            //获取当前节点
            neighborPtr = neighborPtrSets[i];
            //求出扩展节点的g
            tentative_gScore = currentPtr->gScore + edgeCostSets[i];
            if(neighborPtr -> id == 0){ //discover a new node
                /*
                *
                *
                步骤6：对于新节点，请执行所需的操作，然后将邻居放入开放集中并进行记录
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *
                */
                //如果未启发节点则求出其g 和 f，并将其母节点更新
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);
                neighborPtr->cameFrom = currentPtr;
                //更新扩展方向
                for (int i = 0; i < 3; i++)
                {
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if (neighborPtr->dir(i) != 0)
                    {
                        neighborPtr->dir(i) /= abs(neighborPtr->dir(i));
                    }
                    
                }
                openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr->id = 1;
                continue;
            }
            //处于openlist中且需要更新
            else if(neighborPtr->id == 1 && tentative_gScore <= neighborPtr-> gScore){ //in open set and need update
                /*
                *
                *
                步骤7：对于开放集中的节点，对其进行更新，维护该开放集，然后将邻居放入开放集中并进行记录
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                //如果当前的f小于以前的f则更新
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;

                //如果更改其父项，请更新扩展方向
                //这部分是关于JPS的，您在进行Astar工作时可以忽略它
                // if change its parents, update the expanding direction 
                //THIS PART IS ABOUT JPS, you can ignore it when you do your Astar work
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
            }
        }
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in JPS path finding is %f", (time_2 - time_1).toSec() );
}