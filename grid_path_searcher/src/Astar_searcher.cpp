#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

//将前后左右上下26个点加入到neighborPtrSets列表
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
    *
    //完成该函数
    STEP 4: finish AstarPathFinder::AstarGetSucc yourself 
    please write your code below
    *
    *
    */
    //判断当前点是否为空
    if (currentPtr == nullptr)
    {
        cout << "Error : currentPtr is NULL" << endl;
        return;
    }

    //获取当前点的Index 及 coord
    Eigen::Vector3i currentIndex = currentPtr->index;
    Eigen::Vector3d currentCoord = currentPtr->coord;

    int nx, ny, nz;
    double dist;
    GridNodePtr tempPtr = nullptr;
    Eigen::Vector3d nCoord;

    for (int i = -1; i <= 1; i++)
    {
        for (int j = -1; j <= 1; j++)
        {
            for (int k = -1; k <= 1; k++)
            {
                //如果是当前点则跳过
                if (i == 0 && j == 0 && k == 0)
                    continue;
                
                //分别向前后左右上下26个扩展
                nx = currentIndex[0] + i;
                ny = currentIndex[1] + j;
                nz = currentIndex[2] + k;

                //判断是否越界
                if ((nx < 0) || (nx > GLX_SIZE - 1)
                    || (ny < 0) || (ny > GLY_SIZE - 1)
                    || (nz < 0) || (nz > GLZ_SIZE - 1))
                    continue;
                //查询该点是否被占据(即为障碍)
                if (isOccupied(nx, ny, nz))
                    continue;
                //获取当前点
                tempPtr = GridNodeMap[nx][ny][nz];
                //判断该点是否在closelits中
                if (tempPtr->id == -1)
                    continue;
                //判断当前扩展点是否和父节点一样
                if (tempPtr == currentPtr)
                {
                    cout << "Error : tempPtr == currentPtr" << endl;
                    continue;
                }
       
                nCoord = tempPtr->coord;


                if (abs(nCoord[0] - currentCoord[0]) < 1e-6 &&
                    abs(nCoord[1] - currentCoord[1]) < 1e-6 &&
                    abs(nCoord[2] - currentCoord[2]) < 1e-6)
                {
                    cout << "Error : not expanding correctly!" << endl;
                    cout << "nCoord : " << nCoord[0] << ", " << nCoord[1] << ", " << nCoord[2] << endl;
                    cout << "currentCoord : " << currentCoord[0] << ", " << currentCoord[1] << ", " << currentCoord[2] << endl;
                    cout << "neighbor node index :" << nx << ", " << ny << ", " << nz << endl;
                    cout << "current index :" << nx << ", " << ny << ", " << nz << endl;
                    continue;
                }

                //求出当前点到父节点的距离(直线距离)
                dist = sqrt( (nCoord[0] - currentCoord[0]) * (nCoord[0] - currentCoord[0]) +
                             (nCoord[1] - currentCoord[1]) * (nCoord[1] - currentCoord[1]) +
                             (nCoord[2] - currentCoord[2]) * (nCoord[2] - currentCoord[2]) );

                tempPtr->cameFrom = currentPtr;
                neighborPtrSets.push_back(tempPtr);
                edgeCostSets.push_back(dist);
            }
        }
    }
}

#define Manhattan 0
#define Euclidean 1
#define Diagonal  2
#define Octile  3

#define TieBreaker  0

//该函数主要求 f = g + h 中的 h (从该点移动到终点的估算成本)
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    选择您想要的可能的启发式函数
     曼哈顿，欧几里得，对角或0（Dijkstra）
     还记得在讲座中学习到的tie_breaker，在这里添加吗？
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    int Heuristic = Manhattan;
    double h = 0.0;
    
    if (Heuristic == Manhattan){
        //曼哈顿距离
        double dx = std::abs(node1->coord(0) - node2->coord(0));
        double dy = std::abs(node1->coord(1) - node2->coord(1));
        double dz = std::abs(node1->coord(2) - node2->coord(2));
        h = dx + dy + dz;
    }else if (Heuristic == Euclidean){
        //欧氏距离
        h = 1.4 * sqrt( (node1->coord(0) - node2->coord(0)) * (node1->coord(0) - node2->coord(0)) +
                    (node1->coord(1) - node2->coord(1)) * (node1->coord(1) - node2->coord(1)) +
                    (node1->coord(2) - node2->coord(2)) * (node1->coord(2) - node2->coord(2)) );
    }else if (Heuristic == Diagonal){

    }else if (Heuristic == Octile){
        //本质还是欧氏距离，二维可以降低复杂度，三维看来不降反增
        double dcoord[3] = {0};
        dcoord[0] = std::abs(node1->coord(0) - node2->coord(0));
        dcoord[1] = std::abs(node1->coord(1) - node2->coord(1));
        dcoord[2] = std::abs(node1->coord(2) - node2->coord(2));
        double k = sqrt(2) - 1;
        double l = sqrt(3) - sqrt(2);
        //从小到大排序
        sort(dcoord, dcoord+3);
        h = dcoord[2] + k * dcoord[1] + l * dcoord[0];
        h *= 1.5;
    }
    if (TieBreaker)
    {
        h = h * (1 + 1/25);
    }
    
    return h;
}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
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
    // currentPtr表示open_list中具有最低f(n)的节点
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //将开始节点放入openset中
    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
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
    
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    //将起始点的id设为1.即加入了openlist  -1为closelist
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]->id = 1;
    //定义一个当前点的数组下标
    Eigen::Vector3i currentIndex;

    //主循环
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

        //将openlist中的起始点设为当前扩展点
        currentPtr = openSet.begin()->second;
        //将其从openlist中删除
        openSet.erase(openSet.begin());
        currentIndex = currentPtr->index;
        //将当前点id设为-1即加入closelist
        GridNodeMap[currentIndex[0]][currentIndex[1]][currentIndex[2]]->id = -1;

        //如果当前节点是目标
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return;
        }
        //获得继承
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        /*
        *
        *
        步骤5：对于节点“ n”的所有未展开的邻域“ m”，请完成此循环
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            判断近邻是否已扩展
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            //获取当前点
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                步骤6：作为一个新的节点，做你需要做什么，然后把邻居加入到openlist幷计算其f
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                //f = g + h 
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);

                //加如openlist
                openSet.insert( make_pair(neighborPtr->fScore, neighborPtr));
                //其id设为1
                neighborPtr->id = 1;

                continue;
            }
            //该点已经在openlist了
            else if(neighborPtr->id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                步骤7：对于开放集中的节点，对其进行更新，维护开放集，然后将邻居放入开放集中并进行记录
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                //如果以前扩展的g 大于现在扩展的 g 则更新 f g h

                if (neighborPtr->gScore > (currentPtr->gScore + edgeCostSets[i]))
                {
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                    //更新父节点
                    neighborPtr->cameFrom = currentPtr;
                }
                
                continue;
            }else if(neighborPtr->id == -1){//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                //如果已经在 closelist中则忽略它,其实根本没有，因为在上面都不会将已经在closelist中的节点加入队列
                cout << "f : " << neighborPtr->fScore << endl;
                continue;
            }
        }
    }
    //如果搜索失败 
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    步骤8：从当前nodePtr追溯以获取路径上的所有节点
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    auto ptr = terminatePtr;

    while (ptr->cameFrom != nullptr)
    {
        //将当前点加入到 path中
        gridPath.push_back(ptr);
        //将当前点设为下一个
        ptr = ptr->cameFrom;
    }
    

    for (auto ptr: gridPath){
        path.push_back(ptr->coord);
        // cout << "coord :" << ptr->coord[0] << ", " << ptr->coord[1] << ", " << ptr->coord[2] << ", " << ptr->fScore << endl;
    }

    reverse(path.begin(),path.end());

    return path;
}