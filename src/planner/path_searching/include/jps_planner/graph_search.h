/**
 * @file graph_search.h
 * @brief backend of graph search, implementation of A* and JPS
 */

#ifndef JPS_GRAPH_SEARCH_H
#define JPS_GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp>// boost::heap::d_ary_heap
#include <limits>                   // std::numeric_limits
#include <memory>                   // std::shared_ptr
#include <../../planner_map/include/planner_map.hpp>
#include <unordered_map>// std::unordered_map
#include <vector>       // std::vector

#include "../../../../tool/rm_log/include/rm_log.hpp"

namespace JPS {
    ///Heap element comparison
    template <class T>
    struct CompareState{
        bool operator()(T a1, T a2) const
        {
            double f1 = a1->g + a1->h;
            double f2 = a2->g + a2->h;
            if ((f1 >= f2 - 0.000001) && (f1 <= f2 + 0.000001)) {
                return a1->g < a2->g;// if equal compare gvals
            }
            return f1 > f2;
        }
    };


    ///Define priority queue
    struct State;// forward declaration
    
    ///State pointer
    using StatePtr = std::shared_ptr<State>;
    
    //优先队列
    using priorityQueue = boost::heap::d_ary_heap<
    StatePtr,                           // 元素类型：状态指针
    boost::heap::mutable_<true>,        // 启用可变性（可更新节点）
    boost::heap::arity<2>,              // 2叉堆（即二叉堆）
    boost::heap::compare<CompareState<StatePtr>>  // 自定义比较器
    >;

    ///Node of the graph in graph search
    struct State {
        /// ID
        int id;
        /// Coord
        int x, y, z = 0;
        /// direction
        int dx, dy, dz;// discrete coordinates of this node
        /// id of predicessors
        int parentId = -1;

        /// pointer to heap location
        priorityQueue::handle_type heapkey;

        /// g cost
        double g = std::numeric_limits<double>::infinity();
        /// heuristic cost
        double h;
        /// if has been opened
        bool opened = false;
        /// if has been closed
        bool closed = false;

        /// 2D constructor
        State(int id, int x, int y, int dx, int dy)
            : id(id)
            , x(x)
            , y(y)
            , dx(dx)
            , dy(dy)
        {}

        /// 3D constructor
        State(int id, int x, int y, int z, int dx, int dy, int dz)
            : id(id)
            , x(x)
            , y(y)
            , z(z)
            , dx(dx)
            , dy(dy)
            , dz(dz)
        {}
    };

    //💩  
    ///Search and prune neighbors for JPS 2D
    struct JPS2DNeib {
        // for each (dx,dy) these contain:
        //    ns: neighbors that are always added
        //    f1: forced neighbors to check
        //    f2: neighbors to add if f1 is forced
        /**
            第一维 [9]	0-8	9个可能的方向（包括(0,0)）
            第二维 [2]	0-1	方向符号（正或负）
            第三维 [8]/[2]	可变	邻居索引 
        */
        int ns[9][2][8];
        int f1[9][2][2];
        int f2[9][2][2];
        // nsz contains the number of neighbors for the four different types of moves:
        // no move (norm 0):        8 neighbors always added
        //                          0 forced neighbors to check (never happens)
        //                          0 neighbors to add if forced (never happens)
        // straight (norm 1):       1 neighbor always added
        //                          2 forced neighbors to check
        //                          2 neighbors to add if forced
        // diagonal (norm sqrt(2)): 3 neighbors always added
        //                          2 forced neighbors to check
        //                          2 neighbors to add if forced
        //- norm = 0: 初始节点（无方向）
        //- norm = 1: 直线移动 (dx,dy中一个为0)
        //- norm = √2: 对角线移动 (dx,dy都不为0)
        // 每种移动类型有多少个邻居
        static constexpr int nsz[3][2] = {{8, 0}, 
                                          {1, 2}, 
                                          {3, 2}};

        void print();
        JPS2DNeib();

    private:
        void Neib(int dx, int dy, int norm1, int dev, int& tx, int& ty);
        void FNeib(int dx,
            int dy,
            int norm1,
            int dev,
            int& fx,
            int& fy,
            int& nx,
            int& ny);
    };


    ///Search and prune neighbors for JPS 3D
    struct JPS3DNeib {
        // for each (dx,dy,dz) these contain:
        //    ns: neighbors that are always added
        //    f1: forced neighbors to check
        //    f2: neighbors to add if f1 is forced
        int ns[27][3][26];
        int f1[27][3][12];
        int f2[27][3][12];
        // nsz contains the number of neighbors for the four different types of moves:
        // no move (norm 0):        26 neighbors always added
        //                          0 forced neighbors to check (never happens)
        //                          0 neighbors to add if forced (never happens)
        // straight (norm 1):       1 neighbor always added
        //                          8 forced neighbors to check
        //                          8 neighbors to add if forced
        // diagonal (norm sqrt(2)): 3 neighbors always added
        //                          8 forced neighbors to check
        //                          12 neighbors to add if forced
        // diagonal (norm sqrt(3)): 7 neighbors always added
        //                          6 forced neighbors to check
        //                          12 neighbors to add if forced
        static constexpr int nsz[4][2] = {{26, 0}, {1, 8}, {3, 12}, {7, 12}};
        JPS3DNeib();

    private:
        void Neib(int dx,
            int dy,
            int dz,
            int norm1,
            int dev,
            int& tx,
            int& ty,
            int& tz);
        void FNeib(int dx,
            int dy,
            int dz,
            int norm1,
            int dev,
            int& fx,
            int& fy,
            int& fz,
            int& nx,
            int& ny,
            int& nz);
    };


    /**
   * @brief GraphSearch class
   *
   * Implement A* and Jump Point Search
   */
    class GraphSearch
    {
    public:
        //  /**
        //    * @brief 2D graph search constructor
        //    *
        //    * @param cMap 1D array stores the occupancy, with the order equal to \f$x + xDim * y\f$
        //    * @param xDim map length
        //    * @param yDim map width
        //    * @param eps weight of heuristic, optional, default as 1
        //    * @param verbose flag for printing debug info, optional, default as false
        //    */
        //   GraphSearch(const char* cMap, int xDim, int yDim, double eps = 1, bool verbose = false);
        //   /**
        //    * @brief 3D graph search constructor
        //    *
        //    * @param cMap 1D array stores the occupancy, with the order equal to \f$x + xDim * y + xDim * yDim * z\f$
        //    * @param xDim map length
        //    * @param yDim map width
        //    * @param zDim map height
        //    * @param eps weight of heuristic, optional, default as 1
        //    * @param verbose flag for printing debug info, optional, default as False
        //    */
        //   GraphSearch(const char* cMap, int xDim, int yDim, int zDim, double eps = 1, bool verbose = false);

        GraphSearch(std::shared_ptr<planner::map::Map> Map, const double& safe_dis);

        /**
       * @brief start 2D planning thread
       *
       * @param xStart start x coordinate
       * @param yStart start y coordinate
       * @param xGoal goal x coordinate
       * @param yGoal goal y coordinate
       * @param useJps if true, enable JPS search; else the planner is implementing A*
       * @param maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
       */
        bool plan(int xStart,
            int yStart,
            int xGoal,
            int yGoal,
            bool useJps,
            int maxExpand = -1);
        /**
       * @brief start 3D planning thread
       *
       * @param xStart start x coordinate
       * @param yStart start y coordinate
       * @param zStart start z coordinate
       * @param xGoal goal x coordinate
       * @param yGoal goal y coordinate
       * @param zGoal goal z coordinate
       * @param useJps if true, enable JPS search; else the planner is implementing A*
       * @param maxExpand maximum number of expansion allowed, optional, default is -1, means no limitation
       */
        bool plan(int xStart,
            int yStart,
            int zStart,
            int xGoal,
            int yGoal,
            int zGoal,
            bool useJps,
            int maxExpand = -1);
        
        /// Get the optimal path
        std::vector<StatePtr> getPath() const;

        /// Get the states in open set
        std::vector<StatePtr> getOpenSet() const;

        /// Get the states in close set
        std::vector<StatePtr> getCloseSet() const;

        /// Get the states in hash map
        std::vector<StatePtr> getAllSet() const;

        /// Set Safe Dis
        void SetSafeDis(const double& safe_dis);
        double GetSafeDis();

    private:
        /// Main planning loop
        /**
        @param: max_expand: 最大扩展节点数 
        */
        bool plan(StatePtr& currNode_ptr, int max_expand, int start_id, int goal_id);
        /// Get successor function for A*
        void getSucc(const StatePtr& curr,
            std::vector<int>& succ_ids,
            std::vector<double>& succ_costs);
        /// Get successor function for JPS
        void getJpsSucc(const StatePtr& curr,
            std::vector<int>& succ_ids,
            std::vector<double>& succ_costs);
        /// Recover the optimal path
        std::vector<StatePtr> recoverPath(StatePtr node, int id);

        /// Get subscript
        int coordToId(int x, int y) const;
        /// Get subscript
        // int coordToId(int x, int y, int z) const;

        /**  
        @brief: Check if (x, y) is free
        */
        bool isFree(int x, int y) const;
        /// Check if (x, y, z) is free
        // bool isFree(int x, int y, int z) const;

        /// Check if (x, y) is occupied
        bool isUnoccupied(int x, int y) const;

        /// Check if (x, y) is occupied
        bool isOccupied(int x, int y) const;
        /// Check if (x, y, z) is occupied
        // bool isOccupied(int x, int y, int z) const;

        /// Clculate heuristic
        /**
        @brief Calculate heuristic value 
        @param x x coordinate
        @param y y coordinate
        @return heuristic value
         */
        double getHeur(int x, int y) const;
        /// Clculate heuristic
        // double getHeur(int x, int y, int z) const;

        /// Determine if (x, y) has forced neighbor with direction (dx, dy)
        /**
        @brief: 当前节点在给定移动方向上是否存在强制邻居
        */
        bool hasForced(int x, int y, int dx, int dy);
        /// Determine if (x, y, z) has forced neighbor with direction (dx, dy, dz)
        // bool hasForced(int x, int y, int z, int dx, int dy, int dz);

        /// 2D jump, return true if finding the goal or a jump point
        /**
        @brief: 2d , 沿着给定方向跳跃直到找到跳点或障碍物
        */
        bool jump(int x, int y, int dx, int dy, int& new_x, int& new_y);

        /// 3D jump, return true if finding the goal or a jump point
        // bool jump(int x, int y, int z, int dx, int dy, int dz, int& new_x, int& new_y, int& new_z);

        /// Initialize 2D jps arrays
        void init2DJps();

        // const char* cMap_;
        std::shared_ptr<planner::map::Map> map_;
        int xDim_, yDim_, zDim_;
        // heuristic weight
        double eps_;
        //  控制是否输出详细日志信息的标志
        bool verbose_;

        double safe_dis_;

        const char val_free_ = 0;
        int xGoal_, yGoal_, zGoal_;
        // 是否使用2d搜索
        bool use_2d_;
        // 是否使用jps
        bool use_jps_ = false;
        
        // 优先队列，存储待扩展的节点，按照f值（g+h）排序
        priorityQueue pq_;

        // 将节点ID直接作为数组索引(伪哈希表)
        std::vector<StatePtr> hm_;
        
        //标记节点是否已被扩展（Closed Set）
        std::vector<bool> seen_;
        
        // 最终路径
        std::vector<StatePtr> path_;

        std::vector<std::vector<int>> ns_;
        std::shared_ptr<JPS2DNeib> jn2d_;
        std::shared_ptr<JPS3DNeib> jn3d_;
    };
}// namespace JPS
#endif
