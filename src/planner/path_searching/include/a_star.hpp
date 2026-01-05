#pragma once

#include <queue>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>

#ifndef A_STAR_HPP
#define A_STAR_HPP

namespace global_planner {
    using Point2d = Eigen::Vector2d;

    struct Node {
        Point2d point;
        double f;
        bool operator>(const Node& other) const
        {
            return f > other.f;
        }
    };

    struct PairHash {
        std::size_t operator()(const Point2d& p) const noexcept
        {
            return std::hash<long long>()(
                ((long long) p.x() << 32) ^ (long long) p.y());
        }
    };

    double heuristic(Point2d node1, Point2d node2);


    std::vector<Point2d> trace_back();
    class Astar
    {
    public:
        std::vector<Point2d> search(Point2d start,
            Point2d end,
            const std::vector<std::vector<int>>& global_map);

    private:
        std::vector<Point2d> neighbors;
        std::priority_queue<Node, std::vector<Node>, std::greater<>> open_list;
        std::set<Point2d> close_list;
        std::unordered_map<Point2d, Point2d, PairHash> path;
        std::unordered_map<Point2d, double, PairHash> g_score;
        std::vector<Point2d> trace_back(Point2d current_point,
            Point2d start_point);
    };
}// namespace global_planner
#endif//a_star