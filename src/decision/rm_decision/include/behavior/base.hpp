#pragma once 

#ifndef RM_DECISION_BASE_HPP
#define RM_DECISION_BASE_HPP

#include "behaviortree_cpp/behavior_tree.h"

namespace decision{

    class IsGoHome: public BT::ConditionNode{
        public:
            IsGoHome();
            BT::NodeStatus tick() override;
            
    };
}

#endif // RM_DECISION_BASE_HPP