#ifndef ARM_TASK_STATE_MACHINE_HPP
#define ARM_TASK_STATE_MACHINE_HPP

#include <map>
#include <functional>
#include <iostream>
#include "arm2_task/common_units.hpp"

namespace arm2_task {

class ArmStateMachine {
public:
    using StateAction = std::function<void()>;

    ArmStateMachine() : current_state_(TaskState::IDLE) {
        // 定义状态跳转表：{当前状态, 事件} -> 下一个状态
        transition_table_[{TaskState::IDLE,     ArmEvent::START_TASK}]     = TaskState::LOOKOUT;
        transition_table_[{TaskState::LOOKOUT,  ArmEvent::TARGET_SPOTTED}] = TaskState::OVERLOOK;
        transition_table_[{TaskState::OVERLOOK, ArmEvent::IN_POSITION}]    = TaskState::GRASPING;
        transition_table_[{TaskState::GRASPING, ArmEvent::ACTION_SUCCESS}] = TaskState::HOLDING;
        transition_table_[{TaskState::HOLDING,  ArmEvent::TARGET_SPOTTED}] = TaskState::PLACING;
        transition_table_[{TaskState::PLACING,  ArmEvent::ACTION_SUCCESS}] = TaskState::IDLE;
        
        // 故障处理
        transition_table_[{TaskState::GRASPING, ArmEvent::ACTION_FAILURE}] = TaskState::FAULT;
    }

    void handleEvent(ArmEvent event) {
        auto key = std::make_pair(current_state_, event);
        if (transition_table_.count(key)) {
            current_state_ = transition_table_[key];
            if (on_entry_actions_.count(current_state_)) {
                on_entry_actions_[current_state_]();
            }
        }
    }

    void registerOnEntry(TaskState state, StateAction action) {
        on_entry_actions_[state] = action;
    }

    TaskState getCurrentState() const { return current_state_; }

private:
    TaskState current_state_;
    std::map<std::pair<TaskState, ArmEvent>, TaskState> transition_table_;
    std::map<TaskState, StateAction> on_entry_actions_;
};

} // namespace arm2_task

#endif
