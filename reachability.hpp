//
// Created by kris271c on 15/04/2020.
//

#ifndef ASSIGNMENT_REACHABILITY_HPP
#define ASSIGNMENT_REACHABILITY_HPP

#include <functional>
#include <queue>
#include <iostream>
#include <stack>
#include <map>


/*namespace search_order_t{
    struct search_order_t{virtual ~search_order_t() = default;};
    struct breadth_first: search_order_t{};
    struct depth_first: search_order_t{};
};*/

enum class search_order_t{
    breadth_first, depth_first
};

template <typename StateType>
class state_space_t{
    public:
        state_space_t(StateType start_state, std::function<std::vector<StateType>(StateType&)> successors)
        : _start_state{start_state}, _successor_fun{successors}
        {
            _successors = _successor_fun(_start_state);
        }

        std::vector<StateType> check(std::function<bool(StateType)> goal_predicate, search_order_t order = search_order_t::breadth_first) {
            typedef search_order_t search;
            StateType (*pop)    (std::vector<StateType>&) { (order == search::breadth_first)? pop_queue : pop_stack}; //TODO ref i stedet?

            std::map<StateType,std::vector<StateType>> parent_map = std::map<StateType,std::vector<StateType>>(); //for trace and check if visited

            auto goal_states = std::vector<StateType>();
            auto waiting = std::vector<StateType>();
            waiting.push_back(_start_state);

            StateType last_state = _start_state;
            while(!waiting.empty()){
                auto state = pop(waiting);
                if(goal_predicate(state))
                    goal_states.push_back(state);
                if(!parent_map.count(state)) { //not seen before? //TODO
                    parent_map[state].push_back(last_state);
                    for(auto n_state: _successor_fun(state)){
                        waiting.push_back(n_state);
                    }
                }
                else{
                    //crate std:vector
                }
            }

            //Make trace nice
            

            return goal_states;
        }


private:
        StateType                                           _start_state;

        std::function<std::vector<StateType>(StateType&)>   _successor_fun;
        std::vector<StateType>                              _successors;

        static StateType pop_stack(std::vector<StateType>& s) { StateType v = s.back(); s.pop_back(); return v; } //TODO optimize? if(data.back()>-1) then back(); og ikke static?
        static StateType pop_queue(std::vector<StateType>& q) { StateType v = q.front(); q.erase( q.begin() ); return v; } //TODO optimize? og ikke static?
        //TODO fill out
};


template<typename StateType>
std::function<std::vector<StateType>(StateType&)> successors(std::function<std::vector<std::function<void(StateType&)>> (const StateType&)> transitions) {
    return [&transitions](StateType startState){
        auto states = std::vector<StateType>(); //results
        auto functions = transitions(startState);
        for (const auto& transition: functions) {
            StateType stateCpy = startState;
            transition(stateCpy);
            states.push_back(stateCpy);
        }
        return states;
    }; //TODO lav refs her for optimization
}


#endif //ASSIGNMENT_REACHABILITY_HPP