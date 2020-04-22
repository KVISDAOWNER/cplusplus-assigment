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
#include <set>


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

        std::vector<std::vector<StateType>> check(std::function<bool(StateType)> goal_predicate, search_order_t order = search_order_t::breadth_first) {
            typedef search_order_t search;
            StateType (*pop)    (std::vector<StateType>&) { (order == search::breadth_first)? pop_queue : pop_stack}; //TODO ref i stedet?

            std::map<StateType,std::vector<StateType>> parent_map = std::map<StateType,std::vector<StateType>>(); //for trace and check if visited
            std::set<StateType> seen = std::set<StateType>();

            auto goal_states = std::vector<StateType>();
            auto waiting = std::vector<StateType>();
            waiting.push_back(_start_state);

            StateType last_state = _start_state;
            while(!waiting.empty()){
                auto state = pop(waiting);
                if(goal_predicate(state))
                    goal_states.push_back(state);
                if(!seen.count(state)) { //if not seen before?
                    seen.insert(state);
                    for(const auto& n_state: _successor_fun(state)){
                        //if(!parent_map.count(state)) //check to not do duplicate parents
                        parent_map[n_state].push_back(state);
                        waiting.push_back(n_state);
                    }
                }
                else{
                    //crate std:vector //TODO maybe duplicate keys will be an issue with the other puzzles
                }
            }

            auto solutionTrace = std::vector<std::vector<StateType>>();
            //Make tracee
            auto trace = std::vector<StateType>();
            for (auto& state: goal_states) {
                trace.push_back(state);
                while(state != _start_state){
                    state = parent_map.find(state)->second[0]; //Here we guarantee that state is a key //TODO branch out when multiple parents
                    trace.push_back(state);
                }
                std::reverse(std::begin(trace), std::end(trace));
                solutionTrace.push_back(trace);
            }

            return solutionTrace;
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