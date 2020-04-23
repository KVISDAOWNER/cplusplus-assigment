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


enum class search_order_t{
    breadth_first, depth_first
};

template <typename StateType>
class state_space_t{
    public:
        state_space_t(StateType start_state, std::function<std::vector<StateType>(StateType&)> successors)
        : _start_state{start_state}, _successor_fun{successors}{}

        std::vector<std::vector<StateType>> check(std::function<bool(StateType)> goal_predicate, search_order_t order = search_order_t::breadth_first) {
            typedef search_order_t search;
            StateType (*pop)    (std::vector<StateType>&) { (order == search::breadth_first)? pop_queue : pop_stack}; //TODO ref i stedet?

            std::map<StateType,std::vector<StateType>> parent_map = std::map<StateType,std::vector<StateType>>(); //for trace and check if visited
            std::set<StateType> seen = std::set<StateType>();
            auto goal_states = std::vector<StateType>();
            auto goal_state_found = false;
            auto waiting = std::vector<StateType>();
            waiting.push_back(_start_state);

            while(!waiting.empty() && !goal_state_found){
                auto state = pop(waiting);
                if(goal_predicate(state)){
                    goal_state_found = true;
                    goal_states.push_back(state);
                }
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

        static StateType pop_stack(std::vector<StateType>& s) { StateType v = s.back(); s.pop_back(); return v; } //TODO optimize? if(data.back()>-1) then back(); og ikke static?
        static StateType pop_queue(std::vector<StateType>& q) { StateType v = q.front(); q.erase( q.begin() ); return v; } //TODO optimize? og ikke static?
        //TODO fill out
};


template <typename T, typename = void> //Not Container //TODO Why void here?
struct is_container: std::false_type {};

template <typename C>
struct is_container<C,
        std::void_t<
                typename C::iterator,
                typename C::const_iterator,
                decltype(std::begin(std::declval<C&>())),
                decltype(std::end(std::declval<C&>()))
        >
>: std::true_type {};

template<typename Container>
constexpr auto is_container_v = is_container<Container>::value;

template<typename StateType, typename Container>
typename std::enable_if<is_container_v<Container>, std::function<std::vector<StateType>(StateType&)>>::type
successors(Container transitions (const StateType&)) {
    return [transitions](const StateType& startState){ //TODO hvorfor kan den ikke tage &transitions?
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