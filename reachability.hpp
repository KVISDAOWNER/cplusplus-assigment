//
// Created by kris271c on 15/04/2020.
//

#ifndef ASSIGNMENT_REACHABILITY_HPP
#define ASSIGNMENT_REACHABILITY_HPP

#include <functional>
#include <queue>
#include <iostream>
#include <stack>



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

        template<search_order_t o>
        struct container;

        template<>
        struct container <search_order_t::breadth_first> {
            std::queue<StateType> get() {
                return std::queue<StateType>();
            }
        };

        template<>
        struct container <search_order_t::depth_first> {
            std::stack<StateType> get() {
                return std::stack<StateType>();
            }
        };

        //static constexpr search_order_t test = search_order_t::depth_first;
        //static constexpr search_order_t test2 = search_order_t::depth_first;
        std::vector<StateType> check(std::function<bool(StateType)> goal_predicate, search_order_t order = search_order_t::breadth_first) {
            auto goal_states = std::vector<StateType>();


            auto waiting = container<order>().get();
            waiting.push(_start_state);

            auto seen = std::vector<StateType>();

            while(!waiting.empty()){
                auto state = pop(waiting);

                if(goal_predicate(state)){
                    goal_states.push_back(state);
                }

                for(auto n_state: _successor_fun(state)){
                    if(true) //not seen before? //TODO
                    {
                        waiting.push(n_state);
                    }
                }

            }
            return goal_states;
        }


private:
        StateType                                           _start_state;

        std::function<std::vector<StateType>(StateType&)>   _successor_fun;
        std::vector<StateType>                              _successors;


        template <typename T>
        T pop(std::stack<T>& s) { T v = s.top; s.pop(); return v; } //TODO optimize?

        template <typename T>
        T pop(std::queue<T>& q) { T v = q.front(); q.pop(); return v; } //TODO optimize?
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