//
// Created by kris271c on 15/04/2020.
//

#ifndef ASSIGNMENT_REACHABILITY_HPP
#define ASSIGNMENT_REACHABILITY_HPP

#include <functional>
#include <queue>

enum class search_order_t{
    breadth_first, depth_first //TODO fill out
};


template <typename StateType>
class state_space_t{
    public:
        state_space_t(StateType start_state, std::function<std::vector<StateType>(StateType&)> successors)
                :_startState{start_state}, _successor_fun{successors}
                {
                    _successors = _successor_fun(_startState);
                }

        std::vector<StateType> check(std::function<bool(StateType)> goalPredicate) {
            auto res = _successor_fun(_startState);
            return std::vector<StateType>();
        }

private:
        StateType&                                          _startState;

        std::function<std::vector<StateType>(StateType&)>   _successor_fun;
        std::vector<StateType>                              _successors;

        //TODO fill out
};


template<typename StateType>
std::function<std::vector<StateType>(StateType&)> successors(std::function<std::vector<std::function<void(StateType&)>> (const StateType&)> transitions) {
    return [&transitions](StateType& startState){

        auto states = std::vector<StateType>(); //results
        auto frontier = std::queue<StateType>(); //frontier with states to transition from
        states.push_back(startState);
        frontier.push(startState);

        while(!frontier.empty()){
            auto curState = frontier.front();
            frontier.pop();
            auto functions = transitions(curState);
            for (const auto& transition: functions) {
                StateType stateCpy;
                stateCpy = curState;
                transition(stateCpy);
                frontier.push(stateCpy);
                states.push_back(stateCpy);
            }
        }
        return states;
    }; //TODO lav refs her for optimization
}


#endif //ASSIGNMENT_REACHABILITY_HPP