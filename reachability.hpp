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
#include <memory>

enum class search_order_t{
    breadth_first, depth_first
};

//https://stackoverflow.com/questions/15843525/how-do-you-insert-the-value-in-a-sorted-vector
template< typename T, typename Func_T >
typename std::vector<T>::iterator
push( std::vector<T> & vec, T const& item, Func_T cmp)
{
    return vec.insert
            (
                    std::upper_bound( vec.begin(), vec.end(), item, cmp ),
                    item
            );
}

struct default_init_cost{
    size_t depth{0}; // counts the number of transitions
    size_t noise{0}; // kids get bored on shore1 and start making noise there
    bool operator<(const default_init_cost& other) const {
        if (depth < other.depth)
            return true;
        if (other.depth < depth)
            return false;
        return noise < other.noise;
    }
};

template <typename StateType, typename cost_t>
struct node{
    node(StateType s, cost_t c): state{s}, cost{c}{}
    StateType state;
    cost_t cost;
};

template <typename StateType, typename cost_t = default_init_cost, typename cost_fun_t = std::function<cost_t(const StateType&, const cost_t&)>>
class state_space_t{
    public:
        state_space_t() = delete; //should never be instantiated like this
        ~state_space_t() = default; //rule of zero - class doesn't directly manage any resources

        state_space_t(
                StateType start_state,
                std::function<std::vector<StateType>(StateType&)> successors, //TODO better default?
                bool (*is_valid) (const StateType&) = [](const StateType&){return true;})
                : _start_state{start_state}, _initial_cost{default_init_cost{}}, _successor_fun{successors}, _is_valid{is_valid}, _cost_fun()
                {   _cost_fun = [](const StateType&, const cost_t&){return cost_t{};};    }

        state_space_t(
                StateType start_state,
                cost_t initial_cost,
                std::function<std::vector<StateType>(StateType&)> successors = nullptr, //TODO better default?
                bool (*is_valid) (const StateType&) = [](const StateType&){return true;},
                cost_fun_t&& cost_fun = [](const cost_t& prev_cost){return cost_t{prev_cost.depth+1, prev_cost.noise};})
                : _start_state{start_state}, _initial_cost{initial_cost}, _successor_fun{successors}, _is_valid{is_valid}, _cost_fun{cost_fun}{}

        std::vector<std::vector<std::shared_ptr<StateType>>> check(std::function<bool(StateType)> goal_predicate, search_order_t order = search_order_t::breadth_first) {
            typedef search_order_t search;
            node<StateType, cost_t> (*pop)    (std::vector<node<StateType, cost_t>>&) { (order == search::breadth_first)? pop_queue : pop_stack}; //TODO ref i stedet?

            std::map<node<StateType, cost_t>,std::vector<node<StateType, cost_t>>, decltype(cmp_s)> parent_map(cmp_s); //for trace and check if visited
            std::set<StateType> seen = std::set<StateType>();
            auto goal_states = std::vector<node<StateType, cost_t>>();
            auto goal_state_found = false;
            auto waiting = std::vector<node<StateType, cost_t>>();
            if(_is_valid(_start_state))
                push(waiting, node(_start_state, _initial_cost), cmp_c);

            while(!waiting.empty() && !goal_state_found){
                auto state = pop(waiting);
                int size = waiting.size();
                if(goal_predicate(state.state)){
                    goal_state_found = true;
                    goal_states.push_back(state);
                }
                for(const auto& n_state: _successor_fun(state.state)){
                    if(!seen.count(n_state) && _is_valid(n_state)) { //if not seen before?
                        seen.insert(n_state);
                        auto n_node = node(n_state, _cost_fun(state.state, state.cost)); //TODO do the null check smarter or give default func smart way?
                        push(waiting, n_node, cmp_c);
                        parent_map[n_node].push_back(state);
                    //if(!parent_map.count(state)) //check to not do duplicate parents
                    }
                    //else
               }
                //else{
                    //crate std:vector //TODO maybe duplicate keys will be an issue with the other puzzles
                //}
            }

            //Make tracee
            auto solutionTrace = std::vector<std::vector<std::shared_ptr<StateType>>>();
            auto trace = std::vector<std::shared_ptr<StateType>>(); //TODO unique ptrs i stedet i denne funktion? share_ptr forid vi copier nogle gange, kan det undgåes?
            for (auto& node: goal_states) {
                trace.push_back(std::make_shared<StateType>(node.state));
                while(node.state != _start_state){
                    node = parent_map.find(node)->second[0]; //Here we guarantee that state is a key //TODO branch out when multiple parents?
                    trace.push_back(std::make_shared<StateType>(node.state));
                }
                std::reverse(std::begin(trace), std::end(trace));
                solutionTrace.push_back(trace);
            }

            return solutionTrace;
        }


    private:
        StateType                                               _start_state;
        cost_t                                                  _initial_cost;
        std::function<std::vector<StateType>(StateType&)>       _successor_fun;
        std::function<cost_t (const StateType&, const cost_t&)> _cost_fun;
        bool                                                    (*_is_valid) (const StateType&);
        std::function<bool (const node<StateType, cost_t>& a, const node<StateType, cost_t>& b)> cmp_c = [](const node<StateType, cost_t>& a, const node<StateType, cost_t>& b) { return  a.cost < b.cost; };
        std::function<bool (const node<StateType, cost_t>& a, const node<StateType, cost_t>& b)> cmp_s = [](const node<StateType, cost_t>& a, const node<StateType, cost_t>& b) { return  a.state < b.state; };

        static node<StateType, cost_t> pop_stack(std::vector<node<StateType, cost_t>>& s) { node<StateType, cost_t> v = s.back(); s.pop_back(); return v; } //TODO optimize? if(data.back()>-1) then back(); og ikke static?
        static node<StateType, cost_t> pop_queue(std::vector<node<StateType, cost_t>>& q) { node<StateType, cost_t> v = q.front(); q.erase( q.begin() ); return v; } //TODO optimize? og ikke static?
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