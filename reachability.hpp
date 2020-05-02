#ifndef ASSIGNMENT_REACHABILITY_HPP
#define ASSIGNMENT_REACHABILITY_HPP

//Used for better error messages
#define GET_VARIABLE_NAME(Variable) (#Variable)

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

/*
 * default_cost struct is used if none can be deduced from state_space constructor. Simply copied and fitted from
 * family.cpp.
 */
struct default_cost{
    size_t depth{0};
    bool operator<(const default_cost& other) const {
        if (depth < other.depth)
            return true;
        if (other.depth < depth)
            return false;
        return false; // == case
    }
};

/*
 * I chose to model a state and associated cost as a node. This could also have been done as a pair, but I think that
 * .state and .cost is more readable than .first and .second
 */
template <typename TState, typename TCost>
struct node{
    node() = delete; //should never be instantiated like this
    ~node() = default; //rule of zero - struct doesn't directly manage any resources
    node(TState s, TCost c): state{s}, cost{c}{}
    TState state;
    TCost cost;
};


//SFINAE for container
template <typename T, typename = void> //Not Container
struct is_container: std::false_type {};

/*
 * Partly solves Requirement 7. "The library should support any (iterable) containers (for transitions,
 * state representation)"
 * If it quacks like a container then it is a container
 */
template <typename C>
struct is_container<C,
    std::void_t<
        typename C::iterator,
        typename C::const_iterator,
        decltype(std::begin(std::declval<C&>())),
        decltype(std::end(std::declval<C&>()))
    >
>: std::true_type {};

//Constexpr just for more elegant usage of is_container
template<typename Container>
constexpr auto is_container_v = is_container<Container>::value;


/* Partly solves Requirement 7. "The library should support any (iterable) containers (for transitions,
 * state representation)"
 * Note that TState template type is deduced from the constructor thus supporting any state representation that fits
 * the TCost_Fun and goal_predicate
 * I chose to make all template arguments be deduced so that they fit the files frogs, crossing, family.
 */
template <typename TState, typename TCost = default_cost, typename TCost_Fun =
                                                                   std::function<TCost(const TState&, const TCost&)>>
class state_space_t{
    typedef std::function<std::vector<TState>(TState&)> succ_fun;

public:
        ~state_space_t() = default; //rule of zero - default OK class doesn't directly manage any resources


        //Solves Requirement 8. "The library should be generic and applicable to all puzzles using the same library
        //templates. If the search order or cost are not used, the library should assume reasonable defaults."
        state_space_t(
                TState      start_state,
                succ_fun    successors, //TODO better default? std::vector ok here?
                bool        (*is_valid) (const TState&) = [](const TState&){return true;}) //frogs needs default
                : state_space_t(        //constructor chaining to avoid duplicate null argument type check
                        start_state,
                        default_cost{},
                        successors,
                        is_valid,
                        [](const TState&, const TCost&){return TCost{};}) //default cost fun
                        {}

        state_space_t(
                TState      start_state,
                TCost       initial_cost,
                succ_fun    successors,
                bool        (*is_valid) (const TState&),
                TCost_Fun   cost_fun )
                : _start_state{start_state}, _initial_cost{initial_cost}, _successor_fun{successors},
                _is_valid{is_valid}, _cost_fun{cost_fun}
                {
                    //Partly solves Requirement 9. "User friendly to use and fail with a message if the user ..."
                    static_assert(std::is_invocable_r<TCost, decltype(cost_fun), const TState&, const TCost&>::value,
                            "function argument is either not invocable or does not coincide with expected "
                           "argument/return types");
                }

        std::vector<std::vector<std::shared_ptr<TState>>> check(std::function<bool(TState)> goal_predicate,
                                                                search_order_t order = search_order_t::breadth_first);


private:
        typedef  std::function<bool (const node<TState, TCost>& a, const node<TState, TCost>& b)>   cmp_node_func;

        TState          _start_state;
        TCost           _initial_cost;
        succ_fun        _successor_fun;
        TCost_Fun       _cost_fun;
        bool            (*_is_valid) (const TState&);
        cmp_node_func   cmp_c =
                            [](const node<TState, TCost>& a, const node<TState, TCost>& b) { return a.cost < b.cost; };
        cmp_node_func   cmp_s =
                            [](const node<TState, TCost>& a, const node<TState, TCost>& b) { return a.state < b.state;};

        std::string nullptr_err_msg (const std::string& func, const std::string& var)
            {return std::string("In call to function: ") + func + std::string(". Invalid argument: ") +
            var + " is nullptr.";}

        static node<TState, TCost> pop_stack(std::vector<node<TState, TCost>>& s);
        static node<TState, TCost> pop_queue(std::vector<node<TState, TCost>>& q);

        //Generic push to sorted container
        template<typename TFunc, typename Container>
        typename std::enable_if<is_container_v<Container>, void>::type
        push(Container& c, typename Container::value_type item, TFunc cmp)
            {   c.insert(    std::upper_bound( c.begin(), c.end(), item, cmp ),     item); }
};


/* Solves Requirement 2. "Find a state satisfying the goal predicate when given an initial state and successor gene..."
 * Initial state and successor is given through constructor. Check follow to a large degree the Algorithm 1 pseudocode.
 * Biggest difference from Algorithm 1 is that I do "if state ∈/ passed" (my seen = passed) check before adding to
 * waiting, which saves me a push, pop, and check. Also I never do the "if state' ∈/ waiting" check because waiting
 * is a subset of seen, thus it is unnecessary and more efficient to leave the check out.
 */
template<typename TState, typename TCost, typename TCost_Fun>
std::vector<std::vector<std::shared_ptr<TState>>>
state_space_t<TState, TCost, TCost_Fun>::check(std::function<bool(TState)> goal_predicate, search_order_t order) {
    {
        typedef search_order_t search;

        //Partly solves Requirement 9. "User friendly to use and fail with a message if the user ..."
        if(goal_predicate == nullptr)
            throw std::invalid_argument(nullptr_err_msg(__func__, GET_VARIABLE_NAME(goal_predicate)));

        //Solves Requirement 4. "Support various search orders: breadth-first search, depth-first search...
        node<TState, TCost> (*pop)
                (std::vector<node<TState, TCost>>&) {(order == search::breadth_first) ? pop_queue : pop_stack};

        auto parent_map = std::map<node<TState, TCost>,node<TState, TCost>, decltype(cmp_s)>(cmp_s); //for trace and check if visited
        auto seen = std::set<TState>();
        auto goal_states = std::vector<node<TState, TCost>>();
        auto waiting = std::vector<node<TState, TCost>>();
        if(_is_valid(_start_state))
            push(waiting, node(_start_state, _initial_cost), cmp_c);

        while(!waiting.empty()){
            auto state = pop(waiting);
            if(goal_predicate(state.state)){
                goal_states.push_back(state);
                break; //We only want the first solution
            }
            for(const auto& n_state: _successor_fun(state.state)){
                //Solves Requirement 5. "Support a given invariant predicate e (state validation function..."
                if(!seen.count(n_state) && _is_valid(n_state)) { //if not seen before and valid
                    seen.insert(n_state);

                    //Solves Requirement 6. "Support custom cost function over states ..."
                    auto n_node = node(n_state, _cost_fun(n_state, state.cost));
                    push(waiting, n_node, cmp_c);
                    parent_map.insert(std::make_pair(n_node,state));
                }
            }
        }

        //Partly solves Requirement 3. "Print the trace of a state sequence from the initial state to the found goal..."
        //-------Make trace-------
        auto solutionTrace = std::vector<std::vector<std::shared_ptr<TState>>>();
        auto trace = std::vector<std::shared_ptr<TState>>();
        for (auto& node: goal_states) { //actually only one node in goal_states right now
            trace.push_back(std::make_shared<TState>(node.state));
            while(node.state != _start_state){
                node = parent_map.find(node)->second; //reassign node to parent of node
                trace.push_back(std::make_shared<TState>(node.state));
            }
            std::reverse(std::begin(trace), std::end(trace));
            solutionTrace.push_back(trace);
        }

        return solutionTrace;
    }
}

/* Partly solves Requirement 4. "Support various search orders: breadth-first search, depth-first search..."
 * I chose to make the pop_stack function to make check() more simple and elegant when switching between
 * popping stack and queue. They are not made completely generic because .back and .pop_back methods are not applicable
 * across all containers
 */
template<typename TState, typename TCost, typename TCost_Fun>
node<TState, TCost> state_space_t<TState, TCost, TCost_Fun>::pop_stack(std::vector<node<TState, TCost>> &s) {
    node<TState, TCost>& v = s.back();
    s.pop_back();
    return v;
}

/* Partly solves Requirement 4. "Support various search orders: breadth-first search, depth-first search..."
 * I chose to make the pop_queue function to make check() more simple and elegant when switching between
 * popping stack and queue.
 */
template<typename TState, typename TCost, typename TCost_Fun>
node<TState, TCost> state_space_t<TState, TCost, TCost_Fun>::pop_queue(std::vector<node<TState, TCost>> &q) {
    node<TState, TCost>& v = q.front();
    q.erase(q.begin() );
    return v;
}

/* Solves Requirement 1. "Create a generic successor generator function out of a transition generator function..."
 * Solves Requirement 7. "The library should support any (iterable) containers (for transitions, state representation).
 * Takes the puzzle specific transition functions as input.
 * Note that I chose to return a function that returns the reachable states within 1 step. This means that the state
 * space is created lazily during check. A huge benefit is also the simplicity of check() as a result. Check() is almost
 * one-to-one with the Algorithm 1 pseudocode.
 * Also note that the deduced template type Container can be any type that is duck typed according to
 * is_container SFINAE
 * Other design options would be:
 * 1) Return a function that returns the entire state space. Could be represented as a tree (nested std::multimap).
 * Option 1) Would be OK, but is less efficient as it will always generate the entire state space.
 */
template<typename TState, typename Container>
typename std::enable_if<is_container_v<Container>, std::function<std::vector<TState>(TState&)>>::type
successors(Container transitions (const TState&)) {
    static_assert(std::is_invocable<decltype(transitions), const TState&>::value,
                  "function argument must be invocable");
    return [transitions](const TState& startState){
        auto states = std::vector<TState>(); //results
        auto functions = transitions(startState);
        for (const auto& transition: functions) {
            TState stateCpy = startState;
            transition(stateCpy);
            states.push_back(stateCpy);
        }
        return states;
    };
}

#endif //ASSIGNMENT_REACHABILITY_HPP