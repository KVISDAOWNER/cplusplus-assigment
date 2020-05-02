#ifndef ASSIGNMENT_REACHABILITY_HPP
#define ASSIGNMENT_REACHABILITY_HPP

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

template <typename TState, typename TCost>
struct node{
    node() = delete; //should never be instantiated like this
    ~node() = default; //rule of zero - struct doesn't directly manage any resources
    node(TState s, TCost c): state{s}, cost{c}{}
    TState state;
    TCost cost;
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



//TODO remove this if not used
template <typename T, typename  = void> //Not not equal comparable
struct is_not_equal_comparable : std::false_type {};

//https://stackoverflow.com/questions/16399346/c11-static-assert-for-equality-comparable-type
template <typename T>
struct is_not_equal_comparable<T,
        typename std::enable_if<
                true,
                decltype(std::declval<T&>() != std::declval<decltype(nullptr)>(), (void)0)
        >::type
>: std::true_type{};

//TODO remove this if not used
template <typename T, typename  = void> //Not less than comparable
struct is_less_than_comparable : std::false_type {};

//https://stackoverflow.com/questions/16399346/c11-static-assert-for-equality-comparable-type
template <typename T>
struct is_less_than_comparable<T,
        typename std::enable_if<
                true,
                decltype(std::declval<T&>() < std::declval<decltype(nullptr)>(), (void)0)
        >::type
>: std::true_type{};

template <typename T, typename = void> //Not Container //TODO Why void here?
struct is_comparable: std::false_type {};

template <typename T>
struct is_comparable<T,
        std::void_t<
                typename std::enable_if<is_not_equal_comparable<T>::value, void>,
                typename std::enable_if<is_less_than_comparable<T>::value, void>
        >
>: std::true_type {};

template <typename T>
constexpr auto is_comparable_v = is_comparable<T>::value;

//TODO https://stackoverflow.com/questions/40934997/stdmove-or-stdforward-when-assigning-universal-constructor-to-member-variabl
// https://stackoverflow.com/questions/17316386/pass-by-value-or-universal-reference
template <typename TState, typename TCost = default_cost, typename TCost_Fun = std::function<TCost(const TState&, const TCost&)>, typename = std::enable_if<is_comparable_v<TState>>>
class state_space_t{
    typedef std::function<std::vector<TState>(TState&)> succ_fun;
    public:
        state_space_t() = delete; //should never be instantiated like this
        ~state_space_t() = default; //rule of zero - class doesn't directly manage any resources

        state_space_t(
                TState      start_state,
                succ_fun    successors, //TODO better default? std::vector ok here?
                bool        (*is_valid) (const TState&) = [](const TState&){return true;}) //frogs needs default
                : state_space_t(        //constructor chaining to avoid duplicate null argument type check
                        start_state,
                        default_cost{},
                        successors,
                        is_valid,
                        [](const TState&, const TCost&){return TCost{};}){} //default cost fun

        state_space_t(
                TState      start_state,
                TCost       initial_cost,
                succ_fun    successors,
                bool        (*is_valid) (const TState&),
                TCost_Fun   cost_fun )
                : _start_state{start_state}, _initial_cost{initial_cost}, _successor_fun{successors}, _is_valid{is_valid}, _cost_fun{cost_fun}
                {
                    static_assert(std::is_invocable_r<TCost, decltype(cost_fun), const TState&, const TCost&>::value,
                            "function argument is either not invocable or does not coincide with expected "
                           "argument/return types");

                    //static_assert(std::is_function<TCost_Fun>::value, "cost function type must be invocable");
                    //static_assert(!std::is_same<decltype(cost_fun), decltype(nullptr)>::value, "Constructor cannot be called with nullptr arguments."); //if it is null pointer constant type.
                    //static_assert(is_equality_comparable<TCost_Fun>::value, "cost function type must be equality \"==\" comparable to nullptr");
                    //static_assert(is_equality_comparable<decltype(is_valid)>::value, "is valid function type must be equality \"==\" comparable to nullptr");

                    //if(is_valid == nullptr)         throw argument_null_exception(__func__, GET_VARIABLE_NAME(is_valid));
                    //if(cost_fun == nullptr)    throw std::invalid_argument(nullptr_err_msg(__func__, GET_VARIABLE_NAME(cost_fun)));
                }

        std::vector<std::vector<std::shared_ptr<TState>>> check(std::function<bool(TState)> goal_predicate, search_order_t order = search_order_t::breadth_first){
            typedef search_order_t search;

            if(goal_predicate == nullptr)         throw std::invalid_argument(nullptr_err_msg(__func__, GET_VARIABLE_NAME(goal_predicate)));

            node<TState, TCost> (*pop)    (std::vector<node<TState, TCost>>&) {(order == search::breadth_first) ? pop_queue : pop_stack}; //TODO ref i stedet?
            auto parent_map = std::map<node<TState, TCost>,std::vector<node<TState, TCost>>, decltype(cmp_s)>(cmp_s); //for trace and check if visited
            auto seen = std::set<TState>();
            auto goal_states = std::vector<node<TState, TCost>>();
            auto waiting = std::vector<node<TState, TCost>>();
            if(_is_valid(_start_state))
                push(waiting, node(_start_state, _initial_cost), cmp_c);

            while(!waiting.empty()){
                auto state = pop(waiting);
                int size = waiting.size();
                if(goal_predicate(state.state)){
                    goal_states.push_back(state);
                    break; //We only want the first solution
                }
                for(const auto& n_state: _successor_fun(state.state)){
                    if(!seen.count(n_state) && _is_valid(n_state)) { //if not seen before?
                        seen.insert(n_state);
                        auto n_node = node(n_state, _cost_fun(n_state, state.cost)); //TODO do the null check smarter or give default func smart way?
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

            //-------Make trace-------
            auto solutionTrace = std::vector<std::vector<std::shared_ptr<TState>>>();
            auto trace = std::vector<std::shared_ptr<TState>>(); //TODO unique ptrs i stedet i denne funktion? share_ptr forid vi copier nogle gange, kan det undgåes?
            for (auto& node: goal_states) {
                trace.push_back(std::make_shared<TState>(node.state));
                while(node.state != _start_state){
                    node = parent_map.find(node)->second[0]; //Here we guarantee that state is a key //TODO branch out when multiple parents?
                    trace.push_back(std::make_shared<TState>(node.state));
                }
                std::reverse(std::begin(trace), std::end(trace));
                solutionTrace.push_back(trace);
            }

            return solutionTrace;
        }

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
            {return std::string("In call to function: ") + func + std::string(". Invalid argument: ") +  var + " is nullptr.";}

        //TODO lav generic?
        static node<TState, TCost> pop_stack(std::vector<node<TState, TCost>>& s){
            node<TState, TCost> v = s.back(); //TODO std::move her?
            s.pop_back();
            return v;
        } //TODO optimize? if(data.back()>-1) then back(); og ikke static?

        static node<TState, TCost> pop_queue(std::vector<node<TState, TCost>>& q) {
            node<TState, TCost> v = q.front();  //TODO std::move her?
            q.erase(q.begin() );
            return v;
        } //TODO optimize? og ikke static?

        //https://stackoverflow.com/questions/15843525/how-do-you-insert-the-value-in-a-sorted-vector
        template<typename TFunc, typename C >
        typename std::enable_if<is_container_v<C>, void>::type
        push(C& c, node<TState, TCost> const& item, TFunc cmp)
        {
            c.insert(
                std::upper_bound( c.begin(), c.end(), item, cmp ),
                item);
        }
};

template<typename TState, typename Container>
typename std::enable_if<is_container_v<Container>, std::function<std::vector<TState>(TState&)>>::type
successors(Container transitions (const TState&)) {
    static_assert(std::is_invocable<decltype(transitions), const TState&>::value, "function argument must be invocable"); //TODO det her kan vel aldrig være false?
    return [transitions](const TState& startState){ //TODO hvorfor kan den ikke tage &transitions?
        auto states = std::vector<TState>(); //results
        auto functions = transitions(startState);
        for (const auto& transition: functions) {
            TState stateCpy = startState;
            transition(stateCpy);
            states.push_back(stateCpy);
        }
        return states;
    }; //TODO lav refs her for optimization
}

#endif //ASSIGNMENT_REACHABILITY_HPP