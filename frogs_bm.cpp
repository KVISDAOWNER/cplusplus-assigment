#include <benchmark/benchmark.h>
#include "reachability.hpp"
#include "frogs.cpp"

static void frogs_2_breadth(benchmark::State& state) {
    size_t frogs = 2;
    const auto stones = frogs*2+1; // frogs on either side and 1 empty in the middle
    auto start = stones_t(stones, frog::empty);  // initially all empty
    auto finish = stones_t(stones, frog::empty); // initially all empty
    while (frogs-->0) { // count down from frogs-1 to 0 and put frogs into positions:
        start[frogs] = frog::green;                  // green on left
        start[start.size()-frogs-1] = frog::brown;   // brown on right
        finish[frogs] = frog::brown;                 // brown on left
        finish[finish.size()-frogs-1] = frog::green; // green on right
    }
    auto space = state_space_t{
            std::move(start),                 // initial state
            successors<stones_t>(transitions) // successor-generating function from your library
    };
    for (auto _ : state)
        auto solutions = space.check(
                [finish=std::move(finish)](const stones_t& state){ return state==finish; }, search_order_t::breadth_first);
}
BENCHMARK(frogs_2_breadth);


static void frogs_2_depth(benchmark::State& state) {
    size_t frogs = 2;
    const auto stones = frogs*2+1; // frogs on either side and 1 empty in the middle
    auto start = stones_t(stones, frog::empty);  // initially all empty
    auto finish = stones_t(stones, frog::empty); // initially all empty
    while (frogs-->0) { // count down from frogs-1 to 0 and put frogs into positions:
        start[frogs] = frog::green;                  // green on left
        start[start.size()-frogs-1] = frog::brown;   // brown on right
        finish[frogs] = frog::brown;                 // brown on left
        finish[finish.size()-frogs-1] = frog::green; // green on right
    }
    auto space = state_space_t{
            std::move(start),                 // initial state
            successors<stones_t>(transitions) // successor-generating function from your library
    };
    for (auto _ : state)
        auto solutions = space.check(
                [finish=std::move(finish)](const stones_t& state){ return state==finish; }, search_order_t::depth_first);
}
BENCHMARK(frogs_2_depth);


static void frogs_4_breadth(benchmark::State& state) {
    size_t frogs = 4;
    const auto stones = frogs*2+1; // frogs on either side and 1 empty in the middle
    auto start = stones_t(stones, frog::empty);  // initially all empty
    auto finish = stones_t(stones, frog::empty); // initially all empty
    while (frogs-->0) { // count down from frogs-1 to 0 and put frogs into positions:
        start[frogs] = frog::green;                  // green on left
        start[start.size()-frogs-1] = frog::brown;   // brown on right
        finish[frogs] = frog::brown;                 // brown on left
        finish[finish.size()-frogs-1] = frog::green; // green on right
    }
    auto space = state_space_t{
            std::move(start),                 // initial state
            successors<stones_t>(transitions) // successor-generating function from your library
    };
    for (auto _ : state)
        auto solutions = space.check(
                [finish=std::move(finish)](const stones_t& state){ return state==finish; }, search_order_t::breadth_first);
}
BENCHMARK(frogs_4_breadth);


static void frogs_4_depth(benchmark::State& state) {
    size_t frogs = 4;
    const auto stones = frogs*2+1; // frogs on either side and 1 empty in the middle
    auto start = stones_t(stones, frog::empty);  // initially all empty
    auto finish = stones_t(stones, frog::empty); // initially all empty
    while (frogs-->0) { // count down from frogs-1 to 0 and put frogs into positions:
        start[frogs] = frog::green;                  // green on left
        start[start.size()-frogs-1] = frog::brown;   // brown on right
        finish[frogs] = frog::brown;                 // brown on left
        finish[finish.size()-frogs-1] = frog::green; // green on right
    }
    auto space = state_space_t{
            std::move(start),                 // initial state
            successors<stones_t>(transitions) // successor-generating function from your library
    };
    for (auto _ : state)
        auto solutions = space.check(
                [finish=std::move(finish)](const stones_t& state){ return state==finish; }, search_order_t::depth_first);
}
BENCHMARK(frogs_4_depth);





BENCHMARK_MAIN();