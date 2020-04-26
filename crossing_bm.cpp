#include <benchmark/benchmark.h>
#include "reachability.hpp"
#include "crossing.cpp"


// Define another benchmark
static void crossing_breadth(benchmark::State& state) {
    auto state_space = state_space_t{
            actors_t{},                // initial state
            successors<actors_t>(transitions), // successor generator from your library
            &is_valid};                        // invariant over all states
    auto solution = state_space.check(
            [](const actors_t& actors){ // all actors should be on the shore2:
                return std::count(std::begin(actors), std::end(actors), pos_t::shore2)==actors.size();
            });
    for (auto _ : state)
        auto solution = state_space.check(
                [](const actors_t& actors){ // all actors should be on the shore2:
                    return std::count(std::begin(actors), std::end(actors), pos_t::shore2)==actors.size();
                });
}
BENCHMARK(crossing_breadth);
