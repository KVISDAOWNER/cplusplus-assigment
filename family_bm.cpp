
#include <benchmark/benchmark.h>
#include "reachability.hpp"
#include "family.cpp"


// Define another benchmark
static void family_depth_cost(benchmark::State& state) {
    //solve using different noise as a cost -- the "hardest" one
    auto states = state_space_t{
            state_t{}, // initial state
            cost_t{},   // initial cost
            successors<state_t>(transitions), // successor generator from your library
            &river_crossing_valid,            // invariant over states
            [](const state_t&, const cost_t&){return cost_t{};}};
    for (auto _ : state)
        auto solutions = states.check(&goal);
}
BENCHMARK(family_depth_cost);


static void family_noise_old_son_cost(benchmark::State& state) {
    //solve using different noise as a cost -- the "hardest" one
    auto states = state_space_t{
            state_t{}, // initial state
            cost_t{},   // initial cost
            successors<state_t>(transitions), // successor generator from your library
            &river_crossing_valid,            // invariant over states
            [](const state_t& state, const cost_t& prev_cost){
                auto noise = prev_cost.noise;
                if (state.persons[person_t::son1].pos == person_t::shore1)
                    noise += 2; // older son is more noughty, prefer him first
                if (state.persons[person_t::son2].pos == person_t::shore1)
                    noise += 1;
                return cost_t{ prev_cost.depth, noise };}};      // cost over states
    for (auto _ : state)
        auto solutions = states.check(&goal);
}
BENCHMARK(family_noise_old_son_cost);


static void family_noise_young_son_cost(benchmark::State& state) {
    //solve using different noise as a cost -- the "hardest" one
    auto states = state_space_t{
            state_t{}, // initial state
            cost_t{},   // initial cost
            successors<state_t>(transitions), // successor generator from your library
            &river_crossing_valid,            // invariant over states
            [](const state_t& state, const cost_t& prev_cost){
                auto noise = prev_cost.noise;
                if (state.persons[person_t::son1].pos == person_t::shore1)
                    noise += 1;
                if (state.persons[person_t::son2].pos == person_t::shore1)
                    noise += 2; // younger son is more distressed, prefer him first
                return cost_t{ prev_cost.depth, noise };}};      // cost over states
    for (auto _ : state)
        auto solutions = states.check(&goal);
}
BENCHMARK(family_noise_young_son_cost);
