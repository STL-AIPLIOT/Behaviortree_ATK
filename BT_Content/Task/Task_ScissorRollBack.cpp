#include "Task_ScissorRollBack.h"
#include "../BTLog.h"
#include <iostream>
#include <random>
#include <chrono>
#include <algorithm>

using namespace Action;

static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

BT::NodeStatus Task_ScissorRollBack::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res) {
        std::cerr << "[Task_ScissorRollBack] BB nullptr\n";
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_res.value();

    const float D = BB->Distance;
    const int   ecmp = BB->EnergyCompareResult;

    float side = clampf(200.0f + 0.3f * D, 200.0f, 500.0f);
    float up = clampf(200.0f + 0.2f * D, 200.0f, 500.0f);
    float back = clampf(350.0f + (ecmp < 0 ? 100.0f : 0.0f), 300.0f, 600.0f);

    static thread_local std::mt19937 rng(
        static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> uni(0, 1);
    int dir = uni(rng);

    Vector3 evade_offset;
    if (dir == 0) evade_offset = BB->TargetUpVector * up;
    else        evade_offset = BB->TargetRightVector * side;

    evade_offset = evade_offset - BB->TargetForwardVector * back;
    BB->VP_Cartesian = BB->MyLocation_Cartesian + evade_offset;

    BT_VLOG("[Task_ScissorRollBack] RollBack (" << (dir == 0 ? "Up" : "Right")
        << ") | D=" << D << ", side=" << side << ", up=" << up << ", back=" << back << "\n");
    return BT::NodeStatus::SUCCESS;
}
