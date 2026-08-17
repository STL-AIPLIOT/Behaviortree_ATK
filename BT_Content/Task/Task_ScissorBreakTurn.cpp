#include "Task_ScissorBreakTurn.h"
#include "../BTLog.h"
#include <iostream>
#include <random>
#include <chrono>
#include <algorithm>

using namespace Action;

static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

BT::NodeStatus Task_ScissorBreakTurn::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res) {
        std::cerr << "[Task_ScissorBreakTurn] BB nullptr\n";
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_res.value();

    const float D = BB->Distance;
    float side = clampf(150.0f + 0.4f * D, 200.0f, 500.0f);
    if (D < 200.0f) side = 200.0f;

    static thread_local std::mt19937 rng(
        static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count()));
    std::uniform_int_distribution<int> uni(0, 1);
    int dir = uni(rng);

    Vector3 offset = (dir == 0 ? BB->MyRightVector : -BB->MyRightVector) * side;
    BB->VP_Cartesian = BB->MyLocation_Cartesian + offset;

    BT_VLOG("[Task_ScissorBreakTurn] 방향전환 "
        << (dir == 0 ? "Right" : "Left") << " | D=" << D << ", side=" << side << "\n");
    return BT::NodeStatus::SUCCESS;
}
