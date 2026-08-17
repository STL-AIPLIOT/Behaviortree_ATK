#include "SetBFMMode_SCISSORS.h"
#include "../BTLog.h"
#include <iostream>
#include <algorithm>

using namespace Action;

BT::NodeStatus SetBFMMode_SCISSORS::tick()
{
    auto bb_ptr = getInput<CPPBlackBoard*>("BB");
    if (!bb_ptr) {
        std::cerr << "[SetBFMMode_SCISSORS] BB nullptr\n";
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_ptr.value();

    const bool sight = BB->EnemyInSight;
    const float los = BB->Los_Degree_Target;
    const float D = BB->Distance;
    const int   ecmp = BB->EnergyCompareResult;

    bool cond = sight &&
        (los >= 10.0f && los <= 45.0f) &&
        (D >= 150.0f && D <= 800.0f) &&
        (ecmp <= 0);

    if (cond) {
        BB->BFM = SCISSORS;
        BT_VLOG("[SetBFMMode_SCISSORS] t=" << BB->RunningTime << "s | Enter SCISSORS | D=" << D << ", LOS=" << los << ", E=" << ecmp << "\n");
        return BT::NodeStatus::SUCCESS;
    }

    BT_VLOG("[SetBFMMode_SCISSORS] t=" << BB->RunningTime << "s | Blocked | sight=" << sight << ", LOS=" << los << ", D=" << D << ", E=" << ecmp << "\n");
    return BT::NodeStatus::FAILURE;
}
