#include "SetBFMMode_SCISSORS.h"
#include "../BTLog.h"
#include "../STIL_Tuning.h"
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

    // 공격 허용 구간을 더 좁혀서 스키살이 공격 루트 대신 사격 기회를 뺏지 않게 한다.
    const bool cond = sight &&
        (D >= 300.0f && D <= 550.0f) &&
        (los >= 20.0f && los <= 30.0f) &&
        (ecmp <= -1);

    if (cond) {
        BB->BFM = SCISSORS;
        BT_VLOG("[SetBFMMode_SCISSORS] t=" << BB->RunningTime << "s | Enter SCISSORS | D=" << D << ", LOS=" << los << ", E=" << ecmp << "\n");
        return BT::NodeStatus::SUCCESS;
    }

    BT_VLOG("[SetBFMMode_SCISSORS] t=" << BB->RunningTime << "s | Blocked | sight=" << sight << ", LOS=" << los << ", D=" << D << ", E=" << ecmp << "\n");
    return BT::NodeStatus::FAILURE;
}
