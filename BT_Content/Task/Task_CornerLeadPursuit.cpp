#include "Task_CornerLeadPursuit.h"
#include "../BTLog.h"
#include <algorithm>
#include <cmath>

namespace Action {

BT::NodeStatus Task_CornerLeadPursuit::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res || !bb_res.value())
    {
        std::cerr << "[Task_CornerLeadPursuit] BB nullptr\n";
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_res.value();

    const float V = BB->MySpeed_MS;
    const float D = BB->Distance;
    const float ao = std::fabs(BB->MyAngleOff_Degree);
    const float dv = BB->MySpeed_MS - BB->TargetSpeed_MS;

    // --- 3조건 ---
    const bool inCornerSpeedBand =
        (V >= CORNER_SPEED_MIN_MS) && (V <= CORNER_SPEED_MAX_MS);

    const bool inLeadPursuitWindow =
        BB->EnemyInSight && (ao <= LEAD_WINDOW_AO_DEG)
        && (D >= LEAD_D_MIN_M) && (D <= LEAD_D_MAX_M);

    const bool noOvershootRisk =
        !((D < OVERSHOOT_D_M) && (dv > OVERSHOOT_DV_MS));

    // 진입률 집계용. 매 tick 한 줄씩 남긴다 (Docs/12 T8).
    BT_VLOG("[Task_CornerLeadPursuit] cond | speed=" << (inCornerSpeedBand ? 1 : 0)
        << " window=" << (inLeadPursuitWindow ? 1 : 0)
        << " noOvershoot=" << (noOvershootRisk ? 1 : 0)
        << " | V=" << V << ", AO=" << ao << ", D=" << D << ", dv=" << dv << "\n");

    if (!(inCornerSpeedBand && inLeadPursuitWindow && noOvershootRisk))
    {
        return BT::NodeStatus::FAILURE;
    }

    // --- lead pursuit VP ---
    const float tgt_spd = std::max(1.0f, BB->TargetSpeed_MS);
    const float lead_time =
        std::max(LEAD_TIME_MIN_SEC, std::min(LEAD_TIME_MAX_SEC, D / tgt_spd));

    BB->VP_Cartesian = BB->TargetLocaion_Cartesian + BB->PredictedTargetVelocity * lead_time;

    BT_VLOG("[Task_CornerLeadPursuit] ENTER | lead_time=" << lead_time
        << ", V=" << V << ", AO=" << ao << ", D=" << D << "\n");

    return BT::NodeStatus::SUCCESS;
}

} // namespace Action
