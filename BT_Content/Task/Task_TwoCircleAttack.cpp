#include "Task_TwoCircleAttack.h"
#include "../BTLog.h"
#include <algorithm>
#include <cmath>

namespace Action {

static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

void Task_TwoCircleAttack::ResetInternalState()
{
    entry_time_sec_ = 0.0;
    settle_since_sec_ = -1.0;
    vp_saved_ = false;
}

BT::NodeStatus Task_TwoCircleAttack::onStart()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res || !bb_res.value())
    {
        std::cerr << "[Task_TwoCircleAttack] BB nullptr\n";
        ResetInternalState();
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_res.value();

    ResetInternalState();

    if (BB->HABFM_CircleMode != TWO_CIRCLE)
    {
        BT_VLOG("[Task_TwoCircleAttack] skip | CircleMode="
            << (BB->HABFM_CircleMode == ONE_CIRCLE ? "1C" : "NONE") << "\n");
        return BT::NodeStatus::FAILURE;
    }

    entry_time_sec_ = BB->RunningTime;
    vp_on_entry_ = BB->VP_Cartesian;
    vp_saved_ = true;

    return Advance(BB);
}

BT::NodeStatus Task_TwoCircleAttack::onRunning()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res || !bb_res.value())
    {
        std::cerr << "[Task_TwoCircleAttack] BB nullptr\n";
        ResetInternalState();
        return BT::NodeStatus::FAILURE;
    }

    return Advance(bb_res.value());
}

BT::NodeStatus Task_TwoCircleAttack::Advance(CPPBlackBoard* BB)
{
    const double elapsed = BB->RunningTime - entry_time_sec_;

    if (BB->HABFM_CircleMode != TWO_CIRCLE)
    {
        if (vp_saved_) { BB->VP_Cartesian = vp_on_entry_; }
        BT_VLOG("[Task_TwoCircleAttack] abort | CircleMode changed, t=" << elapsed << "\n");
        ResetInternalState();
        return BT::NodeStatus::FAILURE;
    }

    const float myV = BB->MySpeed_MS;
    const float tgV = BB->TargetSpeed_MS;
    const float D = BB->Distance;
    const float dv = tgV - myV;

    // [설계] 바깥 롤: 적기 우측(+) 바깥으로 크게 돌아 나가며, 거리/속도차에 비례해 넓힌다.
    const float side_out = clampf(350.0f + 0.4f * D + 10.0f * dv, 400.0f, 900.0f);
    const float forward = clampf(80.0f + 0.15f * D, 100.0f, 350.0f);

    BB->VP_Cartesian = BB->TargetLocaion_Cartesian
        + BB->TargetRightVector * side_out
        + BB->TargetForwardVector * forward;

    const float aa = std::fabs(BB->MyAspectAngle_Degree);

    if (aa <= EXIT_AA_DEG)
    {
        if (settle_since_sec_ < 0.0) { settle_since_sec_ = BB->RunningTime; }

        if (BB->RunningTime - settle_since_sec_ >= SETTLE_DWELL_SEC)
        {
            BT_VLOG("[Task_TwoCircleAttack] resolved | AA=" << aa
                << ", t=" << elapsed << "\n");
            ResetInternalState();
            return BT::NodeStatus::SUCCESS;
        }
    }
    else
    {
        settle_since_sec_ = -1.0;
    }

    BT_VLOG("[Task_TwoCircleAttack] Two-Circle | dv=" << dv << ", D=" << D
        << " | side_out=" << side_out << ", fwd=" << forward
        << ", AA=" << aa << ", t=" << elapsed << "\n");
    return BT::NodeStatus::RUNNING;
}

void Task_TwoCircleAttack::onHalted()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");

    if (bb_res && bb_res.value() && vp_saved_)
    {
        CPPBlackBoard* BB = bb_res.value();
        BB->VP_Cartesian = vp_on_entry_;
        BT_VLOG("[Task_TwoCircleAttack] halted | t="
            << (BB->RunningTime - entry_time_sec_) << "\n");
    }

    ResetInternalState();
}

} // namespace Action
