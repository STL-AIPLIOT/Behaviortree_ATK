#include "Task_OneCircleAttack.h"
#include "../BTLog.h"
#include <algorithm>
#include <cmath>

// 멤버 정의는 클래스를 감싼 namespace 안에 둔다.
// `using namespace Action;` + 전역 범위 정의는 MSVC가 받아주기는 하지만 표준상 부적합하다.
namespace Action {

static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

void Task_OneCircleAttack::ResetInternalState()
{
    entry_time_sec_ = 0.0;
    settle_since_sec_ = -1.0;
    vp_saved_ = false;
}

BT::NodeStatus Task_OneCircleAttack::onStart()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res || !bb_res.value())
    {
        std::cerr << "[Task_OneCircleAttack] BB nullptr\n";
        ResetInternalState();
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_res.value();

    // Timeout halt 이후 재진입할 때 이전 상태가 이어지지 않도록 항상 초기화한다.
    ResetInternalState();

    if (BB->HABFM_CircleMode != ONE_CIRCLE)
    {
        // 1-circle 차례가 아니다. 2-circle 또는 reposition 에게 넘긴다.
        BT_VLOG("[Task_OneCircleAttack] skip | CircleMode="
            << (BB->HABFM_CircleMode == TWO_CIRCLE ? "2C" : "NONE") << "\n");
        return BT::NodeStatus::FAILURE;
    }

    entry_time_sec_ = BB->RunningTime;
    vp_on_entry_ = BB->VP_Cartesian;
    vp_saved_ = true;

    return Advance(BB);
}

BT::NodeStatus Task_OneCircleAttack::onRunning()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res || !bb_res.value())
    {
        std::cerr << "[Task_OneCircleAttack] BB nullptr\n";
        ResetInternalState();
        return BT::NodeStatus::FAILURE;
    }

    return Advance(bb_res.value());
}

BT::NodeStatus Task_OneCircleAttack::Advance(CPPBlackBoard* BB)
{
    const double elapsed = BB->RunningTime - entry_time_sec_;

    // 기동 도중 선회율 판정이 뒤집히면 즉시 양보한다.
    if (BB->HABFM_CircleMode != ONE_CIRCLE)
    {
        if (vp_saved_) { BB->VP_Cartesian = vp_on_entry_; }
        BT_VLOG("[Task_OneCircleAttack] abort | CircleMode changed, t=" << elapsed << "\n");
        ResetInternalState();
        return BT::NodeStatus::FAILURE;
    }

    const float myV = BB->MySpeed_MS;
    const float tgV = BB->TargetSpeed_MS;
    const float D = BB->Distance;
    const float dv = myV - tgV;

    // [설계] 인사이드 롤: 적기 좌측(-) 방향으로 파고들되, 거리/속도차에 비례해 넓힌다.
    const float side_in = clampf(200.0f + 0.3f * D + 8.0f * dv, 250.0f, 650.0f);
    const float forward = clampf(100.0f + 0.2f * D, 120.0f, 400.0f);

    BB->VP_Cartesian = BB->TargetLocaion_Cartesian
        - BB->TargetRightVector * side_in
        + BB->TargetForwardVector * forward;

    // 머지 해소 판정. 한 tick 스쳐 지나가는 값으로 SUCCESS 가 나지 않도록 유지 시간을 본다.
    const float aa = std::fabs(BB->MyAspectAngle_Degree);

    if (aa <= EXIT_AA_DEG)
    {
        if (settle_since_sec_ < 0.0) { settle_since_sec_ = BB->RunningTime; }

        if (BB->RunningTime - settle_since_sec_ >= SETTLE_DWELL_SEC)
        {
            BT_VLOG("[Task_OneCircleAttack] resolved | AA=" << aa
                << ", t=" << elapsed << "\n");
            ResetInternalState();
            return BT::NodeStatus::SUCCESS;
        }
    }
    else
    {
        settle_since_sec_ = -1.0;
    }

    BT_VLOG("[Task_OneCircleAttack] One-Circle | dv=" << dv << ", D=" << D
        << " | side_in=" << side_in << ", fwd=" << forward
        << ", AA=" << aa << ", t=" << elapsed << "\n");
    return BT::NodeStatus::RUNNING;
}

void Task_OneCircleAttack::onHalted()
{
    // Timeout(Rule.xml HABFM_Action) 만료 시 타이머 스레드에서 haltChild() 로 들어온다.
    auto bb_res = getInput<CPPBlackBoard*>("BB");

    if (bb_res && bb_res.value() && vp_saved_)
    {
        CPPBlackBoard* BB = bb_res.value();
        BB->VP_Cartesian = vp_on_entry_;
        BT_VLOG("[Task_OneCircleAttack] halted | t="
            << (BB->RunningTime - entry_time_sec_) << "\n");
    }

    ResetInternalState();
}

} // namespace Action
