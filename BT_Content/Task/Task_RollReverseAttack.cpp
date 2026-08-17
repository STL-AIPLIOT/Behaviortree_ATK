#include "Task_RollReverseAttack.h"
#include "../BTLog.h"

using namespace Action;

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

BT::NodeStatus Task_RollReverseAttack::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res)
    {
        std::cerr << "[Task_RollReverseAttack] BB 포인터 가져오기 실패\n";
        return BT::NodeStatus::FAILURE;
    }

    CPPBlackBoard* BB = bb_res.value();

    // 근접 가드: 너무 가까우면 롤리버스 대신 "살짝 이격"만
    const float D = BB->Distance;

    float side = 800.0f;  // 원안
    float back = 300.0f;  // 원안

    // 속도, 거리 등에 따라 살짝 클램프 (과조작 방지)
    side = clampf(side, 400.0f, 900.0f);
    back = clampf(back, 150.0f, 400.0f);

    if (D < 250.0f) {
        // 너무 가까우면 작은 롤리버스 (측방 400, 후방 150)
        side = 400.0f;
        back = 150.0f;
    }

    // Roll Reverse: 크게 적기 오른쪽으로 빠지기
    Vector3 reverse_offset = BB->TargetRightVector * side - BB->TargetForwardVector * back;

    BB->VP_Cartesian = BB->MyLocation_Cartesian + reverse_offset;

    BT_VLOG("[Task_RollReverseAttack] Roll Reverse 수행 | D=" << D
        << " | side=" << side << ", back=" << back << "\n");

    return BT::NodeStatus::SUCCESS;
}
