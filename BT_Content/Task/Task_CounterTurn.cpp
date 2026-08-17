#include "Task_CounterTurn.h"
#include "../BTLog.h"
#include <algorithm>

using namespace Action;

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

BT::NodeStatus Task_CounterTurn::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res)
    {
        std::cerr << "[Task_CounterTurn] BB 포인터 가져오기 실패\n";
        return BT::NodeStatus::FAILURE;
    }

    CPPBlackBoard* BB = bb_res.value();

    // 상황 변수
    const float D = BB->Distance;                 // m
    const float dv = BB->TargetSpeed_MS - BB->MySpeed_MS; // 적-아군 속도차
    const int   ecmp = BB->EnergyCompareResult;      // >0 우세, <0 열세

    // ───────────────────────── 오프셋 크기 정책 ─────────────────────────
    // Counter-Turn: 상대 진행 방향을 "벗어나며" 다음 턴 여유를 만듦.
    // 가까울수록 작은 측방, 멀수록 크게. 너무 과도한 이동은 클램프.
    float side = clampf(250.0f + 0.5f * D, 300.0f, 800.0f); // 좌/우 이탈 크기
    // 속도차(적이 빠름)가 크면 약간 더 크게
    if (dv > 8.0f) side = clampf(side * 1.15f, 300.0f, 850.0f);

    // 근접 시(초근접) 과조작 방지: 측방을 낮춤
    if (D < 250.0f) side = clampf(side, 300.0f, 450.0f);

    // 살짝 뒤로 물러나는 성분(과리드 방지): 에너지 열세일수록 조금 더 후퇴
    float back = (ecmp < 0 ? 180.0f : 120.0f);
    back = clampf(back, 80.0f, 220.0f);

    // 오른쪽 기준 Counter-Turn (XML에 좌/우 명시 없으므로 일관된 방향 사용)
    Vector3 offset = BB->TargetRightVector * side - BB->TargetForwardVector * back; // [변경]

    BB->VP_Cartesian = BB->MyLocation_Cartesian + offset;

    BT_VLOG("[Task_CounterTurn] CounterTurn: D=" << D
        << ", dv=" << dv << ", Energy=" << ecmp
        << " | side=" << side << ", back=" << back << "\n");

    return BT::NodeStatus::SUCCESS;
}
