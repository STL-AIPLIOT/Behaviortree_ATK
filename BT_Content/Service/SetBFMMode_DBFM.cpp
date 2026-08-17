#include "SetBFMMode_DBFM.h"
#include "../BTLog.h"
#include <iostream>
#include <algorithm>

using namespace Action;

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

BT::NodeStatus SetBFMMode_DBFM::tick()
{
    auto bb_ptr = getInput<CPPBlackBoard*>("BB");
    if (!bb_ptr)
    {
        std::cerr << "[SetBFMMode_DBFM] BB 포인터 가져오기 실패\n";
        return BT::NodeStatus::FAILURE;
    }

    CPPBlackBoard* BB = bb_ptr.value();

    // === 입력 (이미 BB에 존재한다고 가정; 이름은 업로드 파일 기준) ===
    const bool sight = BB->EnemyInSight;
    const float los_deg = BB->Los_Degree_Target;      // 목표 기준 시선각(가정: 작을수록 정면)
    const float D = BB->Distance;
    const float AA = BB->MyAspectAngle_Degree;        // 존재 시 사용 (없으면 999로 본다)
    const int   energy_cmp = BB->EnergyCompareResult; // >0: 우세, 0: 동등, <0: 열세

    // === DBFM 진입 창 ===
    // - 시야 확보
    // - 거리: 너무 멀면(>1500) 진입X, 너무 가까우면(예: <200) 별도 방어 스텝 필요
    const bool dist_ok = (D >= 200.0f && D <= 1500.0f);

    /*
    [수정 2026-08-17] los_ok = (los_deg >= 15) 조건 삭제.

    los_deg 는 BB->Los_Degree_Target 이고, CheckSight.cpp:76-79 기준
    **적기 기수와 (적기->나) 벡터 사이의 각**이다. 즉 값이 작을수록 적기가 나를 정조준한
    상태다(내가 적기의 사격선 위에 있다).

    그런데 조건이 los_deg >= 15, 곧 "적기가 나를 정조준하고 있으면 DBFM 진입 금지" 였다.
    방어 기동이 가장 필요한 순간에 방어 분기를 스스로 닫은 것이다. 이때 Fallback 은
    DBFM 을 건너뛰고 NormalTracking -> Task_FollowTarget 으로 떨어지므로, 규정 §6 의
    대미지 콘 안에 들어가 있는 동안 우리는 추격 기동을 하고 있었다.

    Rule.xml 의 DBFM_Branch 주석도 "일반 방어 상황, 가장 넓은 조건" 이다. 의도대로
    거리·시야만 보고 진입하게 되돌린다. 반격 여부는 아래 geom_ok 가 따로 판단한다.
    */

    if (sight && dist_ok)
    {
        BB->BFM = DBFM;

        // === 반격 모드 조건 ===
        // 에너지 우세 + (기하 창) : 너무 가깝지 않고(Anti-overshoot 위험), 각도 과대 아님
        //
        // AA < 60 은 BB 규약(0 = 내가 적기 코앞)에서 "적기가 나를 향하고 있다" = 내가 방어
        // 측이라는 뜻이므로, 롤 리버스 반격의 전제로 옳다. 여기는 손대지 않는다.
        bool geom_ok = (D >= 350.0f && D <= 1000.0f) && (AA < 60.0f);
        BB->IsCounterAttack = (energy_cmp > 0) && geom_ok;

        BT_VLOG("[SetBFMMode_DBFM] t=" << BB->MatchTimeSec() << "s | Enter DBFM"
            << " | D=" << D << ", LOSt=" << los_deg
            << ", AA=" << AA << ", Energy=" << energy_cmp
            << " | Counter=" << (BB->IsCounterAttack ? "YES" : "NO") << "\n");
        return BT::NodeStatus::SUCCESS;
    }

    // 진입 실패 사유 로그
    BT_VLOG("[SetBFMMode_DBFM] t=" << BB->MatchTimeSec() << "s | Blocked"
        << " | sight=" << sight
        << ", D=" << D << ", LOSt=" << los_deg
        << ", AA=" << AA << "\n");
    return BT::NodeStatus::FAILURE;
}
