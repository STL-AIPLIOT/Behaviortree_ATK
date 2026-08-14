#pragma once

#include "../../behaviortree_cpp_v3/action_node.h"
#include "../BlackBoard/CPPBlackBoard.h"
#include <iostream>
#include <string>

namespace Action
{
    /*
    공격형 OBFM 단일 노드.

    설계 원칙
    ---------
    "거리·각도·에너지를 동시에 보존한다." 에너지를 extend(이탈)로 지키지 않고,
    각이 벌어졌을 때 추격 강도를 낮춰 in-fight 로 지킨다. 세 라운드 모두 완전 이탈은 없다
    (ALLOW_EXTEND = false). 공격형의 유일한 치명 실패는 오버슛 -> 역할 역전이므로,
    오버슛은 뒤로 빼기가 아니라 하이 요요(수직으로 빼서 공격위치 유지)로 처리한다.

    출력 계약
    ---------
    BB->VP_Cartesian 과 BB->Throttle(0~1) 만 쓴다. 다른 BB 필드는 읽기 전용이다.
    내 담당 상황이 아니면 반드시 FAILURE 를 반환해 Fallback 의 다음 자식
    (Task_AntiOvershoot / Task_CornerLeadPursuit / Task_LeadPursuit / Task_FollowTarget)
    으로 넘긴다.

    왜 진입 게이트(ENTRY_*)가 필요한가
    ----------------------------------
    행동 티어의 CONSERVE 는 |ATA| >= 3.0 deg 전부, 즉 3~180 deg 를 덮는다.
    이 노드는 OBFM_Action Fallback 의 첫 자식이므로, 게이트가 없으면 항상 SUCCESS 가 되어
    뒤의 자식들이 한 번도 실행되지 않는다(Task_AntiOvershoot 이 예전에 겪은 것과 같은 형태의
    starvation 이다 - Task_AntiOvershoot.cpp:22-30 주석 참조). 그래서 시야/거리/ATA 로
    "내가 맡을 상황"을 좁히고, 밖이면 양보한다.

    각도 규약 주의
    --------------
    ATA 는 블랙보드에 없다. MyForwardVector 와 LOS(=Target-My) 로 노드 안에서 acos 로 만든다.
    AA 는 BB->MyAspectAngle_Degree 가 있지만 그 값은 **적기 기수 기준**이다
    (AspectAngleUpdate.cpp:23-29: 0 = 내가 적기 코앞, 180 = 내가 적기 6시).
    Docs 의 "AA 0 = 적기 6시" 와 반대 규약이므로 BB 값에 의존하지 않고 노드가 직접 계산한다.
    이 노드에서 aa_nose = 0 은 "적기가 나를 향하고 있다"(정면), 180 은 "내가 적기 6시"다.

    측정 (종료선)
    -------------
    STIL_AGGR_CSV 에 경로를 주면 tick 단위 CSV 를 남긴다. 미설정 시 완전 비활성이라
    시뮬레이션 동작에 영향이 없다(PredictManeuverCsvLogger / LeadPursuitTelemetry 와 같은 방식).
      핵심 지표 (a) gun_window = (WEZ_MIN_M < D < WEZ_MAX_M) && (ata <= DAMAGE_ATA_DEG) 인 tick 수
      핵심 지표 (b) 피격/역할역전 end_condition 이 함께 늘지 않았는가 (summary.json 쪽)
    gun_window 는 update_damage() 의 실제 판정(|ATA| <= wez.angle_deg/2 = 1 deg)과 같은 값으로만
    집계한다. 행동 티어의 1.5/2.5/3.0 deg 는 "행동 강도"일 뿐 데미지 판정이 아니다.
    */
    class Task_AggressiveOBFM : public BT::SyncActionNode
    {
    public:
        Task_AggressiveOBFM(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config) {}

        static BT::PortsList providedPorts()
        {
            return { BT::InputPort<CPPBlackBoard*>("BB") };
        }

        BT::NodeStatus tick() override;

        // ================= 라운드 프로파일 =================
        /*
        라운드 신호는 코드·블랙보드·DLL export 어디에도 없다(2026-08-13 확인).
          - CPPBlackBoard.h 에 라운드/매치 인덱스 필드 없음
          - native_bt.py:106 CreateBehaviorTree(my_id, my_force_id) - 라운드 인자 없음
        그래서 컴파일 타임 상수로 둔다. 기본은 R12 다.
        재빌드 없이 3R 을 켜야 할 때만 환경변수 STIL_ROUND_PROFILE=R3 로 덮어쓴다
        (BT_RULE_XML / BT_DIAG_LOG 와 같은 방식이며, 미설정 시 동작 불변).
        */
        enum RoundProfile
        {
            ROUND_R12,   // 1R, 2R: 에너지 바닥 존중. CONSERVE 를 약간 더 보수적으로
            ROUND_R3     // 3R   : 열세 + 정면이면 이탈 대신 정면교전으로 커밋(무승부를 정면으로 깬다)
        };
        static constexpr RoundProfile ROUND_PROFILE = ROUND_R12;

        // 세 라운드 공통. 완전 이탈은 하지 않는다.
        static constexpr bool ALLOW_EXTEND = false;

        // ================= 튜닝 상수 (지정값) =================
        static constexpr float D_SWEET_AGGR = 300.0f;   // 공격형이 유지하려는 거리 [m]
        static constexpr float OVERSHOOT_CLOSURE = 18.0f;    // 하이 요요 발동 접근률 [m/s]
        static constexpr float OVERSHOOT_D = 350.0f;   // 하이 요요 발동 거리 [m]
        static constexpr float H_YOYO = 180.0f;   // 하이 요요 수직 오프셋 [m]
        static constexpr float K_MUZZLE = 300.0f;   // lead 비행시간 계산용 유효 탄속 가산 [m/s]
        static constexpr float TURN_IN_BIAS = 180.0f;   // PURSUE 에서 리드점을 더 당기는 횡방향 바이어스 [m]
        static constexpr float ALLOUT_ATA = 1.5f;     // ALL_OUT 진입 ATA [deg]
        static constexpr float TIER_UP_ATA = 2.5f;     // 티어 상승 임계 [deg]
        static constexpr float TIER_DOWN_ATA = 3.0f;     // 티어 하강 임계 [deg]

        // ================= 진입 게이트 =================
        // 이 밖이면 FAILURE 로 양보한다. 위의 "왜 진입 게이트가 필요한가" 참조.
        static constexpr float ENTRY_ATA_MAX_DEG = 60.0f;
        static constexpr float ENTRY_D_MIN_M = 30.0f;
        static constexpr float ENTRY_D_MAX_M = 2500.0f;

        // ================= 파생 튜닝 상수 =================
        static constexpr float LEAD_TIME_MIN_SEC = 0.2f;
        static constexpr float LEAD_TIME_MAX_SEC = 2.0f;
        static constexpr float PURSUE_LEAD_GAIN = 1.3f;     // PURSUE 는 리드를 조금 더 앞에 찍는다
        static constexpr float CONSERVE_LAG_GAIN = 0.15f;    // 약한 lag = 거리의 15%
        static constexpr float CONSERVE_LAG_MAX_M = 250.0f;   // lag 상한. 이 이상은 사실상 extend 다
        static constexpr float R12_CONSERVE_LAG_MUL = 1.2f;     // R12: lag 를 더 줘 선회 G 를 낮춘다
        static constexpr float HEADON_AA_DEG = 60.0f;    // R3 정면 커밋 판정 (적기 기수 기준)

        // 스로틀
        static constexpr float THR_ALLOUT_BASE = 0.95f;
        static constexpr float THR_PURSUE_BASE = 0.85f;
        static constexpr float THR_CONSERVE_BASE = 0.55f;
        static constexpr float THR_YOYO_BASE = 0.45f;
        static constexpr float THR_HEADON_COMMIT = 0.90f;
        static constexpr float THR_CLOSURE_GAIN = 0.02f;    // 접근률 초과분 1 m/s 당 스로틀 감소
        static constexpr float THR_ALLOUT_GAIN = 0.01f;    // ALL_OUT 은 절반만 반응(각을 놓치지 않는다)
        static constexpr float R12_CONSERVE_THR_BIAS = 0.05f;    // R12: CONSERVE 를 더 보수적으로
        static constexpr float CLOSURE_TARGET_MS = 10.0f;    // 스위트 거리 밖에서 허용하는 접근률

        // ================= WEZ (로깅 전용, 절대 넓히지 않는다) =================
        // update_damage() 는 |ATA| <= wez.angle_deg/2 = 1.0 deg 에서만 데미지를 준다.
        static constexpr float WEZ_MIN_M = 152.4f;   // 500 ft
        static constexpr float WEZ_MAX_M = 914.4f;   // 3000 ft
        static constexpr float DAMAGE_ATA_DEG = 1.0f;     // 실효 데미지 콘. 각도 epsilon 없음

    private:
        enum Tier
        {
            TIER_ALL_OUT,
            TIER_PURSUE,
            TIER_CONSERVE
        };

        static const char* TierName(Tier t);

        // 히스테리시스용. 버퍼 구간(2.5~3.0 deg)에서는 직전 티어를 유지해 채터링을 막는다.
        // 담당 상황을 벗어나(FAILURE) 다시 들어올 때는 보수적으로 시작한다.
        Tier prev_tier_ = TIER_CONSERVE;
    };
}
