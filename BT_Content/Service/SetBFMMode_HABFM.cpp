#include "SetBFMMode_HABFM.h"
#include "../STIL_Tuning.h"
#include "../BTLog.h"
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace Action;

namespace
{
    constexpr double TURN_RATE_MARGIN_DEG_SEC = 2.0;        // 1C/2C 전환 데드밴드 (deg/s)
    constexpr double TURN_RATE_SAMPLE_SEC = 0.2;            // 선회율 계산용 최소 샘플 구간 (sec)
    constexpr double MAX_TURN_RATE_SAMPLE_SEC = 1.0;        // 샘플 구간 상한. 다른 BFM 분기로 tick이 끊긴 뒤의 오래된 헤딩 비교 방지 (sec)
    constexpr double MIN_VALID_DT_SEC = 1e-6;               // 0 나눗셈 방지용 최소 dt (sec)
    constexpr double MAX_VALID_TURN_RATE_DEG_SEC = 90.0;    // 비정상 선회율(수직 기동 시 Yaw 급변 등) 배제용 상한

    // 각도 차이를 [-180, 180]으로 접음 (0~360 / -180~180 표기 모두 대응)
    double WrapDeltaDegree(double delta)
    {
        while (delta > 180.0)  delta -= 360.0;
        while (delta < -180.0) delta += 360.0;
        return delta;
    }

    // 헤딩 변화량(deg)과 경과 시간(sec)으로 선회율(deg/s)을 계산. 유효하지 않으면 false
    bool CalculateTurnRateDegPerSec(double headingDeltaDegree, double dtSec, double& outputDegPerSec)
    {
        if (!std::isfinite(headingDeltaDegree) || !std::isfinite(dtSec) || dtSec <= MIN_VALID_DT_SEC)
        {
            return false;
        }

        outputDegPerSec = std::abs(WrapDeltaDegree(headingDeltaDegree)) / dtSec;

        return std::isfinite(outputDegPerSec) && outputDegPerSec <= MAX_VALID_TURN_RATE_DEG_SEC;
    }
}

/*
내 기체 / 타겟의 Yaw(deg) 변화량을 시뮬레이션 시간으로 나누어 선회율(deg/s)을 갱신
- 블랙보드에는 선회 반경/각속도 값이 없으므로 헤딩 변화 / dt 방식만 사용
- 값이 비정상이거나 샘플 구간이 모자라면 직전 선회율을 그대로 유지
*/
void SetBFMMode_HABFM::UpdateTurnRates(CPPBlackBoard* BB)
{
    // [수정 2026-08-17] RunningTime -> MatchTimeSec().
    // RunningTime 은 호스트가 SetBehaviorTreeDeltaTime 을 부르지 않으면 블랙보드 기본값을
    // 누적한다. 그 기본값이 실제 프레임 간격의 1/3 이라 dt 가 1/3 로 들어왔고, 선회율이
    // 3배로 부풀려져 1C/2C 선택이 어긋났다. 아래 임계값들은 모두 '실제 초' 기준이다.
    const double now = BB->MatchTimeSec();
    const double myYaw = BB->MyRotation_EDegree.Yaw;
    const double targetYaw = BB->TargetRotation_EDegree.Yaw;

    if (!std::isfinite(now) || !std::isfinite(myYaw) || !std::isfinite(targetYaw))
    {
        return;     //입력이 비정상이면 상태를 건드리지 않음
    }

    const double dt = now - PrevSampleTime_Sec;

    //첫 tick이거나, 시뮬레이션 시간이 되감겼거나, tick이 오래 끊겼던 경우 기준점만 다시 잡음
    if (!HasPrevHeading || dt < 0.0 || dt > MAX_TURN_RATE_SAMPLE_SEC)
    {
        HasPrevHeading = true;
        PrevSampleTime_Sec = now;
        PrevMyYaw_Degree = myYaw;
        PrevTargetYaw_Degree = targetYaw;
        return;
    }

    if (dt < TURN_RATE_SAMPLE_SEC)
    {
        return;     //샘플 구간 미달. 직전 선회율 유지
    }

    double myRateDeg = 0.0;
    double targetRateDeg = 0.0;

    const bool valid =
        CalculateTurnRateDegPerSec(myYaw - PrevMyYaw_Degree, dt, myRateDeg)
        &&
        CalculateTurnRateDegPerSec(targetYaw - PrevTargetYaw_Degree, dt, targetRateDeg);

    PrevSampleTime_Sec = now;
    PrevMyYaw_Degree = myYaw;
    PrevTargetYaw_Degree = targetYaw;

    if (!valid)
    {
        HasTurnRate = false;    //유효하지 않은 값은 판단에 쓰지 않음 (이전 모드 유지)
        return;
    }

    MyTurnRate_DegSec = myRateDeg;
    TargetTurnRate_DegSec = targetRateDeg;
    HasTurnRate = true;
}

/*
선회율 비교로 1C/2C 확정
- 내 선회율 우세      : 2C
- 타겟 선회율 우세    : 1C
- 차이가 데드밴드 이내: 이전 모드 유지
- 선회율이 유효하지 않으면 이전 모드 유지 (강제 전환 금지)
*/
void SetBFMMode_HABFM::UpdateCircleMode(CPPBlackBoard* BB)
{
    if (!HasTurnRate)
    {
        return;
    }

    if (MyTurnRate_DegSec >= TargetTurnRate_DegSec + TURN_RATE_MARGIN_DEG_SEC)
    {
        BB->HABFM_CircleMode = TWO_CIRCLE;
    }
    else if (TargetTurnRate_DegSec >= MyTurnRate_DegSec + TURN_RATE_MARGIN_DEG_SEC)
    {
        BB->HABFM_CircleMode = ONE_CIRCLE;
    }
    //데드밴드 이내면 BB->HABFM_CircleMode를 그대로 둠
}

BT::NodeStatus SetBFMMode_HABFM::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res)
    {
        std::cerr << "[SetBFMMode_HABFM] BB nullptr\n";
        return BT::NodeStatus::FAILURE;
    }
    CPPBlackBoard* BB = bb_res.value();

    //선회율 샘플링은 HABFM 진입 여부와 무관하게 매 tick 갱신
    UpdateTurnRates(BB);

    const bool sight = BB->EnemyInSight;

    /*
    [수정 2026-08-17] AA 규약 정정. 아래 주석이 틀려 있었고 조건도 반대로 걸려 있었다.

    이전 주석 "0: 같은 방향, 180: 정반대(헤드온)" 은 앵글오프(HCA) 규약이다.
    실제 BB->MyAspectAngle_Degree 는 AspectAngleUpdate.cpp:23-31 이 만드는 **애스펙트각**으로
        0   = 내가 적기 코앞 (적기 기수가 나를 향함) = 헤드온
        180 = 내가 적기 6시                          = 공격 위치
    이다. 따라서 |AA-180| <= 40 (= AA 140~180) 은 헤드온이 아니라 **내가 적기 후방을 잡은**
    상황이었고, 하필 그때 1-circle/2-circle 머지 선회를 걸고 있었다. 애써 만든 공격 기하를
    헤드온 기동으로 되돌려 버린 셈이다. SetBFMMode_OBFM 의 게이트와 정확히 뒤바뀌어 있어
    그쪽도 함께 정정했다.

    두 기체가 서로 마주 접근하는지는 애스펙트각만으로 부족하다(적기 정면에 있어도 내가
    등을 돌리고 있을 수 있다). 기수 교차각 BB->MyAngleOff_Degree 를 함께 본다
    (AngleOffUpdate.cpp:20 — 두 ForwardVector 사이 각, 180 = 정면 대향).
    */
    const double aa = BB->MyAspectAngle_Degree;   // 0 = 적기 정면(헤드온), 180 = 적기 6시
    const double ao = BB->MyAngleOff_Degree;      // 0 = 같은 방향, 180 = 기수 정반대
    const double D = BB->Distance;
    const int    ec = BB->EnergyCompareResult;    // >=0 권장

    // HABFM 진입 창: 적기 정면 반구(AA <= 40) + 기수 대향(AO >= 140),
    //                800m <= D <= 2000m, 에너지 >= 0, 시야 必
    const bool aa_ok = (aa <= 40.0) && (ao >= 140.0);
    /*
    [B/v4 2026-08-19] D 하한 800 -> 1200 (STIL_HABFM_DMIN).

    v3p 실패의 직접 원인이 이 창이었다. WEZ 는 152~914m 인데 HABFM 이 800m 부터
    열려 있어, 사격 진입로(WEZ 바로 바깥)를 1C/2C 선회가 덮었다. 기수를 정렬해야 할
    구간에서 선회가 잡히니 사격 자세가 만들어지지 않았다.

    하한을 1200 으로 올려 800~1200m 를 비운다. Rule.xml 의 분기 순서상
    (SCISSORS -> HABFM -> OBFM -> DBFM) HABFM 이 실패하면 그 구간은 자연히
    OBFM 이 가져간다 - 별도 배선 없이 C 와 맞물린다.

    상한 2000 과 (AA <= 40 && AO >= 140) 은 그대로다. 이 변경 하나만으로
    실험 2단계를 돌릴 수 있어야 하므로 다른 것을 건드리지 않는다.
    구식 복원: STIL_HABFM_DMIN=800
    */
    const bool dist_ok = (D >= STIL::HabfmDMin() && D <= 2000.0);
    const bool e_ok = (ec >= 0);

    if (sight && aa_ok && dist_ok && e_ok)
    {
        BB->BFM = HABFM;
        UpdateCircleMode(BB);
        BT_VLOG("[SetBFMMode_HABFM] t=" << BB->MatchTimeSec() << "s | Enter HABFM | AA=" << aa
            << ", AO=" << ao << ", D=" << D << ", E=" << ec
            << " | myTR=" << MyTurnRate_DegSec << ", tgtTR=" << TargetTurnRate_DegSec
            << ", Circle=" << (BB->HABFM_CircleMode == ONE_CIRCLE ? "1C" :
                              (BB->HABFM_CircleMode == TWO_CIRCLE ? "2C" : "NONE")) << "\n");
        return BT::NodeStatus::SUCCESS;
    }

    BT_VLOG("[SetBFMMode_HABFM] t=" << BB->MatchTimeSec() << "s | Blocked | sight=" << sight
        << ", AA=" << aa << ", AO=" << ao << ", D=" << D << ", E=" << ec << "\n");
    return BT::NodeStatus::FAILURE;
}
