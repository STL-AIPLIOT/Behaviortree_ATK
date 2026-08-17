#pragma once

/*
대회 규정 §6 "대미지 룰" 구현.

무엇이 문제였나
---------------
이 트리는 WEZ 를 |ATA| <= 1.0deg, 152.4~914.4m 하나로 고정하고 있었다
(Task_AggressiveOBFM.h 의 DAMAGE_ATA_DEG / WEZ_MIN_M / WEZ_MAX_M, 주석 "절대 넓히지 않는다").
그 값은 **학습 환경(DogFightEnv)의 update_damage()** 기준으로는 맞다.
config.py 의 wez.angle_deg = 2.0 이고 판정이 angle_deg/2 이므로 1.0deg, 거리 500~3000ft.

그러나 **대회 판정은 경과 시간에 따라 완화된다**(규정 §6).

    Phase 1   0~100s    LOS < 1deg   500~3000ft   계수 1.0
    Phase 2 100~150s    LOS < 2deg   500~3500ft   계수 0.3
    Phase 3 150~200s    LOS < 3deg   500~4000ft   계수 0.1

    "하위 Phase 범위 안에 적기가 있으면 하위 Phase 의 (더 큰) 대미지 적용"
    -> 시각 t 의 실효 계수는 그 시점까지 열린 모든 창 중 **가장 큰 계수**다.
       Phase 3 시점에 1deg/2500ft 를 물면 계수는 0.1 이 아니라 1.0 이다.

콘 부피비는 각도만 1 : 2.72 : 5.34, 거리까지 포함하면 1 : 6.36 : 21.37 이다.
1deg 창만 노리는 로직은 100초 이후 사실상 20배 넓어진 판정면을 통째로 버린다.

시간 기준
---------
경과 시간은 BB->MatchTimeSec() 를 쓴다. BB->RunningTime 이 아니다.
RunningTime 은 DeltaSecond 누적인데 RL 호스트(native_bt.py)가 SetBehaviorTreeDeltaTime 을
바인딩조차 하지 않아 CPPBlackBoard 생성자 기본값이 그대로 쌓인다. 그 기본값이
0.005566170 이라 실제의 약 1/3 로 흘렀고, 그대로 두면 200초 경기가 끝날 때까지
Phase 1 에서 벗어나지 못한다. MatchTimeSec() 는 규정 §4 의 60 Hz 고정 프레임을 근거로
tick 수에서 직접 만든다.

로컬 검증 호환
--------------
STIL_WEZ_MODE=training 을 주면 Phase 완화를 끄고 DogFightEnv update_damage() 와 동일한
고정 창(1deg / 152.4~914.4m, 계수 1.0)만 쓴다. run_local_dogfight 결과를 이전 판과
같은 잣대로 비교해야 할 때 사용한다. 미설정 시 기본은 대회 규정(Phase)이다.
*/

#include <algorithm>
#include <cmath>      // std::abs(float) — <cstdlib> 만 있으면 int 오버로드가 잡혀 각도가 잘린다
#include <cstdlib>
#include <string>

namespace WezPhase
{
    constexpr float FEET_TO_METER = 0.3048f;

    // 모든 Phase 공통 최소 거리 (500 ft). 이 안쪽은 어느 Phase 에서도 판정되지 않는다.
    constexpr float MIN_RANGE_M = 500.0f * FEET_TO_METER;   // 152.4

    struct Window
    {
        float max_ata_deg;
        float max_range_m;
        float coeff;
    };

    // 규정 §6 표. 인덱스 0 = Phase 1.
    constexpr Window kPhase[3] = {
        { 1.0f, 3000.0f * FEET_TO_METER, 1.0f },   //  914.4 m
        { 2.0f, 3500.0f * FEET_TO_METER, 0.3f },   // 1066.8 m
        { 3.0f, 4000.0f * FEET_TO_METER, 0.1f },   // 1219.2 m
    };

    constexpr double kPhase2StartSec = 100.0;
    constexpr double kPhase3StartSec = 150.0;

    // 학습 환경 고정 창 (DogFightEnv update_damage 와 동일)
    constexpr Window kTrainingWindow = { 1.0f, 3000.0f * FEET_TO_METER, 1.0f };

    inline bool TrainingModeFixed()
    {
        static const bool fixed_window = [] {
            const char* env = std::getenv("STIL_WEZ_MODE");
            if (!env || !*env) { return false; }
            const std::string v(env);
            return v == "training" || v == "TRAINING" || v == "fixed";
        }();
        return fixed_window;
    }

    // 1, 2, 3
    inline int PhaseAt(double match_time_sec)
    {
        if (TrainingModeFixed()) { return 1; }
        if (match_time_sec >= kPhase3StartSec) { return 3; }
        if (match_time_sec >= kPhase2StartSec) { return 2; }
        return 1;
    }

    /*
    지금 열려 있는 창 중 **가장 넓은** 것.
    "어디까지 붙어서 어디까지 각을 허용할 것인가" 같은 진입 게이트 판단에 쓴다.
    사격 가치 판단에는 BestCoeff() 를 써야 한다 - 넓은 창은 계수가 작다.
    */
    inline Window WidestOpen(double match_time_sec)
    {
        if (TrainingModeFixed()) { return kTrainingWindow; }
        return kPhase[PhaseAt(match_time_sec) - 1];
    }

    /*
    가장 가치 있는(= 계수가 가장 큰) 창. 조준 목표는 언제나 이쪽이다.
    Phase 가 몇이든 1deg/3000ft 를 물면 계수 1.0 이므로 이 함수는 항상 Phase 1 창을 돌려준다.
    이름을 분리해 둔 이유는 호출부에서 "넓은 창"과 "비싼 창"을 헷갈리지 않게 하기 위함이다.
    */
    inline Window RichestOpen(double /*match_time_sec*/)
    {
        return kPhase[0];
    }

    /*
    실효 대미지 계수. 열려 있는 창 전부를 훑어 **가장 큰 계수**를 돌려준다(규정 §6 단서).
    어느 창에도 들지 않으면 0.
    */
    inline float BestCoeff(double match_time_sec, float ata_deg, float range_m)
    {
        const float ata = std::abs(ata_deg);
        if (range_m < MIN_RANGE_M) { return 0.0f; }

        if (TrainingModeFixed())
        {
            const bool hit = (ata <= kTrainingWindow.max_ata_deg)
                && (range_m <= kTrainingWindow.max_range_m);
            return hit ? kTrainingWindow.coeff : 0.0f;
        }

        const int open = PhaseAt(match_time_sec);
        float best = 0.0f;
        for (int i = 0; i < open; ++i)
        {
            if (ata <= kPhase[i].max_ata_deg && range_m <= kPhase[i].max_range_m)
            {
                best = std::max(best, kPhase[i].coeff);
            }
        }
        return best;
    }

    inline bool InAnyWindow(double match_time_sec, float ata_deg, float range_m)
    {
        return BestCoeff(match_time_sec, ata_deg, range_m) > 0.0f;
    }
}
