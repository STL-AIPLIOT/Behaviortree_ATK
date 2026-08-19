#pragma once

/*
STIL 튜닝 파라미터 단일 지점.

왜 한 헤더에 모으는가 — v3p 실패의 교훈
---------------------------------------
v3p 는 HABFM 진입창(800~2000m)이 WEZ(152~914m) 바로 바깥의 사격 진입로를 덮어,
기수 정렬 구간에서 1C/2C 선회가 잡히는 것이 원인이었다. 여러 변경이 한 번에 들어가
어느 것이 원인인지 귀속할 수 없었던 것이 더 큰 문제였다.

그래서 이번 v4 는 모든 신규/변경 파라미터를 여기 모으고, 각각을 환경변수로
런타임 오버라이드할 수 있게 한다. 재빌드 없이 단계별 실험이 가능하다:

    $env:STIL_HABFM_DMIN = "800"      # B 를 끄고 구식 동작으로
    $env:STIL_TIER_MATRIX = "0"       # D 를 끄고 ATA 단독 티어로

기본값은 v4 신규 값이다. 각 파라미터의 "구식 동작 복원값" 은 주석에 적어 둔다.

주의: getenv 는 프로세스당 1회만 읽고 캐시한다. 규정 §4 가 60Hz 프레임당
0.1667초를 넘기면 페널티인데, 매 tick getenv 를 부르면 그 예산을 갉아먹는다.
*/

#include <cstdlib>
#include <cstring>
#include <string>

namespace STIL
{
    // 프로세스당 1회 파싱 후 캐시. 호출부가 static 지역변수로 붙잡는 것을 전제한다.
    inline double ParseTunable(const char* env_name, double default_value)
    {
        const char* raw = std::getenv(env_name);
        if (raw == nullptr || raw[0] == '\0') { return default_value; }
        try
        {
            return std::stod(std::string(raw));
        }
        catch (...)
        {
            // 오타 하나로 조용히 엉뚱한 값이 들어가는 것보다 기본값이 낫다.
            return default_value;
        }
    }

    /*
    사용법:
        const double dmin = STIL_TUNABLE("STIL_HABFM_DMIN", 1200.0);
    매크로인 이유: static 지역변수를 호출 지점마다 하나씩 만들어 캐시하기 위함.
    함수로 만들면 호출부가 매번 캐시를 직접 선언해야 한다.
    */
    #define STIL_TUNABLE(env_name, default_value) \
        ([]() -> double { static const double v = ::STIL::ParseTunable(env_name, (default_value)); return v; }())

    #define STIL_TUNABLE_F(env_name, default_value) \
        (static_cast<float>(STIL_TUNABLE(env_name, (default_value))))

    #define STIL_TUNABLE_ON(env_name, default_on) \
        (STIL_TUNABLE(env_name, (default_on) ? 1.0 : 0.0) != 0.0)

    // ---- B. HABFM 진입창 --------------------------------------------------
    // 사격 진입로(WEZ 152~914m 바로 바깥)를 비우기 위해 하한을 올린다.
    // 구식 복원: STIL_HABFM_DMIN=800
    inline float HabfmDMin()          { return STIL_TUNABLE_F("STIL_HABFM_DMIN", 1200.0); }

    // ---- C. OBFM 진입 게이트 ----------------------------------------------
    // AA 축을 제거하고 ATA + 거리 + 에너지 완화로 대체한다.
    // 구식 복원: STIL_OBFM_D_MAX=1500, STIL_OBFM_ATA_MAX=180(=사실상 무효화 후 AA 축 복원 필요)
    inline float ObfmDMin()           { return STIL_TUNABLE_F("STIL_OBFM_D_MIN",  150.0); }
    inline float ObfmDMax()           { return STIL_TUNABLE_F("STIL_OBFM_D_MAX", 2500.0); }
    inline float ObfmAtaMax()         { return STIL_TUNABLE_F("STIL_OBFM_ATA_MAX", 60.0); }
    inline float ObfmDeMin()          { return STIL_TUNABLE_F("STIL_OBFM_DE_MIN", -300.0); }
    inline float ObfmDClose()         { return STIL_TUNABLE_F("STIL_OBFM_D_CLOSE", 600.0); }
    // 1 이면 구식 게이트(AA>145 && EnergyCompareResult>0 필수)로 되돌린다.
    inline bool  ObfmLegacyGate()     { return STIL_TUNABLE_ON("STIL_OBFM_LEGACY_GATE", false); }

    // ---- D. 티어 매트릭스 --------------------------------------------------
    // 0 이면 ATA 단독 티어(현행) 유지.
    inline bool  TierMatrix()         { return STIL_TUNABLE_ON("STIL_TIER_MATRIX", true); }
    // dE 밴드 경계 [m]. |dE| 가 이 값 이하이면 E0.
    inline float EnergyBand()         { return STIL_TUNABLE_F("STIL_ENERGY_BAND", 150.0); }
    // 에너지 열세에서 ALL_OUT 을 유지할 수 있는 연속 시간 [s]. 초과 시 CONSERVE 강등.
    inline float AllOutELowMaxSec()   { return STIL_TUNABLE_F("STIL_ALLOUT_ELOW_MAX_S", 3.0); }
    // CONSERVE + 에너지 열세에서 코너속도를 사기 위해 내려가는 스로틀 하한.
    inline float ConserveThrFloor()   { return STIL_TUNABLE_F("STIL_CONSERVE_THR_FLOOR", 0.30); }
    // 이 거리를 넘으면 사격 판정을 시도하지 않는다(접근·각 만들기 전용). 3000ft.
    inline float GunMaxRange()        { return STIL_TUNABLE_F("STIL_GUN_MAX_RANGE", 914.4); }
    // 표적 속도 벡터가 나를 향하는 정면 상황에서는 리드점을 쓰지 않는다.
    // 판정 임계 [deg]: 표적 기수와 "표적->나" 벡터의 사잇각이 이보다 작으면 정면.
    inline float LeadHeadOnDeg()      { return STIL_TUNABLE_F("STIL_LEAD_HEADON_DEG", 30.0); }
    inline bool  LeadHeadOnGuard()    { return STIL_TUNABLE_ON("STIL_LEAD_HEADON_GUARD", true); }

    // ---- E. 지면 회피 롤 수평화 --------------------------------------------
    // 0 이면 현행(즉시 상승 VP)으로 되돌린다.
    inline bool  RollLevelFirst()     { return STIL_TUNABLE_ON("STIL_ROLL_LEVEL_FIRST", true); }
    inline float RollLevelMaxBank()   { return STIL_TUNABLE_F("STIL_ROLL_LEVEL_MAX_BANK", 45.0); }
}
