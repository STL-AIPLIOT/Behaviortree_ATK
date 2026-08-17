#include "SetBFMMode_OBFM.h"
#include "../BTLog.h"
#include <algorithm>

namespace Action {  // ★ 추가

    static inline float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(hi, v));
    }

    BT::NodeStatus SetBFMMode_OBFM::tick() {
        auto bb_ptr = getInput<CPPBlackBoard*>("BB");
        if (!bb_ptr) {
            std::cerr << "[SetBFMMode_OBFM] BB nullptr\n";
            return BT::NodeStatus::FAILURE;
        }
        CPPBlackBoard* BB = bb_ptr.value();

        const bool sight = BB->EnemyInSight;
        const bool e_sup = (BB->EnergyCompareResult > 0);
        const float AA = BB->MyAspectAngle_Degree;
        const float D = BB->Distance;

        /*
        [수정 2026-08-17] AA 규약 불일치 - 진입 조건이 정반대로 걸려 있었다.

        BB->MyAspectAngle_Degree 의 실제 규약은 AspectAngleUpdate.cpp:23-31 이 정한다:
            TPM = (내 위치 - 적기 위치) 를 적기 Up 법선 평면에 투영한 벡터
            AA  = angleBetween(TPM, 적기 ForwardVector)
        즉 **0 = 내가 적기 코앞(적기가 나를 향함), 180 = 내가 적기 6시(공격 위치)** 다.
        Task_AggressiveOBFM.h:35-41 도 같은 결론을 적어 두고 BB 값을 안 쓰고 자체 계산한다.

        그런데 여기는 aa_ok = (AA < 35) 이었다. 이건 "적기가 나를 정면으로 향한" 상황,
        즉 헤드온/피추격 기하에서 공격 기동(OBFM)에 들어간다는 뜻이다. 반대로
        SetBFMMode_HABFM 은 |AA-180| <= 40, 곧 내가 적기 6시를 잡은 공격 기하에서
        정면교전(1C/2C)을 걸고 있었다. 두 게이트가 서로 바뀐 셈이다.

        원인은 규약이 세 갈래로 갈린 것이다. Docs(GeoMathUtil) 는 "AA 0 = 적기 6시",
        BB 구현은 그 반대, HABFM 주석은 앵글오프("0=같은 방향, 180=헤드온") 를 쓰고 있었다.
        여기서는 BB 구현 규약 하나로 통일한다.

        OBFM = 내가 적기 후방 반구를 잡은 상태 -> AA 가 180 에 가까워야 한다.
        */
        const bool aa_ok = (AA > 145.0f);
        const bool dist_ok = (D >= 150.0f && D <= 1500.0f);  // [회전1.5] 하한 400->150: WEZ_MIN_M(152.4m) 아래로 내려 유효 사격 구간 전체에서 OBFM 지원 유지

        if (sight && e_sup && aa_ok && dist_ok) {
            BB->BFM = OBFM;
            BT_VLOG("[SetBFMMode_OBFM] t=" << BB->MatchTimeSec() << "s | Enter OBFM (AA=" << AA
                << ", D=" << D << ", EnergySup=" << e_sup << ")\n");
            return BT::NodeStatus::SUCCESS;
        }

        BT_VLOG("[SetBFMMode_OBFM] t=" << BB->MatchTimeSec() << "s | Blocked: sight=" << sight
            << ", e_sup=" << e_sup
            << ", AA=" << AA
            << ", D=" << D << "\n");
        return BT::NodeStatus::FAILURE;
    }

} // namespace Action
