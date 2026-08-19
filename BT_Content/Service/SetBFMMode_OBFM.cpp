#include "SetBFMMode_OBFM.h"
#include "../STIL_Tuning.h"
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
        // ATA = 내 기수가 표적을 향한 각. CheckSight.cpp:39 가 채운다.
        const float ATA = BB->Los_Degree;
        // dE = (E_own - E_tgt)/2g [m]. EnergyCompare.cpp 가 채운다.
        const float dE  = BB->SpecificEnergyDelta_M;

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
        /*
        [C/v4 2026-08-19] 진입 게이트를 AA 축에서 ATA 축으로 교체한다.

        왜 AA 를 버리는가 - 2026-08-18 진단 로그 실측:
        sight && 150<=D<=1500 인 5,051 tick 중 AA>145 는 0.00% 였다.
        원거리에서는 한쪽이 상대 뒤를 잡지만(AA~180) 1500m 안으로 들어오면 항상
        정면이 된다. "공격 각도"와 "근거리"가 구조적으로 공존하지 않으므로
        AA 기반 진입은 어떤 임계로도 성립하지 않는다.
        ATA(내 기수가 표적을 향한 각)는 접근 구간에서도 유효하다.

        에너지는 차단이 아니라 완화 조건이다. e_sup 필수 조건을 제거한다.
        자기대전에서 두 기체 에너지는 사실상 같아 부호가 계속 뒤집히고,
        열세인 쪽은 영원히 OBFM 에 못 들어갔다. 대신 dE 크기를 보되
        근접(600m)에서는 열세여도 진입한다 - 그 거리에서 물러나는 것이 더 위험하다.

        구식 복원: STIL_OBFM_LEGACY_GATE=1
        */
        if (STIL::ObfmLegacyGate())
        {
            const bool aa_ok_legacy = (AA > 145.0f);
            const bool dist_ok_legacy = (D >= 150.0f && D <= 1500.0f);
            if (sight && e_sup && aa_ok_legacy && dist_ok_legacy) {
                BB->BFM = OBFM;
                BT_VLOG("[SetBFMMode_OBFM] t=" << BB->MatchTimeSec()
                    << "s | Enter OBFM (legacy) AA=" << AA << ", D=" << D << "\n");
                return BT::NodeStatus::SUCCESS;
            }
            BT_VLOG("[SetBFMMode_OBFM] t=" << BB->MatchTimeSec()
                << "s | Blocked(legacy): sight=" << sight << ", e_sup=" << e_sup
                << ", AA=" << AA << ", D=" << D << "\n");
            return BT::NodeStatus::FAILURE;
        }

        const bool dist_ok   = (D >= STIL::ObfmDMin() && D <= STIL::ObfmDMax());
        const bool ata_ok    = (ATA <= STIL::ObfmAtaMax());
        // 에너지 열세여도 근접이면 통과시킨다.
        const bool energy_ok = (dE >= STIL::ObfmDeMin()) || (D <= STIL::ObfmDClose());

        if (sight && dist_ok && ata_ok && energy_ok) {
            BB->BFM = OBFM;
            BT_VLOG("[SetBFMMode_OBFM] t=" << BB->MatchTimeSec() << "s | Enter OBFM (ATA=" << ATA
                << ", D=" << D << ", dE=" << dE << ")\n");
            return BT::NodeStatus::SUCCESS;
        }

        // 어느 축에서 탈락했는지 남긴다 - 단계별 실험의 귀속에 쓴다.
        BT_VLOG("[SetBFMMode_OBFM] t=" << BB->MatchTimeSec() << "s | Blocked: sight=" << sight
            << ", dist_ok=" << dist_ok
            << ", ata_ok=" << ata_ok
            << ", energy_ok=" << energy_ok
            << " | ATA=" << ATA << ", AA=" << AA << ", D=" << D << ", dE=" << dE << "\n");
        return BT::NodeStatus::FAILURE;
    }

} // namespace Action
