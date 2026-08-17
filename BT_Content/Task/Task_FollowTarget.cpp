#include "Task_FollowTarget.h"
#include "../BTLog.h"

using namespace BT_Geometry;

namespace Action
{
    BT::NodeStatus Task_FollowTarget::tick()
    {
        auto bb_opt = getInput<CPPBlackBoard*>("BB");
        if (!bb_opt) return BT::NodeStatus::FAILURE;

        CPPBlackBoard* BB = bb_opt.value();

        // 기본 신호
        Vector3 myPos = BB->MyLocation_Cartesian;
        Vector3 tgtPos = BB->TargetLocaion_Cartesian;
        Vector3 Hm = BB->MyForwardVector;
        Vector3 Ht = BB->TargetForwardVector;

        // 가드 & 정규화
        Vector3 LOS = tgtPos - myPos;
        if (!normalize(LOS))
        {
            // 비정상 LOS → 정면으로 임시 추종
            BB->VP_Cartesian = myPos + Vector3(1, 0, 0) * 1000.0;
            BT_VLOG("[Pure/Lag] Fallback: invalid LOS\n");
            return BT::NodeStatus::SUCCESS;
        }
        if (!normalize(Hm) || !normalize(Ht))
        {
            // 진행방향 실패 → Pure 폴백
            BB->VP_Cartesian = tgtPos;
            BT_VLOG("[Pure/Lag] Fallback: invalid forward vector(s)\n");
            return BT::NodeStatus::SUCCESS;
        }

        // 타깃 진행방향(Ht) 기준 각도
        float theta_los_deg = rad2deg(safe_acos(static_cast<float>(LOS.dot(Ht))));
        float theta_m_deg = rad2deg(safe_acos(static_cast<float>(Hm.dot(Ht))));
        float diff = theta_m_deg - theta_los_deg;

        // 현재 거리
        float D = static_cast<float>(BB->Distance);
        if (D < 1.0f) D = static_cast<float>((tgtPos - myPos).length());

        // --- 분기: Pure / Lag ---
        // Pure: 내 기수와 LOS가 거의 동일
        if (std::fabs(diff) <= EPS_DEG)
        {
            BB->VP_Cartesian = tgtPos; // Pure
            BT_VLOG("[Pure] D=" << D
                << " theta_los=" << theta_los_deg
                << " theta_m=" << theta_m_deg
                << " |diff|=" << std::fabs(diff) << "\n");
        }
        // Lag: 내 기수가 LOS보다 뒤쪽
        else if (diff > (EPS_DEG + HYS_DEG))
        {
            float Dref = clampf(D, D_MIN_REF, D_MAX_REF);
            Vector3 VP = tgtPos - Ht * (K_LAG * Dref); // 타깃 뒤쪽으로
            BB->VP_Cartesian = VP;
            BT_VLOG("[Lag]  D=" << D
                << " Dref=" << Dref
                << " k=" << K_LAG
                << " theta_los=" << theta_los_deg
                << " theta_m=" << theta_m_deg
                << " diff=" << diff << "\n");
        }
        // 경계: Pure로 처리
        else
        {
            BB->VP_Cartesian = tgtPos;
            BT_VLOG("[Pure*] boundary D=" << D
                << " theta_los=" << theta_los_deg
                << " theta_m=" << theta_m_deg
                << " diff=" << diff << "\n");
        }

        // ===== 공통 보정: 거리/AA(0°)/AO(2°) =====
        {
            // 보정용 벡터 재확인
            Vector3 VP = BB->VP_Cartesian;
            Vector3 Ht2 = BB->TargetForwardVector;  normalize(Ht2);
            Vector3 Hm2 = BB->MyForwardVector;      normalize(Hm2);
            Vector3 LOS2 = tgtPos - myPos;          normalize(LOS2);

            // 1) 거리 보정: 300~3000 ft 유효 범위에 수렴
            //  - 멀수록 LOS 방향으로 더 당김
            //  - 300ft 근접 시에는 과도한 근접 완화를 위해 Ht 방향으로 소폭 이동
            {
                float far_ratio = (D - DMOD_MIN) / std::max(1.0f, (DMOD_MAX - DMOD_MIN));
                far_ratio = std::max(0.0f, std::min(1.0f, far_ratio));

                if (D < DMOD_MIN * 1.2f)
                {
                    float tight = (DMOD_MIN * 1.2f - D) / (DMOD_MIN * 1.2f);
                    tight = std::max(0.0f, std::min(1.0f, tight));
                    VP = VP + Ht2 * (K_RANGE_TIGHT * tight * DMOD_MIN);
                }
                else
                {
                    float kR = K_RANGE_CLOSE * far_ratio;
                    VP = VP + LOS2 * (kR * D);
                }
            }

            // 2) AA(Aspect) 보정(→0°): 타깃 테일(-Ht) 방향으로 보정
            {
                float AA = static_cast<float>(BB->MyAspectAngle_Degree);  // 목표 0°
                float Dscale = std::min(D, DMOD_MAX);
                VP = VP + (-Ht2) * (K_AA * AA * 0.001f * Dscale);
            }

            // 3) AO(LOS각) 보정(→2°): lateral_dir = Hm - LOS*(Hm·LOS)
            {
                float ao_err = static_cast<float>(BB->MyAngleOff_Degree) - 2.0f; // 목표 2°
                Vector3 lateral = Hm2 - LOS2 * static_cast<float>(Hm2.dot(LOS2));
                if (lateral.length() > 1e-6) normalize(lateral);
                float Dscale = std::min(D, DMOD_MAX);
                VP = VP + (-lateral) * (K_AO * ao_err * 0.001f * Dscale);
            }

            BB->VP_Cartesian = VP;
        }

        return BT::NodeStatus::SUCCESS;
    }
}
