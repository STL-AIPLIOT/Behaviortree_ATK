#include "Task_AggressiveOBFM.h"
#include "../BTLog.h"
#include "../WezPhase.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>

namespace Action
{
    namespace
    {
        constexpr double RAD2DEG = 57.29577951308232;

        inline float clampf(float v, float lo, float hi)
        {
            return std::max(lo, std::min(hi, v));
        }

        // Vector3::normalize() 는 in-place void 이고 길이 0 이면 아무것도 하지 않는다.
        // 값을 돌려주는 형태가 필요해 감싼다.
        inline Vector3 Normalized(const Vector3& v)
        {
            Vector3 r = v;
            r.normalize();
            return r;
        }

        inline double Dot(const Vector3& a, const Vector3& b)
        {
            return a.dot(b);
        }

        /*
        각도 부호 붕괴 방어(_repair).

        Python 쪽 GeoMathUtil 은 sign = np.sign(p_unit_t[2]) 를 쓰기 때문에 정확히 동고도 +
        정후방이면 ATA 가 0(nose-on)으로 무너진다. 여기서는 acos(dot) 만 쓰므로 그 경로를
        그대로 밟지는 않지만, 동고도(los.Z == 0)는 이 대회 초기 배치의 기본 상태이고
        (initial_scenario.altitude_m 을 두 기체가 공유한다) 이 노드도 cross 로 선회 방향을
        만들기 때문에 퇴화한 입력이 그대로 들어오면 방향이 불안정해진다.
        1e-9 m 오프셋은 기하학적으로 무의미하고(거리 단위가 m), 퇴화만 깬다.
        */
        inline Vector3 RepairCoAltitude(const Vector3& los)
        {
            Vector3 r = los;
            if (std::fabs(r.Z) < 1e-9)
            {
                r.Z = 1e-9;
            }
            return r;
        }

        /*
        tick 단위 CSV 로거. STIL_AGGR_CSV 에 경로가 있을 때만 열린다.
        미설정이면 모든 호출이 즉시 반환되어 동작에 영향이 없다
        (PredictManeuverCsvLogger / LeadPursuitTelemetry 와 같은 방식).
        */
        class AggrCsv
        {
        public:
            static AggrCsv& Instance()
            {
                static AggrCsv inst;
                return inst;
            }

            bool IsEnabled() const { return out.is_open(); }

            void Row(const std::string& line)
            {
                if (!out.is_open())
                {
                    return;
                }
                out << line << "\n";
                out.flush();   // 크래시로 잃지 않도록 매 행 flush
            }

        private:
            AggrCsv()
            {
                const char* path = std::getenv("STIL_AGGR_CSV");
                if (path && *path)
                {
                    out.open(path, std::ios::out | std::ios::trunc);
                    if (out.is_open())
                    {
                        out << "t,mode,tier,D,ata_deg,aa_nose_deg,ao_deg,closure_ms,"
                            "my_spd_ms,tgt_spd_ms,ecmp,round,throttle,gun_window,"
                            "wez_phase,gun_coeff,vp_x,vp_y,vp_z\n";
                        out.flush();
                    }
                }
            }

            std::ofstream out;
        };

        // 컴파일 타임 기본값 위에 얹는 환경변수 오버라이드. 프로세스당 한 번만 읽는다.
        Task_AggressiveOBFM::RoundProfile ResolveRoundProfile()
        {
            static const Task_AggressiveOBFM::RoundProfile resolved = [] {
                Task_AggressiveOBFM::RoundProfile p = Task_AggressiveOBFM::ROUND_PROFILE;
                if (const char* env = std::getenv("STIL_ROUND_PROFILE"))
                {
                    const std::string v(env);
                    if (v == "R3" || v == "r3" || v == "3") { p = Task_AggressiveOBFM::ROUND_R3; }
                    else if (v == "R12" || v == "r12" || v == "12") { p = Task_AggressiveOBFM::ROUND_R12; }
                }
                return p;
            }();
            return resolved;
        }
    }

    const char* Task_AggressiveOBFM::TierName(Tier t)
    {
        switch (t)
        {
        case TIER_ALL_OUT:  return "ALL_OUT";
        case TIER_PURSUE:   return "PURSUE";
        case TIER_CONSERVE: return "CONSERVE";
        }
        return "?";
    }

    BT::NodeStatus Task_AggressiveOBFM::tick()
    {
        auto bb_res = getInput<CPPBlackBoard*>("BB");
        if (!bb_res || !bb_res.value())
        {
            std::cerr << "[Task_AggressiveOBFM] BB nullptr\n";
            return BT::NodeStatus::FAILURE;
        }
        CPPBlackBoard* BB = bb_res.value();

        // ---------------------------------------------------------------
        // 1. 기하 계산 (ATA / closure 는 BB 에 없으므로 여기서 만든다)
        // ---------------------------------------------------------------
        const Vector3 los = RepairCoAltitude(BB->TargetLocaion_Cartesian - BB->MyLocation_Cartesian);
        const float   D = static_cast<float>(los.length());
        const Vector3 losN = Normalized(los);
        const Vector3 fwd = Normalized(BB->MyForwardVector);

        // 방향 벡터가 아직 갱신되지 않았으면(길이 0) 판단 근거가 없다. 양보한다.
        if (fwd.lengthSquared() < 0.5 || D < 1e-3f)
        {
            prev_tier_ = TIER_CONSERVE;
            return BT::NodeStatus::FAILURE;
        }

        const float ata = static_cast<float>(std::acos(std::max(-1.0, std::min(1.0, Dot(fwd, losN)))) * RAD2DEG);

        // 적기 속도 벡터: PredictManeuver 의 예측값이 있으면 그것을, 없으면 기수*속력.
        Vector3 Vt = BB->PredictedTargetVelocity;
        if (Vt.length() <= 0.1)
        {
            Vt = Normalized(BB->TargetForwardVector) * static_cast<double>(BB->TargetSpeed_MS);
        }
        const Vector3 Vm = fwd * static_cast<double>(BB->MySpeed_MS);

        // 양수 = 접근 중. 이전 거리 차분 캐시를 쓰지 않는다(노드는 매 tick 순수 함수로 동작).
        const float closure = static_cast<float>(Dot(Vm - Vt, losN));

        // 적기 기수 기준 aspect: 0 = 적기가 나를 향함(정면), 180 = 내가 적기 6시.
        // BB->MyAspectAngle_Degree 와 같은 규약이지만 BB 의존을 피하려고 직접 만든다.
        const Vector3 tfwd = Normalized(BB->TargetForwardVector);
        const float aa_nose = (tfwd.lengthSquared() < 0.5)
            ? 180.0f
            : static_cast<float>(std::acos(std::max(-1.0, std::min(1.0, Dot(tfwd, -losN)))) * RAD2DEG);

        // ---------------------------------------------------------------
        // 2. 진입 게이트 - 내 담당이 아니면 FAILURE 로 양보
        // ---------------------------------------------------------------
        const bool own_situation =
            BB->EnemyInSight &&
            (D >= ENTRY_D_MIN_M) && (D <= ENTRY_D_MAX_M) &&
            (ata <= ENTRY_ATA_MAX_DEG);

        if (!own_situation)
        {
            prev_tier_ = TIER_CONSERVE;   // 재진입은 보수적으로 시작
            BT_VLOG("[Task_AggressiveOBFM] yield | sight=" << BB->EnemyInSight
                << ", D=" << D << ", ATA=" << ata << "\n");
            return BT::NodeStatus::FAILURE;
        }

        const RoundProfile round = ResolveRoundProfile();
        const int ecmp = BB->EnergyCompareResult;

        /*
        [추가 2026-08-17] 규정 §6 Phase 반영.

        이 노드는 대미지 창을 1.0deg / 152.4~914.4m 로 고정하고 "절대 넓히지 않는다"고
        적어 두었다. 그 값은 학습 환경(DogFightEnv update_damage)에는 맞지만 **대회 판정과는
        다르다**. 규정 §6 은 경과 시간에 따라 판정을 완화한다:

            Phase 1   0~100s   LOS<1deg  500~3000ft  계수 1.0
            Phase 2 100~150s   LOS<2deg  500~3500ft  계수 0.3
            Phase 3 150~200s   LOS<3deg  500~4000ft  계수 0.1

        콘 부피비는 거리까지 포함해 1 : 6.36 : 21.37 이다. 1deg 창만 노리는 행동 티어는
        100초 이후 20배 넓어진 판정면을 통째로 버린다. 2026-08-15 로그 40판이 38판 시간만료로
        끝났고 그중 대부분이 100초를 훌쩍 넘겼으므로, 이 손실은 이론이 아니라 실측 구간이다.

        티어 임계(1.5/2.5/3.0deg)는 "대미지 콘 1deg 를 물기 위한 행동 강도"로 잡힌 값이므로
        현재 열린 창의 폭에 비례해 함께 넓힌다. Phase 1 에서는 배율 1.0 이라 종전과 동일하다.
        */
        const double match_t = BB->MatchTimeSec();
        const WezPhase::Window open_win = WezPhase::WidestOpen(match_t);
        const float phase_scale = open_win.max_ata_deg / WezPhase::kPhase[0].max_ata_deg;

        // 건 리드점: 유효 탄속(내 속도 + K_MUZZLE)으로 비행시간을 만들고 그만큼 앞을 찍는다.
        const float t_flight = clampf(D / std::max(1.0f, BB->MySpeed_MS + K_MUZZLE),
            LEAD_TIME_MIN_SEC, LEAD_TIME_MAX_SEC);
        const Vector3 lead_point = BB->TargetLocaion_Cartesian + Vt * static_cast<double>(t_flight);

        // 스위트 거리 안이면 접근률 0 을, 밖이면 CLOSURE_TARGET_MS 를 원한다.
        const float closure_want = (D > D_SWEET_AGGR) ? CLOSURE_TARGET_MS : 0.0f;
        const float closure_err = closure - closure_want;   // 양수 = 너무 빨리 붙는 중

        Vector3 vp;
        float   thr = 0.0f;
        const char* mode = "";
        Tier    tier = prev_tier_;

        // ---------------------------------------------------------------
        // 3. 오버슛 가드 - 항상 ON, 티어보다 우선
        //    뒤로 빼지 않는다. 내 양력 벡터 방향(MyUpVector)으로 수직으로 빼서
        //    클로저만 죽이고 공격위치를 유지한다 = 하이 요요.
        // ---------------------------------------------------------------
        const bool overshoot_risk = (closure > OVERSHOOT_CLOSURE) && (D < OVERSHOOT_D);

        if (overshoot_risk)
        {
            // [회전3] 요요 오프셋은 반드시 월드 상반구를 향해야 한다.
            // MyUpVector 는 쿼터니언에서 뽑은 *동체* up 이라(DirectionVectorUpdate.cpp:32-34)
            // 인버티드/고뱅크에서 Z<0 이 된다. 월드 +Z 가 상방이므로
            // (Task_ClimbToSafeAltitude.cpp:62-74) 그대로 쓰면 VP 가 타깃 아래에 찍혀
            // 컨트롤러가 지면으로 몬다. 회전 1.5 에서 요요가 처음 발화하자
            // 20판 전부가 "altitude below min" 으로 끝났다.
            Vector3 up = Normalized(BB->MyUpVector);
            if (up.Z < 0.0) up = -up;                  // 하반구를 향하면 뒤집는다
            if (up.Z < YOYO_MIN_UP_Z)                  // 나이프에지에서 수직 성분 소멸 방지
            {
                up.Z = YOYO_MIN_UP_Z;
                up = Normalized(up);                   // MyUpVector 가 0 이어도 (0,0,1) 로 수렴한다
            }
            vp = BB->TargetLocaion_Cartesian + up * static_cast<double>(H_YOYO);
            thr = clampf(THR_YOYO_BASE - THR_CLOSURE_GAIN * (closure - OVERSHOOT_CLOSURE), 0.25f, 0.60f);
            mode = "HIGH_YOYO";
            // 티어 상태는 그대로 유지한다. 요요는 티어 전이가 아니라 덮어쓰기다.
        }
        else
        {
            // -----------------------------------------------------------
            // 4. ATA 기반 행동 티어 (히스테리시스)
            //    2.5~3.0 deg 버퍼 구간에서는 직전 티어를 유지한다.
            // -----------------------------------------------------------
            // 임계는 현재 열린 대미지 창의 폭에 비례한다(Phase 1 = 배율 1.0, 종전과 동일).
            const float allout_ata = ALLOUT_ATA * phase_scale;
            const float tier_up_ata = TIER_UP_ATA * phase_scale;
            const float tier_down_ata = TIER_DOWN_ATA * phase_scale;

            if (ata <= allout_ata)          tier = TIER_ALL_OUT;
            else if (ata <= tier_up_ata)    tier = TIER_PURSUE;    // 좁혀지면 상승
            else if (ata >= tier_down_ata)  tier = TIER_CONSERVE;  // 벌어지면 하강
            else                            tier = prev_tier_;     // 버퍼 구간

            switch (tier)
            {
            case TIER_ALL_OUT:
            {
                // 풀 리드 + 스로틀 상향. 1 deg 건 솔루션까지 밀어붙인다.
                vp = lead_point;
                thr = clampf(THR_ALLOUT_BASE - THR_ALLOUT_GAIN * closure_err, 0.75f, 1.0f);
                mode = "ALL_OUT";
                break;
            }
            case TIER_PURSUE:
            {
                // 공격적 리드로 각을 빠르게 죽인다. 리드점을 기수 기준 횡방향으로 더 밀어
                // 컨트롤러가 더 강한 선회를 물게 한다(TURN_IN_BIAS).
                Vector3 to_lead = (BB->TargetLocaion_Cartesian
                    + Vt * static_cast<double>(t_flight * PURSUE_LEAD_GAIN))
                    - BB->MyLocation_Cartesian;
                Vector3 perp = to_lead - fwd * Dot(to_lead, fwd);   // 기수에 수직인 성분 = 선회해야 할 방향

                vp = BB->MyLocation_Cartesian + to_lead;
                if (perp.lengthSquared() > 1e-6)
                {
                    vp = vp + Normalized(perp) * static_cast<double>(TURN_IN_BIAS);
                }
                thr = clampf(THR_PURSUE_BASE - THR_CLOSURE_GAIN * closure_err, 0.55f, 1.0f);
                mode = "PURSUE";
                break;
            }
            case TIER_CONSERVE:
            default:
            {
                /*
                각이 벌어졌다. 리드를 억지로 당기지 않는다(당기면 G 를 먹고 에너지가 녹는다).
                pure ~ 약한 lag + 스로틀 감소로 에너지를 보존하되, 이탈은 하지 않는다.
                */
                float lag = clampf(CONSERVE_LAG_GAIN * D, 0.0f, CONSERVE_LAG_MAX_M);
                float base = THR_CONSERVE_BASE;

                if (round == ROUND_R12)
                {
                    // 1·2R: 에너지 바닥을 존중해 더 보수적으로. lag 를 늘려 선회 G 를 낮춘다.
                    lag = clampf(lag * R12_CONSERVE_LAG_MUL, 0.0f, CONSERVE_LAG_MAX_M);
                    base -= R12_CONSERVE_THR_BIAS;
                }

                const bool headon_commit =
                    (round == ROUND_R3) && (ecmp < 0) && (aa_nose <= HEADON_AA_DEG);

                if (headon_commit)
                {
                    /*
                    3R, 열세 + 정면. 이탈하면 무승부로 끝난다. 이탈 대신 정면교전으로 커밋한다
                    (HABFM 성격의 nose-on). lag 를 0 으로 두고 리드점을 그대로 문다.
                    */
                    vp = lead_point;
                    thr = THR_HEADON_COMMIT;
                    mode = "R3_HEADON_COMMIT";
                }
                else
                {
                    const Vector3 vt_dir = Normalized(Vt);
                    vp = BB->TargetLocaion_Cartesian;
                    if (vt_dir.lengthSquared() > 0.5)
                    {
                        vp = vp - vt_dir * static_cast<double>(lag);   // 표적 뒤쪽 = lag pursuit
                    }
                    thr = clampf(base - THR_CLOSURE_GAIN * closure, 0.30f, 0.75f);
                    mode = "CONSERVE";
                }
                break;
            }
            }
        }

        const Tier tier_before = prev_tier_;
        prev_tier_ = tier;

        /*
        ALLOW_EXTEND = false 안전장치.
        위 어느 분기도 -fwd 나 -losN 방향으로 VP 를 만들지 않지만(lag 은 표적 뒤
        CONSERVE_LAG_MAX_M 이내의 추적점이지 이탈점이 아니다), 예측 속도 벡터가 이상하게
        들어와 VP 가 표적 반대편으로 넘어가면 그 순간 이탈 기동이 된다. 그 경우 pure 로 되돌린다.
        */
        if (!ALLOW_EXTEND)
        {
            const Vector3 to_vp = vp - BB->MyLocation_Cartesian;
            if (Dot(Normalized(to_vp), losN) <= 0.0)
            {
                vp = BB->TargetLocaion_Cartesian;
                mode = "NO_EXTEND_PURE";
            }
        }

        BB->VP_Cartesian = vp;
        BB->Throttle = clampf(thr, 0.0f, 1.0f);

        // ---------------------------------------------------------------
        // 5. 계측
        //    gun_window 는 **실제 판정과 같은 기준**으로만 센다. 행동 티어의 임계와 절대 섞지 않는다.
        //    [수정 2026-08-17] 고정 1deg/914.4m -> 규정 §6 Phase 판정(WezPhase).
        //    STIL_WEZ_MODE=training 을 주면 학습 환경 update_damage() 와 동일한 고정 창으로
        //    되돌아가므로, 이전 판과 같은 잣대로 비교해야 할 때 그쪽을 쓴다.
        // ---------------------------------------------------------------
        const float gun_coeff = WezPhase::BestCoeff(match_t, ata, D);
        const bool gun_window = (gun_coeff > 0.0f);

        AggrCsv& csv = AggrCsv::Instance();
        if (csv.IsEnabled())
        {
            std::string row;
            row.reserve(224);
            row += std::to_string(match_t);         row += ",";   // 규정 §6 기준 경과 시간
            row += mode;                            row += ",";
            row += TierName(tier);                  row += ",";
            row += std::to_string(D);               row += ",";
            row += std::to_string(ata);             row += ",";
            row += std::to_string(aa_nose);         row += ",";
            row += std::to_string(BB->MyAngleOff_Degree); row += ",";
            row += std::to_string(closure);         row += ",";
            row += std::to_string(BB->MySpeed_MS);  row += ",";
            row += std::to_string(BB->TargetSpeed_MS); row += ",";
            row += std::to_string(ecmp);            row += ",";
            row += (round == ROUND_R3 ? "R3" : "R12"); row += ",";
            row += std::to_string(BB->Throttle);    row += ",";
            row += (gun_window ? "1" : "0");        row += ",";
            row += std::to_string(WezPhase::PhaseAt(match_t)); row += ",";
            row += std::to_string(gun_coeff);       row += ",";
            row += std::to_string(BB->VP_Cartesian.X); row += ",";
            row += std::to_string(BB->VP_Cartesian.Y); row += ",";
            row += std::to_string(BB->VP_Cartesian.Z);
            csv.Row(row);
        }

        // 콘솔은 상태가 바뀔 때만 낸다(매 tick 찍으면 20판 로그가 읽을 수 없게 된다).
        if (tier != tier_before || overshoot_risk || gun_window)
        {
            BT_VLOG("[Task_AggressiveOBFM] " << mode
                << " | tier=" << TierName(tier)
                << ", ATA=" << ata
                << ", D=" << D
                << ", closure=" << closure
                << ", thr=" << BB->Throttle
                << ", gun_window=" << (gun_window ? 1 : 0)
                << "\n");
        }

        return BT::NodeStatus::SUCCESS;
    }
}
