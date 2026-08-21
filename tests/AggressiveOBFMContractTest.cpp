// Task_AggressiveOBFM 계약(contract) 테스트
//
// 빌드/실행: .\tests\build_and_run_aggressive_obfm.ps1
//
// 외부 테스트 프레임워크 없이 실패 개수를 세는 방식이다
// (tests/CanRetryTest.cpp / AntiOvershootContractTest.cpp 와 동일).
//
// 왜 이 테스트가 필요한가
// -----------------------
// 이 노드는 1v1 실행에서 한 번도 tick 되지 않는다. OBFM_Branch 는
// SetBFMMode_OBFM 이 SUCCESS 일 때만 열리고 그 조건에 EnergyCompareResult > 0 이
// 들어 있는데(SetBFMMode_OBFM.cpp:19,26), run_local_dogfight.py 기본 시나리오에서
// 자기기는 교전 내내 에너지 열세다(2026-08-13 측정: OBFM 차단 8541 tick 중 7983 tick 이
// e_sup=0). 원본 트리(AIP_STIL.dll)도 똑같이 OBFM 진입 0회라 이건 이 노드의 문제가 아니다.
//
// 즉 summary.json 의 end_condition 분포로는 이 노드를 잴 수 없다. 노드의 계약은
// 여기서 직접 고정한다.
//
// 좌표 규약
// ---------
// 자기기는 원점, 표적은 +X 축 위 거리 D. ATA 는 내 기수를 XY 평면에서 돌려 만든다.
//   fwd = (cos a, sin a, 0)  ->  acos(dot(fwd, losN)) = a
// 이렇게 하면 LOS 가 항상 +X 라 VP 성분을 눈으로 검산할 수 있다.

#include "../BT_Content/Task/Task_AggressiveOBFM.h"
#include "../behaviortree_cpp_v3/blackboard.h"

#include <cmath>
#include <iostream>
#include <string>

namespace
{

int g_failures = 0;

void Check(bool condition, const std::string& what)
{
    if (condition)
    {
        std::cout << "  [OK]   " << what << "\n";
    }
    else
    {
        std::cout << "  [FAIL] " << what << "\n";
        g_failures++;
    }
}

void CheckNear(double got, double want, double tol, const std::string& what)
{
    const bool ok = std::fabs(got - want) <= tol;
    if (ok)
    {
        std::cout << "  [OK]   " << what << " (got " << got << ")\n";
    }
    else
    {
        std::cout << "  [FAIL] " << what << " (want " << want << " +-" << tol
                  << ", got " << got << ")\n";
        g_failures++;
    }
}

const char* ToStr(BT::NodeStatus s)
{
    switch (s)
    {
        case BT::NodeStatus::IDLE:    return "IDLE";
        case BT::NodeStatus::RUNNING: return "RUNNING";
        case BT::NodeStatus::SUCCESS: return "SUCCESS";
        case BT::NodeStatus::FAILURE: return "FAILURE";
    }
    return "?";
}

BT::NodeConfiguration MakeConfig(BT::Blackboard::Ptr blackboard)
{
    BT::NodeConfiguration config;
    config.blackboard = blackboard;
    config.input_ports["BB"] = "{BB}";
    return config;
}

/*
표적은 +X 축 위, 기수만 ata_deg 만큼 돌린다.

에너지 마진과 closure 를 **따로** 조절할 수 있게 표적 속도를 두 개로 나눠 받는다.
  tgtSpeed : TargetSpeed_MS. 에너지 계산에만 쓰인다(E = V^2 + 2gh).
  tgtVelX  : PredictedTargetVelocity.X. closure 계산에만 쓰인다.
둘이 같아야 물리적으로 자연스럽지만, 그러면 마진을 키우려고 속도를 벌릴 때 closure 도
같이 움직여 스로틀이 clamp 에 걸린다. 검사하려는 축 하나만 움직이려고 분리했다.
두 기체를 같은 고도(Z=0)에 두므로 마진은 속도만으로 정해진다: (V_me^2 - V_tgt^2) / V_tgt^2.
*/
void Setup(CPPBlackBoard& bb, double D, double ata_deg,
           double mySpeed, double tgtSpeed, double tgtVelX)
{
    const double a = ata_deg * 3.14159265358979323846 / 180.0;

    bb.MyLocation_Cartesian = Vector3(0.0, 0.0, 0.0);
    bb.TargetLocaion_Cartesian = Vector3(D, 0.0, 0.0);

    bb.MyForwardVector = Vector3(std::cos(a), std::sin(a), 0.0);
    bb.MyUpVector = Vector3(0.0, 0.0, 1.0);
    bb.MyRightVector = Vector3(0.0, 1.0, 0.0);

    bb.TargetForwardVector = Vector3(1.0, 0.0, 0.0);
    bb.TargetSpeed_MS = static_cast<float>(tgtSpeed);
    bb.MySpeed_MS = static_cast<float>(mySpeed);
    bb.PredictedTargetVelocity = Vector3(tgtVelX, 0.0, 0.0);

    bb.PredictedTurnDirection = "NONE";
    // 노드는 이 값을 읽지 않는다(항상 +1 이라 정보량이 0). 읽지 않음을 확인하려고
    // 일부러 -1 을 넣어 둔다. 읽는다면 R12 보수 분기가 걸려 스로틀 기대값이 어긋난다.
    bb.EnergyCompareResult = -1;
    bb.RunningTime = 0.0;
    bb.Distance = static_cast<float>(D);   // 노드는 읽지 않는다.
    bb.VP_Cartesian = Vector3(0.0, 0.0, 0.0);
    bb.Throttle = 0.0f;
}

// 위 Setup 기준 closure. 기대 스로틀을 손으로 계산하지 않기 위해 같은 식을 쓴다.
double ClosureOf(double ata_deg, double mySpeed, double tgtVelX)
{
    const double a = ata_deg * 3.14159265358979323846 / 180.0;
    return mySpeed * std::cos(a) - tgtVelX;
}

/*
에너지 여유가 충분한 조합(k = 1). 티어 임계값이 지정값 1.5/2.5/3.0 그대로다.
  margin = (250^2 - 200^2)/200^2 = 0.5625 >= E_MARGIN_FULL(0.20)
  closure = 250*cos(ata) - 240 ~ 10
*/
constexpr double kFatMySpd = 250.0, kFatTgtSpd = 200.0, kFatTgtVel = 240.0;

/*
에너지 여유가 얇은 조합(k = 0.5, 하한). 임계값이 절반이 된다: 0.75/1.25/1.5.
  margin = (205^2 - 200^2)/200^2 = 0.050625 -> margin/0.20 = 0.253 -> 하한 0.5
  closure = 205*cos(ata) - 195 ~ 10
*/
constexpr double kThinMySpd = 205.0, kThinTgtSpd = 200.0, kThinTgtVel = 195.0;

} // namespace

int main()
{
    // 노드가 상태 전이마다 std::cout 으로 진단을 뱉는다. 테스트 출력을 가리므로 버린다.
    std::streambuf* saved = std::cout.rdbuf();

    CPPBlackBoard bb;
    BT::Blackboard::Ptr blackboard = BT::Blackboard::create();
    CPPBlackBoard* bbPtr = &bb;
    blackboard->set("BB", bbPtr);
    const BT::NodeConfiguration config = MakeConfig(blackboard);

    // 티어 히스테리시스는 노드 인스턴스 상태(prev_tier_)라, 독립 케이스는 새 인스턴스를 쓴다.
    auto tick = [&](Action::Task_AggressiveOBFM& node) {
        std::cout.rdbuf(nullptr);
        const BT::NodeStatus s = node.executeTick();
        std::cout.rdbuf(saved);
        return s;
    };

    std::cout << "== Task_AggressiveOBFM 계약 테스트 ==\n";

    // -----------------------------------------------------------------
    // 0. 데미지 콘 상수. 행동 티어(1.5/2.5/3.0)와 절대 섞이면 안 된다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[0] WEZ / 데미지 콘 상수 고정\n";
        Check(Action::Task_AggressiveOBFM::DAMAGE_ATA_DEG == 1.0f,
              "DAMAGE_ATA_DEG == 1.0 (update_damage 의 wez.angle_deg/2, epsilon 없음)");
        Check(Action::Task_AggressiveOBFM::WEZ_MIN_M == 152.4f,
              "WEZ_MIN_M == 152.4 (500 ft)");
        Check(Action::Task_AggressiveOBFM::WEZ_MAX_M == 914.4f,
              "WEZ_MAX_M == 914.4 (3000 ft)");
        Check(Action::Task_AggressiveOBFM::ALLOW_EXTEND == false,
              "ALLOW_EXTEND == false (세 라운드 모두 완전 이탈 없음)");
    }

    // -----------------------------------------------------------------
    // 1. 퇴화 기하에서만 FAILURE
    // -----------------------------------------------------------------
    {
        std::cout << "\n[1] 퇴화 기하 -> FAILURE (폴백 안전망이 살아 있는 유일한 경로)\n";

        Action::Task_AggressiveOBFM n1("Task_AggressiveOBFM", config);
        Setup(bb, 800.0, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
        bb.MyForwardVector = Vector3(0.0, 0.0, 0.0);        // 방향 벡터 미갱신
        const BT::NodeStatus sFwd = tick(n1);
        Check(sFwd == BT::NodeStatus::FAILURE,
              std::string("MyForwardVector 길이 0 -> FAILURE (got ") + ToStr(sFwd) + ")");

        Action::Task_AggressiveOBFM n2("Task_AggressiveOBFM", config);
        Setup(bb, 800.0, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
        bb.TargetLocaion_Cartesian = bb.MyLocation_Cartesian;   // D ~ 0
        const BT::NodeStatus sD = tick(n2);
        Check(sD == BT::NodeStatus::FAILURE,
              std::string("D ~ 0 -> FAILURE (got ") + ToStr(sD) + ")");

        Action::Task_AggressiveOBFM n3("Task_AggressiveOBFM", config);
        Setup(bb, 800.0, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
        const BT::NodeStatus sOk = tick(n3);
        Check(sOk == BT::NodeStatus::SUCCESS,
              std::string("정상 기하 -> SUCCESS (got ") + ToStr(sOk) + ")");
    }

    // -----------------------------------------------------------------
    // 2. ATA 행동 티어. 스로틀 식으로 어느 티어가 골라졌는지 판별한다.
    //      ALL_OUT  thr = 0.95 - 0.01*(closure - 10)
    //      PURSUE   thr = 0.85 - 0.02*(closure - 10)
    //      CONSERVE thr = 0.55 - 0.02*closure
    //    D=800 > D_SWEET_AGGR 이라 closure_want = CLOSURE_TARGET_MS = 10.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[2] ATA 티어 선택 (에너지 여유 충분, k=1 -> 임계값 1.5/2.5/3.0)\n";
        const double D = 800.0;

        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            tick(n);
            CheckNear(bb.Throttle, 1.0, 1e-3,
                      "ATA=1.0 -> ALL_OUT 스로틀");
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 2.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            tick(n);
            CheckNear(bb.Throttle, 0.90 - 0.02 * (ClosureOf(2.0, kFatMySpd, kFatTgtVel) - 12.0), 1e-3,
                      "ATA=2.0 -> PURSUE 스로틀");
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 5.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            tick(n);
            CheckNear(bb.Throttle, 0.60 - 0.02 * ClosureOf(5.0, kFatMySpd, kFatTgtVel), 1e-3,
                      "ATA=5.0 -> CONSERVE 스로틀");
        }
    }

    // -----------------------------------------------------------------
    // 2b. 에너지 마진이 티어 임계값을 움직인다.
    //     **같은 ATA=2.0** 이 여유가 충분하면 PURSUE, 얇으면 CONSERVE 가 되어야 한다.
    //     얇을 때 임계값은 1.5*0.5 / 2.5*0.5 / 3.0*0.5 = 0.75 / 1.25 / 1.5 이므로
    //     2.0 은 하강 임계 1.5 를 넘는다.
    //
    //     이 케이스가 실패하면 노드가 에너지를 안 보고 거리+각도만 보는 상태로
    //     되돌아간 것이다(2026-08-13 이전 상태).
    // -----------------------------------------------------------------
    {
        std::cout << "\n[2b] 에너지 마진 -> 티어 임계값 축소\n";
        const double D = 800.0;

        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 2.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            tick(n);
            CheckNear(bb.Throttle, 0.90 - 0.02 * (ClosureOf(2.0, kFatMySpd, kFatTgtVel) - 12.0), 1e-3,
                     "여유 충분(margin 0.5625) + ATA=2.0 -> PURSUE 유지");
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 2.0, kThinMySpd, kThinTgtSpd, kThinTgtVel);
            tick(n);
            CheckNear(bb.Throttle, 0.30, 1e-3,
                     "여유 얇음 + 같은 ATA=2.0 -> CONSERVE 로 하강 (스로틀 하한 0.30)");
            Check(bb.Throttle < 0.65 - 1e-4,
                  "그 스로틀은 여유 충분일 때의 PURSUE(0.65)보다 낮다");
        }
        {
            // CONSERVE 의 lag 도 얇을수록 커진다. lag 는 표적 뒤로 빼는 거리이므로 VP.X 로 보인다.
            double xFat = 0.0, xThin = 0.0;
            {
                Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
                Setup(bb, D, 5.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);  xFat = bb.VP_Cartesian.X;
            }
            {
                Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
                Setup(bb, D, 5.0, kThinMySpd, kThinTgtSpd, kThinTgtVel); tick(n); xThin = bb.VP_Cartesian.X;
            }
            CheckNear(D - xFat, 0.12 * D, 1e-3, "여유 충분: lag = 0.12*D");
            CheckNear(D - xThin, 0.12 * D * Action::Task_AggressiveOBFM::E_CONSERVE_LAG_MUL_MAX, 1e-3,
                     "여유 얇음: lag = 0.12*D * E_CONSERVE_LAG_MUL_MAX");
        }
    }

    // -----------------------------------------------------------------
    // 3. 2.5~3.0 deg 데드밴드 = 직전 티어 유지 (채터링 금지)
    //    같은 ATA=2.7 이 직전 상태에 따라 PURSUE 도, CONSERVE 도 된다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[3] 티어 히스테리시스 (k=1 기준 2.5~3.0 데드밴드)\n";
        const double D = 800.0;
        const double c27 = ClosureOf(2.7, kFatMySpd, kFatTgtVel);

        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 2.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);   // -> PURSUE
            Setup(bb, D, 2.7, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);   // 버퍼: 유지되어야 한다
            CheckNear(bb.Throttle, 0.90 - 0.02 * (c27 - 12.0), 1e-3,
                      "PURSUE 에서 진입한 ATA=2.7 은 PURSUE 유지");
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 3.5, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);   // -> CONSERVE
            Setup(bb, D, 2.7, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);   // 버퍼: 유지되어야 한다
            CheckNear(bb.Throttle, 0.60 - 0.02 * c27, 1e-3,
                      "CONSERVE 에서 진입한 ATA=2.7 은 CONSERVE 유지");
        }
    }

    // -----------------------------------------------------------------
    // 4. 오버슛 가드. 티어보다 우선하고, 뒤로 빼지 않고 위로 뺀다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[4] 오버슛 가드 (하이 요요)\n";
        // closure > 18 && D < 350 을 만족시키되, ATA 는 ALL_OUT 구간에 둔다.
        // 요요는 티어보다 우선하므로 에너지 여유는 결과에 영향을 주지 않아야 한다.
        const double D = 300.0, mySpd = 250.0, tgtSpd = 200.0, tgtVel = 230.0;
        const double closure = ClosureOf(1.0, mySpd, tgtVel);   // ~19.96

        Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
        Setup(bb, D, 1.0, mySpd, tgtSpd, tgtVel);
        tick(n);

        Check(closure >= Action::Task_AggressiveOBFM::OVERSHOOT_CLOSURE,
              "전제: closure >= OVERSHOOT_CLOSURE");
        CheckNear(bb.VP_Cartesian.X, D, 1e-6, "VP.X = 표적 X (뒤로 빼지 않는다)");
        CheckNear(bb.VP_Cartesian.Z, Action::Task_AggressiveOBFM::H_YOYO, 1e-6,
                  "VP.Z = +H_YOYO (MyUpVector 방향 수직 회피)");
        CheckNear(bb.Throttle,
                  0.45 - 0.02 * (closure - Action::Task_AggressiveOBFM::OVERSHOOT_CLOSURE), 1e-3,
                  "요요 스로틀 = THR_YOYO_BASE - 초과 클로저 보정");
    }

    // -----------------------------------------------------------------
    // 5. turn-in 바이어스 (PredictedTurnDirection)
    // -----------------------------------------------------------------
    {
        std::cout << "\n[5] turn-in 바이어스\n";
        const double D = 800.0;
        const double bias = Action::Task_AggressiveOBFM::TURN_IN_BIAS;

        double yNone = 0.0, yLeft = 0.0, yRight = 0.0;
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);  yNone = bb.VP_Cartesian.Y;
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            bb.PredictedTurnDirection = "LEFT";  tick(n);  yLeft = bb.VP_Cartesian.Y;
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            bb.PredictedTurnDirection = "RIGHT"; tick(n);  yRight = bb.VP_Cartesian.Y;
        }

        CheckNear(yLeft - yNone, -bias, 1e-6, "LEFT -> -MyRightVector 방향으로 TURN_IN_BIAS");
        CheckNear(yRight - yNone, bias, 1e-6, "RIGHT -> +MyRightVector 방향으로 TURN_IN_BIAS");

        // CONSERVE 는 바이어스를 쓰지 않는다. 안쪽으로 당기는 것은 G 를 먹어 보존과 반대다.
        double cNone = 0.0, cLeft = 0.0;
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 5.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);  tick(n);  cNone = bb.VP_Cartesian.Y;
        }
        {
            Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
            Setup(bb, D, 5.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
            bb.PredictedTurnDirection = "LEFT"; tick(n);  cLeft = bb.VP_Cartesian.Y;
        }
        CheckNear(cLeft - cNone, 0.0, 1e-9, "CONSERVE 에서는 바이어스 없음");
    }

    // -----------------------------------------------------------------
    // 6. 리드 비행시간 상한 0.6 초
    //    D=1500, mySpd=210 이면 D/(210+300) = 2.94 초라 상한이 실제로 걸린다.
    //    상한이 예전 값(2.0)이면 VP.X 는 1900 이 된다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[6] 리드 비행시간 clamp\n";
        Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
        Setup(bb, 1500.0, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
        tick(n);
        CheckNear(bb.VP_Cartesian.X,
                  1500.0 + kFatTgtVel * Action::Task_AggressiveOBFM::LEAD_TIME_MAX_SEC, 1e-6,
                  "t_lead 가 LEAD_TIME_MAX_SEC(0.7)으로 잘린다");
        Check(Action::Task_AggressiveOBFM::LEAD_TIME_MAX_SEC == 0.7f,
              "LEAD_TIME_MAX_SEC == 0.7");
        Check(Action::Task_AggressiveOBFM::LEAD_TIME_MIN_SEC == 0.05f,
              "LEAD_TIME_MIN_SEC == 0.05");
    }

    // -----------------------------------------------------------------
    // 7. 이탈 금지 가드. 예측 속도가 이상하게 들어와 VP 가 표적 반대편으로
    //    넘어가면 pure 로 되돌린다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[7] ALLOW_EXTEND=false 가드\n";
        Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
        Setup(bb, 800.0, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);
        bb.PredictedTargetVelocity = Vector3(-100000.0, 0.0, 0.0);   // 리드점이 내 뒤로 간다
        tick(n);
        CheckNear(bb.VP_Cartesian.X, 800.0, 1e-6, "VP 가 표적 위치로 되돌려진다 (X)");
        CheckNear(bb.VP_Cartesian.Y, 0.0, 1e-6, "VP 가 표적 위치로 되돌려진다 (Y)");
    }

    // -----------------------------------------------------------------
    // 8. 출력 계약: VP_Cartesian 과 Throttle 만 쓴다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[8] 출력 계약 (다른 BB 필드 불변)\n";
        Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
        Setup(bb, 800.0, 1.0, kFatMySpd, kFatTgtSpd, kFatTgtVel);

        bb.Distance = 12345.0f;
        bb.MyAspectAngle_Degree = 77.0f;
        bb.MyAngleOff_Degree = 33.0f;
        bb.BFM = OBFM;
        const float spd = bb.MySpeed_MS;
        const int ecmp = bb.EnergyCompareResult;

        tick(n);

        Check(bb.EnergyCompareResult == ecmp, "BB->EnergyCompareResult 불변");
        Check(bb.Distance == 12345.0f, "BB->Distance 를 쓰지 않는다 (위치로 D 를 직접 계산)");
        Check(bb.MyAspectAngle_Degree == 77.0f, "BB->MyAspectAngle_Degree 불변");
        Check(bb.MyAngleOff_Degree == 33.0f, "BB->MyAngleOff_Degree 불변");
        Check(bb.MySpeed_MS == spd, "BB->MySpeed_MS 불변");
        Check(bb.BFM == OBFM, "BB->BFM 불변");
    }

    // -----------------------------------------------------------------
    // 9. 스로틀은 항상 0~1. ATA 전 구간 + 여러 거리/속도 조합을 훑는다.
    // -----------------------------------------------------------------
    {
        std::cout << "\n[9] 스로틀 범위 스윕\n";
        Action::Task_AggressiveOBFM n("Task_AggressiveOBFM", config);
        int outOfRange = 0;
        int nonSuccess = 0;
        int samples = 0;

        for (int ai = 0; ai <= 180; ++ai)
        {
            for (double D : { 120.0, 300.0, 800.0, 1500.0 })
            {
                // 에너지 여유가 아주 얇은 것(150 은 표적보다 느려 margin 이 음수)부터
                // 넉넉한 것까지 함께 훑는다. margin 음수는 실제 트리에서는 나오지 않지만
                // 노드가 그 입력에도 [0,1] 을 지키는지 본다.
                for (double ms : { 150.0, 205.0, 250.0, 320.0 })
                {
                    Setup(bb, D, static_cast<double>(ai), ms, 200.0, 195.0);
                    if (tick(n) != BT::NodeStatus::SUCCESS) { ++nonSuccess; }
                    if (bb.Throttle < 0.0f || bb.Throttle > 1.0f) { ++outOfRange; }
                    ++samples;
                }
            }
        }

        Check(samples == 181 * 4 * 4, "스윕 표본 수 2896");
        Check(nonSuccess == 0,
              std::string("모든 표본에서 SUCCESS (아닌 표본=") + std::to_string(nonSuccess) + ")");
        Check(outOfRange == 0,
              std::string("Throttle 이 항상 [0,1] (벗어난 표본=") + std::to_string(outOfRange) + ")");
    }

    std::cout << "\n실패 " << g_failures << "건\n";
    return g_failures == 0 ? 0 : 1;
}
