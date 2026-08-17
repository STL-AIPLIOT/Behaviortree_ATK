#include "Task_ClimbToSafeAltitude.h"
#include "../BTLog.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace Action {

    Task_ClimbToSafeAltitude::Task_ClimbToSafeAltitude(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {
    }

    Task_ClimbToSafeAltitude::~Task_ClimbToSafeAltitude() {}

    PortsList Task_ClimbToSafeAltitude::providedPorts() {
        return { InputPort<CPPBlackBoard*>("BB") };
    }

    static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }
    static inline float length3(const Vector3& v) { return std::sqrt(float(v.X * v.X + v.Y * v.Y + v.Z * v.Z)); }

    NodeStatus Task_ClimbToSafeAltitude::tick() {
        auto BBopt = getInput<CPPBlackBoard*>("BB");
        if (!BBopt || !BBopt.value()) return NodeStatus::FAILURE;
        CPPBlackBoard* BB = BBopt.value();

        const Vector3 myPos = BB->MyLocation_Cartesian;
        const Vector3 fwd = BB->MyForwardVector;
        const Vector3 right = BB->MyRightVector;

        // [회전2] 상승 방향은 월드 +Z 다. MyUpVector 는 쿼터니언에서 뽑은 *동체* up 이라
        // (DirectionVectorUpdate.cpp:32-34) 인버티드/고뱅크에서 Z<0 이 되고,
        // 고도 회복 노드가 오히려 지면으로 몬다. MyLocation_Cartesian.Z 는
        // LLAtoCartesian 의 dD = LLA.Z - BaseLLA.Z 이므로 미터 단위 고도다(LibMain.cpp:254).
        const Vector3 world_up{ 0.0, 0.0, 1.0 };

        // 근사 강하율: 전방벡터 Z성분 × 속도
        const float Vz_est = float(fwd.Z) * BB->MySpeed_MS;
        const float pitch_deg = std::asin(clampf(float(fwd.Z), -1.0f, 1.0f)) * 57.29578f;

        /*
        안전고도 [m]. MyLocation_Cartesian.Z 는 LLAtoCartesian 의 dD 이므로 미터 단위 고도다.

        [수정 2026-08-17] 1200 -> 450. Rule.xml 의 DECO_AltitudeCheck Altitude 와 같은 값이어야 한다.

        규정 §5: 추락선은 1000ft(약 300m), 초기 배치는 "약 2000~3000ft 고도대"(약 610~914m).
        1200m 는 **초기 배치 고도보다 높다**. 그대로 두면 경기 시작 순간부터 지면 회피 분기가
        Fallback 최상단을 계속 점유해 SCISSORS/HABFM/OBFM/DBFM 이 한 번도 실행되지 않는다.
        450m 는 추락선 300m 위로 150m 여유이고, 어떤 초기 배치 고도보다도 낮다.
        급강하 방어는 DECO_AltitudeCheck 의 3초 예측이 맡는다(같은 커밋).
        */
        const float kFloor = 450.0f;
        const float margin = 200.0f;
        const float curZ = float(myPos.Z);

        // ---- 핵심 1: "수평 전방"을 만들어 Z 음수 기여 제거 ----
        // [회전2] 월드 수평면으로 투영한다(Z 성분 제거). 이전에는 동체 up 으로
        // 투영했는데(fwd - up*dot(fwd,up)) 그것은 동체 기준 평면이라 뱅크가 걸리면
        // fwd_h.Z 가 0 이 되지 않아 "Z 음수 기여 제거"라는 의도가 성립하지 않았다.
        Vector3 fwd_h = fwd;  fwd_h.Z = 0.0;
        float mag = length3(fwd_h);
        if (mag < 1e-3f) {
            // 전방이 거의 위/아래로 향하면 수평 전방을 Right 의 수평 성분으로 대체
            fwd_h = right;  fwd_h.Z = 0.0;
            mag = length3(fwd_h);
            if (mag < 1e-3f) {
                // 혹시 모를 특이 케이스
                fwd_h = Vector3{ 1,0,0 };
                mag = 1.0f;
            }
        }
        fwd_h = fwd_h * (1.0f / mag); // 정규화

        // ---- 오프셋 크기: 속도 기반 가변 ----
        const float spd = std::max(50.0f, BB->MySpeed_MS); // 안전 하한
        const float ahead = clampf(150.0f + 1.2f * spd, 200.0f, 600.0f);
        const float climb = clampf(200.0f + 0.6f * spd, 250.0f, 800.0f);

        // ---- 목표점: "수평 전방 + 순수 상승" → Z는 언제나 증가 ----
        Vector3 vp = myPos + fwd_h * ahead + world_up * climb;

        // ---- BFM 차단(선택적이지만 강력 추천): 회복 중엔 다른 BFM 루트 진입 방지 ----
        // DECO_BFMCheck들이 BB->BFM을 검사하므로 NONE으로 비워두면 대부분 전략이 차단됨
        BB->BFM = NONE;

        // ---- 스로틀/VP 적용 ----
        BB->VP_Cartesian = vp;
        BB->Throttle = 1.0f;

        // ---- 종료 조건: 충분히 안전해졌는가? ----
        const bool safe_alt = (curZ >= (kFloor + margin));
        const bool good_pitch = (pitch_deg >= 2.0f);
        const bool climbing = (Vz_est > 5.0f);

        BT_VLOG("[Task_ClimbToSafeAltitude] Recovering"
            << " | pitch=" << pitch_deg
            << " Vz_est=" << Vz_est
            << " | ahead=" << ahead << " climb=" << climb
            << " | fwd_hZ=0 targetZ=" << vp.Z << " curZ=" << curZ
            << " | safe=" << (safe_alt && good_pitch && climbing ? "YES" : "NO")
            << "\n");

        // ---- 핵심 2: 회복 중에도 SUCCESS 를 돌려준다 ----
        // [회전2] 이 노드는 SyncActionNode 이고 SyncActionNode::executeTick() 은
        // 파생 클래스가 RUNNING 을 돌려주면 예외를 던진다(action_node.h:49-60).
        // 예전 구현은 회복 중 RUNNING 을 반환했으므로, Rule.xml 에 배선하는 순간
        // 첫 발화에서 바로 터졌을 것이다.
        //
        // 우선권은 RUNNING 이 아니라 Rule.xml 의 DECO_AltitudeCheck 가 매 tick
        // 재평가해서 유지한다. 트리는 매 프레임 루트부터 다시 tick 되므로,
        // 고도가 임계 아래인 동안 이 분기가 계속 먼저 잡히고 SUCCESS 가
        // Fallback 을 여기서 멈춰 다른 BFM 이 실행되지 않는다.
        return NodeStatus::SUCCESS;
    }

} // namespace Action
