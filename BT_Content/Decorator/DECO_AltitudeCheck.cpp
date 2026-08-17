#include "DECO_AltitudeCheck.h"
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <exception>
#include <iostream>

namespace Action {

    DECO_AltitudeCheck::DECO_AltitudeCheck(const std::string& name, const NodeConfiguration& config)
        : SyncActionNode(name, config) {
    }

    DECO_AltitudeCheck::~DECO_AltitudeCheck() {}

    PortsList DECO_AltitudeCheck::providedPorts() {
        return {
            InputPort<CPPBlackBoard*>("BB"),
            InputPort<std::string>("UpDown"),
            InputPort<std::string>("Altitude")
        };
    }

    static inline float clampf(float v, float lo, float hi) { return std::max(lo, std::min(hi, v)); }

    NodeStatus DECO_AltitudeCheck::tick() {
        auto BBopt = getInput<CPPBlackBoard*>("BB");
        auto updownO = getInput<std::string>("UpDown");
        auto altO = getInput<std::string>("Altitude");
        if (!BBopt || !updownO || !altO) return NodeStatus::FAILURE;

        CPPBlackBoard* BB = BBopt.value();
        const std::string mode = updownO.value();

        // [수정 2026-08-17] std::stof 는 XML 오타에 대해 예외를 던진다. RunCPPBT 의 try-catch 가
        // 받아 주긴 하지만, 그 순간부터 경기 내내 안전 폴백으로 날게 된다. 여기서 막는다.
        float targetAlt = 0.0f;
        try
        {
            targetAlt = std::stof(altO.value());
        }
        catch (const std::exception&)
        {
            std::cerr << "[DECO_AltitudeCheck] bad Altitude port value: '"
                << altO.value() << "'\n";
            return NodeStatus::FAILURE;
        }

        // 현재 고도
        const float Z = static_cast<float>(BB->MyLocation_Cartesian.Z);

        // ★ 강하율 근사(벡터 없이): 전방벡터의 Z성분 × 속도(m/s)
        const float Vz_est = static_cast<float>(BB->MyForwardVector.Z) * BB->MySpeed_MS;

        // 기수 피치(도): 전방벡터 Z성분으로 근사
        const float pitch_deg = std::asin(clampf((float)BB->MyForwardVector.Z, -1.0f, 1.0f)) * 57.29578f;

        /*
        예측 고도.

        [수정 2026-08-17] 2.0s -> 3.0s.
        Rule.xml 의 정적 임계를 1200m 에서 450m 로 내렸기 때문에(규정 §5 초기 배치 고도 대응,
        아래 Rule.xml 주석 참조) 급강하에 대한 방어를 예측 구간이 대신 맡는다.
          -244 m/s 로 꽂히는 경우: 450 + 244*3 = 1182m 에서 발화 -> 예전 정적 1200m 과 사실상 동일
          -60  m/s 로 완만히 내려가는 경우: 450 + 180 = 630m 에서 발화
        즉 위험한 강하에는 예전만큼 일찍 개입하면서, 낮은 고도에서 수평 비행 중일 때
        분기를 통째로 점유하지는 않는다.
        */
        const float horizon = 3.0f;
        const float Z_pred = Z + Vz_est * horizon;

        if (mode == "Greater") {
            return (Z > targetAlt) ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
        }
        else if (mode == "Less") {
            // (1) 현재 고도 미만
            if (Z < targetAlt) return NodeStatus::SUCCESS;

            // (2) 여유 고도라도 급하강/기수하강이면 조기 개입
            const bool bad_descent = (Vz_est < -15.0f) || (pitch_deg < -5.0f);
            if ((Z < targetAlt + 150.0f) && bad_descent) return NodeStatus::SUCCESS;

            // (3) 예측 고도 미만
            if (Z_pred < targetAlt) return NodeStatus::SUCCESS;

            return NodeStatus::FAILURE;
        }
        else {
            return NodeStatus::FAILURE;
        }
    }

} // namespace Action
