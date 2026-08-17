#include "Task_MinimizeAngleOff.h"
#include "../BTLog.h"
#include <algorithm>
#include <cmath>
#include <iostream> // 콘솔 로그

#ifndef DEG2RAD
#define DEG2RAD 0.017453292519943295
#endif
#ifndef RAD2DEG
#define RAD2DEG 57.29577951308232
#endif

using BT::NodeStatus;
using BT::Optional;
using BT::InputPort;
using BT::PortsList;

namespace Action
{
    static inline float vecMag(const Vector3& v)
    {
        return std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
    }

    static inline void normalize(Vector3& v)
    {
        const float m = vecMag(v);
        if (m > 1e-6f) { v.X /= m; v.Y /= m; v.Z /= m; }
    }

    NodeStatus Task_MinimizeAngleOff::tick()
    {
        Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
        if (!BB) {
            BT_VLOG("[MinimizeAngleOff] 조건 미충족 → FAILURE (BB 입력 실패)" << std::endl);
            return NodeStatus::FAILURE;
        }
        CPPBlackBoard* bb = *BB;

        if (bb->Enemy.empty()) {
            BT_VLOG("[MinimizeAngleOff] 조건 미충족 → FAILURE (적기 없음)" << std::endl);
            return NodeStatus::FAILURE;
        }

        // 타겟 포즈/전방
        const PlaneInfo& tgt = bb->Enemy[0];
        Vector3 targetPos = tgt.Location;

        Vector3 targetFwd = bb->TargetForwardVector; // 서비스 노드에서 이미 계산됨
        normalize(targetFwd);
        if (vecMag(targetFwd) < 1e-6f) {
            BT_VLOG("[MinimizeAngleOff] 조건 미충족 → FAILURE (타겟 전방벡터 비정상)" << std::endl);
            return NodeStatus::FAILURE;
        }

        // 나와 타겟 간 거리
        Vector3 myPos = bb->MyLocation_Cartesian;
        const float dx = targetPos.X - myPos.X;
        const float dy = targetPos.Y - myPos.Y;
        const float dz = targetPos.Z - myPos.Z;
        float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (distance < 1e-3f) distance = 1e-3f;

        BT_VLOG("[MinimizeAngleOff] Distance=" << distance << std::endl);

        Vector3 myForward = bb->MyForwardVector;
        normalize(myForward);
        Vector3 toTarget = { dx / distance, dy / distance, dz / distance };

        float dot = myForward.X * toTarget.X + myForward.Y * toTarget.Y + myForward.Z * toTarget.Z;
        if (dot > 1.0f)  dot = 1.0f;
        if (dot < -1.0f) dot = -1.0f;

        float angleDeg = std::acos(dot) * RAD2DEG;
        BT_VLOG("[MinimizeAngleOff] angleDeg=" << angleDeg << std::endl);

        // 파라미터(없으면 기본값 사용)
        double lookMin = 200.0, lookMax = 500.0;
        if (auto v = getInput<double>("LookAheadMin")) lookMin = *v;
        if (auto v = getInput<double>("LookAheadMax")) lookMax = *v;
        if (lookMin > lookMax) std::swap(lookMin, lookMax);

        // 거리 비례 앞지점 (너무 멀면 과도, 너무 가까우면 미약하니 clamp)
        const double raw = static_cast<double>(distance) * 0.20;
        const double look = (raw < lookMin) ? lookMin : (raw > lookMax ? lookMax : raw);

        // 타겟의 전방으로 look만큼 앞지점을 조준
        Vector3 VP;
        VP.X = targetPos.X + targetFwd.X * look;
        VP.Y = targetPos.Y + targetFwd.Y * look;
        VP.Z = targetPos.Z + targetFwd.Z * look;

        // 조준점/스로틀 적용
        bb->VP_Cartesian = VP;
        bb->Throttle = 1.0f; // 속도 관리는 별도 CornerSpeed 노드에서 하도록

        BT_VLOG("[MinimizeAngleOff] Minimize AO 실행!" << std::endl);

        return NodeStatus::SUCCESS;
    }
}
