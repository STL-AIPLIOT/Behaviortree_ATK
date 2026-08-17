#include "Task_MakeLOS.h"
#include "../BTLog.h"
#include <cmath>
#include <algorithm> // std::clamp
#include <iostream>  //  콘솔 로그

#ifndef DEG2RAD
#define DEG2RAD 0.017453292519943295
#endif
#ifndef RAD2DEG
#define RAD2DEG 57.29577951308232
#endif

using BT::InputPort;
using BT::NodeStatus;
using BT::Optional;

PortsList Action::Task_MakeLOS::providedPorts() {
    return { InputPort<CPPBlackBoard*>("BB") };
}

// --- 선택: 디버그 로그를 원하면 매크로 활성화 ---
// #define MAKELOS_DEBUG 1

NodeStatus Action::Task_MakeLOS::tick() {
    Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
    if (!BB) {
        BT_VLOG("[MakeLOS] 조건 미충족 → FAILURE (BB 입력 실패)" << std::endl);
        return NodeStatus::FAILURE;
    }
    CPPBlackBoard* bb = *BB;

    if (!bb || bb->Enemy.empty()) {
        BT_VLOG("[MakeLOS] 조건 미충족 → FAILURE (적기 없음)" << std::endl);
        return NodeStatus::FAILURE;
    }

    const PlaneInfo target = bb->Enemy[0];
    const Vector3   myPos = bb->MyLocation_Cartesian;
    const Vector3   targetPos = target.Location;

    const float dx = targetPos.X - myPos.X;
    const float dy = targetPos.Y - myPos.Y;
    const float dz = targetPos.Z - myPos.Z;
    const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    BT_VLOG("[MakeLOS] Distance=" << distance << std::endl);


    if (distance <= 1e-4f) {
        BT_VLOG("[MakeLOS] 조건 미충족 → FAILURE (거리=0)" << std::endl);
        return NodeStatus::FAILURE;
    }

    // 타깃을 향한 단위 벡터(LOS)
    const Vector3 toTarget = { dx / distance, dy / distance, dz / distance };

    // 내 전방벡터 정규화(동적 보간 알파 계산용)
    Vector3 myForward = bb->MyForwardVector;
    float fmag = std::sqrt(myForward.X * myForward.X + myForward.Y * myForward.Y + myForward.Z * myForward.Z);
    if (fmag > 1e-6f) {
        myForward.X /= fmag; myForward.Y /= fmag; myForward.Z /= fmag;
    }
    else {
        // 폴백: 전방이 비정상이면 LOS를 사용해 과도 조향 방지
        myForward = toTarget;
    }

    // 각도(전방 vs LOS). dot 보호는 반드시 [-1,1].
    float dot = myForward.X * toTarget.X + myForward.Y * toTarget.Y + myForward.Z * toTarget.Z;
    dot = (dot < -1.0f) ? -1.0f : (dot > 1.0f ? 1.0f : dot);
    float angleDeg = std::acos(dot) * RAD2DEG;

    BT_VLOG("[MakeLOS] angleDeg=" << angleDeg << std::endl);

    // === 핵심 정책 ===
    // MakeLOS는 "순수 정렬"만 수행: 항상 타깃 그 자체를 바라보게 함.
    // (리드/앞지점 계산은 Task_LeadPursuit에서 담당)
    Vector3 targetVP = targetPos;

    // === 스무딩(떨림 완화) ===
    // 각도가 클수록 빠르게 따라가고, 작을수록 부드럽게 고정되도록 알파를 동적으로 조정
    // alpha = base + k*(angle/limit), clamp to [min,max]

    /*
    const float baseAlpha = 0.15f;      // 최소 추종 속도
    const float angleForMax = 30.0f;    // 30° 이상이면 최대 알파 적용
    float alpha = baseAlpha + (angleDeg / angleForMax) * 0.45f; // 0.15 ~ 0.60 근방
    alpha = std::clamp(alpha, 0.15f, 0.60f);

    Vector3 vp = bb->VP_Cartesian;
    vp.X = vp.X * (1.0f - alpha) + targetVP.X * alpha;
    vp.Y = vp.Y * (1.0f - alpha) + targetVP.Y * alpha;
    vp.Z = vp.Z * (1.0f - alpha) + targetVP.Z * alpha;

    bb->VP_Cartesian = vp;
    */

    Vector3 tgtFwd = bb->TargetForwardVector;
    float fwdmag = std::sqrt(tgtFwd.X * tgtFwd.X + tgtFwd.Y * tgtFwd.Y + tgtFwd.Z * tgtFwd.Z);
    if (fwdmag > 1e-6f) {
        tgtFwd.X /= fwdmag; tgtFwd.Y /= fwdmag; tgtFwd.Z /= fwdmag;
    }
    else {
        tgtFwd = toTarget; // 폴백
    }

    // 50 m 앞지점
    const float AHEAD_M = 50.0f;
    Vector3 vp;
    vp.X = targetPos.X + tgtFwd.X * AHEAD_M;
    vp.Y = targetPos.Y + tgtFwd.Y * AHEAD_M;
    vp.Z = targetPos.Z + tgtFwd.Z * AHEAD_M;

    bb->VP_Cartesian = vp;

    // 속도 관리는 상위 노드에서 하도록 권장. 필요 시 유지/조정
    bb->Throttle = 1.0f;

    BT_VLOG("[MakeLOS] LOS 정렬 실행!" << std::endl);

#ifdef MAKELOS_DEBUG
    // 원하는 로거로 바꿔서 사용 (예: spdlog, UE_LOG, etc.)
    // printf("[MakeLOS] d=%.1f, ang=%.2f, alpha=%.2f, VP=(%.1f,%.1f,%.1f)\n",
    //        distance, angleDeg, alpha, vp.X, vp.Y, vp.Z);
#endif

    // 중요: 900m 경계에서 체인이 끊기지 않도록 항상 SUCCESS 반환
    // (적 없음/영거리 등 특수 케이스만 FAILURE)
    return NodeStatus::SUCCESS;
}
