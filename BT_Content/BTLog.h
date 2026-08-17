#pragma once

/*
틱 단위 진단 출력 게이트.

왜 필요한가 — 규정 §4
---------------------
서버는 60 Hz(DeltaTime 0.016666s)로 전장 정보를 보내고 접속기는 매 프레임 응답해야 하며,
"AI 연산 시간이 0.1667초를 초과하면 페널티"다. 그런데 이 트리의 여러 노드가 매 tick
std::cout 으로 한 줄씩 찍고 있었다(SetBFMMode_* 4종은 진입/차단 양쪽 모두, Task_FollowTarget·
Task_CornerLeadPursuit·Task_ClimbToSafeAltitude 는 무조건). 콘솔이 리다이렉트되거나 파이프가
막히면 write 가 블로킹되어 프레임 예산을 통째로 날린다. 대회 빌드에서는 조용해야 한다.

사용법
------
    BT_VLOG("[Node] msg " << value << "\n");

기본은 비활성이다. 진단이 필요할 때만 환경변수를 켠다.

    set STIL_BT_VERBOSE=1

파일로 받아야 하면 CPPBehaviorTree.cpp 의 BT_DIAG_LOG 를 쓴다. Python(ctypes)으로 DLL 을
구동할 때 std::cout 은 호출부 stdout 리다이렉트로 수집되지 않기 때문에(2026-08-04 실측),
어차피 이 매크로의 출력은 대부분의 실행 경로에서 보이지도 않았다.
*/

#include <cstdlib>
#include <iostream>
#include <string>

namespace BTLog
{
    // 프로세스당 한 번만 환경변수를 읽는다. 매 tick getenv 를 부르지 않기 위함이다.
    inline bool Verbose()
    {
        static const bool enabled = [] {
            const char* env = std::getenv("STIL_BT_VERBOSE");
            return env && *env && std::string(env) != "0";
        }();
        return enabled;
    }
}

#define BT_VLOG(streamExpr)                          \
    do {                                             \
        if (::BTLog::Verbose())                      \
        {                                            \
            std::cout << streamExpr;                 \
        }                                            \
    } while (false)
