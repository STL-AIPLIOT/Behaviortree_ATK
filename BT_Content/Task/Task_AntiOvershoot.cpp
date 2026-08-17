#include "Task_AntiOvershoot.h"
#include "../BTLog.h"
#include <algorithm>

namespace Action {  // �� �߰�

    static inline float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(hi, v));
    }

    BT::NodeStatus Task_AntiOvershoot::tick() {
        auto bb_res = getInput<CPPBlackBoard*>("BB");
        if (!bb_res) {
            std::cerr << "[Task_AntiOvershoot] BB nullptr\n";
            return BT::NodeStatus::FAILURE;
        }
        CPPBlackBoard* BB = bb_res.value();

        const float D = BB->Distance;
        const float dv = BB->MySpeed_MS - BB->TargetSpeed_MS;
        const bool trigger = (D < 600.0f) && (dv > 12.0f);           // [����]

        if (!trigger) {
            /*
            오버슛 위험이 없으면 이 노드는 할 일이 없다.
            예전에는 여기서 VP를 표적 위치로 덮어쓰고 SUCCESS를 반환했는데, 그러면
            OBFM_Action Fallback이 항상 이 노드에서 끝나 Task_LeadPursuit /
            Task_FollowTarget이 한 번도 실행되지 않았다. FAILURE로 양보한다.
            */
            BT_VLOG("[Task_AntiOvershoot] idle: D=" << D << ", dv=" << dv << "\n");
            return BT::NodeStatus::FAILURE;
        }

        const float back = std::max(150.0f, std::min(450.0f, 150.0f + 10.0f * dv)); // [����]
        BB->VP_Cartesian = BB->TargetLocaion_Cartesian - BB->TargetForwardVector * back;
        BT_VLOG("[Task_AntiOvershoot] triggered: D=" << D
            << ", dv=" << dv << ", back=" << back << "\n");

        return BT::NodeStatus::SUCCESS;
    }

} // namespace Action
