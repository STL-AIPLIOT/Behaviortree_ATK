#include "DECO_CounterAttackCheck.h"
#include "../BTLog.h"
#include <iostream>

using namespace Action;

BT::NodeStatus DECO_CounterAttackCheck::tick()
{
    auto bb_ptr = getInput<CPPBlackBoard*>("BB");

    if (!bb_ptr)
    {
        std::cerr << "[DECO_CounterAttackCheck] BB 포인터 가져오기 실패\n";
        return BT::NodeStatus::FAILURE;
    }

    CPPBlackBoard* BB = bb_ptr.value();

    if (BB->IsCounterAttack)
    {
        BT_VLOG("[DECO_CounterAttackCheck] 반격 조건 만족 → 하위 실행\n");
        return child_node_->executeTick();
    }
    else
    {
        BT_VLOG("[DECO_CounterAttackCheck] 반격 조건 불만족 → 차단\n");
        return BT::NodeStatus::FAILURE;
    }
}
