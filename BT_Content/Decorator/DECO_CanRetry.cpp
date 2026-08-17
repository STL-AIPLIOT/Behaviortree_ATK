#include "DECO_CanRetry.h"
#include "../BTLog.h"
#include <iostream>

using namespace Action;

constexpr const char* DECO_CanRetry::MAX_RETRIES;

bool DECO_CanRetry::IsCounterAttackValid()
{
    auto bb_ptr = getInput<CPPBlackBoard*>("BB");

    // BB가 연결되어 있지 않으면 조건을 판단할 수 없으므로 재시도를 막지 않는다.
    if (!bb_ptr || bb_ptr.value() == nullptr)
    {
        std::cerr << "[CanRetry] BB 포인터를 가져오지 못함 - 반격 조건 재확인 생략\n";
        return true;
    }

    return bb_ptr.value()->IsCounterAttack;
}

void DECO_CanRetry::halt()
{
    retry_count_ = 0;
    BT::DecoratorNode::halt();
}

BT::NodeStatus DECO_CanRetry::tick()
{
    if (!getInput(MAX_RETRIES, max_retries_))
    {
        throw BT::RuntimeError("Missing parameter [", MAX_RETRIES, "] in CanRetry");
    }

    // 무한 재시도(-1 등)는 허용하지 않는다.
    if (max_retries_ < 0)
    {
        max_retries_ = 0;
    }

    setStatus(BT::NodeStatus::RUNNING);

    const BT::NodeStatus child_status = child_node_->executeTick();

    switch (child_status)
    {
        case BT::NodeStatus::RUNNING:
        {
            return BT::NodeStatus::RUNNING;
        }

        case BT::NodeStatus::SUCCESS:
        {
            retry_count_ = 0;
            return BT::NodeStatus::SUCCESS;
        }

        case BT::NodeStatus::FAILURE:
        {
            // Timeout 타이머를 포함한 자식 상태를 초기화한다.
            haltChild();

            if (retry_count_ < max_retries_ && IsCounterAttackValid())
            {
                retry_count_++;
                BT_VLOG("[CanRetry] 실패 - 다음 tick에서 재시도 "
                    << retry_count_ << "/" << max_retries_ << "\n");

                // 같은 tick에서 다시 실행하지 않고 다음 tick으로 넘긴다.
                return BT::NodeStatus::RUNNING;
            }

            retry_count_ = 0;
            BT_VLOG("[CanRetry] 재시도 종료 - FAILURE 반환\n");
            return BT::NodeStatus::FAILURE;
        }

        default:
        {
            throw BT::LogicError("A child node must never return IDLE");
        }
    }
}
