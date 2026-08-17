#include "PredictManeuver.h"
#include "PredictManeuverCsvLogger.h"
#include <cmath>

namespace Action
{
	PredictManeuver::PredictManeuver(
		const std::string& name,
		const NodeConfiguration& config
	)
		: SyncActionNode(name, config)
	{
	}

	PredictManeuver::~PredictManeuver()
	{
	}

	PortsList PredictManeuver::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus PredictManeuver::tick()
	{
		// Behavior Tree로부터 블랙보드 포인터를 가져온다.
		Optional<CPPBlackBoard*> BB =
			getInput<CPPBlackBoard*>("BB");

		// 입력 포트가 없거나 포인터가 nullptr이면 실행할 수 없다.
		if (!BB || !(*BB))
		{
			return NodeStatus::FAILURE;
		}

		/*
		BFM 모드는 Rule.xml에서 이 노드보다 뒤에 결정되므로,
		직전 프레임에 stage해 둔 각도 값을 지금 확정된 BFM 모드와 함께 기록한다.
		PM_CSV_LOG가 설정되지 않았다면 아무 동작도 하지 않는다.
		*/
		PredictManeuverCsvLogger& csvLogger =
			PredictManeuverCsvLogger::Instance();

		csvLogger.FlushPending((*BB)->BFM);

		const Vector3& currentPos =
			(*BB)->TargetLocaion_Cartesian;

		const EulerAngle& currentRot =
			(*BB)->TargetRotation_EDegree;

		const float currentYaw = currentRot.Yaw;

		// 위치 기록이 historySize를 넘지 않도록
		// 가장 오래된 데이터를 제거한다.
		if (prevPositions.size() >= historySize)
		{
			prevPositions.pop_front();
		}

		// 방향 기록이 historySize를 넘지 않도록
		// 가장 오래된 데이터를 제거한다.
		if (prevHeadings.size() >= historySize)
		{
			prevHeadings.pop_front();
		}

		// 현재 프레임의 위치와 방향을 기록한다.
		prevPositions.push_back(currentPos);
		prevHeadings.push_back(currentYaw);

		/*
		[추가 2026-08-17] BB->PredictedTargetVelocity 를 실제로 채운다.

		이 필드는 CPPBlackBoard 생성자에서 (0,0,0) 으로 초기화된 뒤
		**어느 노드도 값을 쓰지 않고 있었다**. 반면 읽는 곳은 넷이다:

		  Task_LeadPursuit.cpp:66        VP = TargetLocation + PredictedTargetVelocity * lead_time
		  Task_CornerLeadPursuit.cpp:49  VP = TargetLocation + PredictedTargetVelocity * lead_time
		  Task_LeadEntry.cpp:51
		  Task_AggressiveOBFM.cpp:155    (여기만 length()<=0.1 가드로 기수*속력 대체)

		즉 앞의 두 노드에서 식이 항상 VP = TargetLocation 으로 축약되어, 리드 추적을 한다고
		믿으면서 실제로는 **순수 추적(pure pursuit)** 을 하고 있었다. 선회하는 표적을 순수
		추적하면 리드각만큼 항상 뒤처지는데 대미지 콘은 규정 §6 Phase 1 기준 1deg 다.
		2026-08-15 로그 40판에서 후방 추적 사격 시간이 정확히 0.00초였던 직접적 원인이다.

		이 노드가 이미 표적 위치 이력(prevPositions)을 쌓고 있으므로 추가 상태 없이 만든다.
		속도는 이력 양 끝의 위치차 / 경과시간이다. 순간 차분(마지막 두 점)은 60Hz 에서
		노이즈가 커서 쓰지 않는다.
		*/
		if (prevPositions.size() >= 2)
		{
			const Vector3 oldest = prevPositions.front();
			const Vector3 newest = prevPositions.back();

			// 표본 간격 = (표본 수 - 1) * tick 간격. 규정 §4 의 60Hz 고정 프레임을 쓴다.
			const double span =
				static_cast<double>(prevPositions.size() - 1) * CPPBlackBoard::SIM_DT_SEC;

			if (span > 1e-6)
			{
				Vector3 v = (newest - oldest) / span;

				// 비유한 값이 블랙보드로 새어 나가면 VP 가 NaN 이 되고, 주최측 패치가
				// 그것을 조용히 [0,0,0](= 지면)으로 치환한다. 여기서 막는다.
				if (std::isfinite(v.X) && std::isfinite(v.Y) && std::isfinite(v.Z))
				{
					(*BB)->PredictedTargetVelocity = v;
				}
			}
		}

		// 충분한 방향 데이터가 쌓이지 않았다면
		// 아직 회전 방향을 판단하지 않는다.
		if (prevHeadings.size() < historySize)
		{
			return NodeStatus::SUCCESS;
		}

		float sumDelta = 0.0f;

		// CSV 기록용으로 가장 최근 한 쌍의 각도 값을 보관한다.
		float lastPreviousYaw = 0.0f;
		float lastCurrentYaw = 0.0f;
		float lastRawDelta = 0.0f;
		float lastNormalizedDelta = 0.0f;

		// 연속된 Yaw 값 사이의 각도 변화를 계산한다.
		for (size_t i = 1; i < prevHeadings.size(); ++i)
		{
			const float previousYaw =
				prevHeadings[i - 1];

			const float currentRecordedYaw =
				prevHeadings[i];

			const float rawDelta =
				currentRecordedYaw - previousYaw;

			// ±180도 경계를 통과할 때 발생하는
			// -358도, +358도 등의 잘못된 차이를 보정한다.
			// 공통 유틸리티(BT_Content/AngleUtil.h)를 통해서만 계산한다.
			const float normalizedDelta =
				BTAngle::SignedDeltaDeg(currentRecordedYaw, previousYaw);

			sumDelta += normalizedDelta;

			lastPreviousYaw = previousYaw;
			lastCurrentYaw = currentRecordedYaw;
			lastRawDelta = rawDelta;
			lastNormalizedDelta = normalizedDelta;
		}

		// 각도 차이의 개수는 방향 기록 개수보다 1개 적다.
		const float deltaCount =
			static_cast<float>(prevHeadings.size() - 1);

		const float avgDelta =
			sumDelta / deltaCount;

		// 기존 코드의 회전 방향 기준을 그대로 유지한다.
		// NaN이 들어오면 두 비교가 모두 false가 되어 STRAIGHT로 남는다.
		if (avgDelta > TURN_THRESHOLD_DEG)
		{
			(*BB)->PredictedTurnDirection = "LEFT";
		}
		else if (avgDelta < -TURN_THRESHOLD_DEG)
		{
			(*BB)->PredictedTurnDirection = "RIGHT";
		}
		else
		{
			(*BB)->PredictedTurnDirection = "STRAIGHT";
		}

		// 이번 프레임의 각도 값을 stage 한다.
		// 실제 기록은 다음 tick에서 BFM 모드가 확정된 뒤 이루어진다.
		// 방향 판정을 마친 뒤에 stage 해야 PredictedTurnDirection이 이번 프레임 값이 된다.
		if (csvLogger.IsEnabled())
		{
			PredictManeuverFrame frame;
			frame.time = (*BB)->RunningTime;
			frame.prevAngle = lastPreviousYaw;
			frame.currAngle = lastCurrentYaw;
			frame.rawDelta = lastRawDelta;
			frame.normalizedDelta = lastNormalizedDelta;
			frame.avgDelta = avgDelta;
			frame.predictedTurn = (*BB)->PredictedTurnDirection;
			// 아래 네 값은 이 노드가 계산하지 않고 블랙보드에서 그대로 읽는다.
			// 의미와 부호 규약은 PredictManeuverCsvLogger.h 의 컬럼 설명 참조.
			frame.distanceM = (*BB)->Distance;
			frame.ownAtaDeg = (*BB)->Los_Degree;
			frame.targetAaDeg = (*BB)->MyAspectAngle_Degree;
			frame.angleOffDeg = (*BB)->MyAngleOff_Degree;
			frame.enemyInSight = (*BB)->EnemyInSight;

			csvLogger.StageFrame(frame);
		}

		return NodeStatus::SUCCESS;
	}
}