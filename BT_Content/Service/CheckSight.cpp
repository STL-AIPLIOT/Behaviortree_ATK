//내 시야에 대한 적기 위치와 적기 시야에 따른 내 위치를 통하여 각각의 시야안에 상대방이 존재하는지 업데이트

#include "CheckSight.h"

#include <algorithm>
#include <cmath>

namespace Action
{
	namespace
	{
		/*
		[추가 2026-08-17] Divide-by-zero / acos 정의역 방어.

		주최측 2026-05-26 패치노트: "기본 제공 Node들 — 극한의 확률이라도 Divide by 0 같은
		문제가 발생할수 있는 부분을 예외 처리". 이 노드는 그 처리를 받지 않은 팀 코드인데
		두 군데가 뚫려 있었다.

		  1) TV = TV / Distance  : 두 기체가 완전히 겹치면 Distance == 0 -> inf/NaN
		  2) acos(ForwardVector.dot(TV)) : 정규화 오차로 dot 이 1.0 을 아주 조금 넘으면 NaN

		예전에는 NaN 이 나도 어차피 크래시로 드러났지만, 같은 날 패치로
		bt_action_provider._vp_to_array() 가 비유한 VP 를 [0,0,0] 으로 조용히 치환하게 되어
		이제는 "지면을 향하라"는 멀쩡한 명령으로 둔갑한다. 반드시 여기서 막아야 한다.

		Vector3::angleBetween() 은 이미 같은 방어가 들어 있다(Vector3.h:243-255).
		이 노드는 그걸 쓰지 않고 직접 acos 를 부르고 있었다.
		*/
		inline float SafeAngleDeg(const Vector3& a, const Vector3& b)
		{
			const double lenProduct = a.length() * b.length();
			if (!(lenProduct > 1e-9))
			{
				// 길이 0 인 벡터끼리는 각을 정의할 수 없다. 시야 밖으로 보수적 판정.
				return 180.0f;
			}
			double d = a.dot(b) / lenProduct;
			d = std::max(-1.0, std::min(1.0, d));
			return static_cast<float>(std::acos(d) * 57.2958);
		}
	}

	PortsList CheckSight::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB"),
		};
	}

	NodeStatus CheckSight::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		
		//내 시야 정보
		Vector3 MyLocation = (*BB)->MyLocation_Cartesian;
		Vector3 TargetLocation = (*BB)->TargetLocaion_Cartesian;
		EulerAngle MyRotation = (*BB)->MyRotation_EDegree;

		MyRotation = MyRotation / 57.2958;
		Quaternion QU = MyRotation.toQuaternion();

		//쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성
		Vector3 ForwardVector;

		ForwardVector.Y = 2 * (QU.X*QU.Z + QU.W * QU.Y);
		ForwardVector.Z = -2 * (QU.Y*QU.Z - QU.W *  QU.X);
		ForwardVector.X = 1 - 2 * (QU.X*QU.X + QU.Y * QU.Y);


		Vector3 TV = TargetLocation - MyLocation;

		// SafeAngleDeg 가 길이로 나누므로 여기서 정규화하지 않는다(0 나눗셈 제거).
		float Los_Degree = SafeAngleDeg(ForwardVector, TV);

		(*BB)->Los_Degree = Los_Degree;



		if (Los_Degree <= 95.739166)
		{
			(*BB)->EnemyInSight = true;
		}
		else
		{
			(*BB)->EnemyInSight = false;
		}

		//적기 시야 정보
		Vector3 MyLocation2 = (*BB)->TargetLocaion_Cartesian;
		Vector3 TargetLocation2 = (*BB)->MyLocation_Cartesian;
		EulerAngle MyRotation2 = (*BB)->TargetRotation_EDegree;

		MyRotation2 = MyRotation2 / 57.2958;
		Quaternion QU2 = MyRotation2.toQuaternion();

		//쿼터니언을 이용하여 전방벡터(ForwardVector)를 생성
		Vector3 ForwardVector2;

		ForwardVector2.Y = 2 * (QU2.X*QU2.Z + QU2.W * QU2.Y);
		ForwardVector2.Z = -2 * (QU2.Y*QU2.Z - QU2.W *  QU2.X);
		ForwardVector2.X = 1 - 2 * (QU2.X*QU2.X + QU2.Y * QU2.Y);


		Vector3 TV2 = TargetLocation2 - MyLocation2;

		float Los_Degree2 = SafeAngleDeg(ForwardVector2, TV2);

		(*BB)->Los_Degree_Target = Los_Degree2;

		if (Los_Degree2 <= 95.739166)
		{
			(*BB)->EnemyInSight_Target = true;
		}
		else
		{
			(*BB)->EnemyInSight_Target = false;
		}


		
		return NodeStatus::SUCCESS;
	}

}