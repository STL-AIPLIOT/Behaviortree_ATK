#include "EnergyCompare.h"

namespace Action
{
	EnergyCompare::EnergyCompare(const std::string& name, const NodeConfiguration& config)
		: SyncActionNode(name, config)
	{
	}

	EnergyCompare::~EnergyCompare() {}

	PortsList EnergyCompare::providedPorts()
	{
		return {
			InputPort<CPPBlackBoard*>("BB")
		};
	}

	NodeStatus EnergyCompare::tick()
	{
		Optional<CPPBlackBoard*> BB = getInput<CPPBlackBoard*>("BB");
		if (!BB || !(*BB)) return NodeStatus::FAILURE;

		const float g = 9.81f;

		float myV = (*BB)->MySpeed_MS;
		float myH = static_cast<float>((*BB)->MyLocation_Cartesian.Z);

		float targetV = (*BB)->TargetSpeed_MS;
		float targetH = static_cast<float>((*BB)->TargetLocaion_Cartesian.Z);

		float myEnergy = myV * myV + 2 * g * myH;
		float targetEnergy = targetV * targetV + 2 * g * targetH;

		/*
		[수정 2026-08-17] 데드밴드 없는 > 비교 -> 히스테리시스.

		무엇이 문제였나
		---------------
		예전 식은 (myEnergy > targetEnergy) 단순 비교였다. 동일 기종끼리 붙는 교전에서 두
		에너지는 사실상 같은 값이라 부호가 쉼 없이 뒤집힌다. 2026-08-15 로그 40판을 재구성해
		세어 보니 **교전당 평균 1,074회**, 초당 약 5회 뒤집혔다.

		이 값은 분기 선택의 1차 조건이다:
		  SetBFMMode_OBFM      : e_sup  = (EnergyCompareResult >  0)  필수
		  SetBFMMode_SCISSORS  : ecmp  <= 0                            필수
		  SetBFMMode_HABFM     : ec    >= 0                            필수
		  SetBFMMode_DBFM      : IsCounterAttack = (energy_cmp > 0)
		따라서 BFM 분기가 초당 5회 진동했고, StatefulActionNode 로 만든 1C/2C·LeadPursuit 이
		기동을 완주할 수 없었다(RUNNING 중에 다른 분기로 넘어가면 halt 된다).

		해결
		----
		직전 판정을 유지하다가 차이가 밴드를 넘을 때만 갈아탄다. 밴드 단위는 "비에너지 고도"[m]다.

		밴드 크기는 2026-08-15 로그 40판으로 직접 골랐다(교전 200초 기준 평균):

		    밴드[m]    부호전환/판    판정이 0이 아닌 시간[s]
		         0         1,075              199.7      <- 수정 전
		        25           671              199.7
		        50           405              199.7
		       100           166              199.6      <- 채택
		       200             8              184.4      <- 15초를 '판정 보류'로 날린다
		       400             4              179.6

		200m 이상은 전환이 거의 사라지지만 EnergyCompareResult 가 0 으로 머무는 시간이 길어져
		OBFM(>0 필수) 진입 자체를 막는다. 100m 는 전환을 1/6.5 로 줄이면서 판정 유효시간
		손실이 0.1초뿐이라 여기서 끊었다.

		직전 값은 블랙보드에 이미 남아 있으므로 노드에 별도 상태를 두지 않는다
		(노드 인스턴스가 재생성돼도 판정이 튀지 않는다).
		*/
		constexpr float DEADBAND_ENERGY_HEIGHT_M = 100.0f;
		const float band = 2.0f * g * DEADBAND_ENERGY_HEIGHT_M;   // v^2 + 2gh 단위로 환산

		const float diff = myEnergy - targetEnergy;

		int result = (*BB)->EnergyCompareResult;   // 밴드 안이면 직전 판정 유지
		if (diff > band)       { result = 1; }
		else if (diff < -band) { result = -1; }

		(*BB)->EnergyCompareResult = result;
		(*BB)->IsEnergySuperior = (result > 0);
		(*BB)->IsEnergyInferior = (result < 0);

		return NodeStatus::SUCCESS;
	}
}
