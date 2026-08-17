#include "CPPBlackBoard.h"

CPPBlackBoard::CPPBlackBoard()
{
	RunningTime = 0;
	TickCount = 0;

	/*
	[수정 2026-08-17] 0.005566170 -> 1/60.

	RL 호스트가 SetBehaviorTreeDeltaTime 을 바인딩하지 않으므로 SetDeltaTime()은 호출되지
	않고 이 기본값이 그대로 누적된다. 0.005566170 은 규정 §4 의 프레임 간격(0.016666s)의
	정확히 1/3 이라, RunningTime 을 읽는 모든 로직이 3배 어긋난 시간 위에서 돌고 있었다.

	  - SetBFMMode_HABFM::UpdateTurnRates : dt 가 1/3 이므로 선회율이 3배로 부풀려져
	    1C/2C 를 잘못 고른다. 샘플 하한 0.2s 는 실제 0.6s, 상한 90deg/s 는 실제 30deg/s.
	  - Task_LeadPursuit  : SETTLE_DWELL_SEC 유지 판정이 실제로는 3배 길게 걸린다.
	  - 규정 §6 Phase 판정 : 200초 경기가 끝날 때까지 Phase 1 에서 못 벗어난다.

	호스트가 값을 넣어 주면 SetDeltaTime()이 이 기본값을 덮어쓴다(동작 불변).
	*/
	DeltaSecond = CPPBlackBoard::SIM_DT_SEC;

	MyLocation_Cartesian		= Vector3(0,0,0);
	TargetLocaion_Cartesian		= Vector3(0, 0, 0);
	VP_Cartesian				= Vector3(0, 0, 0);

	MyForwardVector = Vector3(0, 0, 0);
	MyUpVector		= Vector3(0, 0, 0);
	MyRightVector	= Vector3(0, 0, 0);

	TargetForwardVector = Vector3(0, 0, 0);
	TargetUpVector		= Vector3(0, 0, 0);
	TargetRightVector	= Vector3(0, 0, 0);

	MyRotation_EDegree		= EulerAngle(0,0,0);
	TargetRotation_EDegree	= EulerAngle(0, 0, 0);

	MySpeed_MS		= 0;
	TargetSpeed_MS	= 0;

	Distance = 0;
	Throttle = 0;


	Los_Degree = 0;
	Los_Degree_Target = 0;

	MyAngleOff_Degree = 0;
	MyAspectAngle_Degree = 0;

	BFM = NONE;
	ACM = EF;
	HABFM_CircleMode = CIRCLE_NONE;

	Team = UNKNOWN;

	EnergyCompareResult = 0;

	PredictedTargetVelocity = Vector3(0, 0, 0);

	IsCounterAttack = false; // DBFM 반격

	/*
	[수정 2026-08-17] 초기화 누락분.
	아래 값들은 생성자에서 초기화되지 않아 첫 tick 에서 미초기화 메모리를 읽었다.
	EnemyInSight 는 SetBFMMode_* 네 개 전부가 게이트 조건으로 읽고, AltSpeed 와
	MyAngleAcceleration 은 아무도 쓰지 않지만 값이 남아 돌아다닌다.
	*/
	MyAngleAcceleration = Vector3(0, 0, 0);
	AltSpeed = 0.0f;

	EnemyInSight = false;
	EnemyInSight_Target = false;

	IsEnergySuperior = false;
	IsEnergyInferior = false;

	PredictedTurnDirection = "STRAIGHT";

}

CPPBlackBoard::~CPPBlackBoard()
{
}
