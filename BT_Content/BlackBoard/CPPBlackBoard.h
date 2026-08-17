#pragma once
#include "../../../Geometry/Vector3.h"
#include "../../../Geometry/EulerAngle.h"
#include <vector>

using namespace BT_Geometry;

enum BFM_Mode
{
	OBFM,
	HABFM,
	DBFM,
	DETECTING,
	SCISSORS,
	NONE

};

enum ACM_Mode
{
	EF,
	SF
};

enum TeamColor
{
	BLUE,
	RED,
	UNKNOWN
};

enum S_BFM_Mode
{
	S_OBFM,
	S_HABFM,
	S_DBFM,
	S_Others
};

/*
HABFM 교전에서 1-circle / 2-circle 중 무엇을 할지.
SetBFMMode_HABFM이 양측 선회율을 비교해 결정하고,
Task_OneCircleAttack / Task_TwoCircleAttack이 이 값을 읽어 자기 차례인지 판단한다.
CIRCLE_NONE = 아직 판단 불가(선회율 표본 부족) -> 두 Task 모두 양보한다.
*/
enum HABFM_Circle
{
	CIRCLE_NONE,
	ONE_CIRCLE,
	TWO_CIRCLE
};

enum WeaponMode
{
	Gun,
	Missile
};

/*
비행기들 객체 정보
자세, 위치, 속도, 팀, resv0(리눅스에서 ID), Resv1(비행기의 HP), Resv2(유인기/무인기)
*/
struct PlaneInfo
{
public:
	
	Vector3			Location;	//LLA Alt : Meter, 비헤비어트리로 입력할때는 LLA로 입력하지만 내부에서 사용할때는 Cartesian으로 사용
	
	EulerAngle		Rotation;	//Degree
	Vector3			AngleAcceleration;	//PQR
	
	float			Speed;		//m/s
	
	int				Team;		// 0 , 1
	float			Resv0;		//리눅스에서 ID로 쓰고있음
	float			Resv1;		//HP
	float			Resv2;		//유인기인지 무인기인지 판단용 0 : AI, 1 : Human

	PlaneInfo()
	{
		Location = Vector3(0, 0, 0);
		Rotation = EulerAngle(0, 0, 0);
		Speed = 0;
		Team = 0;
		Resv0 = 0;
		Resv1 = 0;
		Resv2 = 0;
	}
};

struct MissileTarget
{
public:
	int ListIndex;
	int DISID;
};

/*
그지같은 구조의 트리&블랙보드 구조를 개선해보기 위하여 만든 블랙보드 객체
비헤비어트리의 블랙보드 값을 여기에 선언-정의하고 이 블랙보드를 노드에서 호출하여 사용
모든 자세는 Degree이고 평면기준 자세를 기본으로 함

트리뿐만이 아니고 블랙보드의 변수들도 최대 2대 2까지만 상정하고 변수를 생성해둠
*/
class CPPBlackBoard
{
public:
	CPPBlackBoard();
	~CPPBlackBoard();

public:
	/*
	규정 §4: 서버는 60 Hz(DeltaTime 0.016666s)로 송신하고 접속기는 매 프레임 응답한다.
	이 값은 시간 의존 로직(WEZ Phase, 선회율 샘플링, 추종 유지시간)의 유일한 기준이다.
	*/
	static constexpr double SIM_DT_SEC = 1.0 / 60.0;

	double RunningTime;										//해당 시뮬레이션 실행시간
	double DeltaSecond;										//비헤비어트리 작동 틱 판단 및 시간 계산용
	long long TickCount;									//RunCPPBT 호출 횟수. MatchTimeSec()의 근거

	/*
	경기 경과 시간 [sec]. 시간 의존 판단은 RunningTime 이 아니라 이 값을 쓴다.

	왜 RunningTime 을 쓰면 안 되나
	------------------------------
	RunningTime 은 DeltaSecond 누적이다. 그런데 RL 호스트(src/dogfight/ai/native_bt.py)는
	CreateBehaviorTree / ChangeData / Step / GetVP / Reset / RemoveBT 여섯 개만 바인딩하고
	SetBehaviorTreeDeltaTime 은 **바인딩조차 하지 않는다**. 따라서 SetDeltaTime()이 한 번도
	호출되지 않고 CPPBlackBoard 생성자의 기본값이 그대로 쌓인다. 그 기본값이 0.005566170
	이었기 때문에 RunningTime 이 실제의 약 1/3 로 흘렀다(2026-08-15 보고서 실측:
	BFM t 58.21s <-> CSV 174.27s, tick 수는 10,458/10,456 으로 일치).

	그 상태로는 규정 §6 의 Phase 2/3(100s/150s) 진입 판정이 200초 경기 내내 성립하지 않는다.
	DeltaSecond 기본값은 아래 .cpp 에서 SIM_DT_SEC 로 바로잡았지만, 호스트가 다른 값을
	넣어 주는 경로가 생겨도 흔들리지 않도록 경과 시간 자체는 tick 수에서 직접 만든다.
	*/
	double MatchTimeSec() const { return static_cast<double>(TickCount) * SIM_DT_SEC; }

	std::vector<PlaneInfo> Friendly;						//아군기들 정보 Array
	std::vector<PlaneInfo> Enemy;							//적기들 정보 Array

	Vector3 MyLocation_Cartesian;							//내 위치 정보 Cartesian
	Vector3 TargetLocaion_Cartesian;						//타겟 적기 위치 정보 Cartesian
	Vector3 VP_Cartesian;									//추적점 위치 정보 Cartesian

	Vector3 MyForwardVector;								//내 전방 방향 벡터
	Vector3 MyUpVector;										//내 업 방향 벡터
	Vector3 MyRightVector;									//내 오른쪽 방향 벡터

	Vector3 TargetForwardVector;							//타겟 적기 전방 방향 벡터
	Vector3 TargetUpVector;									//타겟 적기 업 방향 벡터
	Vector3 TargetRightVector;								//타겟 적기 오른쪽 방향 벡터

	EulerAngle MyRotation_EDegree;							//내 자세, 평면 기준 자세 ,Degree
	EulerAngle TargetRotation_EDegree;						//타겟 적기 자세, 평면 기준 자세, Degree

	Vector3 MyAngleAcceleration;

	float MySpeed_MS;										//내 속도, meter/sec
	float TargetSpeed_MS;									//타겟 적기 속도. meter/sec

	float Distance;											//타겟 적기와의 거리, meter
	float Throttle;											//Throttle, 0~1


	float Los_Degree;										//타겟에 대한 LOS값
	float Los_Degree_Target;								//타겟이 나에 대한 LOS

	float MyAngleOff_Degree;								//타겟과의 기수 교차각
	float MyAspectAngle_Degree;								//타겟에 대한 AA값

	bool EnemyInSight;
	bool EnemyInSight_Target;

	BFM_Mode BFM;											//현재 BFM (OBFM, HABFM, DBFM, DETECTING, SCISSORS, NONE)
	ACM_Mode ACM;											//현재 ACM (EF, SF)

	HABFM_Circle HABFM_CircleMode;							//SetBFMMode_HABFM이 선회율로 결정한 1C/2C


	TeamColor Team;											//팀 컬러 (BLUE, RED, UNKNOWN)


	float AltSpeed;											//고도 변화량

	std::string PredictedTurnDirection;						// 적기 방향 저장

	bool IsEnergySuperior;									//energy 비교 변수
	bool IsEnergyInferior;									//energy 비교 변수
	int EnergyCompareResult;  // -1: 열세, 0: 같음, +1: 우세

	bool IsCounterAttack; // DBFM 반격

	Vector3 PredictedTargetVelocity;  // PredictManeuver 추가 로직


};
