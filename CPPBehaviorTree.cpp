// Fill out your copyright notice in the Description page of Project Settings.


#include "CPPBehaviorTree.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <functional>
#include <string>


/*
Task가 쓰로틀을 정하지 않았을 때 쓰는 기본값.
1.0은 이 값을 도입하기 전의 동작(Step에서 무조건 1.0으로 덮어쓰던 것)과 같다.
에너지 관리를 실험할 때 여기부터 낮춰 본다.
*/
static constexpr float DEFAULT_THROTTLE = 1.0f;


/*
BT 진단 로그 (환경변수 BT_DIAG_LOG 가 있을 때만 동작)
-----------------------------------------------------
BT 노드가 std::cout 으로 찍는 진단은 Python(ctypes)으로 DLL 을 구동할 때
호출부의 stdout 리다이렉트로 수집되지 않는다(2026-08-04 실측: 3000 tick 에도 0줄,
SetStdHandle 로 핸들을 바꿔도 0바이트). 그래서 tick 이 실제로 도는지조차 확인할 수
없었다. 이 로거는 stdout 을 거치지 않고 파일에 직접 쓴다.

    BT_DIAG_LOG=logs/bt_diag.txt   # 지정하지 않으면 완전히 비활성
*/
namespace
{
	std::ofstream& BtDiagStream()
	{
		static std::ofstream stream = [] {
			std::ofstream out;
			const char* path = std::getenv("BT_DIAG_LOG");
			if (path && *path)
			{
				out.open(path, std::ios::out | std::ios::trunc);
			}
			return out;
		}();
		return stream;
	}

	bool BtDiagEnabled()
	{
		return BtDiagStream().is_open();
	}

	void BtDiag(const std::string& line)
	{
		if (!BtDiagEnabled())
		{
			return;
		}
		BtDiagStream() << line << "\n";
		BtDiagStream().flush();   // 크래시로 잃지 않도록 매번 flush
	}

	const char* StatusName(BT::NodeStatus s)
	{
		switch (s)
		{
		case BT::NodeStatus::IDLE:    return "IDLE";
		case BT::NodeStatus::RUNNING: return "RUNNING";
		case BT::NodeStatus::SUCCESS: return "SUCCESS";
		case BT::NodeStatus::FAILURE: return "FAILURE";
		}
		return "?";
	}

	bool IsFiniteVector(const Vector3& v)
	{
		return std::isfinite(v.X) && std::isfinite(v.Y) && std::isfinite(v.Z);
	}

	/*
	비유한(NaN/Inf) VP 가 나갔을 때 쓸 안전 추종점.

	왜 (0,0,0) 이면 안 되나
	-----------------------
	주최측 패치(2026-05-26)로 bt_action_provider._vp_to_array() 는 GetVP() 결과가 finite 가
	아니면 조용히 SAFE_VP = [0,0,0] 으로 갈아끼운다(vp_valid=False 는 info 에만 실린다).
	그런데 우리 좌표계에서 (0,0,0) 은 기준 LLA(OriLAT/OriLOn) 의 **고도 0m 지점**, 즉 지면이다.
	NaN 한 번이 "지면을 향해 날아가라"는 유효한 명령으로 둔갑한다. 규정 §5 의 300m 추락선을
	생각하면 이건 패배 직행이다.

	그래서 자체적으로 먼저 걸러내고, 수평 전방 + 완만한 상승점을 돌려준다.
	*/
	Vector3 SafeFallbackVP(const Vector3& pos, const Vector3& fwd)
	{
		if (!IsFiniteVector(pos))
		{
			// 위치조차 못 믿으면 상대 오프셋도 의미가 없다. 충분히 높은 절대점.
			return Vector3(0.0, 0.0, 5000.0);
		}

		Vector3 fwd_h(1.0, 0.0, 0.0);
		if (IsFiniteVector(fwd))
		{
			Vector3 h(fwd.X, fwd.Y, 0.0);
			const double mag = h.length();
			if (mag > 1e-6) { fwd_h = h / mag; }
		}

		// 수평 전방 1000m + 상방 300m = 날개 수평 완만 상승
		return Vector3(pos.X + fwd_h.X * 1000.0,
			pos.Y + fwd_h.Y * 1000.0,
			pos.Z + 300.0);
	}
}


Vector3 UCPPBehaviorTree::LLAtoCartesian(Vector3 LLA, Vector3 BaseLLA)
{
	double eccentricitysquare, N, M;
	eccentricitysquare = 1.0 - pow(6356752.3142, 2) / pow(6378137.0, 2);
	N = 6378137.0 / sqrt(1.0 - eccentricitysquare * pow(sin(BaseLLA.X * PI / 180.0), 2)); // prime vertical radius of curvature
	// [수정 2026-08-17] 지수 3 / 2 는 정수 나눗셈이라 1 로 평가된다(자오선 곡률반경 M 이
	// 약 0.34% 어긋난다). 1.5 로 바로잡는다. 원식은 M = a(1-e^2) / (1-e^2 sin^2)^(3/2).
	M = 6378137.0 * (1.0 - eccentricitysquare) / pow(1 - eccentricitysquare * pow(sin(BaseLLA.X * PI / 180.0), 2), 1.5);

	double dlat, dlon;
	dlat = LLA.X - BaseLLA.X;
	dlon = LLA.Y - BaseLLA.Y;

	double dN, dE, dD;
	dN = (M + BaseLLA.Z) * dlat * PI / 180.0;
	dE = (N + BaseLLA.Z) * cos(BaseLLA.X * PI / 180.0) * dlon * PI / 180.0;
	dD = (LLA.Z - BaseLLA.Z);
	Vector3 res(dN, dE, dD);
	return res;
}

// Sets default values for this component's properties
UCPPBehaviorTree::UCPPBehaviorTree()
{
	// [추가 2026-08-17] LibMain.cpp 가 CreateBehaviorTree 에서 덮어쓰지만,
	// 그 전에 읽히는 경로가 생겨도 쓰레기 값이 돌아다니지 않게 초기화한다.
	ID = -1;
	ForceID = -1;
	bInitialized = false;

	f2m = 3.28084;
	EQ_R = 6.378137E+6;
	P_R = 6.3567523142E+6;
	fr = 298.257223563;
	Req = 6.378137E+6;
	d2r = 3.1415926535897931 / 180.0;
	m2f = 3.28084;


	elev0 = 0.2;
	aile0 = 0.0;
	eccen = 1.0 - P_R * P_R / (EQ_R * EQ_R);

	BB = new CPPBlackBoard();

	//std::cout << "Behavior Tree Version : 2022.07.11" << std::endl;
}


UCPPBehaviorTree::~UCPPBehaviorTree()
{
	delete BB;
}


/*
노드 입력 : 구현해둔 노드들을 Factory 객체에 입력해주는 과정

새로 생성한 노드를 여기에 입력해주세요!!!!!!

[분리 2026-08-17] 예전에는 init() 본문이었다. init() 을 try-catch 로 감싸면서
등록 목록까지 한 단계 더 들여쓰기하면 읽기 어려워져 별도 함수로 뺐다. 동작은 같다.
*/
void UCPPBehaviorTree::RegisterNodes()
{
	Factory.registerNodeType<Action::SelectTarget>("SelectTarget");
	Factory.registerNodeType<Action::DistanceUpdate>("DistanceUpdate");
	Factory.registerNodeType<Action::CheckSight>("CheckSight");
	Factory.registerNodeType<Action::AngleOffUpdate>("AngleOffUpdate");
	Factory.registerNodeType<Action::DirectionVectorUpdate>("DirectionVectorUpdate");
	Factory.registerNodeType<Action::AspectAngleUpdate>("AspectAngleUpdate");
	Factory.registerNodeType<Action::DECO_BFMCheck>("DECO_BFMCheck");
	Factory.registerNodeType<Action::DECO_DistanceCheck>("DECO_DistanceCheck");
	Factory.registerNodeType<Action::Task_LeadEntry>("Task_LeadEntry");
	Factory.registerNodeType<Action::Task_Pure>("Task_Pure");

	Factory.registerNodeType<Action::PredictManeuver>("PredictManeuver");
	Factory.registerNodeType<Action::EnergyCompare>("EnergyCompare");
	Factory.registerNodeType<Action::DECO_AltitudeCheck>("DECO_AltitudeCheck");
	Factory.registerNodeType<Action::Task_FollowTarget>("Task_FollowTarget");
	Factory.registerNodeType<Action::Task_ClimbToSafeAltitude>("Task_ClimbToSafeAltitude");

	Factory.registerNodeType<Action::SetBFMMode_OBFM>("SetBFMMode_OBFM");
	Factory.registerNodeType<Action::Task_AggressiveOBFM>("Task_AggressiveOBFM");
	// Task_AntiOvershoot 은 Rule.xml 에서 Task_AggressiveOBFM 으로 교체됐지만 등록은 남긴다.
	// 등록을 지우면 예전 Rule.xml 을 그대로 쓰는 순간 createTreeFromFile 이 예외로 죽는다.
	Factory.registerNodeType<Action::Task_AntiOvershoot>("Task_AntiOvershoot");
	Factory.registerNodeType<Action::Task_CornerLeadPursuit>("Task_CornerLeadPursuit");
	Factory.registerNodeType<Action::Task_LeadPursuit>("Task_LeadPursuit");

	Factory.registerNodeType<Action::SetBFMMode_DBFM>("SetBFMMode_DBFM");
	Factory.registerNodeType<Action::Task_EvasiveRollOrScissors>("Task_EvasiveRollOrScissors");
	Factory.registerNodeType<Action::Task_CounterTurn>("Task_CounterTurn");
	Factory.registerNodeType<Action::DECO_CounterAttackCheck>("DECO_CounterAttackCheck");
	Factory.registerNodeType<Action::DECO_CanRetry>("CanRetry");
	Factory.registerNodeType<Action::Task_RollReverseAttack>("Task_RollReverseAttack");


	Factory.registerNodeType<Action::SetBFMMode_HABFM>("SetBFMMode_HABFM");
	Factory.registerNodeType<Action::Task_OneCircleAttack>("Task_OneCircleAttack");
	Factory.registerNodeType<Action::Task_TwoCircleAttack>("Task_TwoCircleAttack");

	Factory.registerNodeType<Action::SetBFMMode_SCISSORS>("SetBFMMode_SCISSORS");
	Factory.registerNodeType<Action::Task_ScissorBreakTurn>("Task_ScissorBreakTurn");
	Factory.registerNodeType<Action::Task_ScissorRollBack>("Task_ScissorRollBack");

	Factory.registerNodeType<Action::Task_MakeLOS>("Task_MakeLOS");
	Factory.registerNodeType<Action::Task_MinimizeAngleOff>("Task_MinimizeAngleOff");
	Factory.registerNodeType<Action::Task_CloseDistance>("Task_CloseDistance");
}


bool UCPPBehaviorTree::IsInitialized() const
{
	return bInitialized;
}


void UCPPBehaviorTree::init()
{
	/*
	[수정 2026-08-17] 주최측 2026-05-26 패치 대응 — init() 전체를 try-catch 로 감싼다.

	패치된 LibMain.cpp::CreateBehaviorTree 는 init() 을 부른 뒤 IsInitialized() 가 true 일
	때만 BTList 에 등록한다. XML 이름 오타 하나로 createTreeFromFile 이 던지면 예전에는
	ctypes 경계를 넘어 OSError: [WinError -529697949] 라는 정체불명 메시지만 남았다.
	여기서 원인을 사람이 읽을 수 있는 형태로 찍고 다시 던진다.

	init 실패는 삼키지 않는다 — 빌드/배치 실수는 경기 전에 크게 터지는 편이 낫다.
	런타임 예외를 삼키는 곳은 RunCPPBT() 쪽이다(경기 중 크래시 = 규정 §7 응답 불능 = 패배).
	*/
	bInitialized = false;

	std::string rulePath;

	try
	{
	RegisterNodes();

	//파일로 트리 구조 정의
	//
	// 파일명을 하드코딩하지 않는다. 환경변수 BT_RULE_XML 이 있으면 그 경로를 쓰고,
	// 없으면 팀 고유 이름인 "./Rule_STIL_ATK.xml" 이다.
	//
	// [수정 2026-08-17] 기본값 "./Rule.xml" -> "./Rule_STIL_ATK.xml".
	// 규정 §9 제출물이 "코드·모델·XML"이고, Release 루트에는 벤더 DLL 이 읽는 XML 과
	// 우리 XML 이 함께 놓인다. 일반명 Rule.xml 을 쓰면 벤더 AIP_BASE_target.dll 이
	// 우리 XML 을 집어 자기 노드를 못 찾고 죽는다(아래 1번과 같은 사고).
	// DLL 과 XML 은 한 세트이므로 이름도 함께 버전을 맞춘다.
	//
	// 왜 필요했나 — 하드코딩 하나가 세 가지를 동시에 막고 있었다:
	//   1) Release 루트에 Rule.xml 을 하나만 둘 수 있어, 팀 XML 을 놓으면
	//      벤더 AIP_BASE_target.dll 이 자기 노드를 못 찾고 C++ 예외로 죽었다
	//      (ctypes 경계를 넘으면 OSError: [WinError -529697949] 로만 보인다).
	//   2) 제출용으로 Rule_<team>.xml 로 바꿀 수 없었다.
	//   3) XML 을 골라 쓰는 구조가 성립하지 않았다.
	//
	// export 를 늘리지 않고 환경변수로 연 이유: native_bt.py 가 바인딩하는 export 는
	// 6종(CreateBehaviorTree/ChangeData/Step/GetVP/Reset/RemoveBT)으로 고정이고,
	// 여기에 추가하면 호스트 쪽 수정이 필요해진다(수정 금지 영역).
	// 상대경로는 CWD 기준이다 — 실행 디렉터리가 Release 루트여야 한다.
	rulePath = "./Rule_STIL_ATK.xml";
	if (const char* envPath = std::getenv("BT_RULE_XML"))
	{
		if (envPath[0] != '\0') { rulePath = envPath; }
	}
	if (BtDiagEnabled())
	{
		/*
		[A/계측 2026-08-19] 경로만으로는 "어느 XML 이 실제로 읽혔는가" 를 못 가린다.
		v3p 에서 우리 XML(8,404 B) 대신 5,947 B 짜리 옛 파일이 읽힌 사고가 있었는데,
		경로 문자열은 정상으로 보였다. 같은 이름의 다른 파일이었기 때문이다.
		크기를 함께 남겨 파일 동일성을 바로 판별한다.
		*/
		std::streamoff xml_bytes = -1;
		{
			std::ifstream probe(rulePath, std::ios::binary | std::ios::ate);
			if (probe.is_open()) { xml_bytes = probe.tellg(); }
		}
		BtDiag("[init] rule xml = " + rulePath +
			"  bytes=" + (xml_bytes >= 0 ? std::to_string(xml_bytes) : std::string("(열기 실패)")));
	}
	tree = Factory.createTreeFromFile(rulePath);

	// 트리가 실제로 만들어졌는지 (노드 개수 포함) 파일로 남긴다.
	if (BtDiagEnabled())
	{
		BtDiag("[init] createTreeFromFile OK, nodes=" +
			std::to_string(tree.nodes.size()) +
			", root=" + (tree.rootNode() ? tree.rootNode()->name() : std::string("(null)")));

		// 어떤 노드가 실제로 파싱됐는지 전부 남긴다.
		for (const auto& node : tree.nodes)
		{
			BtDiag(std::string("[init]   node: ") + node->name() +
				"  (type=" + node->registrationName() + ")");
		}

		// 실제 부모-자식 계층. tree.nodes 는 평면 목록이라 연결 상태를 보여주지 않는다.
		// 이 벤더 파서는 자식을 노드 '이름 문자열'로 연결하도록 개조돼 있어
		// (xml_parsing.cpp:599-604) 계층이 어긋날 수 있다.
		std::function<void(BT::TreeNode*, int)> dump = [&](BT::TreeNode* n, int depth)
		{
			if (!n) { return; }
			std::string pad(static_cast<size_t>(depth) * 2, ' ');
			if (auto* ctrl = dynamic_cast<BT::ControlNode*>(n))
			{
				BtDiag("[tree] " + pad + n->name() + " [Control, children=" +
					std::to_string(ctrl->children().size()) + "]");
				for (auto* c : ctrl->children()) { dump(c, depth + 1); }
			}
			else if (auto* deco = dynamic_cast<BT::DecoratorNode*>(n))
			{
				BtDiag("[tree] " + pad + n->name() + " [Decorator, child=" +
					(deco->child() ? deco->child()->name() : std::string("(null)")) + "]");
				dump(deco->child(), depth + 1);
			}
			else
			{
				BtDiag("[tree] " + pad + n->name() + " [Leaf]");
			}
		};
		dump(tree.rootNode(), 0);
	}
	
	//문자열로 트리 구조 정의
	//std::string XML = StrCat(xml_text1, xml_text2);
	////std::cout << XML << std::endl;
	//tree = Factory.createTreeFromText(XML);

	//블랙보드 연결 : 원래는 블랙보드 내에 있는 모든 변수를 하나하나 이런식으로 입력해줘야하는 미친 비효율을 보이는 방식이지만 커스텀 블랙보드를 만들어 해당 블랙보드를 입력시킴
	tree.rootBlackboard()->set<CPPBlackBoard*>("BB", BB);

	bInitialized = true;
	std::cout << "Behavior Tree Initialized Successfully (rule=" << rulePath
		<< ", nodes=" << tree.nodes.size() << ")" << std::endl;
	}
	catch (const std::exception& e)
	{
		bInitialized = false;

		std::cout << "Behavior Tree Initialization Failed: " << e.what() << std::endl;
		std::cout << "  rule xml = " << rulePath << " (override: BT_RULE_XML)" << std::endl;
		std::cout << "It appears that the process failed while parsing the XML." << std::endl;
		std::cout << " -Please check whether the XML file is located in the correct path." << std::endl;
		std::cout << " -Please check whether the XML file is calling any node with an invalid or incorrect name." << std::endl;
		std::cout << " -Please check whether the node was added to the Factory when building the DLL." << std::endl;

		if (BtDiagEnabled()) { BtDiag(std::string("[init] FAILED: ") + e.what() + " rule=" + rulePath); }
		throw;
	}
}

StickValue UCPPBehaviorTree::Step(PlaneInfo MyInfo, int NumofOtherPlane, PlaneInfo* OthersInfo, Vector3 & VP, float & Throttle)
{
	/*LLA 좌표를 Cartesian 좌표로 변경
			
	굳이 리눅스의 기준 좌표 37, 127로 맞출 필요 없음
	*/
	Vector3 Mylocation_Cartesian = LLAtoCartesian(MyInfo.Location, Vector3(OriLAT, OriLOn, 0));

	//Cartesiam 좌표계로 위치 정보를 바꾼 내 비행기 정보
	PlaneInfo Myinfo;
	Myinfo.Location = Mylocation_Cartesian;
	Myinfo.Rotation = EulerAngle(MyInfo.Rotation.Yaw, MyInfo.Rotation.Pitch, MyInfo.Rotation.Roll);
	Myinfo.AngleAcceleration = MyInfo.AngleAcceleration;
	Myinfo.Speed = MyInfo.Speed;
	Myinfo.Team = MyInfo.Team;
	Myinfo.Resv0 = MyInfo.Resv0;		//ID
	Myinfo.Resv1 = MyInfo.Resv1;		//HP
	Myinfo.Resv2 = MyInfo.Resv2;		//OperationMode


	//HP가 0이하가 되면 자유 낙하하도록 설정
	if (Myinfo.Resv1 <= 0)
	{
		StickValue R;

		BB->VP_Cartesian = Vector3(BB->MyLocation_Cartesian.X, BB->MyLocation_Cartesian.Y, 0);
		R = Controller.GetStick(
			BB->MyLocation_Cartesian,
			Vector3(BB->MyRotation_EDegree.Roll*DEG2RAD,
				BB->MyRotation_EDegree.Pitch*DEG2RAD,
				BB->MyRotation_EDegree.Yaw*DEG2RAD),
			BB->VP_Cartesian);
		BB->Throttle = 0;
		R.RudderCMD = 100;

		std::cout << " HP : 0 !!!!!!!!!" << std::endl;
		return R;
	}

	//HP가 0이상일때
	else
	{
		//다른 비행기들 위치 좌표계 변환
		PlaneInfo others[4];
		for (int i = 0; i < NumofOtherPlane; i++)
		{
			Vector3 Enemylocation_Cartesian = LLAtoCartesian(OthersInfo[i].Location, Vector3(OriLAT, OriLOn, 0));
			others[i].Location = Enemylocation_Cartesian;
			others[i].Rotation = EulerAngle(OthersInfo[i].Rotation.Yaw, OthersInfo[i].Rotation.Pitch, OthersInfo[i].Rotation.Roll);
			others[i].Speed = OthersInfo[i].Speed;
			others[i].Team = OthersInfo[i].Team;
			others[i].Resv0 = OthersInfo[i].Resv0;
			others[i].Resv1 = OthersInfo[i].Resv1;
			others[i].Resv2 = OthersInfo[i].Resv2;
			
			
		}

		//블랙보드의 아군기, 적군기 List 초기화
		BB->Friendly.clear();
		BB->Enemy.clear();

		//블랙보드에 내 정보(위치, 자세, 속력, 팀) 업데이트
		BB->MyLocation_Cartesian = Mylocation_Cartesian;
		BB->MyRotation_EDegree = EulerAngle(Myinfo.Rotation.Yaw, Myinfo.Rotation.Pitch, Myinfo.Rotation.Roll);
		BB->MyAngleAcceleration = Myinfo.AngleAcceleration;
		BB->MySpeed_MS = Myinfo.Speed;
		BB->Team = (TeamColor)Myinfo.Team;

		//아군기 리스트에 내 정보 추가. Friendly의 index 0번은 무조건 나 자신
		BB->Friendly.push_back(Myinfo);

		//생존중인 비행기들의 적아 구분
		for (int i = 0; i < NumofOtherPlane; i++)
		{
			if (others[i].Resv1 > 0)
			{
				if (others[i].Team == Myinfo.Team)
				{
					BB->Friendly.push_back(others[i]);
				}
				else
				{
					BB->Enemy.push_back(others[i]);
				}
			}
			else
			{

			}
		}


		bool AimmingMode;

		StickValue R;

		//블랙보드에 입력된 정보를 바탕으로 비헤비어트리 Run
		//(예전에는 여기서 Throttle = 1.0 으로 덮어써서 BT가 정한 값이 버려졌다.
		// 지금은 RunCPPBT가 tick 전에 기본값을 넣고, Task가 덮어쓴 값을 그대로 내보낸다.)
		if (BtDiagEnabled())
		{
			std::string m = "[preTick] NumOther=" + std::to_string(NumofOtherPlane) +
				" EnemySize=" + std::to_string(BB->Enemy.size()) +
				" myLLA=(" + std::to_string(MyInfo.Location.X) + "," +
				std::to_string(MyInfo.Location.Y) + "," + std::to_string(MyInfo.Location.Z) + ")";
			if (NumofOtherPlane > 0)
			{
				m += " othLLA=(" + std::to_string(OthersInfo[0].Location.X) + "," +
					std::to_string(OthersInfo[0].Location.Y) + "," +
					std::to_string(OthersInfo[0].Location.Z) + ")" +
					" othCart=(" + std::to_string(others[0].Location.X) + "," +
					std::to_string(others[0].Location.Y) + "," +
					std::to_string(others[0].Location.Z) + ")";
			}
			if (!BB->Enemy.empty())
			{
				m += " Enemy0=(" + std::to_string(BB->Enemy[0].Location.X) + "," +
					std::to_string(BB->Enemy[0].Location.Y) + "," +
					std::to_string(BB->Enemy[0].Location.Z) + ")";
			}
			// 트리 노드가 실제로 받는 BB 와 이 객체의 BB 가 같은지 대조한다.
			{
				CPPBlackBoard* fromTree = nullptr;
				try { fromTree = tree.rootBlackboard()->get<CPPBlackBoard*>("BB"); }
				catch (...) { fromTree = reinterpret_cast<CPPBlackBoard*>(-1); }
				char buf[64];
				snprintf(buf, sizeof(buf), " thisBB=%p treeBB=%p",
					static_cast<void*>(BB), static_cast<void*>(fromTree));
				m += buf;
				m += (fromTree == BB) ? " SAME" : " *** DIFFERENT ***";
				if (fromTree && fromTree != reinterpret_cast<CPPBlackBoard*>(-1))
				{
					m += " treeBB.EnemySize=" + std::to_string(fromTree->Enemy.size());
				}
			}
			BtDiag(m);
		}

		RunCPPBT(VP, Throttle, AimmingMode);


		R = Controller.GetStick(
			BB->MyLocation_Cartesian,
			Vector3(BB->MyRotation_EDegree.Roll*DEG2RAD,
					BB->MyRotation_EDegree.Pitch*DEG2RAD,
					BB->MyRotation_EDegree.Yaw*DEG2RAD),
				VP);

 
		
		return R;
	}
}

Vector3 UCPPBehaviorTree::GetVP()
{
	Vector3 Vp = (*BB).VP_Cartesian;
	return Vp;
}


 void UCPPBehaviorTree::RunCPPBT(Vector3& VP, float& Throttle, bool& AimmingMode)
{

	BB->RunningTime += BB->DeltaSecond;	//시뮬레이선 타임에 따른 델타 타임 설정
	BB->TickCount += 1;					//규정 §6 Phase 판정의 시간 근거 (BB->MatchTimeSec())

	// 예외로 트리가 중단되더라도 이번 tick 의 출력이 남아 있어야 하므로 미리 잡아 둔다.
	const Vector3 posAtEntry = BB->MyLocation_Cartesian;
	const Vector3 fwdAtEntry = BB->MyForwardVector;

	/*
	tick 전에 기본 쓰로틀을 넣어 둔다.
	대부분의 Task는 BB->Throttle을 건드리지 않으므로, 기본값이 없으면 블랙보드 초기값 0이
	그대로 나가 추력이 사라진다. Task가 값을 정하면 그 값이 그대로 살아남는다.
	*/
	BB->Throttle = DEFAULT_THROTTLE;

	/*
	[C-2 실험] tick 전에 트리 상태를 리셋한다.
	루트 Sequence 가 이전 tick 의 상태에 고착돼 자식을 돌지 않는다는 가설을 검증한다.
	BT_FORCE_HALT 를 설정했을 때만 동작한다(기본 동작 불변).
	*/
	{
		static const bool forceHalt = [] {
			const char* v = std::getenv("BT_FORCE_HALT");
			return v && *v && std::string(v) != "0";
		}();
		if (forceHalt && tree.rootNode())
		{
			tree.rootNode()->halt();
		}
	}

	/*
	[수정 2026-08-17] 주최측 2026-05-26 패치 대응 — tickRoot() 를 try-catch 로 감싼다.

	패치노트: "tree의 이상 행동시 경고 로그와 CMD 값 반환하도록 변경".
	여기서는 **다시 던지지 않는다**. 규정 §7 의 즉시 승리 조건 3번이 "상대 에이전트 응답
	불능 — 1회 경고, 2회째 승리 처리"다. 경기 중 예외가 ctypes 경계를 넘으면 그 순간
	응답 불능이 되어 패배로 직결된다. 로그를 남기고 안전한 명령으로 계속 나는 편이 낫다.

	이 트리에서 예외가 나올 수 있는 실제 경로:
	  - SyncActionNode 가 RUNNING 을 반환 (action_node.h:49-60 이 던진다)
	  - Timeout 만료로 타이머 스레드가 haltChild() 를 부르는 도중의 상태 불일치
	  - createTreeFromFile 로 만든 노드가 기대하지 않은 포트를 읽을 때
	*/
	BT::NodeStatus rootStatus = BT::NodeStatus::FAILURE;
	try
	{
		rootStatus = tree.tickRoot(); //트리 작동
	}
	catch (const std::exception& e)
	{
		// 예외는 한 번만 크게 알린다. 매 tick 찍으면 그 자체로 프레임 예산을 먹는다(규정 §4).
		static bool reported = false;
		if (!reported)
		{
			reported = true;
			std::cout << "ERROR!!!!!!!!! Behavior Tree Execution Failed: " << e.what() << std::endl;
			std::cout << "Falling back to wings-level shallow climb for the rest of the match." << std::endl;
		}
		if (BtDiagEnabled()) { BtDiag(std::string("[tick] EXCEPTION: ") + e.what()); }

		BB->VP_Cartesian = SafeFallbackVP(posAtEntry, fwdAtEntry);
		BB->Throttle = DEFAULT_THROTTLE;
		BB->BFM = NONE;
	}

	if (BtDiagEnabled())
	{
		static long long diagTick = 0;
		BtDiag("[tick " + std::to_string(diagTick++) +
			"] t=" + std::to_string(BB->RunningTime) +
			" root=" + StatusName(rootStatus) +
			" BFM=" + std::to_string(static_cast<int>(BB->BFM)) +
			" enemies=" + std::to_string(BB->Enemy.size()) +
			// BFM 게이트가 읽는 값들. 어느 조건에서 막히는지 보려면 이게 필요하다.
			" sight=" + std::to_string(BB->EnemyInSight ? 1 : 0) +
			" D=" + std::to_string(BB->Distance) +
			" LOS=" + std::to_string(BB->Los_Degree) +
			" LOSt=" + std::to_string(BB->Los_Degree_Target) +
			" AA=" + std::to_string(BB->MyAspectAngle_Degree) +
			" AO=" + std::to_string(BB->MyAngleOff_Degree) +
			" E=" + std::to_string(BB->EnergyCompareResult) +
			" myXYZ=(" + std::to_string(BB->MyLocation_Cartesian.X) + "," +
			std::to_string(BB->MyLocation_Cartesian.Y) + "," +
			std::to_string(BB->MyLocation_Cartesian.Z) + ")" +
			" tgtXYZ=(" + std::to_string(BB->TargetLocaion_Cartesian.X) + "," +
			std::to_string(BB->TargetLocaion_Cartesian.Y) + "," +
			std::to_string(BB->TargetLocaion_Cartesian.Z) + ")" +
			" VP=(" + std::to_string(BB->VP_Cartesian.X) + "," +
			std::to_string(BB->VP_Cartesian.Y) + "," +
			std::to_string(BB->VP_Cartesian.Z) + ")" +
			" thr=" + std::to_string(BB->Throttle));
	}

	/*
	[추가 2026-08-17] 출력 경계에서 유한성 검사.

	주최측 2026-05-26 패치로 bt_action_provider._vp_to_array() 는 GetVP() 결과가 finite 가
	아니면 조용히 [0,0,0] 으로 갈아끼우고, action_provider.clip_action() 은 NaN/Inf 를 0.0 으로
	치환한다. 즉 NaN 이 더 이상 크래시로 드러나지 않고 **말이 되는 명령처럼 보이는 값**이 된다.
	우리 좌표계에서 (0,0,0) 은 기준 LLA 의 고도 0m, 곧 지면이다.

	그래서 NaN 을 호스트에 넘기기 전에 여기서 잡는다. 한 tick 이라도 오염되면 이후 판단이
	전부 어긋나므로 블랙보드 값도 함께 되돌린다.
	*/
	if (!IsFiniteVector(BB->VP_Cartesian))
	{
		static bool vpReported = false;
		if (!vpReported)
		{
			vpReported = true;
			std::cout << "WARNING: non-finite VP produced by the tree; substituting a safe point."
				<< std::endl;
		}
		if (BtDiagEnabled()) { BtDiag("[tick] non-finite VP -> safe fallback"); }
		BB->VP_Cartesian = SafeFallbackVP(posAtEntry, fwdAtEntry);
	}

	if (!std::isfinite(BB->Throttle))
	{
		BB->Throttle = DEFAULT_THROTTLE;
	}
	BB->Throttle = std::max(0.0f, std::min(1.0f, BB->Throttle));

	VP = Vector3(BB->VP_Cartesian.X, BB->VP_Cartesian.Y, BB->VP_Cartesian.Z);

	Throttle = BB->Throttle;	// 쓰로틀 값 (Task가 정했으면 그 값, 아니면 DEFAULT_THROTTLE)
}

 void UCPPBehaviorTree::SetDeltaTime(double DT)
 {
	 BB->DeltaSecond = DT;
 }

