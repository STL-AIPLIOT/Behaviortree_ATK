#include "Task_EvasiveRollOrScissors.h"
#include "../BTLog.h"
#include <algorithm>
#include <chrono>
#include <random>

static thread_local std::mt19937 g_evasive_rng(
    static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count())
);

using namespace Action;

static inline float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(hi, v));
}

BT::NodeStatus Task_EvasiveRollOrScissors::tick()
{
    auto bb_res = getInput<CPPBlackBoard*>("BB");
    if (!bb_res)
    {
        std::cerr << "[Task_EvasiveRollOrScissors] BB ������ �������� ����\n";
        return BT::NodeStatus::FAILURE;
    }

    CPPBlackBoard* BB = bb_res.value();

    // ��Ȳ ���� (�̹� BB�� �����Ѵٰ� ����)
    const float D = BB->Distance;                 // m
    const float dv = BB->TargetSpeed_MS - BB->MySpeed_MS; // ��-�Ʊ� �ӵ���(+)�� ���� ����
    const int   ecmp = BB->EnergyCompareResult;      // >0 �켼, 0 ����, <0 ����

    // �������������������������������������������������� ���� ���� ��å ��������������������������������������������������
    // 1) �ʱ��� ��� (D < 250): �ް��� Ⱦ�⵿ (��/��) �켱
    // 2) ������ ����(ecmp<0) & ���� ����(dv>0): ��/��ũ����(���� ����)�� ���� ����
    // 3) ������ �켼(ecmp>0): ��+���(�Ѹ� Ŭ����)�� ���� Ȯ��
    // 4) �� ��: ��/�� ���� ��Ż
    enum class Dir { Right, Left, Up };
    Dir choice;

    if (D < 250.0f) {
        std::uniform_int_distribution<int> uni(0, 1);
        choice = (uni(g_evasive_rng) ? Dir::Right : Dir::Left); // [����] ������ ���� ȸ�� ����
    }
    else if (ecmp < 0 && dv > 0) {
        std::uniform_int_distribution<int> uni(0, 1);
        choice = (uni(g_evasive_rng) ? Dir::Right : Dir::Left); // ����+���� ���� �� ���� ȸ��
    }
    else if (ecmp > 0) {
        choice = Dir::Up; // �켼 �� ��� ����
    }
    else {
        // ������ ���� ���� (����: ���ػ� �ð�)
        static thread_local std::mt19937 rng(
            static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count())
        );
        std::uniform_int_distribution<int> uni(0, 2);
        int rnd = uni(rng);
        choice = (rnd == 0 ? Dir::Right : (rnd == 1 ? Dir::Left : Dir::Up));
    }

    // �������������������������������������������������� ������ ũ�� ��å ��������������������������������������������������
    // D, dv�� ���� ����. �������� �۰�, �ּ��� ũ��.
    float side = clampf(200.0f + 0.6f * D, 250.0f, 700.0f); // ��/��
    float up = clampf(150.0f + 0.3f * D, 200.0f, 500.0f); // ���

    // �ӵ����� ũ��(���� ����) �� ũ�� �̰�
    if (dv > 10.0f) {
        side = clampf(side * 1.2f, 250.0f, 800.0f);
    }

    Vector3 evasive_offset;
    switch (choice) {
    case Dir::Right: evasive_offset = BB->MyRightVector * side; break;
    case Dir::Left:  evasive_offset = BB->MyRightVector * (-side); break;
    case Dir::Up:    evasive_offset = BB->MyUpVector * up;   break;
    }

    // ���� ���� ��ǥ��
    BB->VP_Cartesian = BB->MyLocation_Cartesian + evasive_offset;

    BT_VLOG("[Task_EvasiveRollOrScissors] ȸ��: "
        << (choice == Dir::Right ? "Right" : choice == Dir::Left ? "Left" : "Up")
        << " | D=" << D << ", dv=" << dv
        << " | side=" << side << ", up=" << up << "\n");

    return BT::NodeStatus::SUCCESS;
}
