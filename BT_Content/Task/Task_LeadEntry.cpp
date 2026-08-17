#include "Task_LeadEntry.h"
#include "../BTLog.h"

using namespace BT_Geometry;

namespace Action
{
    BT::NodeStatus Task_LeadEntry::tick()
    {
        auto bb_opt = getInput<CPPBlackBoard*>("BB");
        if (!bb_opt) return BT::NodeStatus::FAILURE;
        CPPBlackBoard* BB = bb_opt.value();

        Vector3 tgtPos = BB->TargetLocaion_Cartesian;
        Vector3 myPos = BB->MyLocation_Cartesian;

        // --- �Ÿ� ��� ---
        float D = static_cast<float>(BB->Distance);
        if (D < 1.0f) D = static_cast<float>((tgtPos - myPos).length());

        // --- ����: ���� ��� �Ÿ� & AA ���� ---
        float AA = static_cast<float>(std::fabs(BB->MyAspectAngle_Degree));  // ��ǥ 0��
        if (D < LEAD_D_MIN || D > LEAD_D_MAX || AA > AA_MAX)
        {
            // ���� ������ �� �������� �ѱ�
            // (�ð� Ÿ�̸Ӵ� �Ʒ����� �ʱ�ȭ)
            BT_VLOG("[Lead] skip: D=" << D << " (allow "
                << LEAD_D_MIN << "~" << LEAD_D_MAX << "), AA=" << AA << " deg\n");
            // ���� Ÿ�̸� ����
            lead_timer_ = 0.0f;
            return BT::NodeStatus::FAILURE;
        }

        // --- ���� ���� �ð� ����(���ڷ����� ���� ���� �ð� ����) ---
        // BB->DeltaSecond �� �������忡 �����Ѵٰ� ����(����� ȯ�� ���� ���)
        float dt = 0.0f;
        if (BB->DeltaSecond > 0.0f) dt = static_cast<float>(BB->DeltaSecond);
        lead_timer_ += dt;
        if (lead_timer_ > LEAD_TIME_MAX)
        {
            BT_VLOG("[Lead] stop: time_limit " << lead_timer_ << "s > " << LEAD_TIME_MAX << "s\n");
            lead_timer_ = 0.0f;
            return BT::NodeStatus::FAILURE;
        }

        // --- ���� Ÿ�̹� ---
        float Vm = static_cast<float>(BB->MySpeed_MS);
        if (Vm < 1.0f) Vm = 1.0f;
        float t_lead = clampf(D / Vm, T_MIN, T_MAX);

        // --- Ÿ�� �ӵ� ���� ���� ---
        Vector3 Vt = BB->PredictedTargetVelocity;
        float vt_len = static_cast<float>(Vt.length());
        if (vt_len < 0.1f)
        {
            Vector3 Ht = BB->TargetForwardVector;
            if (!normalize(Ht))
            {
                BT_VLOG("[Lead] skip: invalid target forward\n");
                lead_timer_ = 0.0f;
                return BT::NodeStatus::FAILURE;
            }
            Vt = Ht * BB->TargetSpeed_MS;
            vt_len = static_cast<float>(Vt.length());
            if (vt_len < 0.1f)
            {
                BT_VLOG("[Lead] skip: target too slow\n");
                lead_timer_ = 0.0f;
                return BT::NodeStatus::FAILURE;
            }
        }

        // --- �⺻ ���� ���� ---
        Vector3 VP = tgtPos + Vt * t_lead;

        Vector3 Ht_dir = BB->TargetForwardVector;
        if (!normalize(Ht_dir)) {
            // forward ���Ͱ� �������� ���� Vt �������� ��ü
            Ht_dir = Vt;
            if (!normalize(Ht_dir)) {
                // �̰͵� ���и� ������ ��ŵ
                Ht_dir = Vector3(0, 0, 0);
            }
        }
        VP = VP + Ht_dir * 50.0f;   // �� ��� ����������� 50m ������

        // --- ��ǥ ���� ����: AA �� 0��, AO(LOS��) �� 2�� ---
        // �غ� ����
        Vector3 Ht = BB->TargetForwardVector;  normalize(Ht);
        Vector3 Hm = BB->MyForwardVector;      normalize(Hm);
        Vector3 LOSv = tgtPos - myPos;         normalize(LOSv);

        // �Ÿ� ������(������ ���� ��, �ָ� ���� ��)
        float Dscale = std::min(D, LEAD_D_MAX);

        // 1) AA ����(��0��): Ÿ�� ����(-Ht) �������� �ҷ� ����
        VP = VP + (-Ht) * (K_AA * AA * 0.001f * Dscale);

        // 2) AO ����(��2��): lateral_dir = Hm - LOS*(Hm��LOS)
        float ao_err = static_cast<float>(BB->MyAngleOff_Degree) - 2.0f; // ��ǥ 2��
        Vector3 lateral = Hm - LOSv * static_cast<float>(Hm.dot(LOSv));
        if (lateral.length() > 1e-6) normalize(lateral);
        VP = VP + (-lateral) * (K_AO * ao_err * 0.001f * Dscale);

        // --- ��� & ���� ---
        BB->VP_Cartesian = VP;
        BT_VLOG("[Lead] OK  D=" << D
            << " Vm=" << BB->MySpeed_MS
            << " |Vt|=" << vt_len
            << " t_lead=" << t_lead
            << " AA=" << AA
            << " AO=" << BB->MyAngleOff_Degree
            << " t_used=" << lead_timer_ << "s\n");

        return BT::NodeStatus::SUCCESS;
    }
}
