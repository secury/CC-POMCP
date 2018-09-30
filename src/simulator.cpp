#include "simulator.h"

using namespace std;
using namespace UTILS;

SIMULATOR::KNOWLEDGE::KNOWLEDGE()
:   TreeLevel(LEGAL),
    RolloutLevel(LEGAL),
    SmartTreeCount(10),
    SmartTreeValue(RC(1.0, 1.0))
{
}

SIMULATOR::STATUS::STATUS()
:   Phase(TREE),
    Particles(CONSISTENT)
{
}

SIMULATOR::SIMULATOR() 
:   Discount(1.0),
    NumActions(0),
    NumObservations(0),
    RewardRange(1.0)
{
}

SIMULATOR::SIMULATOR(int numActions, int numObservations, double discount)
:   NumActions(numActions),
    NumObservations(numObservations),
    Discount(discount)
{ 
    assert(discount > 0 && discount <= 1);
}

SIMULATOR::~SIMULATOR() 
{ 
}

void SIMULATOR::Validate(const STATE& state) const 
{ 
}

bool SIMULATOR::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    return true;
}

void SIMULATOR::GenerateLegal(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status) const
{
    for (int a = 0; a < NumActions; ++a)
        actions.push_back(a);
}

void SIMULATOR::GeneratePreferred(const STATE& state, const HISTORY& history, 
    std::vector<int>& actions, const STATUS& status) const
{
}

int SIMULATOR::SelectRandom(const STATE& state, const HISTORY& history,
    const STATUS& status) const
{
    static vector<int> actions;

    if (Knowledge.RolloutLevel >= KNOWLEDGE::SMART)
    {
        actions.clear();
        GeneratePreferred(state, history, actions, status);
        if (!actions.empty())
            return actions[Random(actions.size())];
    }
        
    if (Knowledge.RolloutLevel >= KNOWLEDGE::LEGAL)
    {
        actions.clear();
        GenerateLegal(state, history, actions, status);
        if (!actions.empty())
            return actions[Random(actions.size())];
    }

    return Random(NumActions);
}

void SIMULATOR::Prior(const STATE* state, const HISTORY& history,
    VNODE* vnode, const STATUS& status) const
{
    static vector<int> actions;
    
    if (Knowledge.TreeLevel == KNOWLEDGE::PURE || state == 0)
    {
        vnode->SetChildren(0, RC(0, 0));
        return;
    }
    else
    {
        vnode->SetChildren(+LargeInteger, RC(-Infinity, Infinity));
    }

    if (Knowledge.TreeLevel >= KNOWLEDGE::LEGAL)
    {
        actions.clear();
        GenerateLegal(*state, history, actions, status);

        for (vector<int>::const_iterator i_action = actions.begin(); i_action != actions.end(); ++i_action)
        {
            int a = *i_action;
            QNODE& qnode = vnode->Child(a);
            qnode.Value.Set(0, RC(0, 0));
            qnode.AMAF.Set(0, RC(0, 0));
        }
    }
    
    if (Knowledge.TreeLevel >= KNOWLEDGE::SMART)
    {
        actions.clear();
        GeneratePreferred(*state, history, actions, status);

        for (vector<int>::const_iterator i_action = actions.begin(); i_action != actions.end(); ++i_action)
        {
            int a = *i_action;
            QNODE& qnode = vnode->Child(a);
            qnode.Value.Set(Knowledge.SmartTreeCount, Knowledge.SmartTreeValue);
            qnode.AMAF.Set(Knowledge.SmartTreeCount, Knowledge.SmartTreeValue);
        }    
    }
}

bool SIMULATOR::HasAlpha() const
{
    return false;
}

void SIMULATOR::AlphaValue(const QNODE& qnode, double& q, int& n) const
{
}

void SIMULATOR::UpdateAlpha(QNODE& qnode, const STATE& state) const
{
}

void SIMULATOR::DisplayBeliefs(const BELIEF_STATE& beliefState, 
    ostream& ostr) const
{
}

void SIMULATOR::DisplayState(const STATE& state, ostream& ostr) const 
{
}

void SIMULATOR::DisplayAction(int action, ostream& ostr) const 
{
    ostr << "Action " << action << endl;
}

void SIMULATOR::DisplayObservation(const STATE& state, int observation, ostream& ostr) const
{
    ostr << "Observation " << observation << endl;
}

void SIMULATOR::DisplayRewardCost(RC rewardcost, std::ostream& ostr) const
{
    ostr << "Reward " << rewardcost.R << " / Cost " << rewardcost.C << endl;
}

double SIMULATOR::GetHorizon(double accuracy, int undiscountedHorizon) const 
{ 
    if (Discount == 1)
        return undiscountedHorizon;
    return log(accuracy) / log(Discount);
}
