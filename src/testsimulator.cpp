#include "testsimulator.h"
#include "utils.h"

using namespace UTILS;

STATE* TEST_SIMULATOR::Copy(const STATE& state) const
{
    const TEST_STATE& tstate = safe_cast<const TEST_STATE&>(state);
    TEST_STATE* newstate = new TEST_STATE;
    newstate->Depth = tstate.Depth;
    return newstate;
}

STATE* TEST_SIMULATOR::CreateStartState() const
{
    return new TEST_STATE;
}

void TEST_SIMULATOR::FreeState(STATE* state) const
{
    delete state;
}

bool TEST_SIMULATOR::Step(STATE& state, int action, 
    int& observation, RC& rewardcost) const
{
    // Up to MaxDepth action 0 is good independent of observations
    TEST_STATE& tstate = safe_cast<TEST_STATE&>(state);
    if (tstate.Depth < MaxDepth && action == 0)
        rewardcost = RC(1.0, 1.0);
    else
        rewardcost = RC(0.0, 0.0);

    observation = Random(0, GetNumObservations());
    tstate.Depth++;
    return false;
}

RC TEST_SIMULATOR::OptimalValue() const
{
    double discount = 1.0;
    RC totalRewardCost(0.0, 0.0);
    for (int i = 0; i < MaxDepth; i++)
    {
        totalRewardCost += RC(discount, discount);
        discount *= GetDiscount();
    }
    return totalRewardCost;
}

RC TEST_SIMULATOR::MeanValue() const
{
    double discount = 1.0;
    RC totalRewardCost(0.0, 0.0);
    for (int i = 0; i < MaxDepth; i++)
    {
        totalRewardCost += RC(discount / GetNumActions(), discount / GetNumActions());
        discount *= GetDiscount();
    }
    return totalRewardCost;
}
