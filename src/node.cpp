#include "node.h"
#include "history.h"
#include "utils.h"

using namespace std;

//-----------------------------------------------------------------------------

int QNODE::NumChildren = 0;

void QNODE::Initialise()
{
    assert(NumChildren);
    Children.resize(NumChildren);
    for (int observation = 0; observation < QNODE::NumChildren; observation++)
        Children[observation] = 0;
    AlphaData.AlphaSum.clear();
}

void QNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": " << Value.GetValue().R << ", " << Value.GetValue().C << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back().Observation = observation;
            Children[observation]->DisplayValue(history, maxDepth, ostr);
        }
    }
}

void QNODE::DisplayPolicy(double lambda, HISTORY& history, int maxDepth, ostream& ostr) const
{
    history.Display(ostr);
    ostr << ": R=" << Value.GetValue().R << ", C=" << Value.GetValue().C
            << ", H=" << Value.GetValue().R - lambda * Value.GetValue().C
            << " (" << Value.GetCount() << ")\n";
    if (history.Size() >= maxDepth)
        return;

    for (int observation = 0; observation < NumChildren; observation++)
    {
        if (Children[observation])
        {
            history.Back().Observation = observation;
            Children[observation]->DisplayPolicy(lambda, history, maxDepth, ostr);
        }
    }
}

//-----------------------------------------------------------------------------

MEMORY_POOL<VNODE> VNODE::VNodePool;

int VNODE::NumChildren = 0;

void VNODE::Initialise()
{
    assert(NumChildren);
    Children.resize(VNODE::NumChildren);
    for (int action = 0; action < VNODE::NumChildren; action++)
        Children[action].Initialise();
}

VNODE* VNODE::Create()
{
    VNODE* vnode = VNodePool.Allocate();
    vnode->Initialise();
    return vnode;
}

void VNODE::Free(VNODE* vnode, const SIMULATOR& simulator)
{
    vnode->BeliefState.Free(simulator);
    VNodePool.Free(vnode);
    for (int action = 0; action < VNODE::NumChildren; action++)
        for (int observation = 0; observation < QNODE::NumChildren; observation++)
            if (vnode->Child(action).Child(observation))
                Free(vnode->Child(action).Child(observation), simulator);
}

void VNODE::FreeAll()
{
	VNodePool.DeleteAll();
}

void VNODE::SetChildren(int count, RC value)
{
    for (int action = 0; action < NumChildren; action++)
    {
        QNODE& qnode = Children[action];
        qnode.Value.Set(count, value);
        qnode.AMAF.Set(count, value);
    }
}

void VNODE::DisplayValue(HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    for (int action = 0; action < NumChildren; action++)
    {
        history.Add(action);
        Children[action].DisplayValue(history, maxDepth, ostr);
        history.Pop();
    }
}

void VNODE::DisplayPolicy(double lambda, HISTORY& history, int maxDepth, ostream& ostr) const
{
    if (history.Size() >= maxDepth)
        return;

    double bestq = -Infinity;
    int besta = -1;
    for (int action = 0; action < NumChildren; action++)
    {
        double scalarizedValue = Children[action].Value.GetValue().R - lambda * Children[action].Value.GetValue().C;
        if (scalarizedValue > bestq)
        {
            besta = action;
            bestq = scalarizedValue;
        }
    }

    if (besta != -1)
    {
        history.Add(besta);
        Children[besta].DisplayPolicy(lambda, history, maxDepth, ostr);
        history.Pop();
    }
}

//-----------------------------------------------------------------------------
