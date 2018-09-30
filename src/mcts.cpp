#include "mcts.h"
#include "testsimulator.h"
#include <math.h>
#include <stdio.h>

#include <algorithm>

using namespace std;
using namespace UTILS;

//-----------------------------------------------------------------------------

MCTS::PARAMS::PARAMS()
:   Verbose(0),
    MaxDepth(100),
    NumSimulations(1000),
    NumStartStates(1000),
    UseTransforms(true),
    NumTransforms(0),
    MaxAttempts(0),
    ExpandCount(1),
    ExplorationConstant(1),
    UseRave(false),
    RaveDiscount(1.0),
    RaveConstant(0.01),
    DisableTree(false),
    LambdaMax(100.0),
    TreeAlgorithm(0)
{
}

MCTS::MCTS(const SIMULATOR& simulator, const PARAMS& params)
:   Simulator(simulator),
    Params(params),
    TreeDepth(0),
	lambda(2),
	lambdaMax(params.LambdaMax),
	c_hat(params.c_hat),
	initial_c_hat(params.c_hat),
	TreeAlgorithm(params.TreeAlgorithm)
{
    VNODE::NumChildren = Simulator.GetNumActions();
    QNODE::NumChildren = Simulator.GetNumObservations();

    Root = ExpandNode(Simulator.CreateStartState());

    for (int i = 0; i < Params.NumStartStates; i++)
        Root->Beliefs().AddSample(Simulator.CreateStartState());
}

MCTS::~MCTS()
{
    VNODE::Free(Root, Simulator);
    VNODE::FreeAll();
}

bool MCTS::Update(int action, int observation, RC rewardcost)
{
    History.Add(action, observation);
    BELIEF_STATE beliefs;

    // Find matching vnode from the rest of the tree
    QNODE& qnode = Root->Child(action);
    VNODE* vnode = qnode.Child(observation);
    if (vnode)
    {
        if (Params.Verbose >= 1)
            cout << "Matched " << vnode->Beliefs().GetNumSamples() << " states" << endl;
        beliefs.Copy(vnode->Beliefs(), Simulator);
    }
    else
    {
        if (Params.Verbose >= 1)
            cout << "No matching node found" << endl;
    }

    // Generate transformed states to avoid particle deprivation
    if (Params.UseTransforms)
        AddTransforms(Root, beliefs);

    // If we still have no particles, fail
    if (beliefs.Empty() && (!vnode || vnode->Beliefs().Empty()))
        return false;

    if (Params.Verbose >= 1)
        Simulator.DisplayBeliefs(beliefs, cout);

    // Find a state to initialise prior (only requires fully observed state)
    const STATE* state = 0;
    if (vnode && !vnode->Beliefs().Empty())
        state = vnode->Beliefs().GetSample(0);
    else
        state = beliefs.GetSample(0);

    // Delete old tree and create new root
    VNODE::Free(Root, Simulator);
    VNODE* newRoot = ExpandNode(state);
    newRoot->Beliefs() = beliefs;
    Root = newRoot;
    return true;
}

Policy MCTS::SelectAction()
{
    if (Params.DisableTree)
        RolloutSearch();
    else
        UCTSearch();
    return GreedyUCB(Root, false, true);
}

void MCTS::RolloutSearch()
{
	std::vector<double> totals(Simulator.GetNumActions(), 0.0);
	int historyDepth = History.Size();
	std::vector<int> legal;
	assert(BeliefState().GetNumSamples() > 0);
	Simulator.GenerateLegal(*BeliefState().GetSample(0), GetHistory(), legal, GetStatus());
	random_shuffle(legal.begin(), legal.end());

	for (int i = 0; i < Params.NumSimulations; i++)
	{
		int action = legal[i % legal.size()];
		STATE* state = Root->Beliefs().CreateSample(Simulator);
		Simulator.Validate(*state);

		int observation;
		RC immediateRewardCost, delayedRewardCost, totalRewardCost;
		bool terminal = Simulator.Step(*state, action, observation, immediateRewardCost);

		VNODE*& vnode = Root->Child(action).Child(observation);
		if (!vnode && !terminal)
		{
			vnode = ExpandNode(state);
			AddSample(vnode, *state);
		}
		History.Add(action, observation);

		delayedRewardCost = Rollout(*state);
		totalRewardCost = immediateRewardCost + Simulator.GetDiscount() * delayedRewardCost;
		Root->Child(action).Value.Add(totalRewardCost);

		Simulator.FreeState(state);
		History.Truncate(historyDepth);
	}
}

void MCTS::UCTSearch()
{
    ClearStatistics();
    int historyDepth = History.Size();

    for (int n = 0; n < Params.NumSimulations; n++)
    {
        STATE* state = Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);
        Status.Phase = SIMULATOR::STATUS::TREE;
        if (Params.Verbose >= 2)
        {
            cout << "Starting simulation" << endl;
            Simulator.DisplayState(*state, cout);
        }

        TreeDepth = 0;
        PeakTreeDepth = 0;
        RC totalRewardCost = SimulateV(*state, Root);
        StatTotalReward.Add(totalRewardCost.R);
        StatTotalCost.Add(totalRewardCost.C);
        StatTreeDepth.Add(PeakTreeDepth);

        /////////////////////
        // Update lambda
        if (TreeAlgorithm == 0) {
            Policy policy = GreedyUCB(Root, false, false);
            int action = policy.sampleAction();
            double V_C = Root->Child(action).Value.GetValue().C;
            double gradient = (V_C - c_hat) < 0 ? -1 : 1;
            lambda += 1.0 / (n + 1.0) * gradient;
            if (lambda < 0.0) {
                lambda = 0.0;
            }
            if (lambda > lambdaMax) {
                lambda = lambdaMax;
            }
        }
        ////////////////////

        if (Params.Verbose >= 2) {
            cout << "Total reward = " << totalRewardCost.R << ", cost = " << totalRewardCost.C << endl;
        }
        if (Params.Verbose >= 3)
            DisplayValue(4, cout);

        Simulator.FreeState(state);
        History.Truncate(historyDepth);
    }
    DisplayStatistics(cout);
}

RC MCTS::Simulate(const BELIEF_STATE &beliefs, int iteration)
{
    RC totalRewardCost(0, 0);
    for (int i = 0; i < iteration; i++) {
        STATE* state = Root->Beliefs().CreateSample(Simulator);
        Simulator.Validate(*state);

        VNODE* vnode = Root;
        int observation;
        RC immediateRewardCost;
        double discount = 1.0;
        while (true) {
            int action = GreedyUCB(vnode, false, true).sampleAction();
            bool terminal = Simulator.Step(*state, action, observation, immediateRewardCost);
            assert(observation >= 0 && observation < Simulator.GetNumObservations());

            totalRewardCost += discount * immediateRewardCost;
            discount *= Simulator.GetDiscount();

            if (terminal) {
                break;
            }
            // next state
            vnode = vnode->Child(action).Child(observation);
            if (!vnode) {
                RC delayedRewardCost = Rollout(*state);
                totalRewardCost += discount * delayedRewardCost;
                break;
            }
        }
        Simulator.FreeState(state);
    }

    return totalRewardCost / (double) iteration;
}

RC MCTS::SimulateV(STATE& state, VNODE* vnode)
{
    int action = GreedyUCB(vnode, true, true).sampleAction();

    PeakTreeDepth = TreeDepth;
    if (TreeDepth >= Params.MaxDepth) // search horizon reached
        return RC(0, 0);

    if (TreeDepth == 1)
        AddSample(vnode, state);

    QNODE& qnode = vnode->Child(action);
    RC totalRewardCost = SimulateQ(state, qnode, action);
    vnode->Value.Add(totalRewardCost);
    AddRave(vnode, totalRewardCost);
    return totalRewardCost;
}

RC MCTS::SimulateQ(STATE& state, QNODE& qnode, int action)
{
    int observation;
    RC immediateRewardCost, delayedRewardCost(0.0, 0.0);

    if (Simulator.HasAlpha())
        Simulator.UpdateAlpha(qnode, state);
    bool terminal = Simulator.Step(state, action, observation, immediateRewardCost);
    assert(observation >= 0 && observation < Simulator.GetNumObservations());
    History.Add(action, observation);

    if (Params.Verbose >= 3)
    {
        Simulator.DisplayAction(action, cout);
        Simulator.DisplayObservation(state, observation, cout);
        Simulator.DisplayRewardCost(immediateRewardCost, cout);
        Simulator.DisplayState(state, cout);
    }

    VNODE*& vnode = qnode.Child(observation);
    if (!vnode && !terminal && qnode.Value.GetCount() >= Params.ExpandCount)
        vnode = ExpandNode(&state);

    if (!terminal)
    {
        TreeDepth++;
        if (vnode)
            delayedRewardCost = SimulateV(state, vnode);
        else
            delayedRewardCost = Rollout(state);
        TreeDepth--;
    }

    RC totalRewardCost = immediateRewardCost + Simulator.GetDiscount() * delayedRewardCost;
    qnode.Value.Add(totalRewardCost);
    return totalRewardCost;
}

void MCTS::AddRave(VNODE* vnode, RC totalRewardCost)
{
    double totalDiscount = 1.0;
    for (int t = TreeDepth; t < History.Size(); ++t)
    {
        QNODE& qnode = vnode->Child(History[t].Action);
        qnode.AMAF.Add(totalRewardCost, totalDiscount);
        totalDiscount *= Params.RaveDiscount;
    }
}

VNODE* MCTS::ExpandNode(const STATE* state)
{
    VNODE* vnode = VNODE::Create();
    vnode->Value.Set(0, RC(0.0, 0.0));
    Simulator.Prior(state, History, vnode, Status);

    if (Params.Verbose >= 2)
    {
        cout << "Expanding node: ";
        History.Display(cout);
        cout << endl;
    }

    return vnode;
}

void MCTS::AddSample(VNODE* node, const STATE& state)
{
    STATE* sample = Simulator.Copy(state);
    node->Beliefs().AddSample(sample);
    if (Params.Verbose >= 2)
    {
        cout << "Adding sample:" << endl;
        Simulator.DisplayState(*sample, cout);
    }
}

Policy MCTS::GreedyUCB(VNODE* vnode, bool ucb, bool stochastic) const
{
    static vector<int> besta; besta.clear();
    double bestQ = -Infinity, bestQplus = -Infinity;
    int N = vnode->Value.GetCount();
    double logN = log(N + 1);
    bool hasalpha = Simulator.HasAlpha();

    for (int action = 0; action < Simulator.GetNumActions(); action++)
    {
        double Q, Qplus, alphaq;
        int n, alphan;

        QNODE& qnode = vnode->Child(action);
        Q = qnode.Value.GetValue().R - lambda * qnode.Value.GetValue().C;  // scalarized value
        n = qnode.Value.GetCount();

        if (Params.UseRave && qnode.AMAF.GetCount() > 0)
        {
            double n2 = qnode.AMAF.GetCount();
            double beta = n2 / (n + n2 + Params.RaveConstant * n * n2);
            Q = (1.0 - beta) * Q + beta * (qnode.AMAF.GetValue().R - lambda * qnode.AMAF.GetValue().C);  // scalarized value
            assert(false); // test...
        }

        if (hasalpha && n > 0)
        {
            Simulator.AlphaValue(qnode, alphaq, alphan);
            Q = (n * Q + alphan * alphaq) / (n + alphan);
            assert(false); // test...
        }

        Qplus = Q;
        if (ucb) {
            Qplus += FastUCB(N, n, logN);
        }

        if (TreeAlgorithm == 0) { //CCPOMCP
            if (Qplus >= bestQplus)
            {
                if (Qplus > bestQplus)
                    besta.clear();
                bestQplus = Qplus;
                bestQ = Q;
                besta.push_back(action);
            }
        } else if (TreeAlgorithm == 1) { // Baseline
            if (qnode.Value.GetValue().C < c_hat && Qplus >= bestQplus) {
                if (Qplus > bestQplus)
                    besta.clear();
                bestQplus = Qplus;
                bestQ = Q;
                besta.push_back(action);
            }
        }
    }

    //Baseline
    if (TreeAlgorithm == 1){
        if (besta.size() == 0) {
            // Random action (among legal actions)
            while (true) {
                int action = Random(vnode->NumChildren);
                if (vnode->Child(action).Value.GetCount() == 0 || vnode->Child(action).Value.GetValue().R > -Infinity) {
                    Policy policy;
                    policy.setPolicy(0, 0, action, action, 0);
                    return policy;
                }
            }
        } else {
            int action = besta[Random(besta.size())];
            Policy policy;
            policy.setPolicy(0, 0, action, action, 0);
            return policy;
        }
    }
    //Else: CCPOMCP

    int bestAction = besta[Random(besta.size())];

    // additionally, add tied actions
    const double biasconstant = exp(-TreeDepth) * 0.1;
    double minCost = Infinity, maxCost = -Infinity;
    int minCostAction = -1, maxCostAction = -1;

    int bestActionN = vnode->Child(bestAction).Value.GetCount();
    double bestActionQ = vnode->Child(bestAction).Value.GetValue().R - lambda * vnode->Child(bestAction).Value.GetValue().C;
    double bestActionBias = biasconstant * (log(bestActionN + 1) / (bestActionN + 1));
    for (int action = 0; action < Simulator.GetNumActions(); action++) {
        QNODE& qnode = vnode->Child(action);
        double Q_R = qnode.Value.GetValue().R;
        double Q_C = qnode.Value.GetValue().C;
        double q = Q_R - lambda * Q_C;  // scalarized value
        int n = qnode.Value.GetCount();
        double actionBias = biasconstant * (log(n + 1) / (n + 1));

        double threshold = (stochastic) ? bestActionBias + actionBias : 0.0;
        if (fabs(bestQ - q) <= threshold) {
            if (Q_C < minCost) {
                minCost = Q_C;
                minCostAction = action;
            }
            if (Q_C > maxCost) {
                maxCost = Q_C;
                maxCostAction = action;
            }
        }
    }

    Policy policy;
    policy.setPolicy(minCost, maxCost, minCostAction, maxCostAction, c_hat);

    return policy;
}

RC MCTS::Rollout(STATE& state)
{
    Status.Phase = SIMULATOR::STATUS::ROLLOUT;
    if (Params.Verbose >= 3)
        cout << "Starting rollout" << endl;

    RC totalRewardCost(0.0, 0.0);
    double discount = 1.0;
    bool terminal = false;
    int numSteps;
    for (numSteps = 0; numSteps + TreeDepth < Params.MaxDepth && !terminal; ++numSteps)
    {
        int observation;
        RC rewardcost;

        int action = Simulator.SelectRandom(state, History, Status);
        terminal = Simulator.Step(state, action, observation, rewardcost);
        History.Add(action, observation);

        if (Params.Verbose >= 4)
        {
            Simulator.DisplayAction(action, cout);
            Simulator.DisplayObservation(state, observation, cout);
            Simulator.DisplayRewardCost(rewardcost, cout);
            Simulator.DisplayState(state, cout);
        }

        totalRewardCost += rewardcost * discount;
        discount *= Simulator.GetDiscount();
    }

    StatRolloutDepth.Add(numSteps);
    if (Params.Verbose >= 3)
        cout << "Ending rollout after " << numSteps
            << " steps, with total reward " << totalRewardCost.R << "and total cost " << totalRewardCost.C << endl;
    return totalRewardCost;
}

void MCTS::AddTransforms(VNODE* root, BELIEF_STATE& beliefs)
{
    int attempts = 0, added = 0;

    // Local transformations of state that are consistent with history
    while (added < Params.NumTransforms && attempts < Params.MaxAttempts)
    {
        STATE* transform = CreateTransform();
        if (transform)
        {
            beliefs.AddSample(transform);
            added++;
        }
        attempts++;
    }

    if (Params.Verbose >= 1)
    {
        cout << "Created " << added << " local transformations out of "
            << attempts << " attempts" << endl;
    }
}

STATE* MCTS::CreateTransform() const
{
    int stepObs;
    RC stepRewardCost;

    STATE* state = Root->Beliefs().CreateSample(Simulator);
    Simulator.Step(*state, History.Back().Action, stepObs, stepRewardCost);
    if (Simulator.LocalMove(*state, History, stepObs, Status))
        return state;
    Simulator.FreeState(state);
    return 0;
}

double MCTS::UCB[UCB_N][UCB_n];
bool MCTS::InitialisedFastUCB = true;

void MCTS::InitFastUCB(double exploration)
{
    cout << "Initialising fast UCB table... ";
    for (int N = 0; N < UCB_N; ++N)
        for (int n = 0; n < UCB_n; ++n)
            if (n == 0)
                UCB[N][n] = Infinity;
            else
                UCB[N][n] = exploration * sqrt(log(N + 1) / n);
    cout << "done" << endl;
    InitialisedFastUCB = true;
}

inline double MCTS::FastUCB(int N, int n, double logN) const
{
    if (InitialisedFastUCB && N < UCB_N && n < UCB_n)
        return UCB[N][n];

    if (n == 0)
        return Infinity;
    else
        return Params.ExplorationConstant * sqrt(logN / n);
}

void MCTS::ClearStatistics()
{
    StatTreeDepth.Clear();
    StatRolloutDepth.Clear();
    StatTotalReward.Clear();
}

void MCTS::DisplayStatistics(ostream& ostr) const
{
    if (Params.Verbose >= 1)
    {
        StatTreeDepth.Print("Tree depth", ostr);
        StatRolloutDepth.Print("Rollout depth", ostr);
        StatTotalReward.Print("Total reward", ostr);
    }

    if (Params.Verbose >= 2)
    {
        ostr << "Policy after " << Params.NumSimulations << " simulations" << endl;
        DisplayPolicy(6, ostr);
        ostr << "Values after " << Params.NumSimulations << " simulations" << endl;
        DisplayValue(6, ostr);
    }
}

void MCTS::DisplayValue(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Values:" << endl;
    Root->DisplayValue(history, depth, ostr);
}

void MCTS::DisplayPolicy(int depth, ostream& ostr) const
{
    HISTORY history;
    ostr << "MCTS Policy:" << endl;
    Root->DisplayPolicy(lambda, history, depth, ostr);
}

//-----------------------------------------------------------------------------

void MCTS::UnitTest()
{
    UnitTestGreedy();
    UnitTestUCB();
    UnitTestRollout();
    for (int depth = 1; depth <= 3; ++depth)
        UnitTestSearch(depth);
}

void MCTS::UnitTestGreedy()
{
    TEST_SIMULATOR testSimulator(5, 5, 0);
    PARAMS params;
    MCTS mcts(testSimulator, params);
    int numAct = testSimulator.GetNumActions();
    int numObs = testSimulator.GetNumObservations();

    VNODE* vnode = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode->Value.Set(1, RC(0, 0));
    vnode->Child(0).Value.Set(0, RC(1, 1));
    for (int action = 1; action < numAct; action++)
        vnode->Child(action).Value.Set(0, RC(0, 0));
    assert(mcts.GreedyUCB(vnode, false, true).sampleAction() == 0);
}

void MCTS::UnitTestUCB()
{
    TEST_SIMULATOR testSimulator(5, 5, 0);
    PARAMS params;
    MCTS mcts(testSimulator, params);
    int numAct = testSimulator.GetNumActions();
    int numObs = testSimulator.GetNumObservations();

    // With equal value, action with lowest count is selected
    VNODE* vnode1 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode1->Value.Set(1, RC(0, 0));
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode1->Child(action).Value.Set(99, RC(0, 0));
        else
            vnode1->Child(action).Value.Set(100 + action, RC(0, 0));
    assert(mcts.GreedyUCB(vnode1, true, true).sampleAction() == 3);

    // With high counts, action with highest value is selected
    VNODE* vnode2 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode2->Value.Set(1, RC(0, 0));
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode2->Child(action).Value.Set(99 + numObs, RC(1, 1));
        else
            vnode2->Child(action).Value.Set(100 + numAct - action, RC(0, 0));
    assert(mcts.GreedyUCB(vnode2, true, true).sampleAction() == 3);

    // Action with low value and low count beats actions with high counts
    VNODE* vnode3 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode3->Value.Set(1, RC(0, 0));
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode3->Child(action).Value.Set(1, RC(1, 1));
        else
            vnode3->Child(action).Value.Set(100 + action, RC(1, 1));
    assert(mcts.GreedyUCB(vnode3, true, true).sampleAction() == 3);

    // Actions with zero count is always selected
    VNODE* vnode4 = mcts.ExpandNode(testSimulator.CreateStartState());
    vnode4->Value.Set(1, RC(0, 0));
    for (int action = 0; action < numAct; action++)
        if (action == 3)
            vnode4->Child(action).Value.Set(0, RC(0, 0));
        else
            vnode4->Child(action).Value.Set(1, RC(1, 1));
    assert(mcts.GreedyUCB(vnode4, true, true).sampleAction() == 3);
}

void MCTS::UnitTestRollout()
{
    TEST_SIMULATOR testSimulator(2, 2, 0);
    PARAMS params;
    params.NumSimulations = 1000;
    params.MaxDepth = 10;
    MCTS mcts(testSimulator, params);
    RC totalRewardCost;
    for (int n = 0; n < mcts.Params.NumSimulations; ++n)
    {
        STATE* state = testSimulator.CreateStartState();
        mcts.TreeDepth = 0;
        totalRewardCost += mcts.Rollout(*state);
    }
    RC rootValue = totalRewardCost / mcts.Params.NumSimulations;
    RC meanValue = testSimulator.MeanValue();
    assert(fabs(meanValue.R - rootValue.R) < 0.1);
}

void MCTS::UnitTestSearch(int depth)
{
    TEST_SIMULATOR testSimulator(3, 2, depth);
    PARAMS params;
    params.MaxDepth = depth + 1;
    params.NumSimulations = pow(10, depth + 1);
    MCTS mcts(testSimulator, params);
    mcts.UCTSearch();
    RC rootValue = mcts.Root->Value.GetValue();
    RC optimalValue = testSimulator.OptimalValue();
    assert(fabs(optimalValue.R - rootValue.R) < 0.1);
}

//-----------------------------------------------------------------------------
