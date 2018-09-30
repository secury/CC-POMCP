#include "experiment.h"
#include "boost/timer.hpp"

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(1000),
    NumSteps(100000),
    SimSteps(1000),
    TimeOut(3600),
    MinDoubles(0),
    MaxDoubles(20),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(1000),
    AutoExploration(true)
{
}

EXPERIMENT::EXPERIMENT(const SIMULATOR& real,
    const SIMULATOR& simulator, const string& outputFile,
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams)
:   Real(real),
    Simulator(simulator),
    OutputFile(outputFile.c_str()),
    ExpParams(expParams),
    SearchParams(searchParams)
{
    if (ExpParams.AutoExploration)
    {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}

void EXPERIMENT::Run()
{
    boost::timer timer;

    MCTS mcts(Simulator, SearchParams);

    RC undiscountedReturn(0.0, 0.0);
    RC discountedReturn(0.0, 0.0);
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;

    STATE* state = Real.CreateStartState();
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

    for (t = 0; t < ExpParams.NumSteps; t++)
    {
        int observation;
        RC rewardcost;
        Policy policy = mcts.SelectAction();
        int action = policy.sampleAction();
        terminal = Real.Step(*state, action, observation, rewardcost);

        Results.Reward.Add(rewardcost.R);
        Results.Cost.Add(rewardcost.C);
        undiscountedReturn += rewardcost;
        discountedReturn += rewardcost * discount;
        discount *= Real.GetDiscount();

        // Update admissible cost
        double nextAdmissibleCost = mcts.getNextAdmissibleCost(policy, action, rewardcost);
        mcts.setAdmissibleCost(nextAdmissibleCost);

        if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayRewardCost(rewardcost, cout);
        }

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }
        outOfParticles = !mcts.Update(action, observation, rewardcost);
        if (outOfParticles)
            break;

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }

    if (outOfParticles)
    {
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
        HISTORY history = mcts.GetHistory();
        while (++t < ExpParams.NumSteps)
        {
            int observation;
            RC rewardcost;

            // This passes real state into simulator!
            // SelectRandom must only use fully observable state
            // to avoid "cheating"
            int action = Simulator.SelectRandom(*state, history, mcts.GetStatus());
            terminal = Real.Step(*state, action, observation, rewardcost);

            Results.Reward.Add(rewardcost.R);
            Results.Cost.Add(rewardcost.C);
            undiscountedReturn += rewardcost;
            discountedReturn += rewardcost * discount;
            discount *= Real.GetDiscount();

            if (SearchParams.Verbose >= 1)
            {
                Real.DisplayAction(action, cout);
                Real.DisplayState(*state, cout);
                Real.DisplayObservation(*state, observation, cout);
                Real.DisplayRewardCost(rewardcost, cout);
            }

            if (terminal)
            {
                cout << "Terminated" << endl;
                break;
            }

            history.Add(action, observation);
        }
    }

    Results.Time.Add(timer.elapsed());
    Results.OneStepTime.Add(timer.elapsed() / t);
    Results.TimeSteps.Add(t);
    Results.UndiscountedRewardReturn.Add(undiscountedReturn.R);
    Results.UndiscountedCostReturn.Add(undiscountedReturn.C);
    Results.DiscountedRewardReturn.Add(discountedReturn.R);
    Results.DiscountedCostReturn.Add(discountedReturn.C);
    cout << "==============================" << endl;
    cout << "Discounted reward return = " << discountedReturn.R
        << ", average = " << Results.DiscountedRewardReturn.GetMean() << endl;
    cout << "Discounted cost return = " << discountedReturn.C
        << ", average = " << Results.DiscountedCostReturn.GetMean() << endl;

    cout << "Undiscounted reward return = " << undiscountedReturn.R
        << ", average = " << Results.UndiscountedRewardReturn.GetMean() << endl;
    cout << "Undiscounted cost return = " << undiscountedReturn.C
        << ", average = " << Results.UndiscountedCostReturn.GetMean() << endl;
    cout << "Lambda = " << mcts.getLambda() << endl;
    cout << "NumSteps = " << t << endl;
    cout << "Time per steps = " << timer.elapsed() / t << endl;
    cout << "==============================" << endl;
}

void EXPERIMENT::MultiRun()
{
    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
            << SearchParams.NumSimulations << " simulations... " << endl;
        Run();
        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
}

void EXPERIMENT::DiscountedReturn()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations" << "\t"
            << "TimeSteps" << "\t"
            << "Runs" << "\t"
            << "Undiscounted reward return" << "\t"
            << "Undiscounted reward error" << "\t"
            << "Undiscounted cost return" << "\t"
            << "Undiscounted cost error" << "\t"
            << "Discounted reward return" << "\t"
            << "Discounted reward error" << "\t"
            << "Discounted cost return" << "\t"
            << "Discounted cost error" << "\t"
            << "Time" << "\t"
            << "TimePerStep" << "\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        MultiRun();

        cout << "=============================" << endl;
        cout << "Simulations = " << SearchParams.NumSimulations << endl
            << "Runs = " << Results.Time.GetCount() << endl
            << "Undiscounted reward return = " << Results.UndiscountedRewardReturn.GetMean()
            << " +- " << Results.UndiscountedRewardReturn.GetStdErr() << endl
            << "Undiscounted cost return = " << Results.UndiscountedCostReturn.GetMean()
            << " +- " << Results.UndiscountedCostReturn.GetStdErr() << endl
            << "Discounted reward return = " << Results.DiscountedRewardReturn.GetMean()
            << " +- " << Results.DiscountedRewardReturn.GetStdErr() << endl
            << "Discounted cost return = " << Results.DiscountedCostReturn.GetMean()
            << " +- " << Results.DiscountedCostReturn.GetStdErr() << endl
            << "Time = " << Results.Time.GetMean() << endl
            << "Time per time step = " << Results.OneStepTime.GetMean() << endl;
        cout << "=============================" << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.TimeSteps.GetMean() << "\t"
            << Results.Time.GetCount() << "\t"
            << Results.UndiscountedRewardReturn.GetMean() << "\t"
            << Results.UndiscountedRewardReturn.GetStdErr() << "\t"
            << Results.UndiscountedCostReturn.GetMean() << "\t"
            << Results.UndiscountedCostReturn.GetStdErr() << "\t"
            << Results.DiscountedRewardReturn.GetMean() << "\t"
            << Results.DiscountedRewardReturn.GetStdErr() << "\t"
            << Results.DiscountedCostReturn.GetMean() << "\t"
            << Results.DiscountedCostReturn.GetStdErr() << "\t"
            << Results.Time.GetMean() << "\t"
            << Results.OneStepTime.GetMean() << endl;
    }
}


//----------------------------------------------------------------------------
