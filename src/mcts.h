#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "utils.h"

class Policy {
    // assume that the number of stochastic actions is not larger than 2. (since the number of cost types is 1).
public:
    Policy() : minCostAction(-1), maxCostAction(1), probMinCostAction(-1), probMaxCostAction(-1) {
    }

    int sampleAction() {
        assert(probMinCostAction + probMaxCostAction >= 1 - 1e-6);
        double r = UTILS::RandomDouble(0, 1);
        if (r <= probMinCostAction){
            return minCostAction;
        } else {
            return maxCostAction;
        }
    }

    int getNumActions() {
        return (minCostAction == maxCostAction) ? 1 : 2;
    }

    int getOtherAction(int action) {
        assert((action == minCostAction || action == maxCostAction) && (minCostAction != maxCostAction));
        if (action == minCostAction) {
            return maxCostAction;
        } else {
            return minCostAction;
        }
    }

    double getActionProbability(int action) {
        assert(action == minCostAction || action == maxCostAction);
        if (action == minCostAction) {
            return probMinCostAction;
        } else {
            return probMaxCostAction;
        }
    }

    void setPolicy(double minCost, double maxCost, int minCostAction, int maxCostAction, double c_hat)
    {
        assert(minCost <= maxCost);
        if (maxCost <= c_hat) {
            this->minCostAction = this->maxCostAction = maxCostAction;
            this->probMinCostAction = this->probMaxCostAction = 1;
        }
        else if (minCost >= c_hat) {
            this->minCostAction = this->maxCostAction = minCostAction;
            this->probMinCostAction = this->probMaxCostAction = 1;
        }
        else {
            this->minCostAction = minCostAction;
            this->maxCostAction = maxCostAction;
            this->probMinCostAction = (c_hat - maxCost) / (minCost - maxCost);
            this->probMaxCostAction = 1 - this->probMinCostAction;
        }
    }

    double getProbMinCostAction() { return probMinCostAction; }
    double getProbMaxCostAction() { return probMaxCostAction; }
private:
    int minCostAction, maxCostAction;
    double probMinCostAction, probMaxCostAction;
};

class MCTS
{
public:

    struct PARAMS
    {
        PARAMS();

        int Verbose;
        int MaxDepth;
        int NumSimulations;
        int NumStartStates;
        bool UseTransforms;
        int NumTransforms;
        int MaxAttempts;
        int ExpandCount;
        double ExplorationConstant;
        bool UseRave;
        double RaveDiscount;
        double RaveConstant;
        bool DisableTree;
        double LambdaMax;
        double c_hat;
        int TreeAlgorithm;
    };

    MCTS(const SIMULATOR& simulator, const PARAMS& params);
    ~MCTS();

    Policy SelectAction();
    bool Update(int action, int observation, RC rewardcost);

    void UCTSearch();
    void RolloutSearch();

    RC Rollout(STATE& state);

    const BELIEF_STATE& BeliefState() const { return Root->Beliefs(); }
    const HISTORY& GetHistory() const { return History; }
    const SIMULATOR::STATUS& GetStatus() const { return Status; }
    void ClearStatistics();
    void DisplayStatistics(std::ostream& ostr) const;
    void DisplayValue(int depth, std::ostream& ostr) const;
    void DisplayPolicy(int depth, std::ostream& ostr) const;
    void setAdmissibleCost(double c_hat) {
        this->c_hat = c_hat;
    }
    double getAdmissibleCost() {
        return c_hat;
    }
    double getLambda() {
        return lambda;
    }
    VNODE* getRoot() {
        return Root;
    }
    double getNextAdmissibleCost(Policy policy, int sampledAction, RC rewardcost) {
        double newAdmisslbleCost;
        if (policy.getNumActions() == 1) {
            newAdmisslbleCost = (getAdmissibleCost() - rewardcost.C) / Simulator.GetDiscount();
        } else {
            int otherAction = policy.getOtherAction(sampledAction);
            double probAction = policy.getActionProbability(sampledAction);
            double otherActionQ_C = Root->Child(otherAction).Value.GetValue().C;
            newAdmisslbleCost = (c_hat - probAction * rewardcost.C - (1 - probAction) * otherActionQ_C) / (Simulator.GetDiscount() * probAction);
        }
        if (newAdmisslbleCost < 0) {
            newAdmisslbleCost = 0;
        }
        return newAdmisslbleCost;
    }

    static void UnitTest();
    static void InitFastUCB(double exploration);

private:

    const SIMULATOR& Simulator;
    int TreeDepth, PeakTreeDepth;
    PARAMS Params;
    VNODE* Root;
    HISTORY History;
    SIMULATOR::STATUS Status;
    double lambda;
    double lambdaMax;
    double c_hat;
    double initial_c_hat;
    int TreeAlgorithm;  // 0: CCPOMCP, 1: Baseline

    STATISTIC StatTreeDepth;
    STATISTIC StatRolloutDepth;
    STATISTIC StatTotalReward;
    STATISTIC StatTotalCost;

    Policy GreedyUCB(VNODE* vnode, bool ucb, bool stochastic) const;
    int SelectRandom() const;
    RC SimulateV(STATE& state, VNODE* vnode);
    RC SimulateQ(STATE& state, QNODE& qnode, int action);
    void AddRave(VNODE* vnode, RC totalRewardCost);
    VNODE* ExpandNode(const STATE* state);
    void AddSample(VNODE* node, const STATE& state);
    void AddTransforms(VNODE* root, BELIEF_STATE& beliefs);
    STATE* CreateTransform() const;
    void Resample(BELIEF_STATE& beliefs);
    RC Simulate(const BELIEF_STATE &beliefs, int iteration);

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

    static void UnitTestGreedy();
    static void UnitTestUCB();
    static void UnitTestRollout();
    static void UnitTestSearch(int depth);
};

#endif // MCTS_H
