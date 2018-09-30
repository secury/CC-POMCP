#ifndef EXPERIMENT_H
#define EXPERIMENT_H

#include "mcts.h"
#include "simulator.h"
#include "statistic.h"
#include <fstream>

//----------------------------------------------------------------------------

struct RESULTS
{
    void Clear();

    STATISTIC Time;
    STATISTIC OneStepTime;
    STATISTIC TimeSteps;
    STATISTIC Reward;
    STATISTIC Cost;
    STATISTIC DiscountedRewardReturn;
    STATISTIC DiscountedCostReturn;
    STATISTIC UndiscountedRewardReturn;
    STATISTIC UndiscountedCostReturn;
};

inline void RESULTS::Clear()
{
    Time.Clear();
    OneStepTime.Clear();
    TimeSteps.Clear();
    Reward.Clear();
    Cost.Clear();
    DiscountedRewardReturn.Clear();
    DiscountedCostReturn.Clear();
    UndiscountedRewardReturn.Clear();
    UndiscountedCostReturn.Clear();
}

//----------------------------------------------------------------------------

class EXPERIMENT
{
public:

    struct PARAMS
    {
        PARAMS();
        
        int NumRuns;
        int NumSteps;
        int SimSteps;
        double TimeOut;
        int MinDoubles, MaxDoubles;
        int TransformDoubles;
        int TransformAttempts;
        double Accuracy;
        int UndiscountedHorizon;
        bool AutoExploration;
    };

    EXPERIMENT(const SIMULATOR& real, const SIMULATOR& simulator, 
        const std::string& outputFile, 
        EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams);

    void Run();
    void MultiRun();
    void DiscountedReturn();

private:

    const SIMULATOR& Real;
    const SIMULATOR& Simulator;
    EXPERIMENT::PARAMS& ExpParams;
    MCTS::PARAMS& SearchParams;
    RESULTS Results;

    std::ofstream OutputFile;
};

//----------------------------------------------------------------------------

#endif // EXPERIMENT_H
