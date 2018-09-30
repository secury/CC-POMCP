#ifndef BELIEF_STATE_H
#define BELIEF_STATE_H

#include <vector>

class STATE;
class SIMULATOR;

class BELIEF_STATE
{
public:

    BELIEF_STATE();

    // Free memory for all states
    void Free(const SIMULATOR& simulator);

    // Creates new state, now owned by caller
    STATE* CreateSample(const SIMULATOR& simulator) const;

    // Added state is owned by belief state
    void AddSample(STATE* state);

    // Make own copies of all samples
    void Copy(const BELIEF_STATE& beliefs, const SIMULATOR& simulator);

    // Move all samples into this belief state
    void Move(BELIEF_STATE& beliefs);

    bool Empty() const { return Samples.empty(); }
    int GetNumSamples() const { return Samples.size(); }
    const STATE* GetSample(int index) const { return Samples[index]; }
    
private:

    std::vector<STATE*> Samples;
};

#endif // BELIEF_STATE_H
