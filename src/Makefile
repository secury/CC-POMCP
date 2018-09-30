FLAGS=-lboost_program_options -O3

all: ccpomcp

ccpomcp: beliefstate.o coord.o experiment.o main.o mcts.o node.o rocksample.o simulator.o utils.o testsimulator.o
	g++ -lm -o $@ beliefstate.o coord.o experiment.o main.o mcts.o node.o rocksample.o simulator.o utils.o testsimulator.o ${FLAGS}

.cpp.o:
	g++ -c $< ${FLAGS}

beliefstate.o: beliefstate.cpp beliefstate.h simulator.h utils.h
coord.o: coord.cpp coord.h utils.h
experiment.o: experiment.cpp experiment.h 
main.o: main.cpp mcts.h rocksample.h experiment.h
mcts.o: mcts.cpp mcts.h testsimulator.h
node.o: node.cpp node.h history.h utils.h
rocksample.o: rocksample.cpp rocksample.h utils.h
simulator.o: simulator.cpp simulator.h
utils.o: utils.cpp utils.h coord.h memorypool.h
testsimulator.o: testsimulator.cpp testsimulator.h utils.h

clean:
	rm -f *.o ccpomcp

