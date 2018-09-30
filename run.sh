#--problem: rocksample
#--size, --number: (5, 5) / (5, 7) / (7, 8) / (11, 11) / (15, 15)
#--timeout: 43200 (12 hours)

SIZE=7
NUMBER=8
CHAT=1   # cost constraint
TREE=0   # 0: ccpomcp, 1: baseline


./src/ccpomcp --size "$SIZE" \
        --number "$NUMBER" \
        --timeout 43200 \
        --runs 100 \
        --mindoubles 3 \
        --maxdoubles 20 \
        --accuracy 0.001 \
        --autoexploration false \
        --exploration 20 \
        --verbose 0 \
        --outputfile result_rocksample_"$SIZE"_"$NUMBER"_TREE_"$TREE".txt \
        --c_hat "$CHAT" \
        --lambdamax 30 \
        --transformattempts 10000 \
        --treealgorithm "$TREE" \
        --problem rocksample

