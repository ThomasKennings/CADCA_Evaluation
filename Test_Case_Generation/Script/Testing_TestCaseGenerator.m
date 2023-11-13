%% Basic Test

tcg = TestCaseGenerator();
traceCell = tcg.performTraceGeneration();
tcg.plotLocTrace(traceCell);

%% Steering Test

tcg = TestCaseGenerator();
tcg.patternEgo = [0 0 0; 3 0 pi/8; 4 0 -pi/8; 5 0 0];
tcg.patternOther{1} = [0 0 0; 5 1 -pi/8; 6 1 pi/8; 7 0 0];
traceCell = tcg.performTraceGeneration();
tcg.plotLocTrace(traceCell);