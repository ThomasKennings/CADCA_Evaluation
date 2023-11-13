%% One additional vehicle 1 faulty data

%[solTable, eqVal, eqSys] = AnomalyEquationSolver.genTestCase([1 1; 0 1], 1)

[~, eqVal, eqSys] = AnomalyEquationSolver.genTestCase([1 1; 0 1], 1);

%AnomalyEquationSolver.getFaultyIndividualSolution(solTable, eqVal)

solCell = AnomalyEquationSolver.solveEquationSystem(eqSys, eqVal, 2);

[TPR, FPR, TP, TN, FP, FN] = AnomalyEquationSolver.verifyResult([1 1; 0 1], solCell{3})

%%

solTable = [...
    nan nan 0 nan nan nan 0; 
    nan nan nan 0 0 nan nan; 
    0 nan nan nan 0 nan nan; 
    nan nan nan 0 0 0 nan; 
    0 nan nan nan  nan nan 0];
eqVal = [0, 0, 0, 0, 0]';

AnomalyEquationSolver.getSolutionSpace(solTable,eqVal)