function [plotresult] = build_plotresult_MOPSO(plotresult, BestParticles, state)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to adapt MOPSO result to plotnsga
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - plotresult: struct containing data to be passed to plotnsga
%           - BestParticles: table containing best particles which already sorted by ranks
%           - state: struct containing data about current optimization state
% ------------
% Output:   - plotresult: struct plotresult with adapted data for current iteration
% ------------    
    
%% Implementation    
    % collect plotresult.pops from BestParticles
    for i = 1:height(BestParticles)
        plotresult.pops(state.Iteration, i).var = BestParticles.Positions(i,:);
        plotresult.pops(state.Iteration, i).obj = BestParticles.Fitness(i,:);
        plotresult.pops(state.Iteration, i).cons = BestParticles.cons(i);
        plotresult.pops(state.Iteration, i).nViol = BestParticles.nViol(i);
        plotresult.pops(state.Iteration, i).violSum = BestParticles.violSum(i);
    end
    
    % collect plotresult.states from state
    plotresult.states(state.Iteration).Iteration = state.Iteration;
    plotresult.states(state.Iteration).FunEval = state.FunEval;
    plotresult.states(state.Iteration).totaltime = toc(state.StartTime);
    
end
        