function [AllSolutions, NonDom] = collect_Pareto_solutions(state)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to find nondominated solutions without ranking the rest
%              solutions for MOPSO
%              based on ndsort from NSGA-II by LSSSSWC, NWPU
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - state: swarm/subswarm containing particles with evaluated fitnesses
% ------------
% Output:   - AllSolutions: all solutions with ranking 
%           - NonDom: non dominated solutions
% ------------

%% Implementation
    % collect the number of particles
    N = size(state.Positions,1);
    
    % create a table containing np, ID and rank to analyze the domination
    % status
    ind = table;
    % np defines how oft a solution get dominated by the rest particles
    ind.np = zeros(N,1);
    % ID contains particles unique IDs
    ind.ID = [1:N]';
    % rank contains the ranking
    ind.rank = inf(N,1);
    
    % create a NonDom table containing the particles data
    NonDom = table;
    % set unique particles ID
    NonDom.ParticleID = [1:N]';
    % set rank to infinite
    NonDom.rank = inf(N,1);
    % collect nViol, violSum and cons from state
    NonDom.nViol   = state.nViol;
    NonDom.violSum = state.violSum;
    NonDom.cons = state.cons;
    % collect Fitness and Positions from state
    NonDom.Fitness     = state.Fitness;
    NonDom.Positions     = state.Positions;
    
    % calculate domination matrix for time efficiency
    domMat  = calcDominationMatrix(NonDom.nViol, NonDom.violSum, NonDom.Fitness);

    % Compute np & sp and collect the domination status in ind table
    % Collect the dominated status
    Dominated = (domMat == -1);
    % np defines how oft a solution get dominated by the rest solutions
    ind.np = sum(Dominated,2);
    % create a matrix with particles ID
    ID = repmat(1:N,N,1);
    % collect dominate status
    Dominate = (domMat == 1);
    % sp defines which solutions are dominated by a solution
    ind.sp = ID .* Dominate;


    % find the non dominated solutions, because MOPSO only needs the leaders 
    % and doesn't require a complete ranking fo the rest solutions
    
    % collect the nps of all solutions
    dyn_np = ind.np;
    % start with first ranking
    ranking = 1;
    
    % find solutions with np = 0: means non dominated by other solutions
    rank_Pos = find(dyn_np == 0);
    % set the rank of these solutions as 1
    ind.rank(rank_Pos,1) = ranking;
    
    % transfer the ranking from ind into temptable
    NonDom.rank = ind.rank;
    
    % save uncut version of NonDom
    AllSolutions = NonDom;
    
    % select only solutions with ranking = 1
    NonDom = NonDom(NonDom.rank == 1, :);


end






function domMat = calcDominationMatrix(nViol, violSum, Fitness)
% Function: domMat = calcDominationMatrix(nViol, violSum, Fitness)
% Description: Calculate the domination maxtir which specified the domination
%   releation between two individual using constrained-domination.
%
% Return: 
%   domMat(N,N) : domination matrix
%       domMat(p,q)=1  : p dominates q
%       domMat(p,q)=-1 : q dominates p
%       domMat(p,q)=0  : non dominate
%
%    Copyright 2011 by LSSSSWC
%    Revision: 1.0  Data: 2011-07-13
%*************************************************************************

N       = size(Fitness, 1);
numObj  = size(Fitness, 2);

domMat  = zeros(N, N);

    for p = 1:N-1
        for q = p+1:N
            %*************************************************************************
            % 1. p and q are both feasible
            %*************************************************************************
            if(nViol(p) == 0 && nViol(q)==0)
                pdomq = false;
                qdomp = false;
                for i = 1:numObj
                    if( Fitness(p, i) < Fitness(q, i) )         % Fitnessective function is minimization!
                        pdomq = true;
                    elseif(Fitness(p, i) > Fitness(q, i))
                        qdomp = true;
                    end
                end

                if( pdomq && ~qdomp )
                    domMat(p, q) = 1;
                elseif(~pdomq && qdomp )
                    domMat(p, q) = -1;
                end
            %*************************************************************************
            % 2. p is feasible, and q is infeasible
            %*************************************************************************
            elseif(nViol(p) == 0 && nViol(q)~=0)
                domMat(p, q) = 1;
            %*************************************************************************
            % 3. q is feasible, and p is infeasible
            %*************************************************************************
            elseif(nViol(p) ~= 0 && nViol(q)==0)
                domMat(p, q) = -1;
            %*************************************************************************
            % 4. p and q are both infeasible
            %*************************************************************************
            else
                if(violSum(p) < violSum(q))
                    domMat(p, q) = 1;
                elseif(violSum(p) > violSum(q))
                    domMat(p, q) = -1;
                end
            end
        end
    end

    domMat = domMat - domMat';

end










