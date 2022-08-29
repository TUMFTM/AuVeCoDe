function state = swarmfitness(state, pos, options, numParticles, varargin)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to calculate the fitness of particles in MOPSO/Multiswarm MOPSO
%              based on evaluate from NSGA-II by LSSSSWC, NWPU

% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - state: state of swarm or subswarm
%           - pos: positions matrix of particles
%           - options: optimization options
%           - numParticles: number of particles in swarm/subswarm
%           - varargin: vehicle Parameters
% ------------
% Output:   - state: state of swarm or subswarm containing evaluated particles' fitnesses
% ------------      

%% Implementation
    
    % allTime : use to calculate average evaluation times
    allTime = zeros(numParticles, 1);  
    
    % if parallel pool for evaluation is to be used
    if string(options.useParallel) == "yes"
        % use parfor
        parfor i = 1:numParticles
            % turn off all warnings
            warning('off','all')
            %fprintf('\nEvaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
            % evaluate particles fitness
            [fitness(i,:), allTime(i), nviol(i), violSum(i)] ...
                = evalparticle(pos(i,:), options.objfun, varargin{:});
            fprintf('Current Population: %d/%d...\n',i,numParticles);
        end

    %*************************************************************************
    % Evaluate objective function in serial
    %*************************************************************************
    % if parallel pool isn't used
    else
        % use for
        for i = 1:numParticles
            %fprintf('\nEvaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
            % evaluate particles fitness
            [fitness(i,:), allTime(i), nviol(i), violSum(i)]...
                = evalparticle(pos(i,:), options.objfun, varargin{:});
            fprintf('Current Population: %d/%d...\n',i,numParticles);
        end
    end
    
    % store calculated fitnesses in state
    state.Fitness = fitness;
    % state the violations in state (for Pareto sorting later)
    state.nViol = nviol';
    % state the violations sum in state (for Pareto sorting later)
    state.violSum = violSum';

    %*************************************************************************
    % Statistics
    %*************************************************************************
    % calculate average evaluation time for each vehicle concept
    state.avgEvalTime   = sum(allTime) / length(allTime);
    % collect total number of fitness evaluations
    state.FunEval = state.FunEval + numParticles;
    
end

function [y, evalTime, nViol, violSum] = evalparticle(indi, objfun, varargin)
% Function: [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Description: Evaluate one objective function.
%
%         LSSSSWC, NWPU
%    Revision: 1.1  Data: 2011-07-25
%*************************************************************************

    tStart = tic;
    [y, cons] = objfun( indi, varargin{:} );
    evalTime = toc(tStart);

    idx = find( cons );
    if( ~isempty(idx) )
        nViol = length(idx);
        violSum = sum( abs(cons) );
    else
        nViol = 0;
        violSum = 0;
    end
end