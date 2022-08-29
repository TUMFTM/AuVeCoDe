function Swarm = create_swarm(lbMatrix, ubMatrix, options, Parameters, ConstantVariableColumn, ConstantVariable)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to create swarm
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - lbMatrix, ubMatrix: Matrix with upper and lower bounds of the variable
%           - options: options containing settings of the optimizer
%           - Parameter: vehicle Parameter (AuVeCoDe specific)
%           - ConstantVariableColumn: Column of the position vector/matrix with constant variable 
%           - ConstantVariable: value of the constant variable column for this subswarm
% ------------
% Output:   - Swarm: initialized swarm
% ------------      
%% Implementation
    % makeState needs the vector of bounds, not the expanded matrix.
    lb = lbMatrix(1,:);
    ub = ubMatrix(1,:);
    
    numParticles = options.SwarmSize;
    
    % generate population
    Swarm.Positions = random_SwarmPositions(ubMatrix,lbMatrix,numParticles,options.numVar,...
        options.vartype);
    
    
    % Enforce bounds
    if any(any(Swarm.Positions < lbMatrix)) || any(any(Swarm.Positions > ubMatrix))
        Swarm.Positions = max(lbMatrix, Swarm.Positions);
        Swarm.Positions = min(ubMatrix, Swarm.Positions);
    end

    % Initialize velocities by randomly sampling over the smaller of 
    % options.InitialSwarmSpan or ub-lb. Note that min will be
    % InitialSwarmSpan if either lb or ub is not finite.
    vmax = ub-lb;
    Swarm.Velocities = random_velocity(vmax, options.numVar, numParticles);
    
    % set a variable constant for multiswarm PSO
    if ConstantVariableColumn > 0
        % set constant variable in a position
        Swarm.Positions(:, ConstantVariableColumn) = repmat(ConstantVariable, numParticles, 1);
        % set velocity in the position to zero
        Swarm.Velocities(:, ConstantVariableColumn) = zeros(numParticles, 1);
    end
    
    Swarm.FunEval = 0;
    
    Swarm = swarmfitness(Swarm, Swarm.Positions,...
        options, numParticles, Parameters);

    Swarm.IndividualBestFitness = Swarm.Fitness;
    Swarm.IndividualBestPositions = Swarm.Positions;
    
end