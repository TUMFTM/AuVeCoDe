function SubSwarm = create_subswarm(lb, ub, lbMatrix, ubMatrix, options, Parameters, ...
    ConstantVariableColumn, ConstantVariable)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to check the validity of generated swarm positions
%              based on implementation in particleswarm from MATLAB
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - lb, ub: vector of upper and lower bounds of the variable
%           - lbMatrix, ubMatrix: Matrix with upper and lower bounds of the variable
%           - options: options containing settings of the optimizer
%           - Parameter: vehicle Parameter (AuVeCoDe specific)
%           - ConstantVariableColumn: Column of the position vector/matrix with constant variable 
%           - ConstantVariable: value of the constant variable column for this subswarm
% ------------
% Output:   - SubSwarm: initialized subswarm
% ------------   

%% Implementation    
    % define the number of particles in the subswarm
    numParticles = options.SwarmSize;
    
    % generate population
    SubSwarm.Positions = random_SwarmPositions(ub, lb, ubMatrix,lbMatrix,numParticles,options.numVar,...
        options.vartype);
    
    % define maximum velocity as the difference between the upper and the
    % lower bounds
    vmax = ub-lb;
    % initialize velocity by random
    SubSwarm.Velocities = random_velocity(vmax, options.numVar, numParticles);
    
    % set constant variable value in the constant variable column
    SubSwarm.Positions(:, ConstantVariableColumn) = repmat(ConstantVariable, numParticles, 1);
    % set velocity in the constant variable column to zero
    SubSwarm.Velocities(:, ConstantVariableColumn) = zeros(numParticles, 1);
    
    % initialize the number of function evaluation in current subswarm
    SubSwarm.FunEval = 0;
    
    % calculate the particles' fitness of the current subswarms
    SubSwarm = swarmfitness(SubSwarm, SubSwarm.Positions,...
        options, numParticles, Parameters);
    
    % Initialize the individual best fitness and best position for all
    % particles in subswarm
    SubSwarm.IndividualBestFitness = SubSwarm.Fitness;
    SubSwarm.IndividualBestPositions = SubSwarm.Positions;
    
end