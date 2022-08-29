function SwarmPositions = random_SwarmPositions(ub, lb, ubMatrix,lbMatrix,numParticles,nvars,vartype)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to generate swarm/subswarm particles' positions by random
%              based on the generation of particles in particleswarm in MATLAB
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - lb, ub: vector of upper and lower bounds of the variable
%           - lbMatrix, ubMatrix: Matrix with upper and lower bounds of the variable
%           - numParticles: number of particles in current swarm/subswarm
%           - nvar: number of input variables
%           - vartype: vector containing the variable type of every input variable
% ------------
% Output:   - SwarmPositions: random generated particles' positions
% ------------   

%% Implementation
    % define the span by deriving from upper and lower bound
    span = ub - lb;
    
    % create particles' positions by adding lower bound with span times a
    % randomized number
    SwarmPositions = repmat(lb,numParticles,1) + ...
    repmat(span,numParticles,1) .* rand(numParticles,nvars);
    
    % check the validity of the generated swarm positions
    SwarmPositions = checkswarm(SwarmPositions,vartype,lbMatrix,ubMatrix);

end