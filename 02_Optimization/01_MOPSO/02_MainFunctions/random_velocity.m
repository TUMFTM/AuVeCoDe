function [velocities] = random_velocity(vmax, nvars, numParticles)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function for random velocity
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vmax: vector of maximum velocity
%           - nvars: number of input variables
%           - numParticles: number of particles in current swarm/subswarm
% ------------
% Output:   - velocities: randomized particles' velocities
% ------------     

%% Implementation
    % velocities: randomized particles' velocities
     velocities = repmat(-vmax,numParticles,1) + ...
        repmat(2*vmax,numParticles,1) .* rand(numParticles,nvars);
end