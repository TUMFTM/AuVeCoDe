function update_Verbosity(Verbosity, state, options)
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
% Input:    - Verbosity: struct containing Object for waitbar
%           - state: state of the optimization
%           - options: optimization options
% ------------
% Output:   - No output
% ------------    

%% Implementation   
    % define message for waitbar
    Message = sprintf("Iteration %i / %i:",...
        state.Iteration, options.MaxIter);
    % update waitbar
    waitbar(state.Iteration/options.MaxIter, Verbosity.Object,  Message); 
    
end