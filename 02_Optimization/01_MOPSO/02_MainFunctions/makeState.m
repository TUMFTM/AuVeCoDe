function state = makeState
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to initialize state for Multiswarm MOPSO 
%              based on state initialization in particleswarm from MATLAB
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - no input
% ------------
% Output:   - state: initialized state as struct
% ------------      
    state = struct;
        
    % current iteration counter, initial state means zero iteration
    state.Iteration = 0; 
    % initialize FunEval to count number of function evaluation during the
    % optimization process
    state.FunEval = 0;

end