function swarmpos = checkswarm(swarmpos,vartype,lbMatrix,ubMatrix)
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
% Input:    - swarmpos: matrix containing particles' positions
%           - vartype: vector containing variables type of the input variables
%           - lbMatrix, ubMatrix: Matrix with upper and lower bounds of the
%           - variable
% ------------
% Output:   - swarmpos: validated swarmpos
% ------------

%% Implementation   
    %% Step 1: check variable step
    % expand vartype into matrix to simplify the variable check's process
    vartype = repmat(vartype,height(swarmpos),1);
    % find the integers type from the vartype matrix
    integers = vartype == 2;
    % round all variables in swarmpos which have to be integer
    swarmpos(integers) = round(swarmpos(integers));
    
    %% Step 2: Keep Positions between bounds
    % if there is any position that is outside the lower bounds and the
    % upper bounds, then set the value of the position as lower bound (if
    % position below lower bound) or as upper bound (if position higher
    % than upper bound)
    if any(any(swarmpos < lbMatrix)) || any(any(swarmpos > ubMatrix))
        swarmpos = max(lbMatrix, swarmpos);
        swarmpos = min(ubMatrix, swarmpos);
    end
end