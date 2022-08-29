function [plot_options] = transform_options_MOPSO(options)
 %% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to adapt options from MOPSO to NSGA-II regardings plotnsga
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - options: options from MOPSO
% ------------
% Output:   - plot_options: options for plotnsga
% ------------    
       
%% Implementation    
    plot_options = options;
    plot_options.maxGen = options.MaxIter;
    plot_options.refPoints = [];

end