function [v] = Package_Shock_absorber_dim(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the dimensions of the shock absorber. The upper
%               part of the shochk absorber has a different diameter than the lower part.

%------------------ Overview Diameters shock absorber ---------------------
%   ------------------------------------ 
%   ? height upper bearing      >|     |<upper_bearing_diameter_shock_absorber
%   -----------------------------|     |
%                                 |   |
%                                 |   |
%                                 |   |
%                                >|   |< piston_diameter
%                                 |   |
%                                 -----
%-------------------------------------------------------------------------
% ------------
% Sources:  [1] M. Spreng, „Maßkettenanalyse am Hinterwagen zur Erstellung von Ersatzmodellen,“Bachelor thesis, Faculty of Mechanical Engineering, Ostbayerische Technische Hochschule Regensburg, Regensburg, 2020.
%           [2] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%           [3] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
% ------------
% Output:   - vehicle(v): struct with data of the vehicle
% ------------

%% Implementation
%1) Assign shock absorber dimensions
%2) Assign Outputs


%% 1) Assign shock absorber dimensions
%Weight limit for the assignment of the shock absorber diameter. After this
%weight limit the diameter of the shock absorber has to increase
weight_limit=Par.rear_axle.weight_limit_shock_absorber;
axis_load_rear=v.masses.axle_load_r_max;           %in kg
axis_type=v.topology.axis_type_r;                
spring_layout=v.topology.spring_layout_r;

if axis_load_rear<weight_limit

    switch axis_type
        
        case 'torsion_beam'
            
            if strcmp(spring_layout,'Feder in X vor Daempfer')
                piston_diameter=Par.dimensions.CX.piston_diameter_r.torsion_beam;        %[mm]Average after categorisation
            else 
                piston_diameter=Par.dimensions.CX.piston_diameter_r.torsion_beam_2;      %[mm]Average after categorisation
            end

        case 'trapezoidal_link'
                piston_diameter=Par.dimensions.CX.piston_diameter_r.trapezoidal_link;    %[mm]Average after categorisation

        case 'sword_arm_link'
                piston_diameter=Par.dimensions.CX.piston_diameter_r.sword_arm_link;

        otherwise %five_link
                piston_diameter=Par.dimensions.CX.piston_diameter_r.five_link;           %[mm]Average after categorisation
    end
    
else 
        %the axle load has a particularly high value, a schock absorber with a bigger diameter is therefore required
        piston_diameter=Par.dimensions.CX.piston_diameter_r.max_weight;                  %[mm]Average after categorisation                                      

end

%Assign the diameter the upper part of the shock absorber (constant for all vehicles)
upper_bearing_diameter_shock_absorber=Par.dimensions.CX.upper_bearing_diameter_shock_absorber;    % in mm

%Assign the height of the shock absorber upper part in mm (constant from all the vehicles)
height_upper_bearing_shock_absorber=Par.dimensions.CZ.height_upper_bearing_shock_absorber;        % in mm

%% 2) Assign the Outputs:
v.dimensions.CX.piston_diameter=piston_diameter;
v.dimensions.CX.upper_bearing_diameter_shock_absorber=upper_bearing_diameter_shock_absorber;
v.dimensions.CZ.height_upper_bearing_shock_absorber=height_upper_bearing_shock_absorber;

end

