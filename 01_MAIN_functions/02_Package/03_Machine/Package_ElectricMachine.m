function [v] = Package_ElectricMachine(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the measurements of the elcetric machince and is derived from the Master thesis of Koehler (FTM Master Thesis).
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
% ------------
% Output:   - vehicle(v): struct with data of the vehicle
% ------------

%% Implementation
% 1) Read in Parameters
% 2) Calculate measurements
% 3) Write values in vehicle struct

%% 1) Read in Parameters:
% Load empirical parameters for the dimensional chain
CX_hous_thick = Par.e_machine.CX.housing_thickness;             %Machine housing thickness in mm [1, p. 46]
CX_hous_rib_thick = Par.e_machine.CX.housing_rib_thickness;     %Thickness of machine housing ribs in mm [1, p. 46]
EY_stat_to_mach = Par.e_machine.EY.stator_to_machine_length;    %Difference between stator length and total motor length in mm [1, p. 46]

%Load power requirements from the LDS
v.e_machine=v.LDS.MOTOR;  %Assign the results from the LDS to the struct e_machine

%Load the stator ratio and the staor regression
ratios=Par.e_machine.stator_ratio;                                           %The ratio between stator diameter and stator length (dimensionless)
regr_e_machine=Par.regr.e_machine.stator_volume;                             %Regression between P_max and stator volume [1](p. 45-47)
regr_e_machine_not_filtered=Par.regr.e_machine.stator_volume_not_filtered;   %Regression stator volume considering also "outliner vehicles" see [1](p. 45-47)


%% 2) Calculate measurements
for axle=1:2 %-> axle=1 is the front axle; axle=2 is the rear axle    
    if ~isempty(v.e_machine{axle}) %Check if there is a machine on the axle
        
        P_max = v.e_machine{axle}.P_max_mech;     %Maximum Power of the e-machine in kW
        e_machine_type = v.e_machine{axle}.type;  %Electric machine type (ASM or PSM)
        
        % Empirical length/diameter-ratio of the stator depending on machine type (Koehler [1])
        switch char(e_machine_type)
            case 'PSM'
                stator_ratio = ratios.PMSM;
            case 'ASM'
                stator_ratio = ratios.IM;
            case 'SSM'
                stator_ratio = ratios.SSM;
        end
        
        % Regression to calculate the volume of the stator in L [1, p. 45]
        volume_stator = regr_e_machine.eq(P_max);
        
        %Check if the machine power is within the given regression limits.
        %In
        if P_max>regr_e_machine.Limits.Indep_var_1(2)+30
            v=errorlog(v,'Attention, the value of Pmax, required for the calculation of EM volume, is outside of the regression limits: the calculated values will be extrapolated');
            %We observed that if we are outside the regression limit, the "not_filtered" option offers a better extrapolation
            volume_stator = regr_e_machine_not_filtered.eq(P_max);
        end
        
        %Diameter of the stator in mm [1, p. 43]
        CX_stator_diameter = ((4*volume_stator*10^6)/(pi*stator_ratio))^(1/3);
        
        %Length of the stator in mm [1, p. 43]
        CY_stator_length = stator_ratio*CX_stator_diameter;
        
        %Include the dimensions of the housing to estimate the total machine dimensions [1, p. 43]
        e_machine_diameter = CX_stator_diameter+2*CX_hous_thick+2*CX_hous_rib_thick;    %Total e machine diameter in mm
        e_machine_length = CY_stator_length+EY_stat_to_mach;                            %Total e machine length in mm
        %A visualization of the dimensional chain is documented in the Appendix of [2]
        
        %% 3) Write values in vehicle struct
        v.e_machine{axle}.CY_e_machine_length = e_machine_length;       %Length of the entire machine (includes housings and covers) in mm
        v.e_machine{axle}.CX_e_machine_diameter = e_machine_diameter;   %Diameter of the entire machine (includes housings ribs and thickness) in mm
        v.e_machine{axle}.CX_stator_diameter=CX_stator_diameter;        %Diameter of the stator in mm
        v.e_machine{axle}.CY_stator_length=CY_stator_length;            %Length of the stator in mm
        v.e_machine{axle}.volume_stator=volume_stator;                  %Volume of the stator in L
    end
end
