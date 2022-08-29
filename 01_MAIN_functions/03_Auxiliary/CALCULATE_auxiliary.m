function [veh] = CALCULATE_auxiliary(veh,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 10.03.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Calculate total auxiliary consumption
% ------------
% Sources:  [1] Adrian König, S. Mayer, L. Nicoletti, S. Tumphart, und M. Lienkamp, “The Impact of HVAC on the Development of Autonomous and Electric Vehicle Concepts,” Energies, Bd. 15, Rn. 2, S. 441, 2022.
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Par: struct with input and constant values
%           - veh: struct including the data of the vehicle
% ------------
% Output:   - veh: struct including the data of the vehicle
% ------------

%% Implementation:
% 1) Calculate HVAC
% 2) Calculate Sensors and ECU
% 3) Write into vehicle

%% Read in parameters
aux_base=veh.aux.base; %base auxiliary consumption

%% 1) HVAC
[veh] = Auxiliary_HVAC(veh,Par);

%% 2) Sensors and ECU
[veh] = Auxiliary_SensECU(veh,Par);

%% 3) Write into vehicle
%Calculate total consumption
aux_tot=aux_base+veh.aux.HVAC_con/1000; %total concumption = base+ HVAC + Sensors/ECU in kW

%Write value in vehicle struct
veh.Input.power_auxiliaries=aux_tot+veh.aux.SensECU_con ; %auxiliary power use in kW
    
end

