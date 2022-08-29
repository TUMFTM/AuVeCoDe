function v=Mass_EE(v,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the total weight of the electric (Low Voltage)
%               and electronic(High Voltage) components. 
%               The LV contains the LV battery and its cables, the LV wiring throughout the vehicle,
%               the fuses box and the vehicle ECUs.
% ------------
% Sources: [1] Lorenzo Nicoletti, "Parametric Modeling of Battery Electric Vehicles in the Early Development Phase", Technical University of Munich, Institute of Automotive Technology, 2022
%          [2] A. Romano, „Data-based Analysis for Parametric Weight Estimation of new BEV Concepts,“Master thesis, Institute of Automotive Technology, Technical University of Munich, Munich, 2021.
%          [3] L. Nicoletti, A. Romano, A. König, P. Köhler, M. Heinrich and M. Lienkamp, „An Estimation of the Lightweight Potential of Battery Electric Vehicles,“ Energies, vol. 14, no. 15, p. 4655, 2021, DOI: 10.3390/en14154655.
%          [4] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters (Par): struct with input and constant values
%           - vehicle(v): struct including the data of the vehicle
% ------------
% Output:   - vehicle(v): struct including the data of the vehicle
% ------------

%% Implementation
%1) Initialize the variables
%2) Weight of LV components
%3) Weight of HV components
%4) Weight of plugin system

%% 1) Initialize the variables
wheelbase = v.dimensions.GX.wheelbase;                              %Wheelbase in mm
battery_voltage=v.battery.battery_voltage;                          %The total nominal voltage of the battery in V
gross_energy=v.battery.energy_is_gross_in_kWh;                      %The gross energy of the battery, in kWh

%Regressions for the mass calculation of EE components
regr_LV_wiring=Par.regr.mass.LV_wiring_harness.eq;
regr_HV_cables=Par.regr.mass.HV_cables.eq;
regr_DCDC_converter=Par.regr.mass.DC_DC_converter.eq;
regr_HV_charger=Par.regr.mass.HV_charger.eq;

%% 2) Weight of LV components
%Low Voltage Electric components: 12V battery and cables, fuse box, wires
%throughout the vehicle leading to the vehicle ECUs.

% LV battery and its cables
low_voltage_battery_weight = Par.masses.LV_battery + Par.masses.LV_battery_cables; %21.048;           % [kg]

% Fuse box
fuse_box_weight = Par.masses.fuse_box; %1.044;                                                        % [kg] 

% LV wiring
LV_wiring_weight = regr_LV_wiring(wheelbase);%-73.590+0.035*wheelbase;                                % [kg] 

%Other LV components
additional_LV_weight = Par.masses.additional_LV_components; %4.2275; 

%Check that the result is not negative
check_mass(LV_wiring_weight); 

LV_components_weight=low_voltage_battery_weight+...
    fuse_box_weight+LV_wiring_weight+additional_LV_weight;            % [kg]

%% 3) Weight of HV components
%
%HV cables
HV_wiring_weight = regr_HV_cables(wheelbase); %-13.690+0.007*wheelbase;    % [kg]

%DC/DC converter and supports
DC_DC_converter_weight = regr_DCDC_converter(battery_voltage);               % [kg]
DC_DC_converter_support_weight = Par.masses.DCDC_converter_supports;%0.598;  % [kg]

%HV charger with supports
HV_charger_weight=regr_HV_charger(gross_energy) + Par.masses.HV_charger_support;%1.58+5.346+0.11*gross_energy;   % [kg]

%Inverters: one inverter is needed for each EM in the vehicle
%Constant value computed for one inverter with its supports
Inverter_weight = Par.masses.inverter + Par.masses.inverter_protection + Par.masses.inverter_supports;%9.1570; % [kg]

%One inverter for each e-machine. quantity is the number of EMs per axle
for axle=1:2
    if ~isempty(v.gearbox{axle}) %-> axle=1 is the front axle; axle=2 is the rear axle
        
        %DFor each machine on the axle a correspondingt inverter is required
        quantity=v.e_machine{axle}.quantity;
        inverters_weight(axle) = Inverter_weight*quantity;       
    end
end

%sum the weight of the inverters
inverters_weight=sum(inverters_weight);

%Account for other HV components like cable brackets and electric unit control
additional_HV_components = Par.masses.additional_HV_components;%1.8055;  % [kg]

%Check that the result is not negative
check_mass(HV_wiring_weight); 
check_mass(DC_DC_converter_weight); 
check_mass(HV_charger_weight)
check_mass(inverters_weight)

%Add up all HV components
HV_components_weight = HV_wiring_weight+DC_DC_converter_weight +...
    DC_DC_converter_support_weight+HV_charger_weight+inverters_weight+...
    additional_HV_components;

%% 4) Weight of plugin system
%This section calculates the weight of the plug-in system, i.e. AC
%charging cables, DC charging plug and cables to charger and battery, and
%other components like charging port supports, covers, lids, trim pieces,
%and power connectors.

% Home AC charging cable 3kW and fast AC charging cable for public charging stations
AC_charging_cables_weight = Par.masses.home_charging_cable + Par.masses.public_charging_cable;   % [kg]

%Charging plug with cables to HV charger and battery
charging_plug_weight = Par.masses.charging_plug;%3.4335;                                         % [kg]

%Additional charging components (charging port lid, supports etc)
additional_charging_weight = Par.masses.additional_charging_plug_components;%1.2595;             % [kg]

%Add up all plug-in components
plugin_components_weight = AC_charging_cables_weight+charging_plug_weight+...
    additional_charging_weight;

%% Assign output

v.masses.EE.LV_weight=LV_components_weight;
v.masses.EE.HV_weight=HV_components_weight;
v.masses.EE.plugin_sys_weight=plugin_components_weight;
end

