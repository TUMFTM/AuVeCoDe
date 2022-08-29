function [veh] = Auxiliary_HVAC(veh,Par)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 10.03.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: Calculate energy consumption due to HVAC
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
% 1) Preprocessing calculations
% 2) Load Par of Cabin
% 3) Calculate auxiliary use
% 4) Assign output

%% Check if calculation is switched on
if veh.Input.HVAC_calc==1
%% 1) Preprocessing calculations
veh=CALCULATE_Cabin(veh,Par); %Read an calculate cabin measurements

%% 2) Load vehicle parameters needed for calculation
%Read in parameters
Alpha       = veh.aux.exterior_measurements.Alpha;      %Angle between horizontal and windshield
Beta        = veh.aux.exterior_measurements.Betta;      %Angle between horizontal and sidewindow
Gamma       = veh.aux.exterior_measurements.Gamma;      %Angle between horizontal and rearwindow
Area_Front  = veh.aux.exterior_measurements.Area_Front; %Front glas area in m^2
Area_Side   = veh.aux.exterior_measurements.Area_Side;  %Side glas area in m^2
Area_Back   = veh.aux.exterior_measurements.Area_Back;  %Back glas area in m^2
Area_Cabin  = veh.aux.exterior_measurements.Area_Cabin; %Cabin surface in m^2
T_amb       = str2double(veh.Input.HVAC_t_amb);         %Ambient temperature in °C
Scenario    = veh.Input.HVAC_scenario;                  %Scenario (name)
m_flow      = veh.Input.HVAC_m_flow;                    %massflow in kg/min
perc_ca     = veh.Input.HVAC_perc_ca;                   %pecantage of circulated air in %/100
T_set       = str2double(veh.Input.HVAC_T_set);         %Temperature set (only Winter)


% Calculations needed to calculate input:



%% 3) Load regresssion function

if strcmp(Scenario,'Munich - Summer')
    Input= [Alpha, Beta, Gamma, Area_Front, Area_Side, Area_Back, Area_Cabin];
    switch T_amb
        case 25
            num=1;
        case 28
            num=2;
        case 30
            num=3;
        case 32
            num=4;
    end   
    
    %Check using Alpha (front window angle) which regression to use
    if (Alpha<75 && Alpha>40) %if angle is between two regressions
        perc_conv=(75-Alpha)./35;
        perc_OB=1-perc_conv;
        
        fun_conv=Par.aux.HVAC.reg_cooling{3,num}; %Read function
        par_conv=Par.aux.HVAC.reg_cooling{2,num}; %Read parameters
        aux_HVAC_conv=fun_conv(par_conv,Input); %Calculate consumption in W
        
        fun_OB=Par.aux.HVAC.reg_cooling{3,num+4}; %Read function
        par_OB=Par.aux.HVAC.reg_cooling{2,num+4}; %Read parameters
        aux_HVAC_OB=fun_OB(par_OB,Input); %Calculate consumption in W
        
        aux_HVAC=perc_conv.*aux_HVAC_conv+perc_OB.*aux_HVAC_OB;
        
    elseif Alpha>=75 %if angle inside one-box regression
        fun_OB=Par.aux.HVAC.reg_cooling{3,num+4}; %Read function
        par_OB=Par.aux.HVAC.reg_cooling{2,num+4}; %Read parameters
        aux_HVAC=fun_OB(par_OB,Input); %Calculate consumption in W
    elseif Alpha<=40 %if angle inside conventional regression
        fun_conv=Par.aux.HVAC.reg_cooling{3,num}; %Read function
        par_conv=Par.aux.HVAC.reg_cooling{2,num}; %Read parameters
        aux_HVAC=fun_conv(par_conv,Input); %Calculate consumption in W
    else
        error('HVAC Scenario not covered')
    end
    
elseif strcmp(Scenario,'Munich - Winter')
    switch T_set
        case 22
            num=1;
        case 23
            num=2;
        case 24
            num=3;
    end
    
    Input= [T_amb, m_flow, Area_Cabin, perc_ca];
    fun=Par.aux.HVAC.reg_heating{3,num}; %Read function
    par=Par.aux.HVAC.reg_heating{2,num}; %Read parameters
    aux_HVAC=fun(par,Input); %Calculate consumption in W
    
elseif strcmp(Scenario,'Munich - Spring/Fall')
    %Define Input
    Input= [Alpha, Beta, Gamma, Area_Front, Area_Side, Area_Back, Area_Cabin, m_flow];
    %Check using Alpha (front window angle) which regression to use
    if (Alpha<75 && Alpha>40) %if angle is between two regressions
        perc_conv=(75-Alpha)./35;
        perc_OB=1-perc_conv;
        
        fun_conv=Par.aux.HVAC.reg_moderate{3,1}; %Read function
        par_conv=Par.aux.HVAC.reg_moderate{2,1}; %Read parameters
        aux_HVAC_conv=fun_conv(par_conv,Input); %Calculate consumption in W
        
        fun_OB=Par.aux.HVAC.reg_moderate{3,2}; %Read function
        par_OB=Par.aux.HVAC.reg_moderate{2,2}; %Read parameters
        aux_HVAC_OB=fun_OB(par_OB,Input); %Calculate consumption in W
        
        aux_HVAC=perc_conv.*aux_HVAC_conv+perc_OB.*aux_HVAC_OB;
        
    elseif Alpha>=75 %if angle inside one-box regression
        fun_OB=Par.aux.HVAC.reg_moderate{3,2}; %Read function
        par_OB=Par.aux.HVAC.reg_moderate{2,2}; %Read parameters
        aux_HVAC=fun_OB(par_OB,Input); %Calculate consumption in W
    elseif Alpha<=40 %if angle inside conventional regression
        fun_conv=Par.aux.HVAC.reg_moderate{3,1}; %Read function
        par_conv=Par.aux.HVAC.reg_moderate{2,1}; %Read parameters
        aux_HVAC=fun_conv(par_conv,Input); %Calculate consumption in W
    else
        error('HVAC Scenario not covered')
    end    
end



%% 4) Assign output

veh.aux.HVAC_con=aux_HVAC; %write consumption of HVAC in W

%% If HVAC calculation is switched off
else
    veh.aux.HVAC_con=0; %write consumption of HVAC in W
end

%% Examples
%Point_Polo = [28.75, 65, 40.75, 1.168, 0.516,0.532,5];
%Point_eGolf = [29, 65, 49, 1.159, 0.608,0.511,8];
%Point_UNICAR = [72, 86.8, 72, 1.8, 1.163, 1.63, 11];
%Point_ID3 = [26.5, 65, 41, 1.36, 0.67, 0.553, 6.5];

end

