function gearbox = Package_gearbox_dim(v,Parameters,gearbox_id)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function computes gearboxes in lay-shaft or planetary design according to input from the vehicle- and Parameters-structs.
% ------------
% Sources:  [1] Peter Köhler, Semi-physikalische Modellierung von Antriebsstrangkomponenten für Elektrofahrzeuge , Master Thesis, Institute of Automotive Technology, TUM, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - vehicle(v): struct with data of the vehicle
%           - gearbox_id: 1=front axle and 2=rear axle
% ------------
% Output:   - gearbox: struct that includes the data of the gearbox
% ------------

%% 2) Loading and storage of static gearbox parameters:
gearbox = Parameters.gearbox;

gearbox.Input.T_max            = v.LDS.MOTOR{gearbox_id}.T_max;                                           % Maximum torque of el. motor [Nm]
gearbox.Input.T_nom            = v.LDS.MOTOR{gearbox_id}.T_max/v.LDS.MOTOR{gearbox_id}.overload_factor;   % Nominal torque of el. motor [Nm]
gearbox.Input.i_tot            = v.LDS.GEARBOX{gearbox_id}.i_gearbox;                                     % Total transmission ratio [-]
gearbox.Input.d_mot            = v.e_machine{gearbox_id}.CX_e_machine_diameter;                           % Diameter of el. motor [mm]
gearbox.Input.overload_factor  = v.LDS.MOTOR{gearbox_id}.overload_factor;                                 % Overload factor of el. motor [-]
gearbox.Input.type             = v.Input.gearbox_type{gearbox_id};                                        % Type of transmission
gearbox.Input.axles            = v.Input.gearbox_axles{gearbox_id};                                       % In- and output shafts parallel or coaxial
gearbox.Input.num_EM           = v.e_machine{gearbox_id}.quantity;                                        % Number of machine on the considered axle (front or rear axle). If there are 2 Machines (one per wheel) the differential is not required and won't be calculated
gearbox.diff.diff_orientation  = 'in';                                                                    % Desired differential orientation (empty for automatic selection) see Master Thesis Köhler pag. 52. Options: 'in'/'out'/''; The last option ('') tries first 'in' and if it does not fit chooses 'out'

if ~isfield(gearbox.Input,'opt_gears')%If there is no assigned optimization goal, then set the default gearbox optimization
    gearbox.Input.opt_gears        = {'Manual'};                                                          % Optimization goal (Height/Length/Mass/Manual)
end

gearbox.Input.inc_init         = 0;                                                                       % Angle between shafts with manual selection [deg]
gearbox.Input.manTR            = 0;                                                                       % 0 for automatic selection, 1 for manual selection    
gearbox.Errorlog               =[];                                                                       % Initialize structure to store Error logs

%Allocate the regression equations to the gearbox struct
gearbox.Regression = Parameters.regr.gearbox;

% Nominal engine speed from the engine characteristics in rpm
motor_characteristic = v.LDS.MOTOR{gearbox_id}.characteristic;
T_max = max(round(motor_characteristic(:,1),4));
idx = find(round(motor_characteristic(:,1),4) == T_max);
gearbox.Input.n_nom = motor_characteristic(idx(end),2);

% Sorting of the bearing catalogue (ascending inner diameter d)
gearbox.BearCtlg.ball = sortrows(Parameters.gearbox.BearCtlg.ball,'d','ascend');
gearbox.BearCtlg.ang_ball = sortrows(Parameters.gearbox.BearCtlg.ang_ball,'d','ascend');
gearbox.BearCtlg.roller = sortrows(Parameters.gearbox.BearCtlg.roller,'d','ascend');

gearbox.GearingConst.alpha_t = atan(tan(gearbox.GearingConst.alpha_n)/cos(gearbox.GearingConst.beta));  % Pressure angle in transverse section [rad]                                                    
gearbox.GearingConst.epsilon_beta = 2;                                                                  % Desired overlap ratio [-]
gearbox.GearingConst.beta_b = acos(sin(gearbox.GearingConst.alpha_n)/sin(gearbox.GearingConst.alpha_t));% Base helix angle of first stage [rad]
gearbox.CalcFactors.L_10 = gearbox.CalcFactors.L_10h*60*(gearbox.Input.n_nom)/(10^6);                   % Lifetime factor regarding differential shaft speed [10^6 revolutions]

%% 3) Switch for calculation of gear data and main dimensions for selected topology
if strcmpi(gearbox.Input.type,'lay-shaft')
    
    % Calculation of lay-shaft gearbox parameters
    gearbox = set_laysh_transmission(gearbox);
    
    % Calculation of gearbox main dimensions in mm
    gearbox = calc_dimensions(gearbox);
    
elseif strcmpi(gearbox.Input.type,'planetary')
    
    % The user input for the planetary gearbox teeth is not implemented!
    gearbox.Input.manTR = 0;
    
    % Calculation of planetary gearbox parameters
    gearbox = set_plan_transmission(gearbox);
    % Calculation of gearbox main dimensions in mm
    gearbox = calc_dimensions_plan(gearbox);
end

end
