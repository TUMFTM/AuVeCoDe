function vehicle = Package_Powertrain(vehicle,Parameters,drivetrain_type)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Korbinian Moller (Technical University of Munich)
%-------------
% Created on: 01.10.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  This function calculates the dimensions of the electric machines, of the gearbox and of the battery)
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - Parameters: struct with input and constant values
%           - drivetrain_type (see Package_assign_topology)
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------

%% Implementation
%1) Calculate electric machine
%2) Calculate the gearbox dimensions

%% 1) Calculate electric machine
%Dimensions of the electric machine (Modell Felgenhauer)
vehicle = Package_ElectricMachine(vehicle,Parameters);

%% 2) Calculate the gearbox
for axle=1:2
    if ~isempty(vehicle.LDS.GEARBOX{axle}) % gearbox cannot compute lower than 50Nm and higher than 700Nm
        % Check if range of torque is outreached
        if (vehicle.LDS.MOTOR{axle}.T_max <50) || (vehicle.LDS.MOTOR{axle}.T_max >600)
            vehicle.Error=6;
            return
        end
        
        %Calculate gearbox dimensions
        [vehicle.gearbox{axle}]=Package_gearbox_dim(vehicle,Parameters,axle);
                
        %Assign new measrurements        
        if drivetrain_type(axle) == 1 % 1: motor with parallel gear
            [vehicle] = set_dim_gear_paral(vehicle,axle); %dimensions of gear wheels
            
        elseif drivetrain_type(axle) == 2 % 2: motor with coaxial gear (planetary)
            [vehicle] = set_dim_gear_planetary(vehicle,axle,0);
            
        elseif drivetrain_type(axle) == 3 % 3: two motors in coaxial position
            [vehicle] = set_dim_gear_planetary(vehicle,axle,1);
            vehicle.dimensions.p_gear_nax{axle} = [0 0 0; vehicle.gearbox{axle}.results.d_gearbox/2 vehicle.gearbox{axle}.results.t_gearbox 3];
            
        elseif drivetrain_type(axle)==4 % 4: motor with coaxial gear (layshaft)
            [vehicle] = set_dim_gear_coaxial_ls(vehicle,axle); %dimensions of gear wheels            
        end
        
    else
        vehicle.gearbox{axle}={};
    end
end

end

%% Function to calculate the wheels of the gearboxes
function [vehicle] = set_dim_gear_coaxial_ls(vehicle,i)
    
    vehicle.dimensions.p_gear_coax_ls{i}(7,:)=[0 0 0]; %position of gear wheel connected to motor
    
    %% Initialize required Parameters
    gearbox = vehicle.gearbox{i};
    b_gap = gearbox.ConstDim.b_gap;             % Gap between components on the shafts [mm]
    b_seal = gearbox.ConstDim.b_seal;           % Width of seals on output shafts as average of database [mm]
    b_A = gearbox.bearings_1.b_A;               % Width of bearing A [mm]
    b_B = gearbox.bearings_1.b_B;               % Width of bearing B [mm]
    b_C = gearbox.bearings_2.b_C;               % Width of bearing C [mm]
    b_D = gearbox.bearings_2.b_D;               % Width of bearing D [mm]
    b_E = gearbox.bearings_3.b_E;               % Width of bearing E [mm]
    b_F = gearbox.bearings_3.b_F;               % Width of bearing F [mm]
    r_A = gearbox.bearings_1.d_A_A/2;           % Radius of bearing A [mm]
    r_B = gearbox.bearings_1.d_A_B/2;           % Radius of bearing B [mm]
    r_C = gearbox.bearings_2.d_A_C/2;           % Radius of bearing C [mm]
    r_D = gearbox.bearings_2.d_A_D/2;           % Radius of bearing D [mm]
    r_E = gearbox.bearings_3.d_A_E/2;           % Radius of bearing E [mm]
    r_F = gearbox.bearings_3.d_A_F/2;           % Radius of bearing F [mm]
    b_Diff = gearbox.diff.b_diffcage;           % Width of Diffcage [mm]
    r_Diff = gearbox.diff.d_diffcage/2;         % Radius of Diffcage [mm]
    % General
    psi=vehicle.Input.psi_motor(i); 
    
    %% Calculate gearbox dimensions
    
    % Housing
    l_gearbox=vehicle.gearbox{i}.results.l_gearbox;
    h_gearbox=vehicle.gearbox{i}.results.h_gearbox;
    t_gearbox=vehicle.gearbox{i}.results.t_gearbox;
    
    % Distance between axles, % x and z Position of Gearwheels
    if abs(vehicle.gearbox{i}.results.a_12-vehicle.gearbox{i}.results.a_23)<0.001
    p2_x=vehicle.gearbox{i}.results.a_12*cosd(psi);
    p2_z=vehicle.gearbox{i}.results.a_12*sind(psi);
    p3_x=p2_x; %3rd and 2nd stage are on the same shaft (coaxial) 
    p3_z=p2_z; %3rd and 2nd stage are on the same shaft (coaxial)
    else
        error('The axles of coaxial lay-shaft have to be at the same distance!')
    end
    
    % Size of Gearwheels
    r_1=vehicle.gearbox{i}.gears_12.d_1/2;
    r_2=vehicle.gearbox{i}.gears_12.d_2/2;
    r_3=vehicle.gearbox{i}.gears_34.d_3/2;
    r_4=vehicle.gearbox{i}.gears_34.d_4/2;
    b_1=vehicle.gearbox{i}.gears_12.b_1;
    b_3=vehicle.gearbox{i}.gears_34.b_3;
    
    % y Position of Gearwheels
    p1_y = b_seal + b_A + b_gap; % motor side of gear 1
    p2_y = p1_y;
    p4_y = p1_y + b_1 + b_gap + b_B + b_gap + b_F + b_gap;
    p3_y = p4_y;
    
    %y_position of auxiliary gearbox components
    P_A = b_seal + b_A/2;                   %center point of Bearing A
    P_B = p1_y + b_1 + b_gap + b_B/2;       %center point of Bearing B
    P_C = p2_y - b_gap - b_C/2;             %center point of Bearing C
    P_D = p3_y + b_3 + b_gap + b_D/2;       %center point of Bearing D
    P_E = p4_y + b_3 + b_Diff + b_E/2;      %center point of Bearing E
    P_F = p4_y - b_gap - b_F/2;             %center point of Bearing F
    P_Diff = p4_y + b_3 + b_Diff/2;         %center point of Differential cage
  
    
    %% Write parameters in gear struct
    vehicle.dimensions.p_gear_coax_ls{i}=[...
        0 p1_y 0;r_1 b_1 6;... %pos/dim 1st stage
        p2_x p2_y p2_z; r_2 b_1 6;... %pos/dim 2nd stage
        p3_x p3_y p3_z; r_3 b_3 6;... %pos/dim 3rd stage
        0 p4_y 0; r_4 b_3 6;... %pos/dim 4th stage
        0 0 0; l_gearbox t_gearbox h_gearbox];%%pos/dim of housing
    
    vehicle.dimensions.p_gear_coax_ls_aux{i} = [...
        0    P_A 0   ;r_A b_A 0;...     %pos/dim Bearing A 
        0    P_B 0   ;r_B b_B 0;...     %pos/dim Bearing B
        p2_x P_C p2_z;r_C b_C 0;...     %pos/dim Bearing C
        p2_x P_D p2_z;r_D b_D 0;...     %pos/dim Bearing D
        0    P_E 0   ;r_E b_E 0;...     %pos/dim Bearing E
        0    P_F 0   ;r_F b_F 0;...     %pos/dim Bearing F
        0    P_Diff 0;r_Diff b_Diff 1]; %pos/dim Differential -- 1 is indicator for Differential (important for plotting color)
    
end


function [vehicle] = set_dim_gear_paral(vehicle,i)

%% Initialize required Parameters
    gearbox = vehicle.gearbox{i};
    b_gap = gearbox.ConstDim.b_gap;             % Gap between components on the shafts [mm]
    b_park = gearbox.ConstDim.b_park;           % Width of parking gear [mm]
    b_seal = gearbox.ConstDim.b_seal;           % Width of seals on output shafts as average of database [mm]
    b_A = gearbox.bearings_1.b_A;               % Width of bearing A [mm]
    b_B = gearbox.bearings_1.b_B;               % Width of bearing B [mm]
    b_C = gearbox.bearings_2.b_C;               % Width of bearing C [mm]
    b_D = gearbox.bearings_2.b_D;               % Width of bearing D [mm]
    b_E = gearbox.bearings_3.b_E;               % Width of bearing E [mm]
    b_F = gearbox.bearings_3.b_F;               % Width of bearing F [mm]
    r_A = gearbox.bearings_1.d_A_A/2;           % Radius of bearing A [mm]
    r_B = gearbox.bearings_1.d_A_B/2;           % Radius of bearing B [mm]
    r_C = gearbox.bearings_2.d_A_C/2;           % Radius of bearing C [mm]
    r_D = gearbox.bearings_2.d_A_D/2;           % Radius of bearing D [mm]
    r_E = gearbox.bearings_3.d_A_E/2;           % Radius of bearing E [mm]
    r_F = gearbox.bearings_3.d_A_F/2;           % Radius of bearing F [mm]
    b_Diff = gearbox.diff.b_diffcage;           % Width of Diffcage [mm]
    r_Diff = gearbox.diff.d_diffcage/2;         % Radius of Diffcage [mm]

%% Calculate gearbox dimensions
    phi=vehicle.Input.psi_motor(i);
    gamma=phi-(360*vehicle.gearbox{i}.results.zeta/(2*pi()));

    if phi<=90
        lambda = 90-phi-(360*vehicle.gearbox{i}.results.theta/(2*pi()));
        n1=1;
    else 
        lambda = -(90-phi-(360*vehicle.gearbox{i}.results.theta/(2*pi())));
        n1=-1;
    end
    
    % Housing
    l_gearbox = gearbox.results.l_gearbox;
    h_gearbox = gearbox.results.h_gearbox;
    t_gearbox = gearbox.results.t_gearbox;
    
    % Distance between axles
    a_23_x = gearbox.results.a_23*cosd(gamma);
    a_12_x = gearbox.results.a_12*sind(lambda);
    a_23_z = gearbox.results.a_23*sind(gamma);
    a_12_z = gearbox.results.a_12*cosd(lambda);
    
    % x and z Position of Gearwheels
    p1_x = a_23_x+n1*a_12_x;
    p1_z = a_23_z+a_12_z;
    
    p2_x = gearbox.results.a_23*cosd(gamma);
    p2_z = gearbox.results.a_23*sind(gamma);
    
    % Size of Gearwheels
    r_1 = gearbox.gears_12.d_1/2;
    r_2 = gearbox.gears_12.d_2/2;
    r_3 = gearbox.gears_34.d_3/2;
    r_4 = gearbox.gears_34.d_4/2;
    b_1 = gearbox.gears_12.b_1;
    b_3 = gearbox.gears_34.b_3;
    
    % y Position of Gearwheels
    p1_y = b_seal+b_A+b_gap+b_park+b_gap; % motor side of gear 1
    p2_y = p1_y;
    p3_y = p2_y + b_1 + b_gap;
    p4_y = p3_y;
    
    %y_position of auxiliary gearbox components
    P_A = b_seal + b_A/2;                   %center point of Bearing A
    P_B = p1_y + b_1 + b_gap + b_B/2;       %center point of Bearing B
    P_C = p2_y - b_gap - b_C/2;             %center point of Bearing C
    P_D = p3_y + b_3 + b_gap + b_D/2;       %center point of Bearing D
    P_E = p4_y + b_3 + b_Diff + b_E/2;      %center point of Bearing E
    P_F = p4_y - 2*b_gap - b_F/2 - b_1;     %center point of Bearing F
    P_Diff = p4_y + b_3 + b_Diff/2;         %center point of Differential cage

    %% Write parameters in gear struct
    vehicle.dimensions.p_gear_paral{i}=[...
        p1_x p1_y p1_z;r_1 b_1 2;... %pos/dim 1st stage
        p2_x p2_y p2_z;r_2 b_1 2;... %pos/dim 2nd stage
        p2_x p3_y p2_z; r_3 b_3 2;... %pos/dim 3rd stage
        0 p4_y 0; r_4 b_3 2;... %pos/dim 4th stage
        0 0 0; l_gearbox, t_gearbox, h_gearbox]; %pos/dim of housing
    
    vehicle.dimensions.p_gear_paral_aux{i} = [...
        p1_x P_A p1_z;r_A b_A 0;...     %pos/dim Bearing A 
        p1_x P_B p1_z;r_B b_B 0;...     %pos/dim Bearing B
        p2_x P_C p2_z;r_C b_C 0;...     %pos/dim Bearing C
        p2_x P_D p2_z;r_D b_D 0;...     %pos/dim Bearing D
        0    P_E 0   ;r_E b_E 0;...     %pos/dim Bearing E
        0    P_F 0   ;r_F b_F 0;...     %pos/dim Bearing F
        0    P_Diff 0;r_Diff b_Diff 1]; %pos/dim Differential -- 1 is indicator for Differential (important for plotting color)
        
        

end


function [vehicle] = set_dim_gear_planetary(vehicle,i,nax)
%% Description:
%Assigns position and dimension of planetary gearbox components to
%corrponding struct
%-------------
% Inputs:
% vehicle:  vehicle structr
% i:        axle ID
% nax:      near axle topology yes/no (1,0)
%-------------

%% Initialization of required values:
gearbox = vehicle.gearbox{i};                   % Load gearbox
b_gap = gearbox.ConstDim.b_gap;                 % Gap between components on the shafts [mm]
b_seal = gearbox.ConstDim.b_seal;               % Width of seals on output shafts as average of database [mm]
ang_plan = 60*pi/180;                           % Angel of planets

%% Gearbox gear dimensions

% Housing
r_gearbox = gearbox.results.d_gearbox/2;        % Radius of gearbox housing [mm]
t_gearbox = gearbox.results.t_gearbox;          % Width of gearbox housing [mm]

% Size of Gearwheels
%Sun Gear
r_sun_o = gearbox.gears.d_1/2;                  % Outer radius of sun gear [mm]      
r_sun_i = gearbox.shafts.d_inn_1/2;             % Inner radius of sun gear [mm]  
b_sun  	= gearbox.gears.b_1;                    % Width of sun gear [mm]

%Planets stage 1
d_p1 = gearbox.gears.d_p1;                      % Pitch diameter of the planet 1 [mm]
r_pl_1 = d_p1/2;                                % Radius of planets stage 1 [mm]
b_pl_1 = b_sun;                                 % Width of planets stage 1  [mm]

%Planets stage 2
r_pl_2 = gearbox.gears.d_p2/2;                  % Radius of planets stage 2 [mm]
b_pl_2 = gearbox.gears.b_2;                     % Width of planets stage 2  [mm]

%Ring gear
d_f2 = gearbox.gears.d_f2;                      % Root diameter of ring gear [mm]
m_n_2 = gearbox.gears.m_n_2;                    % Normal module of the second stage [mm]
r_ring_o = (d_f2 + 6 * m_n_2)/2;                % Outer radius of ring gear [mm]      
r_ring_i = gearbox.gears.d_f2 /2;                % Inner radius of ring gear [mm]  
b_ring   = b_pl_2;                              % Width of planets stage 2  [mm]

%Diffcage
b_diff = gearbox.diff.b_diffcage;               % Width of Diffcage  [mm]
r_diff_o = gearbox.diff.d_diffcage/2;           % Outer Radius of Diffcage  [mm]
r_diff_i = gearbox.shafts.d_sh_3/2;             % Inner Radius of Diffcage  [mm]

%% Gearbox Bearing dimensions

b_A = gearbox.bearings_1.b_A;                   % Width of bearing A [mm]
b_B = gearbox.bearings_1.b_B;                   % Width of bearing B [mm]
b_C = gearbox.bearings_2.b_C;                   % Width of bearing C [mm]
b_D = gearbox.bearings_2.b_D;                   % Width of bearing D [mm]
b_E = gearbox.bearings_3.b_E;                   % Width of bearing E [mm]
b_F = gearbox.bearings_3.b_F;                   % Width of bearing F [mm]

r_A_o = gearbox.bearings_1.d_A_A/2;             % Outer Radius of bearing A [mm]
r_A_i = gearbox.bearings_1.d_sh_A/2;            % Inner Radius of bearing A [mm]

r_B_o = gearbox.bearings_1.d_A_B/2;             % Outer Radius of bearing B [mm]
r_B_i = gearbox.bearings_1.d_sh_B/2;            % Inner Radius of bearing B [mm]

r_C_o = gearbox.bearings_2.d_A_C/2;             % Outer Radius of bearing C [mm]
r_C_i = gearbox.bearings_2.d_sh_C/2;            % Inner Radius of bearing C [mm]

r_D_o = gearbox.bearings_2.d_A_D/2;             % Outer Radius of bearing D [mm]
r_D_i = gearbox.bearings_2.d_sh_D/2;            % Inner Radius of bearing D [mm]

r_E_o = gearbox.bearings_3.d_A_E/2;             % Outer Radius of bearing E [mm]
r_E_i = gearbox.bearings_3.d_sh_E/2;            % Inner Radius of bearing E [mm]

r_F_o = gearbox.bearings_3.d_A_F/2;             % Outer Radius of bearing F [mm]
r_F_i = gearbox.bearings_3.d_sh_F/2;            % Inner Radius of bearing F [mm]

%% Gearbox gear position x and z
%sun gear
p_sun_x = 0;
p_sun_z = 0;

%planets first stage
d_s = gearbox.gears.d_s;                                % Diameter of planet circle [mm]
p_pl_1_1_x = p_sun_x + d_s/2*cos(ang_plan/2);           % x position of planet 1 stage 1
p_pl_1_1_z = p_sun_z - d_s/2*sin(ang_plan/2);           % z position of planet 1 stage 1

p_pl_1_2_x = p_sun_x - d_s/2*cos(ang_plan/2);           % x position of planet 2 stage 1
p_pl_1_2_z = p_sun_x - d_s/2*sin(ang_plan/2);           % z position of planet 2 stage 1

p_pl_1_3_x = p_sun_x;                                   % x position of planet 3 stage 1
p_pl_1_3_z = p_sun_x + d_s/2;                           % z position of planet 3 stage 1

%planets second stage (same x and z coordinates as planets from stage 1)
p_pl_2_1_x = p_pl_1_1_x;                                % x position of planet 1 stage 2
p_pl_2_1_z = p_pl_1_1_z;                                % z position of planet 1 stage 2

p_pl_2_2_x = p_pl_1_2_x;                                % x position of planet 2 stage 2
p_pl_2_2_z = p_pl_1_2_z;                                % z position of planet 2 stage 2
    
p_pl_2_3_x = p_pl_1_3_x;                                % x position of planet 3 stage 2
p_pl_2_3_z = p_pl_1_3_z;                                % z position of planet z stage 2

%ring gear (same x and z coordinates as sun gear)
p_ring_x = p_sun_x;                                     % x position of ring gear
p_ring_z = p_sun_z;                                     % z position of ring gear

%diffcage (same x and z coordinates as sun gear)
p_diff_x = p_sun_x;                                     % x position of diffcage
p_diff_z = p_sun_z;                                     % z position of diffcage


%% Gearbox gear position y
%sun gear
p_sun_y = b_seal + b_F + b_A + 5 + b_sun/2;             % y position of sun gear (-5 is adapted from PAUMANI) -- Reference Position for all other wheels 

%planets first stage
p_pl_1_1_y = p_sun_y;                                   % y position of planet 1 stage 1
p_pl_1_2_y = p_sun_y;                                   % y position of planet 2 stage 1
p_pl_1_3_y = p_sun_y;                                   % y position of planet 3 stage 1

% planets second stage
p_pl_2_1_y = p_sun_y + b_sun/2 + b_gap + b_pl_2/2;      % y position of planet 1 stage 2
p_pl_2_2_y = p_pl_2_1_y;                                % y position of planet 2 stage 2
p_pl_2_3_y = p_pl_2_1_y;                                % y position of planet 3 stage 2

%ring gear (same y position as planets from second stage)
p_ring_y = p_pl_2_1_y;                                  % y position of ring gear

%% Bearing and Differential y Position (x and z positions are the same as x and z position of the corresponding gears --> written directly into comp matrix)

%Bearings A-D
p_A = p_sun_y - b_sun/2 - 5 - b_A/2;                    % y position of bearing A (-5 is adapted from PAUMANI)
p_C = p_sun_y - b_sun/2 - b_gap - b_C/2;                % y position of bearing C
p_D = p_pl_2_1_y + b_pl_2/2 + b_gap + b_D/2;            % y position of bearing D

%Differential
if gearbox.Input.num_EM == 1        % version 1 -- with differential
    p_diff_y = p_sun_y + b_sun/2 + b_gap + b_diff/2;      % y position of diffcage 
elseif gearbox.Input.num_EM == 2    % version 2 -- no differential - two motors per axle 
    p_diff_y = p_D + b_D/2 + 5 + b_diff/2;                % y position of diffcage 
end

%Bearings E-F
p_E = p_diff_y + b_diff/2 + b_E/2;                      % y position of bearing E
p_F = p_A - b_A/2 - b_F/2;                              % y position of bearing F



%% Define component matrix

components =[...
   %pos x       pos y       pos z       radius out  radius in   width   hollow  color 
    p_sun_x     p_sun_y     p_sun_z     r_sun_o     r_sun_i     b_sun   1       1;...    % sun gear
    p_pl_1_1_x  p_pl_1_1_y  p_pl_1_1_z  r_pl_1      0           b_pl_1  0       2;...    % planet_1_1
    p_pl_1_2_x  p_pl_1_2_y  p_pl_1_2_z  r_pl_1      0           b_pl_1  0       2;...    % planet_1_2
    p_pl_1_3_x  p_pl_1_3_y  p_pl_1_3_z  r_pl_1      0           b_pl_1  0       2;...    % planet_1_3
    p_pl_2_1_x  p_pl_2_1_y  p_pl_2_1_z  r_pl_2      0           b_pl_2  0       3;...    % planet_2_1
    p_pl_2_2_x  p_pl_2_2_y  p_pl_2_2_z  r_pl_2      0           b_pl_2  0       3;...    % planet_2_2
    p_pl_2_3_x  p_pl_2_3_y  p_pl_2_3_z  r_pl_2      0           b_pl_2  0       3;...    % planet_2_3
    p_ring_x    p_ring_y    p_ring_z    r_ring_o    r_ring_i    b_ring  1       4;...    % ring gear
    p_diff_x    p_diff_y    p_diff_z    r_diff_o    r_diff_i    b_diff  1       5;...    % differential
    p_sun_x     p_A         p_sun_z     r_A_o       r_A_i       b_A     1       6;...    % Bearing A
    p_pl_1_1_x  p_C         p_pl_1_1_z  r_C_o       r_C_i       b_C     1       6;...    % Bearing C-1
    p_pl_1_2_x  p_C         p_pl_1_2_z  r_C_o       r_C_i       b_C     1       6;...    % Bearing C-2
    p_pl_1_3_x  p_C         p_pl_1_3_z  r_C_o       r_C_i       b_C     1       6;...    % Bearing C-3
    p_pl_2_1_x  p_D         p_pl_2_1_z  r_D_o       r_D_i       b_D     1       6;...    % Bearing D-1
    p_pl_2_2_x  p_D         p_pl_2_2_z  r_D_o       r_D_i       b_D     1       6;...    % Bearing D-2
    p_pl_2_3_x  p_D         p_pl_2_3_z  r_D_o       r_D_i       b_D     1       6;...    % Bearing D-3
    p_sun_x     p_E         p_sun_z     r_E_o       r_E_i       b_E     1       6;...    % Bearing E
    p_sun_x     p_F         p_sun_z     r_F_o       r_F_i       b_F     1       6];      % Bearing F



%% Write parameters in gear struct

if nax == 0 % one engine per axle
    
    %housing (pos, dim and type)
    vehicle.dimensions.p_gear_coax{i} = [0 0 0;r_gearbox t_gearbox 3];
    
    %components
    vehicle.dimensions.comp_gear_plan{i} = components;

elseif nax == 1 % one engine per axle
    
    %housing (pos, dim and type)
    vehicle.dimensions.p_gear_nax{i} = [0 0 0; r_gearbox t_gearbox 3];

    % mirror components for second gearbox
    components_mirrored = components;
    components_mirrored(:,2) = - components(:,2);
    
    vehicle.dimensions.comp_gear_nax_1{i} = components_mirrored;
    vehicle.dimensions.comp_gear_nax_2{i} = components;
    
end

end
