function [vehicle]=Package_CCSystem(vehicle,wagontype,Parameters)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.02.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  Package_CCS calculates the package of the crash structure and the
%               cooling components
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - wagontype: 1=front, 2=rear
%           - Parameters: struct with input and constant values
% ------------
% Output:   - vehicle: struct with includes the data of the vehicle
% ------------
%% Implementation
% 1) Calculate position and dimension of bumper
%   a) Read/Calculate bumper dimensions
%   b) Bumper z-pos depended on axle height
% 2) Calculate position and dimension of cooler

%% 1) Calculate position and dimension of bumper
%   a) Read/Calculate bumper dimensions
    % Calculation
    [vehicle] = Package_Crossmember(vehicle,Parameters);
    % Read in inputs
    bumper_f=zeros(1,3); %Preallocate front bumper
    bumper_r=zeros(1,3); %Preallocate rear bumper

    bumper_f(1)     =   vehicle.dimensions.CX.crossmember_f; %The length of the front cross member 
    bumper_f(3)     =   vehicle.dimensions.CZ.crossmember_f; %The height of the front cross member

    bumper_r(1)     =   vehicle.dimensions.CX.crossmember_r; %The length of the rear cross memeber
    bumper_r(3)     =   vehicle.dimensions.CZ.crossmember_r; %The height of the rear cross member

    % Write front or rear bumper values into vehicle
    if wagontype==1    %1=Front, 2=Rear
    vehicle.dimensions.p_bumper{1}=[0 0 0;bumper_f(1)  0 bumper_f(3)]; % Front Bumper dim
    elseif wagontype==2
    vehicle.dimensions.p_bumper{2}=[0 0 0;bumper_r(1) 0 bumper_r(3)]; % Rear Bumper dim
    end

    % Bumper width (total width - diff)
    vehicle.dimensions.p_bumper{wagontype}(2,2)=vehicle.dimensions.GY.vehicle_width-Parameters.bumper.diff2width;

%   b) Bumper z-pos depended on axle height
    if vehicle.dimensions.p_drivetrain{wagontype}(2,3)==0
        vehicle.dimensions.p_bumper{wagontype}(1,3)=vehicle.dimensions.p_axle_height;
    elseif (vehicle.dimensions.p_wheelhouse_left{wagontype}(2,2)+115)>((vehicle.dimensions.GY.vehicle_width-vehicle.dimensions.p_drivetrain{wagontype}(2,2))/2);
        vehicle.dimensions.p_bumper{wagontype}(1,3)=vehicle.dimensions.p_drivetrain{wagontype}(1,3)+vehicle.dimensions.p_axle_height-vehicle.dimensions.p_powerel{wagontype}(2,3);
    else
        vehicle.dimensions.p_bumper{wagontype}(1,3)=vehicle.dimensions.p_axle_height+vehicle.gearbox{wagontype}.shafts.d_sh_3;
    end

%% 2) Calculate position and dimension of cooler
cooler_type=vehicle.aux.cooler_type;
if cooler_type==1 %Cooler in front of wheelhouse (Not implemented)
    vehicle.Error=7;
elseif cooler_type==2 % Cooler behind bumper
    % cooler width = available width
    w=vehicle.dimensions.GY.vehicle_width-(2*(vehicle.dimensions.p_wheelhouse_left{1}(2,2)+Parameters.body.F_Frame_w));
    vehicle.dimensions.p_cooler_ctr(2,2) = w;
    
    % cooler height = volume/(height*length)
    h=Parameters.aux.cooler_vol/(w*vehicle.dimensions.p_cooler_ctr(2,1));
    h=min(h,vehicle.dimensions.wheelhouse_width(1)); %maximum cooler height is wheelhouse height
    vehicle.dimensions.p_cooler_ctr(2,3) = h;
    
    % Cooler y-pos (Center of vehicle)
    vehicle.dimensions.p_cooler_ctr(1,2)=vehicle.dimensions.GY.vehicle_width/2;
    
    % Cooler z-pos (Underbody height + half height) --> -axle height due to
    % z-pos-shift in Package_Layer
    vehicle.dimensions.p_cooler_ctr(1,3)=-vehicle.dimensions.p_axle_height+vehicle.dimensions.GZ.H156+h/2;
end

end
