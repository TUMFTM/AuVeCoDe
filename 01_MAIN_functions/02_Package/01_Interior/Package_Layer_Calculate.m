function [boundary,vehicle] = Package_Layer_Calculate(vehicle,wagontype,body,size,psi)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates a layers for a specific component taking the
%              block formation into account
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - body: body for the calculated boundary lines
%           - vehicle: struct including the data of the vehicle
%           - Psi: Angle of Motor to Gear in Degree
%           - Wagon type (1=front, 2=rear)
%           - size: needed size of vector (interior + interior heigth)
% ------------
% Output:   - vehicle: struct including the data of the vehicle
% ------------

%% Implementation
% 1) Presize and initialize boundary
% 2) Calculate boundary layer
% 3) Optional: Plot to check results

%% 1) Presize and initialize boundary
boundary=NaN(size,2);
body=round(body);
% Move Body with axle height into positive z-values to avoid negative indexing
body(1,3)=ceil(body(1,3)+vehicle.dimensions.p_axle_height);

% Move Body relative to bumper front surface (position - half x-length), because bumper is reference for block formation
body(1,1)=ceil(body(1,1)-(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1)));

%% 2) Calculate boundary layer
% Differentiate between possible body shapes
if body(2,3)>10 %body is a rectangle
    % Calculate Square Shape
    boundary(ceil(body(1,3)-0.5*body(2,3)):floor(body(1,3)+0.5*body(2,3)),1)=body(1,1)-0.5*body(2,1); %Calculate back boundary line (in x-direction)
    boundary(ceil(body(1,3)-0.5*body(2,3)):floor(body(1,3)+0.5*body(2,3)),2)=body(1,1)+0.5*body(2,1); %Calulate front boundary line (in x-direction)
    
elseif body(2,3)==1 || body(2,3)==3 %body is cylinder
    % Calculate Radius Shape
    int_a=0:1:2*body(2,1); %presize z-coordinates
    int_a=int_a';
    boundary(body(1,3)-body(2,1):body(1,3)+body(2,1),1)=-sqrt(body(2,1).^2-(abs(body(2,1)-int_a)).^2)+body(1,1); %Calculate back boundary line (in x-direction)
    boundary(body(1,3)-body(2,1):body(1,3)+body(2,1),2)=sqrt(body(2,1).^2-(abs(body(2,1)-int_a)).^2)+body(1,1); %Calculate front boundary line (in x-direction)
    
elseif body(2,3)==2 || body(2,3)==6 %body is parallel/coaxial powertrain
    %% Initialize boundary vectors 
    % Legend: 1=motor, 2=upper circle shape of wheelhouse, 3=rectangle shape of wheelhouse, 4=lower circle shape of wheelhouse
    
    % Presize z-coordinates
    int_a1=0:1:2*body(2,1); %motor diameter
    int_a1=int_a1';
    int_a2=0:1:body(12,3); %cirlce diameter is gearbox housing height
    int_a2=int_a2';
    height_psi=sind(psi)*(body(12,1)-body(12,3)); %height of only rectangle part of gearbox housing (length-height(=2*radius))
    int_a3=0:1:height_psi; 
    int_a3=int_a3';
    int_a4=0:1:body(12,3); %cirlce diameter is gearbox housing height
    int_a4=int_a4';
    
    %housing radius
    r_house=body(12,3)/2; %housing height/2
    
    %% Calculate points for housing circle/rectangle
    % Calculate point for lower circle of housing
    if body(2,3)==2 %parallel gear => use 4th stage (on motor)
        int_a4_x=body(11,1)+body(9,1); %offset+position of axle
        int_a4_z=body(11,3)+body(9,3); %offset+position of axle
    else %coaxial layshaft => use 1st stage (on motor)
        int_a4_x=body(11,1)+body(3,1); %offset+position of axle
        int_a4_z=body(11,3)+body(3,3); %offset+position of axle
    end
        
    
    % Calculate point for upper circle of housing
    int_a2_x=int_a4_x+cosd(psi)*(body(12,1)-body(12,3)); %
    int_a2_z=int_a4_z+sind(psi)*(body(12,1)-body(12,3)); %
    
    % Calculate rectangle part of housing front/rear
    %front:
    int_a3_f_x=int_a4_x+sind(psi)*0.5*body(12,3); %starting point (x) of front line
    int_a3_f_z=int_a4_z-cosd(psi)*0.5*body(12,3); %starting point (z) of front line
    
    %rear
    int_a3_r_x=int_a4_x-sind(psi)*0.5*body(12,3); %starting point of front line
    int_a3_r_z=int_a4_z+cosd(psi)*0.5*body(12,3); %starting point (z) of front line
    
    %% Move points to avoid negative indexing
    % Move points relative to bumper front surface (position - half x-length), because bumper is reference for block formation
    int_a2_x=ceil(int_a2_x-(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1)));
    int_a3_f_x=ceil(int_a3_f_x-(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1)));
    int_a3_r_x=ceil(int_a3_r_x-(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1)));
    int_a4_x=ceil(int_a4_x-(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1)));
    
    % Move points with axle height into positive z-values to avoid negative indexing
    int_a2_z=ceil(int_a2_z+vehicle.dimensions.p_axle_height);  
    int_a3_f_z=ceil(int_a3_f_z+vehicle.dimensions.p_axle_height);
    int_a3_r_z=ceil(int_a3_r_z+vehicle.dimensions.p_axle_height);
    int_a4_z=ceil(int_a4_z+vehicle.dimensions.p_axle_height);
    
    %% Calculate boundary surfaces
    % Legend: 1=motor, 2=upper circle shape of wheelhouse, 3=rectangle shape of wheelhouse, 4=lower circle shape of wheelhouse
    % Initialize temp. back and front boundaries
    boundary_temp_1_back=NaN(size,2);
    boundary_temp_1_front=boundary_temp_1_back;
    boundary_temp_2_back=boundary_temp_1_back;
    boundary_temp_2_front=boundary_temp_1_back;
    boundary_temp_3_back=boundary_temp_1_back;
    boundary_temp_3_front=boundary_temp_1_back;
    boundary_temp_4_back=boundary_temp_1_back;
    boundary_temp_4_front=boundary_temp_1_back;
    
    boundary_temp_1_back((body(1,3)-body(2,1)):(body(1,3)+body(2,1)),1)=-sqrt(body(2,1).^2-(abs(body(2,1)-int_a1)).^2)+body(1,1); %Calculate back boundary line (in x-direction)
    boundary_temp_1_front((body(1,3)-body(2,1)):(body(1,3)+body(2,1)),2)=sqrt(body(2,1).^2-(abs(body(2,1)-int_a1)).^2)+body(1,1); %Calculate front boundary line (in x-direction)
    
    boundary_temp_2_back(round((int_a2_z-r_house)):round((int_a2_z+r_house)),1)=-sqrt(r_house.^2-(abs(r_house-int_a2)).^2)+int_a2_x; %Calculate back boundary line (in x-direction)
    boundary_temp_2_front(round((int_a2_z-r_house)):round((int_a2_z+r_house)),2)=sqrt(r_house.^2-(abs(r_house-int_a2)).^2)+int_a2_x; %Calculate front boundary line (in x-direction)
        
    boundary_temp_3_back(int_a3_r_z:floor(int_a3_r_z+height_psi),1)=int_a3/(tand(psi))+int_a3_r_x; %Calculate front boundary line (in x-direction)
    boundary_temp_3_front(int_a3_f_z:floor(int_a3_f_z+height_psi),2)=int_a3/(tand(psi))+int_a3_f_x; %Calculate back boundary line (in x-direction)
    
    boundary_temp_4_back(round(int_a4_z-r_house):round(int_a4_z+r_house),1)=-sqrt(r_house.^2-(abs(r_house-int_a4)).^2)+int_a4_x; %Calculate back boundary line (in x-direction)
    boundary_temp_4_front(round(int_a4_z-r_house):round(int_a4_z+r_house),2)=sqrt(r_house.^2-(abs(r_house-int_a4)).^2)+int_a4_x; %Calculate front boundary line (in x-direction)
    
    boundary(:,1)=min([boundary_temp_1_back boundary_temp_2_back boundary_temp_3_back boundary_temp_4_back],[],2);
    boundary(:,2)=max([boundary_temp_1_front boundary_temp_2_front boundary_temp_3_front boundary_temp_4_front],[],2);
    
    %% Relocate boundary and save it to vehicle (for DISPLAY_vehicle)
    %Move boundary back to axle and save it to value
    if body(2,3)==2
        vehicle.dimensions.p_gearbox_paral_boundary{wagontype}=boundary+(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1));
    else
        vehicle.dimensions.p_gearbox_coax_ls_boundary{wagontype}=boundary+(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1));
    end
    
elseif body(2,3)==4 %body is an arch (wheelhouse)
    
    % Correction of wheelhouse z-position (wheelhouse arch higher than radius (width) around axle middle point)
    body(1,3)=ceil(vehicle.dimensions.CZ.wheelhouse_r_height-0.5*vehicle.dimensions.CX.wheelhouse_r_length);
    
    % Initialize temp. back and front boundaries
    boundary_temp_back=NaN(size,2);
    boundary_temp_front=boundary_temp_back;
    
    for i=1:2
        % Presize z-coordinates
        int_a=body(2,1):1:2*body(2,1); 
        int_a=int_a';
        
        if i==1 %vertical part
            boundary_temp_back(1:body(1,3),1)=body(1,1)-body(2,1); %Calculate back boundary line (in x-direction)
            boundary_temp_front(1:body(1,3),1)=body(1,1)+body(2,1); %Calulate front boundary line (in x-direction)
        else % arch
            boundary_temp_back(body(1,3):body(1,3)+body(2,1),2)=-sqrt(body(2,1).^2-(abs(body(2,1)-int_a)).^2)+body(1,1); %Calculate back boundary line (in x-direction)
            boundary_temp_front(body(1,3):body(1,3)+body(2,1),2)=sqrt(body(2,1).^2-(abs(body(2,1)-int_a)).^2)+body(1,1); %Calculate front boundary line (in x-direction)
        end
        
    end
    
    boundary(:,1)=min(boundary_temp_back,[],2);
    boundary(:,2)=max(boundary_temp_front,[],2);   
     
elseif body(2,3)==5 %body is coaxial planetary or near-axle powertrain
    % Legend: 1=motor, 2=gear
    
    % Presize z-coordinates
    int_a1=0:1:2*body(2,1);
    int_a1=int_a1';
    int_a2=0:1:2*body(4,1);
    int_a2=int_a2';
    
    % Move Body relative to bumper front surface (position - half x-length), because bumper is reference for block formation
    body(3,1)=ceil(body(3,1)-(vehicle.dimensions.p_bumper{wagontype}(1,1)+0.5*vehicle.dimensions.p_bumper{wagontype}(2,1)));
    
    % Move Body with axle height into positive z-values to avoid negative indexing
    body(3,3)=ceil(body(3,3)+vehicle.dimensions.p_axle_height);
    
    % Initialize temp. back and front boundaries
    boundary_temp_1_back=NaN(size,2);
    boundary_temp_1_front=boundary_temp_1_back;
    boundary_temp_2_back=boundary_temp_1_back;
    boundary_temp_2_front=boundary_temp_1_back;
    
    boundary_temp_1_back(body(1,3)-body(2,1):body(1,3)+body(2,1),1)=-sqrt(body(2,1).^2-(abs(body(2,1)-int_a1)).^2)+body(1,1); %Calculate back boundary line (in x-direction)
    boundary_temp_1_front(body(1,3)-body(2,1):body(1,3)+body(2,1),2)=sqrt(body(2,1).^2-(abs(body(2,1)-int_a1)).^2)+body(1,1); %Calculate front boundary line (in x-direction)
    boundary_temp_2_back(body(3,3)-body(4,1):body(3,3)+body(4,1),1)=-sqrt(body(4,1).^2-(abs(body(4,1)-int_a2)).^2)+body(3,1); %Calculate back boundary line (in x-direction)
    boundary_temp_2_front(body(3,3)-body(4,1):body(3,3)+body(4,1),2)=sqrt(body(4,1).^2-(abs(body(4,1)-int_a2)).^2)+body(3,1); %Calculate front boundary line (in x-direction)
    boundary(:,1)=min([boundary_temp_1_back boundary_temp_2_back],[],2);
    boundary(:,2)=max([boundary_temp_1_front boundary_temp_2_front],[],2);
end

%% 3) Optional: Plot to check results
% Get all base workspace variables into this function.
%Plot_Layer_Calculate(body,boundary,boundary_temp_1_front,boundary_temp_1_back,boundary_temp_2_back,boundary_temp_2_front,...
%boundary_temp_3_back,boundary_temp_3_front,boundary_temp_4_back,boundary_temp_4_front,...
%int_a2_x,int_a2_z,int_a3_f_x,int_a3_f_z,int_a3_r_x,int_a3_r_z,int_a4_x,int_a4_z);

end
