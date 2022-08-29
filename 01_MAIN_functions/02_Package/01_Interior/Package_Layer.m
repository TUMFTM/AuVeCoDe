function [vehicle, Boundary_max] = Package_Layer(vehicle,p_bodies,psi,wagontype,block_building)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow
%-------------
% Created on: 01.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Package_Layer calculates the layers, two for each component (max. and min. value in y-direction)
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - p_bodies: bodies used for calculation
%           - vehicle: struct including the data of the vehicle
%           - Psi: Angle of Motor to Gear in Degree
%           - Wagon type (1=front, 2=rear)
%           - block_building: body is blockbuilding or not (0=no, 1=yes)
% ------------
% Output:   - vehicle: struct including the data of the vehicle
% ------------
%% Implementation
% 1) Initialize Bodies
% 2) Calculate Boundary Lines
% 3) Calculate Block measurements
% 4) Move Objects to achieve free crash length

%% 1) Initialize bodies
% Free Crash Length
crash_length=vehicle.dimensions.p_crash_length(wagontype); %Free Crash, which has to be achieved

% size (total interior height + interior height)
size=ceil(vehicle.interior.int_z_tot(wagontype)+vehicle.Input.int_height);

% Prelocate boundary lines
Boundary_Front=zeros(size,length(p_bodies));
Boundary_Back=zeros(size,length(p_bodies));

%% 2) Calculate boundary lines
for i=1:length(p_bodies)
    [boundary,vehicle] = Package_Layer_Calculate(vehicle,wagontype,eval(p_bodies(i)),size,psi); %Calculate boundary surfaces
     
    Boundary_Back(:,i)=boundary(:,1);
    Boundary_Front(:,i)=boundary(:,2);
end

%% 3) Calculate block measurements
% Calculate sum of blocked length due to objects
length_block_obj=nansum((Boundary_Front-Boundary_Back),2);

% Calculate if free crash length is achieved
crash_length=max(Boundary_Front,[],2)-length_block_obj-crash_length;
crash_length=abs((crash_length<0).*crash_length);

%% 4) Move objects to achieve free crash length
for i=1:length(p_bodies)
    ind_nan=find(~isnan(Boundary_Back(:,i)));   
    x_bottom=min(ind_nan);
    x_top=max(ind_nan);  
    
    if block_building
        Boundary_Front(x_bottom:x_top,i)=Boundary_Front(x_bottom:x_top,i)+max(crash_length(x_bottom+1:x_top-1)); %x_bottom and x_top reduced with 1 to avoid jumps at edges 
    end
end

Boundary_max=max(Boundary_Front,[],2);
Boundary_max(isnan(Boundary_max))=vehicle.dimensions.p_crash_length(wagontype); % set NaN values to crash-length

end