function PlotInterior(v,axis)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 30.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function calculates and plots the positions of the seats and armrests depending on the number of seats per row and the seating layout
% ------------
% Sources:        [1] Felix Fahn, “Innenraummodellierung von autonomen Elektrofahrzeugen” Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2021
%                 [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Plotted seats of interior
% ------------

%% Implementation
% 1) Read in needed parameters
% 2) Calculate angles
% 3) Calculate x-values
% 4) Calculate y-values
%   a) Prealocate variables
%   b) Pre-Calculations
%   c)Calculation of seats
%   d) Calculation of armrests
%   e)Write values of front and rear row in separate functions
% 6) Plot seats, backrests and armrests
%   a)Plot seats
%   b)Plot backrest
%   c)Plot armrest

%% 1) Read in needed parameters
int_type            =   v.Input.int_type;                 % interior type (con,vav,btb,sr)
width               =   v.dimensions.GY.vehicle_width;    % vehicle width in mm
int_width(1)        =   v.interior.int_y_front;           % front interior width in mm
int_width(2)        =   v.interior.int_y_rear;            % rear interior width in mm
int_boundary_f_new  =   v.interior.int_boundary_f_new;    % initialize interior front boundary
int_boundary_r_new  =   v.interior.int_boundary_r_new;    % initialize interior rear boundary
n_seat              =   v.Input.n_seat;                   % number of seats
y_ar_s              =   v.Input.armrest_width_single;     % width of single armrest
y_ar_d              =   v.Input.armrest_width_double;     % width of double armrest
z_ar                =   50;                               % height of armrest (50 mm temp)
z_offset_ar         =   225;                              % Offset of armrest to seat surface
eu_def              =   [0;0;0];                          % Default angles of armrests and seat surface (no rotation)
int_height          =   v.Input.int_height;               % Interior height in mm
seat_depth          =   v.Input.seat_depth;               % Seat depth in mm
bound_seat          =   v.interior.boundary_seat;         % Boundary of seat
t_seat              =   v.Input.thickness_seat;           % Thickness of seat in mm
l_backrest          =   v.Input.length_br;                % Length of backrest in mm
adj_z               =   v.Input.seat_adjustment_z;        % Adjustment of seat in z
adj_x               =   v.Input.seat_adjustment_x;        % Adjustment of seat in x
t_br                =   v.Input.thickness_br;             % Thickness of backrest
a_br_max            =   v.Input.angle_br_max;             % Max. angle of backrest
a_br_min            =   v.Input.angle_br_min;             % Min. angle of backrest
y_wallgap           =   v.Input.wallgap;                  % Distance sidewall to seat/armrest
y_seatgap           =   v.Input.seatgap;                  % Distance between armrest(s) and seat
y_backrest          =   v.Input.backrest_width;           % Width of backrest
s_ar_out            =   v.Input.single_ar_outside;        % Switch for single armrest outside (yes/no)
s_ar_betw           =   v.Input.single_ar_between;        % Switch for single armrest between seats (yes/no)
d_ar_betw           =   v.Input.double_ar;                % Switch for double armrest between seat (yes/no)
int_3rows           =   v.Input.int_3rows;                % Switch for 3 rows (on/off)
if int_3rows
    int_3rows_type  =   v.Input.int_3rows_type;           % Type of three rows
    offset_3rows    =   v.interior.offset_3rows;          % Offset due to third rows
end


%Prealocate variables
armrest_size_f      =   zeros(6,3);                       % Size of front armrest single
armrest_size_r      =   zeros(6,3);                       % Size of rear armrest single


%Calculate starting point of interior (wall thickness)
y_wall              =   (width-int_width)/2;              % Distance between outer vehicle skin and interior

%Reduce n_seat for single row
if strcmp(int_type,'sr')
    n_seat=n_seat(1);
end

%% 2) Calculate angles
% Calculate angles of first position
if strcmp(int_type,'vav')
    eu_br(:,1)    =   [0; pi+2*pi/360*a_br_max(1) ;0];
    eu_br_a(:,1)  =   [0; pi+2*pi/360*a_br_min(1) ;0];
else
    eu_br(:,1)    =   [0; -2*pi/360*a_br_max(1);0];
    eu_br_a(:,1)  =   [0; -2*pi/360*a_br_min(1);0];
end

if strcmp(int_type,'btb')
    eu_br(:,2)    =   [0; pi+2*pi/360*a_br_max(2) ;0];
    eu_br_a(:,2)  =   [0; pi+2*pi/360*a_br_min(2) ;0];
elseif ~strcmp(int_type,'sr')
    eu_br(:,2)    =   [0; -2*pi/360*a_br_max(2);0];
    eu_br_a(:,2)  =   [0; -2*pi/360*a_br_min(2);0];
end

%% 3) Calculate x-values
for i=1:length(n_seat)   
    %Check if front or rear row
    if i==1
        %Prealocate variables
        seat_positions    =   zeros(n_seat(1),3);    % prealocate vector for seat position (3 for every seat)
        backrest_positions=   zeros(n_seat(1),3);    % prealocate vector for backrest position (3 for every seat)
        armrest_positions =   zeros(6,3);            % position of armrests (n_rows x n_max armrest)
    elseif i==2
        seat_positions    =   zeros(n_seat(2),3);    % prealocate vector for seat position (3 for every seat)
        backrest_positions=   zeros(n_seat(2),3);    % prealocate vector for backrest position (3 for every seat)
        armrest_positions =   zeros(6,3);            % position of armrests (n_rows x n_max armrest)
    end
    %Calculate x-values depending on seating layout
    for j=1:n_seat(i)
        if i==1
            if ~strcmp(int_type,'vav')%btb %con %sr
                seat_positions(j,1)=int_boundary_f_new(int_height + 1)-adj_x(i);
                backrest_positions(j,1)=int_boundary_f_new(int_height + 1) + seat_depth(i);
                armrest_positions(:,1)=seat_positions(j,1)+cosd(a_br_max(i))*z_offset_ar+adj_x(i);
            else%vav
                seat_positions(j,1)=int_boundary_f_new(int_height + 1)-seat_depth(i);
                backrest_positions(j,1)= seat_positions(j,1);
                armrest_positions(:,1)=seat_positions(j,1)-cosd(a_br_max(i))*z_offset_ar;
            end
            
        else
            if strcmp(int_type,'btb')%btb
                seat_positions(j,1)=int_boundary_r_new(int_height + 1)-seat_depth(i);
                backrest_positions(j,1)=int_boundary_r_new(int_height + 1) - seat_depth(i);
                armrest_positions(:,1)=seat_positions(j,1)-cosd(a_br_max(i))*z_offset_ar;
            elseif ~strcmp(int_type,'sr') %vav %con
                seat_positions(j,1)=int_boundary_r_new(int_height + 1)-adj_x(i);
                backrest_positions(j,1)=seat_positions(j,1) + seat_depth(i)+adj_x(i);
                armrest_positions(:,1)=seat_positions(j,1)+cosd(a_br_max(i))*z_offset_ar+adj_x(i);
            end
        end
    end
    %Write values of front and rear row in separate functions and armrest x-size
    if i==1
        seat_positions_f    =   seat_positions;    
        backrest_positions_f=   backrest_positions;
        armrest_positions_f =   armrest_positions;
        armrest_size_f(:,1)=    seat_depth(i);
    elseif i==2
        seat_positions_r    =   seat_positions;    
        backrest_positions_r=   backrest_positions;
        armrest_positions_r =   armrest_positions;
        armrest_size_r(:,1)=    seat_depth(i);
    end    
end
%% 4) Calculate y-values
%Differentiate between number of seats and armrest configurations
for i=1:length(n_seat)
    %a) Prealocate variables 
    if i==1
        seat_positions    =   zeros(n_seat(1),1);    % prealocate vector for seat position in y-direction        
        armrest_positions =   zeros(6,1);            % position of armrests (n_rows x n_max armrest)
        armrest_size      =   zeros(6,1);            % sizes of armrests (only y direction)
    elseif i==2
        seat_positions    =   zeros(n_seat(2),1);    % prealocate vector for seat position in y-direction        
        armrest_positions =   zeros(6,1);            % position of armrests (n_rows x n_max armrest)
        armrest_size      =   zeros(6,1);            % sizes of armrests (only y direction)
    end
    
    %b) Pre-Calculations
    %Special case only one seat per row    
    if n_seat(i)==1 %one seat per row
        s_ar_betw(i)=0; %No armrest between seats
        s_ar_betw(i)=0; %No armrest between seats
    end
    %Calculate outer length
    y1=y_wallgap(i)+s_ar_out(i)*y_ar_s(i); %distance between wall and outer seat
    %Calculate length of seatmodules
    if s_ar_betw(i)==1 %single armrests between seats
        y2=y_backrest(i)+2*y_ar_s(i)+y_seatgap(i);
    elseif d_ar_betw(i)==1 %double armrest between seats
        y2=y_backrest(i)+y_ar_d(i)+y_seatgap(i);
    else %No armrest between seats
        y2=y_backrest(i)+y_seatgap(i);
    end
    %Calculate final seat
    y3=y_backrest(i);
  
    %c)Calculation of seats
    for j=1:n_seat(i)
        %Position of seats
        seat_positions(j,1)=y_wall(i)+y1+y2*(j-1);        
    end
    
    %d) Calculation of armrests
    if n_seat(i)==1 %one seat per row
        s_ar_betw(i)=0; %No armrest between seats
        d_ar_betw(i)=0; %No armrest between seats
        %Position of armrest
        armrest_positions(1,1)=(y_wall(i)+y1-y_ar_s(i))*s_ar_out(i);
        armrest_positions(2,1)=(y_wall(i)+y1+y3)*s_ar_out(i);
        %Width of armrest (always single)
        armrest_size(:,1)=y_ar_s(i);
    elseif n_seat(i)==2 %two seats per row
        %Position of armrest
        armrest_positions(1,1)=(y_wall(i)+y1-y_ar_s(i))*s_ar_out(i);
        armrest_size(1,1)=y_ar_s(i);
        if s_ar_betw(i)==1 %single armrests between seats
            armrest_positions(2,1)=y_wall(i)+y1+y_backrest(i);
            armrest_positions(3,1)=armrest_positions(2,1)+y_ar_s(i)+y_seatgap(i);
            armrest_positions(4,1)=(y_wall(i)+y1+y2+y3)*s_ar_out(i);
            armrest_size(2:4,1)=y_ar_s(i);
        elseif d_ar_betw(i)==1 %double armrest between seats
            armrest_positions(2,1)=y_wall(i)+y1+y_backrest(i)+0.5*y_seatgap(i);
            armrest_size(2,1)=y_ar_d(i);
            armrest_positions(3,1)=(y_wall(i)+y1+y2+y3)*s_ar_out(i);
            armrest_size(3,1)=y_ar_s(i);
        else %no armrest between seats
            armrest_positions(2,1)=(y_wall(i)+y1+y2+y3)*s_ar_out(i);
            armrest_size(2,1)=y_ar_s(i);
        end
    elseif n_seat(i)==3 %three seats per row
        %Position of armrest
        armrest_positions(1,1)=(y_wall(i)+y1-y_ar_s(i))*s_ar_out(i);
        armrest_size(1,1)=y_ar_s(i);
        if s_ar_betw(i)==1 %single armrests between seats
            armrest_positions(2,1)=y_wall(i)+y1+y_backrest(i);
            armrest_positions(3,1)=armrest_positions(2,1)+y_ar_s(i)+y_seatgap(i);
            armrest_positions(4,1)=armrest_positions(2,1)+y2;
            armrest_positions(5,1)=armrest_positions(3,1)+y2;
            armrest_positions(6,1)=(y_wall(i)+y1+2*y2+y3)*s_ar_out(i);
            armrest_size(2:6,1)=y_ar_s(i);
        elseif d_ar_betw(i)==1 %double armrest between seats
            armrest_positions(2,1)=y_wall(i)+y1+y_backrest(i)+0.5*y_seatgap(i);
            armrest_positions(3,1)=armrest_positions(2,1)+y2;
            armrest_size(2:3,1)=y_ar_d(i);
            armrest_positions(4,1)=(y_wall(i)+y1+2*y2+y3)*s_ar_out(i);
            armrest_size(4,1)=y_ar_s(i);
        else %no armrest between seats
            armrest_positions(2,1)=(y_wall(i)+y1+2*y2+y3)*s_ar_out(i);
            armrest_size(2,1)=y_ar_s(i);
        end
    end
    
    
    %e)Write values of front and rear row in separate functions
    if i==1
        seat_positions_f(:,2)       =   seat_positions(:);
        backrest_positions_f(:,2)   =   seat_positions(:);
        armrest_positions_f(:,2)    =   armrest_positions(:);
        armrest_size_f(:,2)         =   armrest_size(:);
    elseif i==2
        seat_positions_r(:,2)       =   seat_positions(:);
        backrest_positions_r(:,2)   =   seat_positions(:);
        armrest_positions_r(:,2)    =   armrest_positions(:);
        armrest_size_r(:,2)         =   armrest_size(:);
    end
    
end


%% 5) Calculates z-values
for i=1:length(n_seat)
    %Position of seats
    if i==1
        seat_positions_f(:,3)       =   int_height+bound_seat(i)+adj_z(i);
        backrest_positions_f(:,3)   =   int_height+bound_seat(i)+adj_z(i)+t_seat(i);
        
    elseif i==2
        seat_positions_r(:,3)       =   int_height+bound_seat(i)+adj_z(i);
        backrest_positions_r(:,3)   =   int_height+bound_seat(i)+adj_z(i)+t_seat(i);
    end
    
    %Position of armrests
    if i==1
    armrest_positions_f(:,3)     =   seat_positions_f(1,3)+t_seat(i)+z_offset_ar*sind(a_br_max(i)); %Position of armrest in z-direction
    elseif i==2
    armrest_positions_r(:,3)     =   seat_positions_r(1,3)+t_seat(i)+z_offset_ar*sind(a_br_max(i)); %Position of armrest in z-direction
    end        
end
%Assign z-length of armrest
armrest_size_f(:,3) =   z_ar; %Height of armrest
armrest_size_r(:,3) =   z_ar; %Height of armrest

%% 6) Plot seats, backrests and armrests
%Optional for three rows: Calculate x value for mirror plane 
if int_3rows==1
    if strcmp(int_3rows_type,'3vavcon')
        x_mirror        =   min(int_boundary_f_new)+offset_3rows;
    else
        x_mirror        =   max(int_boundary_r_new)-offset_3rows;
    end
end

%a)Plot seats
for i=1:length(n_seat)
    for j=1:n_seat(i)
        Lx=seat_depth(i)+adj_x(i);
        Ly=y_backrest(i);
        Lz=t_seat(i);
        eu=eu_def;
        if i==1
            offset=seat_positions_f(j,:)';
        elseif i==2
            offset=seat_positions_r(j,:)';
        end
        alpha=0;
        plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
        
        %Optional: 3rows
        if int_3rows==1
            switch int_3rows_type
                case'3con'
                    if i==2
                        offset(1)=offset(1)-offset_3rows;
                        plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
                    end
                case'3convav'
                    if i==2
                        plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror) 
                    end
                case'3vavcon'
                    if i==1
                        plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror)
                    end
            end
        end
    end
end

%b)Plot backrest
for i=1:length(n_seat)
    for j=1:n_seat(i)      
        Lx=l_backrest(i);
        Ly=y_backrest(i);
        Lz=t_br(i);
        eu=eu_br(:,i);
        if i==1
            offset=backrest_positions_f(j,:)';
            if ~strcmp(int_type,'vav') %if seat is faced in driving direction
                Lz=-Lz;
            end
        elseif i==2
            offset=backrest_positions_r(j,:)';
            if ~strcmp(int_type,'btb')
                Lz=-Lz;
            end
        end
        alpha=0;
        plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
        eu=eu_br_a(:,i);
        alpha=1;
        plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)               
        
        %Optional: 3rows
        if int_3rows==1
            switch int_3rows_type
                case'3con'
                    if i==2
                        offset(1)=offset(1)-offset_3rows;
                        plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
                        eu=eu_br(:,i);
                        alpha=0;
                        plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
                    end
                case'3convav'
                    if i==2
                        plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror) 
                        eu=eu_br(:,i);
                        alpha=0;
                        plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror) 
                    end
                case'3vavcon'
                    if i==1
                        plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror)
                        eu=eu_br(:,i);
                        alpha=0;
                        plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror)
                    end
            end
        end
    end
end
    
%c) Plot armrest
eu=eu_def;
alpha=0;
for i=1:length(n_seat)
    for j=1:length(armrest_positions)
        if i==1
            offset=armrest_positions_f(j,:)';
            Lx=armrest_size_f(j,1);
            Ly=armrest_size_f(j,2);
            Lz=armrest_size_f(j,3);
        elseif i==2
            offset=armrest_positions_r(j,:)';
            Lx=armrest_size_r(j,1);
            Ly=armrest_size_r(j,2);
            Lz=armrest_size_r(j,3);
        end
        if ~offset(2)==0 %y-value is 0 if no armrest is selected
            plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)

            %Optional: 3rows
            if int_3rows==1
                switch int_3rows_type
                    case'3con'
                        if i==2
                            offset(1)=offset(1)-offset_3rows;
                            plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
                        end
                    case'3convav'
                        if i==2
                            plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror) 
                        end
                    case'3vavcon'
                        if i==1
                            plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror) 
                        end
                end
            end
        end
    end
end


    function plot_rectangle(axis,offset,eu,Lx,Ly,Lz,alpha)
        %% Description: Plot square with given inputs
        [F_b,V_b]=Object3d_Block(offset,eu,Lx,Ly,Lz);        
        if alpha==1
            patch(axis,'Faces', F_b, 'Vertices', V_b, 'FaceColor', [96 96 86]./255,'FaceAlpha',0.2);
        else
            patch(axis,'Faces', F_b, 'Vertices', V_b, 'FaceColor', [96 96 86]./255);
        end
    end

    function plot_rectangle_mirror(axis,offset,eu,Lx,Ly,Lz,alpha,x_mirror)
        %% Description: Plot square for 3 row mirrored
        %Calculate original points
        [F_b,V_b]=Object3d_Block(offset,eu,Lx,Ly,Lz);        
        %Move values to x=0 and mirror them
        V_b(:,1)=2*x_mirror-V_b(:,1);        
        if alpha==1
            patch(axis,'Faces', F_b, 'Vertices', V_b, 'FaceColor', [96 96 86]./255,'FaceAlpha',0.2);
        else
            patch(axis,'Faces', F_b, 'Vertices', V_b, 'FaceColor', [96 96 86]./255);
        end
    end

end
