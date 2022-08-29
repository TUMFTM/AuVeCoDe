function PlotWheelhouse(axis,v,wheel_envelope)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots wheelhouses including wheels(tire+rim) and (optional) the wheel envelopes
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - axis: figure where vehicle is plotted
%           - wheel_envelope: 1= plot, 2= no plot
% ------------
% Output:   - Plotted wheelhouses and wheels (incl. optional wheel envelopes)
% ------------

%% Implementation
% 1) Calculate wheelhouse surface
% 2) Plot Wheelhouse and Wheels (tire and rim)

%% 1) Calculate wheelhouse surface
for wagontype=1:2 %Front and rear wheelhouses
    %Read wheelhouse dimensions
    if wagontype==1 %front wheelhouse
        r_wh = floor(v.dimensions.wheelhouse_width(1)/2); %Radius wheelhouse front
        r_cor=v.dimensions.CZ.wheelhouse_r_height-2*r_wh; %Correction factor if wheelhouse height is smaller/bigger then width (wheelhouse is not a perfect circle due to high/low deflection)
        wheelhouse2bumper=v.Input.dist_wheelhouse2bumper_f; %Read in front wheelhouse2bumper
    else %rear wheelhouse
        r_wh = floor(v.dimensions.wheelhouse_width(2)/2); %Radius wheelhouse rear
        r_cor=v.dimensions.CZ.wheelhouse_r_height-2*r_wh; %Correction factor if wheelhouse height is smaller/bigger then width (wheelhouse is not a perfect circle due to high/low deflection)
        wheelhouse2bumper=v.Input.dist_wheelhouse2bumper_r; %Read in rear wheelhouse2bumper
    end    
    h_wh = floor(v.dimensions.wheelhouse_length(wagontype)); %width wheelhouses    
    r_w = v.dimensions.CX.wheel_f_diameter/2; %Radius wheel
    h_w = v.dimensions.CY.wheel_f_width; %width wheel
    
    % Calculate surface of wheelhouse
    i_x=0:1:2*(round(wheelhouse2bumper)+r_wh); % wheelhouse x-coordinates
    wheelhouse=zeros(2*(round(wheelhouse2bumper)+r_wh),2); % preallocate wheelhouse
    
    wheelhouse(i_x<=round(wheelhouse2bumper),:)=v.dimensions.GZ.H156; % start of wheelhouse = underbody height
    
    for i_x2=2:r_wh % front part of arch (until sym point of wheel)
        wheelhouse(i_x2+round(wheelhouse2bumper),:)=sqrt(r_wh.^2-(abs(r_wh-i_x2)).^2)+r_wh +r_cor;
    end
    
    % rear part (== symmetry of front part)
    wheelhouse((r_wh+round(wheelhouse2bumper)+1):(2*r_wh+2*round(wheelhouse2bumper)),:)=wheelhouse((r_wh+round(wheelhouse2bumper)):-1:1,:);
    
    
    % --------------------------- b) Wheels and Rims -----------------------------------
    %Wheels
    p_w=zeros(3,2); % preallocate wheels
    sc_w = 80; %Sidecount
    
    if wagontype==1 % front wheels
        p_w(:,1) = [0 h_w r_w]; %Position Offset from Origin (x y z)
        p_rw(:,1) = [0 0.5*h_w r_w]; %Position Offset from Origin (x y z)
        p_w(:,2) = [0 v.dimensions.GY.vehicle_width r_w]; %Position Offset from Origin (x y z)
        p_rw(:,2) = [0 v.dimensions.GY.vehicle_width-0.5*h_w r_w]; %Position Offset from Origin (x y z)
    else % rear wheels
        p_w(:,1) = [v.dimensions.GX.wheelbase h_w r_w]; %Position Offset from Origin (x y z)
        p_rw(:,1) = [v.dimensions.GX.wheelbase 0.5*h_w r_w]; %Position Offset from Origin (x y z)
        p_w(:,2) = [v.dimensions.GX.wheelbase v.dimensions.GY.vehicle_width r_w]; %Position Offset from Origin (x y z)
        p_rw(:,2) = [v.dimensions.GX.wheelbase v.dimensions.GY.vehicle_width-0.5*h_w r_w]; %Position Offset from Origin (x y z)
    end
    eu_w=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
    
    %Rims
    r_rim = v.dimensions.CX.rim_f_diameter/2*25.4; %Radius wheel in mm (inch to mm)
    h_rim = v.dimensions.CY.wheel_f_width; %width wheel
    
    p_rim=zeros(3,2); % preallocate wheels
    sc_rim = 80; %Sidecount (level of detail of cylinder plot)
    
    if wagontype==1 % front wheels
        p_rim(:,1) = [0 0 r_w]; %Position Offset from Origin (x y z)
        p_rim(:,2) = [0 v.dimensions.GY.vehicle_width r_w]; %Position Offset from Origin (x y z)
    else % rear wheels
        p_rim(:,1) = [v.dimensions.GX.wheelbase 0 r_w]; %Position Offset from Origin (x y z)
        p_rim(:,2) = [v.dimensions.GX.wheelbase v.dimensions.GY.vehicle_width r_w]; %Position Offset from Origin (x y z)
    end
    eu_rim=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
    
    %Rotated wheels due to steering angle
    %Read steering angles
    a_steering_f = deg2rad(v.wheels.steering_angle_f); %Steering angle front
    a_steering_r = deg2rad(v.wheels.steering_angle_r); %Steering angle rear
    %Assign roation matrix
    eu_rw         = zeros(2,3);
    eu_rw_f(1,:)  = [pi/2; a_steering_f; 0]; %Rotation Matrix for Euler Angles
    eu_rw_f(2,:)  = [pi/2; a_steering_f; 0]; %Rotation Matrix for Euler Angles
    eu_rw_r(1,:)  = [pi/2; -a_steering_r; 0]; %Rotation Matrix for Euler Angles
    eu_rw_r(2,:)  = [pi/2; -a_steering_r; 0]; %Rotation Matrix for Euler Angles
    
    %% 2) Plot Wheelhouse and wheels
    %load stl file of rim
    load('rim.mat','rim')
    %read measurements of stl to scale it correctly
    d_ori=max(rim.points(:,1))-min(rim.points(:,1)); %stl diameter
    d_ratio=(2*r_rim)/d_ori; %ratio between actual rim diameter and stl diameter
    h_ori=max(rim.points(:,2))-min(rim.points(:,2)); %stl height
    h_ratio=h_rim/h_ori; %ratio between actual rim width and stl width
    
    rim.points(:,1)=rim.points(:,1)*d_ratio; %resize rim in x-direction
    rim.points(:,3)=rim.points(:,3)*d_ratio; %resize rim in z-direction
    rim.points(:,2)=rim.points(:,2)*h_ratio; %resize rim in y-direction
    
    
    
    %Two plots per axle
    for n=1:2 %n: number of wheels per axle
        % Plot wheel
        [F_w,F_w_b,V_w]=Object3d_Cylinder_hollow(p_w(:,n),eu_w,r_w,r_rim,h_w,sc_w); %create face and vertices of one wheel
        patch(axis,'Faces', F_w, 'Vertices', V_w, 'FaceColor', 'k'); %use patch to plot cylinder surface (round surface)
        patch(axis,'Faces', F_w_b, 'Vertices', V_w, 'FaceColor', 'k'); %use patch to plot cylinder surface (flat surface)
        
        
        %Plot rims
        rimcolor=[100, 100, 100]./255;
        rim_plot=rim;
        rim_plot.points=rim_plot.points(1:end,:)+p_rim(:,n)';
        if n==1
            rim_plot.points(:,2)=rim_plot.points(:,2)*-1;
        end
        RIM=triangulation(rim_plot.cv,rim_plot.points);
        trisurf(RIM,'FaceColor',rimcolor);
        
        
        if wagontype==1
            x=-(round(v.Input.dist_wheelhouse2bumper_f)+r_wh-1):5:(round(v.Input.dist_wheelhouse2bumper_f)+r_wh); % x-vector (- 2*r until 2*r) ((...)-1 because vector starts with x(1) and not x(0)
            X=[x;x];
            if n==1 % left front wheelhouse
                Y=[zeros(length(X),1)';h_wh*ones(length(X),1)']; % y-vector (0 until width of wheelhouse)
            else % right front wheelhouse
                Y=[(v.dimensions.GY.vehicle_width-h_wh)*ones(length(X),1)';v.dimensions.GY.vehicle_width*ones(length(X),1)']; % y-vector (v width - width until v width)
            end
            Z=wheelhouse(1:5:end,:)';
            surface(axis,X,Y,Z,'FaceAlpha',0.7,'FaceColor',[128 128 128]./255,'EdgeColor','none');
            
            %Plot wheel envelope 
            if wheel_envelope==1
                % Plot steered wheel
                [F_rw,F_rw_b,V_rw]=Object3d_Cylinder_Middle(p_rw(:,n),eu_rw_f(n,:),r_w,h_w,sc_w); %create face and vertices of one wheel
                patch(axis,'Faces', F_rw, 'Vertices', V_rw, 'FaceColor', 'k','FaceAlpha',0.3,'EdgeColor','none'); %use patch to plot cylinder surface (round surface)
                patch(axis,'Faces', F_rw_b, 'Vertices', V_rw, 'FaceColor', 'k','FaceAlpha',0.3); %use patch to plot cylinder surface (flat surface)
            end
        else
            x=v.dimensions.GX.wheelbase-(round(v.Input.dist_wheelhouse2bumper_r)+r_wh-1):5:v.dimensions.GX.wheelbase+round(v.Input.dist_wheelhouse2bumper_r)+r_wh; % x-vector (- 2*r + wheelbase until 2*r + wheelbase) ((...)-1 because vector starts with x(1) and not x(0)
            X=[x;x];
            if n==1 % left rear wheelhouse
                Y=[zeros(length(X),1)';h_wh*ones(length(X),1)']; % y-vector (0 until width of wheelhouse)
            else% right rear wheelhouse
                Y=[(v.dimensions.GY.vehicle_width-h_wh)*ones(length(X),1)';v.dimensions.GY.vehicle_width*ones(length(X),1)'];
            end
            Z=wheelhouse(1:5:end,:)';
            surface(axis,X,Y,Z,'FaceAlpha',0.7,'FaceColor',[128 128 128]./255,'EdgeColor','none');
            
            
        % Plot wheel envelope 
        if wheel_envelope==1
            % Plot steered wheel
            [F_rw,F_rw_b,V_rw]=Object3d_Cylinder_Middle(p_rw(:,n),eu_rw_r(n,:),r_w,h_w,sc_w); %create face and vertices of one wheel
            patch(axis,'Faces', F_rw, 'Vertices', V_rw, 'FaceColor', 'k','FaceAlpha',0.3,'EdgeColor','none'); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_rw_b, 'Vertices', V_rw, 'FaceColor', 'k','FaceAlpha',0.3); %use patch to plot cylinder surface (flat surface)
        end
        end
    end
end
end



