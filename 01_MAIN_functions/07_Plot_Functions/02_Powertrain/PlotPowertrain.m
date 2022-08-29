function PlotPowertrain(axis,v)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function plots the powertrain including gearbox motor, power electronics and drive shafts for both front and rear axle
% ------------
% Sources:      [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle: struct with includes the data of the vehicle
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Powertrain
% ------------


%% 1) Define Colors and read in paramters
C1 = [0, 101, 189]./255;            % Wheels of the first stage (blue)
C2 = [100, 160, 200]./255;          % Wheels of the second stage (light blue)
CD = [204,204,204]./255;            % Differential cage (light grey)
CB = [10, 10, 10]./255;             % Bearings (black)
CS = [51,51,51]./255;               % Shafts (dark grey)
r_w = v.dimensions.CX.wheel_f_diameter/2; %Radius wheel

%% 2) Calculate dimensions and positions and direct plot of gearbox
for wagontype=1:2 %Front and rear powertrain
%Motor, gearbox and powerelectronics
    if v.LDS.settings.filled_axles(wagontype) % ignore motor/gear if not driven
        if v.e_machine{wagontype}.quantity==1 % single motor topology
            if (wagontype==1 && strcmp(v.Input.gear_type_f,'Parallel')) || (wagontype==2 && strcmp(v.Input.gear_type_r,'Parallel'))
                %% Parallel gears
                % Sizing of wheels (parallel gearbox) 
                r_g1 = v.dimensions.p_gear_paral{wagontype}(2,1); %Radius wheel 1
                h_g1 = v.dimensions.p_gear_paral{wagontype}(2,2); %Height wheel 1
                r_g2 = v.dimensions.p_gear_paral{wagontype}(4,1); %Radius wheel 2
                h_g2 = v.dimensions.p_gear_paral{wagontype}(4,2); %Height wheel 2
                r_g3 = v.dimensions.p_gear_paral{wagontype}(6,1); %Radius wheel 3
                h_g3 = v.dimensions.p_gear_paral{wagontype}(6,2); %Height wheel 3
                r_g4 = v.dimensions.p_gear_paral{wagontype}(8,1); %Radius wheel 4
                h_g4 = v.dimensions.p_gear_paral{wagontype}(8,2); %Height wheel 4
                sc_g = 40; %Sidecount
                
                % Sizing of gearbox auxiliary components (parallel gearbox --> Bearings, Differential, ...)
                r_aux = zeros(size(v.dimensions.p_gear_paral_aux{wagontype},1)/2,1);
                h_aux = zeros(size(v.dimensions.p_gear_paral_aux{wagontype},1)/2,1);
                for i = 1:size(v.dimensions.p_gear_paral_aux{wagontype},1)/2
                    r_aux(i) = v.dimensions.p_gear_paral_aux{wagontype}(2*i,1); %Radius of auxiliary components
                    h_aux(i) = v.dimensions.p_gear_paral_aux{wagontype}(2*i,2); %Height of auxiliary components
                end                
                
                % Housing
                % 1) Close surface from the sides
                % front part
                X=[v.dimensions.p_gearbox_paral_boundary{wagontype}(:,1)';v.dimensions.p_gearbox_paral_boundary{wagontype}(:,1)'];
                Y=[(v.dimensions.p_gear_paral{wagontype}(9,2)-0.5*v.dimensions.p_gear_paral{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_paral_boundary{wagontype}(:,1)));...
                    (v.dimensions.p_gear_paral{wagontype}(9,2)+0.5*v.dimensions.p_gear_paral{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_paral_boundary{wagontype}(:,1)))]; % y-vector (0 until width of wheelhouse)
                z_f=1:1:length(v.dimensions.p_gearbox_paral_boundary{wagontype}(:,1));
                Z=[z_f;z_f];
                if wagontype==2
                    X=v.dimensions.GX.wheelbase-X;
                    X=flip(X);
                end
                surface(axis,X,Y,Z,'FaceAlpha',0.7,'FaceColor',[60 60 60]./255,'EdgeColor','none');
                % Vertices for front part
                V_house_f_1=[X(1,:)' Y(1,:)' Z(1,:)']; %first side of housing
                V_house_f_2=[X(2,:)' Y(2,:)' Z(2,:)']; %second side of housing
                V_house_f_1(any(isnan(V_house_f_1),2),:) = []; %filter nan out of V_house
                V_house_f_2(any(isnan(V_house_f_2),2),:) = []; %filter nan out of V_house
                
                % rear part
                X=[v.dimensions.p_gearbox_paral_boundary{wagontype}(:,2)';v.dimensions.p_gearbox_paral_boundary{wagontype}(:,2)'];
                Y=[(v.dimensions.p_gear_paral{wagontype}(9,2)-0.5*v.dimensions.p_gear_paral{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_paral_boundary{wagontype}(:,2)));...
                    (v.dimensions.p_gear_paral{wagontype}(9,2)+0.5*v.dimensions.p_gear_paral{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_paral_boundary{wagontype}(:,2)))]; % y-vector (0 until width of wheelhouse)
                z_f=1:1:length(v.dimensions.p_gearbox_paral_boundary{wagontype}(:,2));
                Z=[z_f;z_f];
                if wagontype==2
                    X=v.dimensions.GX.wheelbase-X;
                    X=flip(X);
                end
                surface(axis,X,Y,Z,'FaceAlpha',0.7,'FaceColor',[60 60 60]./255,'EdgeColor','none');
                % Vertices for rear part
                V_house_r_1=[X(1,:)' Y(1,:)' Z(1,:)']; %first side of housing
                V_house_r_2=[X(2,:)' Y(2,:)' Z(2,:)']; %second side of housing
                V_house_r_1(any(isnan(V_house_r_1),2),:) = []; %filter nan out of V_house
                V_house_r_2(any(isnan(V_house_r_2),2),:) = []; %filter nan out of V_house
                
                                
                % 2) Close surface from the sides
                % Vertices
                V_house_1=[V_house_f_1;flip(V_house_r_1)];
                V_house_2=[V_house_f_2;flip(V_house_r_2)];
                % Face
                F_house_1=1:1:length(V_house_1);
                F_house_2=1:1:length(V_house_2);
                patch(axis,'Faces', F_house_1, 'Vertices', V_house_1,'FaceAlpha',0.7, 'FaceColor', [60 60 60]./255); %use patch to plot trunk
                patch(axis,'Faces', F_house_2, 'Vertices', V_house_2,'FaceAlpha',0.7, 'FaceColor', [60 60 60]./255); %use patch to plot trunk
                
                %% Sizing of motor
                r_mot = v.dimensions.p_motor_paral{wagontype}(2,1); %Radius
                h_mot = v.dimensions.p_motor_paral{wagontype}(2,2); %Height
                sc_mot = 40; %Sidecount
                
                %% Positioning of motor (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
                % adjustment of position due to plot functions (see sources) and wheelstart at z=0
                p_mot=[v.dimensions.p_motor_paral{wagontype}(1,1); v.dimensions.p_motor_paral{wagontype}(1,2)+0.5*h_mot; v.dimensions.p_motor_paral{wagontype}(1,3)+r_w];
                eu_mot=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
                
                %% Positioning of wheels (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
                % adjustment of position due to plot functions (see sources) and wheelstart at z=0
                p_g1=[v.dimensions.p_gear_paral{wagontype}(1,1); v.dimensions.p_gear_paral{wagontype}(1,2)+0.5*h_g1; v.dimensions.p_gear_paral{wagontype}(1,3)+r_w]; % first wheel
                p_g2=[v.dimensions.p_gear_paral{wagontype}(3,1); v.dimensions.p_gear_paral{wagontype}(3,2)+0.5*h_g2; v.dimensions.p_gear_paral{wagontype}(3,3)+r_w]; % second wheel
                p_g3=[v.dimensions.p_gear_paral{wagontype}(5,1); v.dimensions.p_gear_paral{wagontype}(5,2)+0.5*h_g3; v.dimensions.p_gear_paral{wagontype}(5,3)+r_w]; % third wheel
                p_g4=[v.dimensions.p_gear_paral{wagontype}(7,1); v.dimensions.p_gear_paral{wagontype}(7,2)+0.5*h_g4; v.dimensions.p_gear_paral{wagontype}(7,3)+r_w]; % fourth wheel
                eu_g=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
                
                %% Positioning of auxiliary components -- copied from above
                p_aux = zeros(3,size(v.dimensions.p_gear_paral_aux{wagontype},1)/2);
                for i = 1:size(v.dimensions.p_gear_paral_aux{wagontype},1)/2
                    x_aux = v.dimensions.p_gear_paral_aux{wagontype}(2*i-1,1);
                    y_aux = v.dimensions.p_gear_paral_aux{wagontype}(2*i-1,2) + 0.5 * h_aux(i);
                    z_aux = v.dimensions.p_gear_paral_aux{wagontype}(2*i-1,3) + r_w;
                    p_aux(:,i) = [x_aux;y_aux;z_aux];
                end
                
            elseif (wagontype==1 && strcmp(v.Input.gear_type_f,'Coaxial')) || (wagontype==2 && strcmp(v.Input.gear_type_r,'Coaxial'))
                %% Coaxial configuration
                % Sizing of gearbox
                r_g = v.dimensions.p_gear_coax{wagontype}(2,1); %Radius
                h_g = v.dimensions.p_gear_coax{wagontype}(2,2); %Height
                sc_g = 40; %Sidecount
                
                %% Sizing of motor
                r_mot = v.dimensions.p_motor_coax{wagontype}(2,1); %Radius
                h_mot = v.dimensions.p_motor_coax{wagontype}(2,2); %Height
                sc_mot = 40; %Sidecount
                
                %% Positioning of motor (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
                % adjustment of position due to plot functions (see sources) and wheelstart at z=0
                p_mot=[v.dimensions.p_motor_coax{wagontype}(1,1); v.dimensions.p_motor_coax{wagontype}(1,2)+0.5*h_mot; v.dimensions.p_motor_coax{wagontype}(1,3)+r_w];
                eu_mot=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
                
                %% Positioning of gearbox (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
                % adjustment of position due to plot functions (see sources) and wheelstart at z=0
                p_g=[v.dimensions.p_gear_coax{wagontype}(1,1); v.dimensions.p_gear_coax{wagontype}(1,2)+0.5*h_g; v.dimensions.p_gear_coax{wagontype}(1,3)+r_w];
                eu_g=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
            elseif (wagontype==1 && strcmp(v.Input.gear_type_f,'Coaxial-layshaft')) || (wagontype==2 && strcmp(v.Input.gear_type_r,'Coaxial-layshaft'))
                %% Coaxial layshaft gears
                % Sizing of wheels (Coaxial layshaft gearbox) 
                r_g1 = v.dimensions.p_gear_coax_ls{wagontype}(2,1); %Radius wheel 1
                h_g1 = v.dimensions.p_gear_coax_ls{wagontype}(2,2); %Height wheel 1
                r_g2 = v.dimensions.p_gear_coax_ls{wagontype}(4,1); %Radius wheel 2
                h_g2 = v.dimensions.p_gear_coax_ls{wagontype}(4,2); %Height wheel 2
                r_g3 = v.dimensions.p_gear_coax_ls{wagontype}(6,1); %Radius wheel 3
                h_g3 = v.dimensions.p_gear_coax_ls{wagontype}(6,2); %Height wheel 3
                r_g4 = v.dimensions.p_gear_coax_ls{wagontype}(8,1); %Radius wheel 4
                h_g4 = v.dimensions.p_gear_coax_ls{wagontype}(8,2); %Height wheel 4
                sc_g = 40; %Sidecount
                
                % Sizing of gearbox auxiliary components (coaxial layshaft gearbox --> Bearings, Differential, ...)
                r_aux = zeros(size(v.dimensions.p_gear_coax_ls_aux{wagontype},1)/2,1);
                h_aux = zeros(size(v.dimensions.p_gear_coax_ls_aux{wagontype},1)/2,1);
                for i = 1:size(v.dimensions.p_gear_coax_ls_aux{wagontype},1)/2
                    r_aux(i) = v.dimensions.p_gear_coax_ls_aux{wagontype}(2*i,1); %Radius of auxiliary components
                    h_aux(i) = v.dimensions.p_gear_coax_ls_aux{wagontype}(2*i,2); %Height of auxiliary components
                end
                
                % Housing
                % 1) Close surface from the sides
                % front part
                X=[v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,1)';v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,1)'];
                Y=[(v.dimensions.p_gear_coax_ls{wagontype}(9,2)-0.5*v.dimensions.p_gear_coax_ls{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,1)));...
                    (v.dimensions.p_gear_coax_ls{wagontype}(9,2)+0.5*v.dimensions.p_gear_coax_ls{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,1)))]; % y-vector (0 until width of wheelhouse)
                z_f=1:1:length(v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,1));
                Z=[z_f;z_f];
                if wagontype==2
                    X=v.dimensions.GX.wheelbase-X;
                    X=flip(X);
                end
                surface(axis,X,Y,Z,'FaceAlpha',0.7,'FaceColor',[60 60 60]./255,'EdgeColor','none');
                % Vertices for front part
                V_house_f_1=[X(1,:)' Y(1,:)' Z(1,:)']; %first side of housing
                V_house_f_2=[X(2,:)' Y(2,:)' Z(2,:)']; %second side of housing
                V_house_f_1(any(isnan(V_house_f_1),2),:) = []; %filter nan out of V_house
                V_house_f_2(any(isnan(V_house_f_2),2),:) = []; %filter nan out of V_house
                
                % rear part
                X=[v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,2)';v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,2)'];
                Y=[(v.dimensions.p_gear_coax_ls{wagontype}(9,2)-0.5*v.dimensions.p_gear_coax_ls{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,2)));...
                    (v.dimensions.p_gear_coax_ls{wagontype}(9,2)+0.5*v.dimensions.p_gear_coax_ls{wagontype}(10,2))*ones(1,length(v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,2)))]; % y-vector (0 until width of wheelhouse)
                z_f=1:1:length(v.dimensions.p_gearbox_coax_ls_boundary{wagontype}(:,2));
                Z=[z_f;z_f];
                if wagontype==2
                    X=v.dimensions.GX.wheelbase-X;
                    X=flip(X);
                end
                surface(axis,X,Y,Z,'FaceAlpha',0.7,'FaceColor',[60 60 60]./255,'EdgeColor','none');
                % Vertices for rear part
                V_house_r_1=[X(1,:)' Y(1,:)' Z(1,:)']; %first side of housing
                V_house_r_2=[X(2,:)' Y(2,:)' Z(2,:)']; %second side of housing
                V_house_r_1(any(isnan(V_house_r_1),2),:) = []; %filter nan out of V_house
                V_house_r_2(any(isnan(V_house_r_2),2),:) = []; %filter nan out of V_house
                
                                
                % 2) Close surface from the sides
                % Vertices
                V_house_1=[V_house_f_1;flip(V_house_r_1)];
                V_house_2=[V_house_f_2;flip(V_house_r_2)];
                % Face
                F_house_1=1:1:length(V_house_1);
                F_house_2=1:1:length(V_house_2);
                patch(axis,'Faces', F_house_1, 'Vertices', V_house_1,'FaceAlpha',0.7, 'FaceColor', [60 60 60]./255); %use patch to plot trunk
                patch(axis,'Faces', F_house_2, 'Vertices', V_house_2,'FaceAlpha',0.7, 'FaceColor', [60 60 60]./255); %use patch to plot trunk
                
                %% Sizing of motor
                r_mot = v.dimensions.p_motor_coax{wagontype}(2,1); %Radius
                h_mot = v.dimensions.p_motor_coax{wagontype}(2,2); %Height
                sc_mot = 40; %Sidecount
                
                %% Positioning of motor (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
                % adjustment of position due to plot functions (see sources) and wheelstart at z=0
                p_mot=[v.dimensions.p_motor_coax{wagontype}(1,1); v.dimensions.p_motor_coax{wagontype}(1,2)+0.5*h_mot; v.dimensions.p_motor_coax{wagontype}(1,3)+r_w];
                eu_mot=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
                
                %% Positioning of wheels (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
                % adjustment of position due to plot functions (see sources) and wheelstart at z=0
                p_g1=[v.dimensions.p_gear_coax_ls{wagontype}(1,1); v.dimensions.p_gear_coax_ls{wagontype}(1,2)+0.5*h_g1; v.dimensions.p_gear_coax_ls{wagontype}(1,3)+r_w]; % first wheel
                p_g2=[v.dimensions.p_gear_coax_ls{wagontype}(3,1); v.dimensions.p_gear_coax_ls{wagontype}(3,2)+0.5*h_g2; v.dimensions.p_gear_coax_ls{wagontype}(3,3)+r_w]; % second wheel
                p_g3=[v.dimensions.p_gear_coax_ls{wagontype}(5,1); v.dimensions.p_gear_coax_ls{wagontype}(5,2)+0.5*h_g3; v.dimensions.p_gear_coax_ls{wagontype}(5,3)+r_w]; % third wheel
                p_g4=[v.dimensions.p_gear_coax_ls{wagontype}(7,1); v.dimensions.p_gear_coax_ls{wagontype}(7,2)+0.5*h_g4; v.dimensions.p_gear_coax_ls{wagontype}(7,3)+r_w]; % third wheel
                eu_g=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
                
                %% Positioning of auxiliary components -- copied from above
                p_aux = zeros(3,size(v.dimensions.p_gear_coax_ls_aux{wagontype},1)/2);
                for i = 1:size(v.dimensions.p_gear_coax_ls_aux{wagontype},1)/2
                    x_aux = v.dimensions.p_gear_coax_ls_aux{wagontype}(2*i-1,1);
                    y_aux = v.dimensions.p_gear_coax_ls_aux{wagontype}(2*i-1,2) + 0.5 * h_aux(i);
                    z_aux = v.dimensions.p_gear_coax_ls_aux{wagontype}(2*i-1,3) + r_w;
                    p_aux(:,i) = [x_aux;y_aux;z_aux];
                end
            end
        else % double motor topology 
            % Near axle configuration
            % Sizing of gearboxes
            r_g = v.dimensions.p_gear_nax{wagontype}(2,1); %Radius
            h_g = v.dimensions.p_gear_nax{wagontype}(2,2); %Height
            sc_g = 40; %Sidecount
            
            % Sizing of motors
            r_mot = v.dimensions.p_motor_nax{wagontype}(2,1); %Radius
            h_mot = v.dimensions.p_motor_nax{wagontype}(2,2); %Height
            sc_mot = 40; %Sidecount
            
            % Positioning of motors (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
            % adjustment of position due to plot functions (see sources) and wheelstart at z=0
            p_mot=[v.dimensions.p_motor_nax{wagontype}(1,1); v.dimensions.p_motor_nax{wagontype}(1,2)+0.5*h_mot; v.dimensions.p_motor_nax{wagontype}(1,3)+r_w]; %left motor
            p_mot2=[v.dimensions.p_motor_nax{wagontype}(1,1); v.dimensions.p_motor_nax{wagontype}(3,2)+0.5*h_mot; v.dimensions.p_motor_nax{wagontype}(3,3)+r_w]; % right motor
            eu_mot=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
            
            % Positioning of gearboxes (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
            % adjustment of position due to plot functions (see sources) and wheelstart at z=0
            p_g=[v.dimensions.p_gear_nax{wagontype}(1,1); v.dimensions.p_gear_nax{wagontype}(1,2)+0.5*h_g; v.dimensions.p_gear_nax{wagontype}(1,3)+r_w]; % left gearbox
            p_g2=[v.dimensions.p_gear_nax{wagontype}(3,1); v.dimensions.p_gear_nax{wagontype}(3,2)+0.5*h_g; v.dimensions.p_gear_nax{wagontype}(3,3)+r_w]; % right gearbox
            eu_g=[pi/2;0; 0]; %Rotation Matrix for Euler Angles
            
        end
        
        % Powerelectronics
        %Sizing of Power Electronic
        Lx_pe=v.dimensions.p_powerel{wagontype}(2,1); %Length of Power Electronic
        Ly_pe=v.dimensions.p_powerel{wagontype}(2,2); %Width of Power Electronic
        Lz_pe=v.dimensions.p_powerel{wagontype}(2,3); %Height of Power Electronic
        
        % Positioning of powerelectronics (For further explanation see Package_WagonWidth or Package_WagonHeightLength)
        % adjustment of position due to plot functions (see sources) and wheelstart at z=0
        p_pe=[v.dimensions.p_powerel{wagontype}(1,1)-0.5*Lx_pe; v.dimensions.p_powerel{wagontype}(1,2)-0.5*Ly_pe;v.dimensions.p_powerel{wagontype}(1,3)+r_w-0.5*Lz_pe];
        eu_pe=[0;0;0]; %Rotation Matrix for Euler Angles
          
    end

    %% Adjust x-pos for rear axle
    % Adjustment: new x-start = wheelbase - (x-pos) --> due to negative relative position to rear axle 
    if (wagontype==2) && v.LDS.settings.filled_axles(wagontype)
        p_pe(1,1)=v.dimensions.GX.wheelbase-p_pe(1,1)-Lx_pe; % powerelectronics
        p_mot(1,1)=v.dimensions.GX.wheelbase-p_mot(1,1); % rear motor
        if v.e_machine{wagontype}.quantity==1 && strcmp(v.Input.gear_type_r,'Parallel')
            p_g1(1,1)=v.dimensions.GX.wheelbase-p_g1(1,1); % first wheel
            p_g2(1,1)=v.dimensions.GX.wheelbase-p_g2(1,1); % second wheel
            p_g3(1,1)=v.dimensions.GX.wheelbase-p_g3(1,1); % third wheel
            p_g4(1,1)=v.dimensions.GX.wheelbase-p_g4(1,1); % third wheel
            
            %adjust gear auxiliary components
            for i = 1:size(p_aux,2)
                p_aux(1,i) = v.dimensions.GX.wheelbase - p_aux(1,i);
            end
            
        elseif v.e_machine{wagontype}.quantity==1 && strcmp(v.Input.gear_type_r,'Coaxial-layshaft')
            p_g1(1,1)=v.dimensions.GX.wheelbase-p_g1(1,1); % first wheel
            p_g2(1,1)=v.dimensions.GX.wheelbase-p_g2(1,1); % second wheel
            p_g3(1,1)=v.dimensions.GX.wheelbase-p_g3(1,1); % third wheel
            p_g4(1,1)=v.dimensions.GX.wheelbase-p_g4(1,1); % fourth wheel
            
            %adjust gear auxiliary components
            for i = 1:size(p_aux,2)
                p_aux(1,i) = v.dimensions.GX.wheelbase - p_aux(1,i);
            end
            
        else
            p_g(1,1)=v.dimensions.GX.wheelbase-p_g(1,1); % coaxial gearbox
            
            if v.e_machine{wagontype}.quantity == 1 % one motor per axle
                %update gear components of planetary gear (coaxial)
                for i = 1:size(v.dimensions.comp_gear_plan{wagontype},1)
                    v.dimensions.comp_gear_plan{wagontype}(i,1) = v.dimensions.GX.wheelbase - v.dimensions.comp_gear_plan{wagontype}(i,1);
                end
                
            else % two motors per axle --> near axle engine (nax)
                %update position of motor and gearbox housing
                p_mot2(1,1) = v.dimensions.GX.wheelbase - p_mot2(1,1); % second motor of near-axle configuration
                p_g2(1,1) = v.dimensions.GX.wheelbase - p_g2(1,1);    % second gearbox of near-axle configuration
                
                %update nax gear components
                for i = 1:size(v.dimensions.comp_gear_nax_1{wagontype},1)
                    v.dimensions.comp_gear_nax_1{wagontype}(i,1) = v.dimensions.GX.wheelbase - v.dimensions.comp_gear_nax_1{wagontype}(i,1);
                    v.dimensions.comp_gear_nax_2{wagontype}(i,1) = v.dimensions.GX.wheelbase - v.dimensions.comp_gear_nax_2{wagontype}(i,1);
                end
            end
        end
        
    end
    
    %% 3) Plot Powertrain components
    if v.LDS.settings.filled_axles(wagontype)
        
        %% Plot Power electronics
        [F_pe,V_pe]=Object3d_Block(p_pe,eu_pe,Lx_pe,Ly_pe,Lz_pe);
        patch(axis,'Faces', F_pe, 'Vertices', V_pe, 'FaceColor', [51 51 51]./255); %use patch to plot power electronics
        
        %% Plot Motor and Gearbox
        % Motor
        r_mot_i = v.gearbox{wagontype}.shafts.d_sh_3+20/2;
        [F_mot,F_mot_b,V_mot]=Object3d_Cylinder_hollow(p_mot,eu_mot,r_mot,r_mot_i,h_mot,sc_mot); %create face and vertices of one motor
        patch(axis,'Faces', F_mot, 'Vertices', V_mot, 'FaceColor', [127 89 100]./255); %use patch to plot cylinder surface (round surface)
        patch(axis,'Faces', F_mot_b, 'Vertices', V_mot, 'FaceColor', [127 89 100]./255); %use patch to plot cylinder surface (flat surface)
        
        % Plot Gearbox
        % Gearbox - Parallel
        if v.e_machine{wagontype}.quantity==1 && ((wagontype==1 && strcmp(v.Input.gear_type_f,'Parallel')) || (wagontype==2 && strcmp(v.Input.gear_type_r,'Parallel')))
            % Parallel gearbox
            [F_g1,F_g_b1,V_g1]=Object3d_Cylinder(p_g1,eu_g,r_g1,h_g1,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g1, 'Vertices', V_g1, 'FaceColor', C1); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b1, 'Vertices', V_g1, 'FaceColor', C1); %use patch to plot cylinder surface (flat surface)
            
            [F_g2,F_g_b2,V_g2]=Object3d_Cylinder(p_g2,eu_g,r_g2,h_g2,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g2, 'Vertices', V_g2, 'FaceColor', C1); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b2, 'Vertices', V_g2, 'FaceColor', C1); %use patch to plot cylinder surface (flat surface)
            
            [F_g3,F_g_b3,V_g3]=Object3d_Cylinder(p_g3,eu_g,r_g3,h_g3,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g3, 'Vertices', V_g3, 'FaceColor', C2); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b3, 'Vertices', V_g3, 'FaceColor', C2); %use patch to plot cylinder surface (flat surface)
            
            [F_g4,F_g_b4,V_g4]=Object3d_Cylinder(p_g4,eu_g,r_g4,h_g4,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g4, 'Vertices', V_g4, 'FaceColor', C2); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b4, 'Vertices', V_g4, 'FaceColor', C2); %use patch to plot cylinder surface (flat surface)
            
            %Plot auxiliary gearbox components
            for i = 1:size(v.dimensions.p_gear_paral_aux{wagontype},1)/2
                color = CB; %dark grey for Bearings
                if v.dimensions.p_gear_paral_aux{wagontype}(2*i,3) == 1
                    color = CD; %blue for Differential
                end
                [F_g_aux,F_g_b_aux,V_g_aux] = Object3d_Cylinder(p_aux(:,i),eu_g,r_aux(i),h_aux(i),sc_g); %create face and vertices of gearbox
                patch(axis,'Faces', F_g_aux, 'Vertices', V_g_aux, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
                patch(axis,'Faces', F_g_b_aux, 'Vertices', V_g_aux, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
            end
            
            %% Optional -- Plot shafts
            color = CS;
            r_shaft = 0.5 * min(r_aux);
            
            %shaft 1
            r_shaft_1 =v.gearbox{wagontype}.shafts.d_sh_1/2;
            p_shaft_1 = p_aux(:,2) - [0;0.5 * h_aux(2);0];
            h_shaft_1 = p_shaft_1(2) - p_aux(2,1) + 0.5 * h_aux(1);
            [F_g_sh1,F_g_b_sh1,V_g_sh1]=Object3d_Cylinder(p_shaft_1,eu_g,r_shaft_1,h_shaft_1,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g_sh1, 'Vertices', V_g_sh1, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b_sh1, 'Vertices', V_g_sh1, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
            
            %shaft 2
            r_shaft_2 =v.gearbox{wagontype}.shafts.d_sh_2/2;
            p_shaft_2 = p_aux(:,4) - [0;0.5 * h_aux(4);0];
            h_shaft_2 = p_shaft_2(2) - p_aux(2,3) + 0.5 * h_aux(3);
            [F_g_sh2,F_g_b_sh2,V_g_sh2]=Object3d_Cylinder(p_shaft_2,eu_g,r_shaft_2,h_shaft_2,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g_sh2, 'Vertices', V_g_sh2, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b_sh2, 'Vertices', V_g_sh2, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
            
            %shaft 3
            r_shaft_3 =v.gearbox{wagontype}.shafts.d_sh_3/2;
            p_shaft_3 = p_aux(:,5) - [0;0.5 * h_aux(5);0];
            h_shaft_3 = p_shaft_3(2) - p_aux(2,6) + 0.5 * h_aux(6);
            [F_g_sh3,F_g_b_sh3,V_g_sh3]=Object3d_Cylinder(p_shaft_3,eu_g,r_shaft_3,h_shaft_3,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g_sh3, 'Vertices', V_g_sh3, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b_sh3, 'Vertices', V_g_sh3, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
        
        % Gearbox - Coaxial
        elseif ((wagontype==1 && strcmp(v.Input.gear_type_f,'Coaxial')) || (wagontype==2 && strcmp(v.Input.gear_type_r,'Coaxial')))
            % Coaxial gearbox
            [F_g,F_g_b,V_g]=Object3d_Cylinder(p_g,eu_g,r_g,h_g,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g, 'Vertices', V_g, 'FaceColor', [127 127 127]./255, 'FaceAlpha', 0.5);   %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b, 'Vertices', V_g, 'FaceColor', [127 127 127]./255, 'FaceAlpha', 0.5); %use patch to plot cylinder surface (flat surface)
            
            %Plot gear components of planetary gear
            if v.e_machine{wagontype}.quantity == 1 
                components = v.dimensions.comp_gear_plan{wagontype}; % Load gearbox components from dimensions struct
            else % 2 engines per axle 
                components = v.dimensions.comp_gear_nax_1{wagontype}; % Load gearbox components from dimensions struct (left v side (driving direction))
            end
            Plot_Gear_Plan(axis,v,components);
        
            
        % Gearbox - Coaxial Layshaft
        elseif v.e_machine{wagontype}.quantity==1 && ((wagontype==1 && strcmp(v.Input.gear_type_f,'Coaxial-layshaft')) || (wagontype==2 && strcmp(v.Input.gear_type_r,'Coaxial-layshaft')))
            % Coaxial layshaft gearbox
            [F_g1,F_g_b1,V_g1]=Object3d_Cylinder(p_g1,eu_g,r_g1,h_g1,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g1, 'Vertices', V_g1, 'FaceColor', C1); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b1, 'Vertices', V_g1, 'FaceColor', C1); %use patch to plot cylinder surface (flat surface)
            
            [F_g2,F_g_b2,V_g2]=Object3d_Cylinder(p_g2,eu_g,r_g2,h_g2,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g2, 'Vertices', V_g2, 'FaceColor', C1); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b2, 'Vertices', V_g2, 'FaceColor', C1); %use patch to plot cylinder surface (flat surface)
            
            [F_g3,F_g_b3,V_g3]=Object3d_Cylinder(p_g3,eu_g,r_g3,h_g3,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g3, 'Vertices', V_g3, 'FaceColor', C2); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b3, 'Vertices', V_g3, 'FaceColor', C2); %use patch to plot cylinder surface (flat surface)
            
            [F_g4,F_g_b4,V_g4]=Object3d_Cylinder(p_g4,eu_g,r_g4,h_g4,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g4, 'Vertices', V_g4, 'FaceColor', C2); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b4, 'Vertices', V_g4, 'FaceColor', C2); %use patch to plot cylinder surface (flat surface)
            
            %Plot auxiliary gearbox components
            for i = 1:size(v.dimensions.p_gear_coax_ls_aux{wagontype},1)/2
                color = CB; %dark grey for Bearings
                if v.dimensions.p_gear_coax_ls_aux{wagontype}(2*i,3) == 1
                    color = CD; %blue for Differential
                end
                [F_g_aux,F_g_b_aux,V_g_aux] = Object3d_Cylinder(p_aux(:,i),eu_g,r_aux(i),h_aux(i),sc_g); %create face and vertices of gearbox
                patch(axis,'Faces', F_g_aux, 'Vertices', V_g_aux, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
                patch(axis,'Faces', F_g_b_aux, 'Vertices', V_g_aux, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
            end
            
            %% Optional -- Plot shafts
            color = CS;
            
            %shaft 1
            r_shaft_1 =v.gearbox{wagontype}.shafts.d_sh_1/2;
            p_shaft_1 = p_aux(:,5) - [0;0.5 * h_aux(5);0];
            h_shaft_1 = p_shaft_1(2) - p_aux(2,1) + 0.5 * h_aux(1);
            [F_g_sh1,F_g_b_sh1,V_g_sh1]=Object3d_Cylinder(p_shaft_1,eu_g,r_shaft_1,h_shaft_1,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g_sh1, 'Vertices', V_g_sh1, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b_sh1, 'Vertices', V_g_sh1, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
            
            %shaft 2
            r_shaft_2 =v.gearbox{wagontype}.shafts.d_sh_2/2;
            p_shaft_2 = p_aux(:,4) - [0;0.5 * h_aux(4);0];
            h_shaft_2 = p_shaft_2(2) - p_aux(2,3) + 0.5 * h_aux(3);
            [F_g_sh2,F_g_b_sh2,V_g_sh2]=Object3d_Cylinder(p_shaft_2,eu_g,r_shaft_2,h_shaft_2,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g_sh2, 'Vertices', V_g_sh2, 'FaceColor', color); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g_b_sh2, 'Vertices', V_g_sh2, 'FaceColor', color); %use patch to plot cylinder surface (flat surface)
            
        end
        
        % Second set of motor and gears (for near-axle topology)
        if v.e_machine{wagontype}.quantity==2
            % Second gearbox
            [F_g2,F_g2_b,V_g2]=Object3d_Cylinder(p_g2,eu_g,r_g,h_g,sc_g); %create face and vertices of gearbox
            patch(axis,'Faces', F_g2, 'Vertices', V_g2, 'FaceColor', [127 127 127]./255, 'FaceAlpha', 0.5); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_g2_b, 'Vertices', V_g2, 'FaceColor', [127 127 127]./255, 'FaceAlpha', 0.5); %use patch to plot cylinder surface (flat surface)
            
            %Plot components of second gearbox
            components = v.dimensions.comp_gear_nax_2{wagontype}; % Load gear components from dimensions struct
            Plot_Gear_Plan(axis,v,components);
            
            % Second motor
            r_mot_i = v.gearbox{wagontype}.shafts.d_sh_3+20/2;
            [F_mot,F_mot_b,V_mot]=Object3d_Cylinder_hollow(p_mot2,eu_mot,r_mot,r_mot_i,h_mot,sc_mot); %create face and vertices of one motor
            patch(axis,'Faces', F_mot, 'Vertices', V_mot, 'FaceColor', [127 89 100]./255); %use patch to plot cylinder surface (round surface)
            patch(axis,'Faces', F_mot_b, 'Vertices', V_mot, 'FaceColor', [127 89 100]./255); %use patch to plot cylinder surface (flat surface)
        end
    end
    

end

%% 4) Plot Drive Shafts
Plot_Drive_Shaft(v);

end

