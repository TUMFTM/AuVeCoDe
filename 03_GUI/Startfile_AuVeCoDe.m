function [vehicle, options]=Startfile_AuVeCoDe(Parameters,app)
%-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
%                               START FILE                                %
%-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

% !!! Do not run this script. Start userform instead. !!!

%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Daniel Telschow, Korbinian
% Moller, Fabian Liemawan Adji
%-------------
% Created on: 01.05.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Startfile which launches either the MAIN_AuVeCoDe for the single vehicle calculation
%              or the selected optimizer (NSGA-II,MOPSO etc.). After calculations the vehicle is plotted
% ------------
% Input:    - Parameters: struct with input and constant values
%           - app: app file which includes selected userform inputs
% ------------
% Output:   - vehicle: struct including the data of the vehicle
%           - options: 
%           - plot of single vehicle / best vehicles and optimization results

%% Implementation
% 1) Clear and Folderlocation
% 2) Save Parameters
% 3) Initialization and simulation start (either Single Vehicle or Optimization

%% 1) Clear and Folderlocation
clc; %Clear command window
close all %close all windows
%Add that folder plus all subfolders to the path.
folder = fileparts(which('Run_AuVeCoDe')); %Determine where Run_AuVeCoDe is.
addpath(genpath(folder),'-begin');         %Add that folder plus all subfolders to the path.
fprintf('\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n');
fprintf('*               DEVELOPMENT AND OPTIMIZATION OF AUTONOMOUS VEHICLE CONCEPTS                 *\n');
fprintf('*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n\n');

%% 2) Save Parameters
% Save temp Parameters for vehicle plot and comparison later on
[filepath,~,~] = fileparts(which('Run_AuVeCoDe'));
if Parameters.input.single_vehicle==1
    path_Par= fullfile(filepath,'06_Results','Parameters_SingleVehicle.mat');
    save(path_Par','Parameters'); % save parameters struct
end

%% 3) Initialization and simulation start
% Plot date and time of simulation start
t_start = datestr(datetime('now'));   
disp(['Simulation start: ' , t_start]);

   
%%---------------------------Single vehicle development--------------------
    if Parameters.input.single_vehicle==1
        
        fprintf('Calculating Single Vehicle...\n');
        [vehicle] = MAIN_AuVeCoDe(Parameters);
        
        % Output results
        if vehicle.feasible==1 % vehicle is feasible
            [filepath,~,~] = fileparts(which('Run_AuVeCoDe'));
            path_veh= fullfile(filepath,'06_Results','single_vehicle_result.mat');            
            vehicle=orderfields(vehicle); %Sort vehicle in alhpabetic matter
            save(path_veh,'vehicle'); % save vehicle struct
            
            fprintf('Vehicle empty weight is...%d kg\n',vehicle.masses.vehicle_empty_weight);
            fprintf('Vehicle length is...%d mm\n',vehicle.dimensions.GX.vehicle_length);
            fprintf('Total cost of the vehicle is...%d €\n', vehicle.Cost.Manufacturing_Cost);
            
            app.Run_Text.Visible='off';           
            % Plot in second window
            figure('Name','Vehicle Package','NumberTitle','off');%,'Renderer', 'Painters');
            clf
            ax1 = axes;
            %Optional fixed axes, better zoom but not true to scale after zoom
            ax1.Clipping = "off";
            DISPLAY_vehicle(vehicle,Parameters,ax1);
            ax1.Projection = 'perspective';
            %set(gca, 'Visible', 'off');  %disable axes        
            
            %Write results in Excel sheet
            WRITE_results(vehicle)
            
            %Optional: plot important data
            %plot_resultdata(vehicle);
        else % vehicle not feasible
            [Msg] = Error_Reason_Msg(vehicle); % relate error to specific message
            fprintf('Vehicle not feasible...\n');
            fprintf('Reason: %s\n',Msg);
            fprintf('Please adjust the parameters and try again.\n');
            if exist('app')
            app.Run_Text.Visible='off';
            app.Output_Label.Text=Msg;
            app.Output_Label.Visible='on';
            app.Error_Label.Visible='on';
            end
        end
        % initialize empty options for function output structure
        options = [];
%%---------------------------concept optimization--------------------------
    else 
		% Initializes optimization options and specify messages
        if Parameters.optimization.NSGA == 1
            [options,Parameters] = NSGAII_Options(Parameters);
            Message = "Starting NSGA II optimization...please wait\n";
        elseif Parameters.optimization.MOPSO == 1
            [options,Parameters] = MOPSO_Options(Parameters);
            Message = "Starting MOPSO optimization...please wait\n";
        elseif Parameters.optimization.MPNSGA == 1
            [options,Parameters] = MultiPopNSGAII_Options(Parameters);
            Message = "Starting MultiPopulation NSGA II optimization...please wait\n";
        elseif Parameters.optimization.MSMOPSO == 1
            [options,Parameters] = MultiSwarmMOPSO_Options(Parameters);
            Message = "Starting MultiSwarm MOPSO optimization...please wait\n";
        end
        fprintf(Message);
        
        % Prepare path for saving results and save plotvehicles
        options.ResultPath = fullfile(app.SaveFolderName, (options.FolderName + "_Optimization" + sprintf('%02d',app.CurrentIndex)));
        % Check if folder exists
        if ~isfolder(options.ResultPath)
            mkdir(options.ResultPath); % create target folder if it doesn't exist
        end
        
        % Check checkbox for user's preference to save additional results
        switch Parameters.optimization.result_separate_save
            case 0 % user wants to save additional results in 06_Results and overwrite old additional results
                options.AdditionalResultPath = fullfile(filepath,'06_Results');
            case 1 % user wants to save additional results in current defined result folder
                options.AdditionalResultPath = options.ResultPath;
        end
        
        % Save Parameters_Pareto for plotting
        save(fullfile(options.AdditionalResultPath, "Parameters_Pareto.mat"), "Parameters");
        
        % Run optimization
        [state] = options.func(options, Parameters);
        
        
        % Set the full path of the folder for optimization results
        SaveResultFile = fullfile(options.ResultPath, ...
            ("OptimizationResult.mat"));
        
        % Save results
        save(SaveResultFile, "state");
		
        vehicle=0;
    end
fprintf('Current processes finished...\n');
%%---------------------------runtime information---------------------------
t_end = datestr(datetime('now')); 
disp(['Simulation end: ' , t_end]);
t_start = datevec(datenum(t_start));
t_end = datevec(datenum(t_end));
time = etime(t_end,t_start);
fprintf('Required computation time: %s seconds \n' , string(time));
fprintf('\n*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n');
fprintf('*                                    END OF SIMULATION                                      *\n');
fprintf('*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n\n');
end
