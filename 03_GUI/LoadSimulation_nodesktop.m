%% Starts simulation - nodesktop version to work with linux console
% IMPORTANT NOTE: In the nodesktop version the path of the file/folder and the variation settings have to be manually filled!
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 25.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function reads in saved App-Data (from the Userform) and starts the simulation 
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Saved userform configuration for external simulation
% ------------
% Output:   - Saved Optimization(s) in 06_Results folder
% ------------

clear
clc
%% Manually set needed parameters
%Simulation mode: 1= single file, 2= folder with files
simulation_mode     =   1;

% Load either single file or add path of folder
InputPath=fileparts(which("Run_AuVeCoDe.m"));
if simulation_mode==1
    load(fullfile(InputPath,"VS4_Konfig_Acceleration_NSGAII_ext.mat"));
elseif simulation_mode==2
    FilePath=fullfile(InputPath,"Simulation");
end

%1) Change current path
cd(InputPath);
%2) Add folders and subfolders
addpath(genpath(InputPath));


%% Start single or multiple optimization
if simulation_mode==1
    % Only single configuration is loaded
    n_config=0;
    
    if exist('var_vector')
        %Variation mode: 1= vary selected parameter 0= do not vary any parameter
        Variation           =   var_vector.Varation;
        %If variation mode==1: Select parameter to be varied
        par_select          =   var_vector.par_select;
        %If variation mode==1: Select limit of variation (10 means +-10%)
        Limit               =   var_vector.Limit;
    else
        Variation=0;
    end
    
    
    if Variation==1
        Name=par_select;
        Value_vect=create_variation_vec(Name,Limit,Parameters);        
    else
        Value_vect=0;
        Name=0;
    end
    Optimization_main(Parameters,options,InputPath,Value_vect,Variation,n_config,Name)
elseif simulation_mode==2
    % Multiple configurations are loaded
    file=dir(fullfile(FilePath,'*.mat'));
    %Load and optimize for each configuration
    for n_config=1:length(file) 
        clear('Parameters');
        clear('options');
        clear('var_vector');
        
        load(fullfile(file(n_config).folder,file(n_config).name));
        
        if exist('var_vector')
            %Variation mode: 1= vary selected parameter 0= do not vary any parameter
            Variation           =   var_vector.Varation;
            %If variation mode==1: Select parameter to be varied
            par_select          =   var_vector.par_select;
            %If variation mode==1: Select limit of variation (10 means +-10%)
            Limit               =   var_vector.Limit;
        else
            Variation=0;
        end
        
        if Variation==1
            Name=par_select;
            Value_vect=create_variation_vec(Name,Limit,Parameters);
        else
            Value_vect=0;
            Name=0;
        end
        Optimization_main(Parameters,options,InputPath,Value_vect,Variation,n_config,Name)
    end
end
    
%% Simulation
function Optimization_main(Parameters,options,InputPath,Value_vect,Variation,n_config,Name)
    SaveFolder = fullfile(InputPath,"06_Results");
    %Optimize for selected parameter range
    if Variation==1 
        for n_var=1:length(Value_vect)
            switch Name
                case 'Acceleration'
                    Parameters.input.acceleration_time_req=Value_vect(n_var);
                case 'Speed'
                    Parameters.input.max_speed=Value_vect(n_var);
                case 'Range'
                    Parameters.input.range=Value_vect(n_var);
                case 'Front Window Angle'
                    Parameters.input.window_angle_front_min=Value_vect(n_var);
                case 'Rear Window Angle'
                    Parameters.input.window_angle_rear_min=Value_vect(n_var);
                case 'Free Crash Length'
                    Parameters.input.free_crash_length=Value_vect(:,n_var)';
            end            
            %% Call optimization
            Optimization_coordinator(options,Parameters,n_config,n_var,SaveFolder)
            
        end
    %Only optimize once
    else
        n_var=0;
        Optimization_coordinator(options,Parameters,n_config,n_var,SaveFolder)
    end
end
    
    
    %% Optimization
    function Optimization_coordinator(options,Parameters,n_config,n_var,SaveFolder)
    %Repeat for every step    
    for n_rep = 1:options.RepeatNumber
        Optimization_start(options,Parameters,n_config,n_var,n_rep,SaveFolder)
    end
    end
    
    function Optimization_start(options,Parameters,n_config,n_var,n_rep,SaveFolder)
    % prepare the path to save folder
    if n_var==0 && n_config==0        
        options.ResultPath = fullfile(SaveFolder, (options.FolderName + "_Optimization"));
    elseif n_var==0 && n_config>0
        options.ResultPath = fullfile(SaveFolder, (options.FolderName + "_Optimization_" + sprintf('Config%02d_',n_config) ));
    elseif n_var>0 && n_config==0
        options.ResultPath = fullfile(SaveFolder, (options.FolderName + "_Optimization_" + sprintf('Var%02d_',n_var) ));
    elseif n_var>0 && n_config>0
        options.ResultPath = fullfile(SaveFolder, (options.FolderName + "_Optimization_" + sprintf('Config%02d_',n_config) + sprintf('Var%02d_',n_var)));   
    end
    
    switch Parameters.optimization.result_separate_save
        case 0 % user wants to save additional results in 06_Results and overwrite old additional results
            options.AdditionalResultPath = SaveFolder;
        case 1 % user wants to save additional results in current defined result folder
            options.AdditionalResultPath = options.ResultPath;
    end
    
    % save Parameters_Pareto for plotting
    save(fullfile(options.AdditionalResultPath, "Parameters_Pareto.mat"), "Parameters");
    
    % run optimization
    [state] = options.func(options, Parameters);
    
    % prepare path for saving results
    formatOut='yymmdd'; %Right format
    currentdate=datestr(now,formatOut); %current date in right format
    if n_var==0 && n_config==0        
        SaveResultFile = fullfile(SaveFolder,(currentdate + "_Optimization_" + sprintf('%02d',n_rep) + ".mat"));
    elseif n_var==0 && n_config>0
        SaveResultFile = fullfile(SaveFolder,(currentdate + "_Optimization_" + sprintf('Config%02d_',n_config) + sprintf('%02d',n_rep) + ".mat"));
    elseif n_var>0 && n_config==0
        SaveResultFile = fullfile(SaveFolder,(currentdate + "_Optimization_" + sprintf('Var%02d_',n_var) + sprintf('%02d',n_rep) + ".mat"));
    elseif n_var>0 && n_config>0
        SaveResultFile = fullfile(SaveFolder,(currentdate + "_Optimization_" + sprintf('Config%02d_',n_config) + sprintf('Var%02d_',n_var) + sprintf('%02d',n_rep) + ".mat"));
    end    
    % Save results
    save(SaveResultFile, 'state');
    end       
    

    function [Value_vect]=create_variation_vec(Name,Limit,Parameters)        
    % Calculate the range of values
    switch Name
        case 'Acceleration'
            Value=Parameters.input.acceleration_time_req;
        case 'Speed'
            Value=Parameters.input.max_speed;
        case 'Range'
            Value=Parameters.input.range;
        case 'Front Window Angle'
            Value=Parameters.input.window_angle_front_min;
        case 'Rear Window Angle'
            Value=Parameters.input.window_angle_rear_min;
        case 'Free Crash Length'
            Value=Parameters.input.free_crash_length;
    end
        %Change step size of variation for chosen percentage
        switch true
            case Limit<10
                num_step=0.01;
            case Limit>=20 && Limit<50
                num_step=0.05;
            case Limit>=50 && Limit<100
                num_step=0.10; 
            case Limit==100
                num_step=0.20; 
        end
                
        % Create variation vector
        vect=((-Limit/100):num_step:(Limit/100))+1;
        Value_vect=vect.*Value';
    end
    

    