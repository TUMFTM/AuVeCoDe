function [Parameters] = ConvertUserform(app)
%% Description:
% Designed by:  Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.12.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Called by userform. Reads settings and input from userform and calls Startfile
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - app: Userform data including user input
% ------------
% Output:   - Parameters: struct with input and constant values
%
% ------------

%% Description: 
%Called by userform. Reads settings and input from userform and calls Startfile
%Author:   Adrian König
%Modified: December 2021

%% Input: Settings from userform
%% Output: Parameters with inputs

%% 1) Load Paramters struct
% Load Fixed Parameters
warning('off','all') %Switch off warnings when loading parameters (due to function links)
load 'Parameters_AuVeCoDe.mat' Parameters;
warning('on')

%% x) Temporary: Update Parameters Struct
%Update Parameters with values of Lorenzo
%[Parameters] = UpdateParameters(Parameters);

%% 2) Update cost database
[Databank] = databank_update('Cost_Database.xlsx'); %Load updated Cost Database
Parameters.Databank=Databank; %% Assign to struct

% GUI Label Visibility
app.Output_Label.Visible='off';
app.Error_Label.Visible='off';

%% Read in parameters
[Parameters] = ReadApp(Parameters,app); %Read parameters from userform

end

