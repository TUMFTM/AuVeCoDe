%% WELCOME TO THE AUVECODE TOOL %%

%            ____________________
%           /   /     |      \   \
%          /   |      |       |   \
%         |    |      |       |    |
%         | ___|      |       |___ |
%         |(   ) ------------ (   )|  
% ----------------------------------------------

% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 21.01.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Adds the folders to matlab and opens userform to start the
% calculation or optimization of autonomous vehicle concepts
% ------------
%% Start AuVeCoDe Userform
%% MATLAB Release Check
%Matlab Version supported:
release = 'R2020b';
try
Old=isMATLABReleaseOlderThan(release);
catch
    error('The Matlab version for the AuVeCoDe-Tool has to be R2020b or newer!')
end
if Old==1
    error('The Matlab version for the AuVeCoDe-Tool has to be R2020b or newer!')
end

%1) Find path
InputPath=fileparts(which("Run_AuVeCoDe.m"));
cd(InputPath);
%2) Add folders and subfolders
addpath(genpath(InputPath))
%3) Run userform
run(fullfile(InputPath,"03_GUI","App_AuVeCoDe.mlapp"));