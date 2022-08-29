function [Databank] = databank_update(Excel_Doc)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Dila Akguel (Technical University of Munich), Fabian Liemawan Adji (Technical University of Munich)
%-------------
% Created on: 01.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function updates the databank before running the total_cost function
%              This is the first function if the tool: please call the function as shown: [Databank]=databank_update('XXXXXXX.xlsx')
% ------------
% Sources:  [1] Dila Akguel, "Development of a Tool for the Cost Estimation of Autonomous Vehicles", Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2020
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%          
% ------------
% Input:    - Excel_Doc
% ------------
% Output:   - Databank
% ------------

All_=readtable(Excel_Doc,'Sheet','Matlab_Export','PreserveVariableNames',true);
All_=table2cell(All_);
Databank=All_(:,2:6);

end

