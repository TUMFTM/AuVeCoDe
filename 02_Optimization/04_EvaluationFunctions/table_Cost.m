function [Cost] = table_Cost(varargin)
    
    %% Description
    % Functions to display cost comparison of multiples vehicle concepts
    % Current Features:
    %       -Group Cost
    %       -Powertrain Cost
    %       -Chassis Cost
    %       -PSM Cost
    
    % Author: Fabian Liemawan Adji
    % Date: September 2021
    
    %% Input:
    % varargin: e.g. table_Cost(veh1, veh2, veh3, ... , veh10)
    % veh represents vehicle structs
    
    %% Output:
    % Cost: struct with cost tables
    % Result.xlsx:
    %           -Sheet Überblick Kosten compare group costs
    %           -Sheet Kosten Powertrain compare powertrain costs
    %           -Sheet Kosten Chassis compare chassis costs
    %           -Sheet Kosten Motor compare PSM costs
    
    % initialize costs tables
    CostTable = table;
    PowertrainCost = table;
    ChassisCost = table;
    PSMCost = table;
    
    % initialize table row index
    index = 1;
    
    for i = 1:nargin
        
        % collect fieldnames of every vehicle struct in string
        FieldNames = string(fieldnames(varargin{1,i}));
        
        for ii = 1:length(FieldNames)
            %% Cost Table
            CostTable.(FieldNames(ii))= ...
                varargin{1, i}.(FieldNames(ii)).Cost.Component_Groups.Cost;
            
            
            %% Powertrain Table
            % sort by names for comparison
            PowertrainData = sortrows(varargin{1, i}.(FieldNames(ii)).Cost.Group_Cost.Powertrain, 1);
            % get category
            PowertrainCost.Kategorie = PowertrainData.Name;
            % get total
            PowertrainCost.(FieldNames(ii))= PowertrainData.Total;
            
            %% Chassis Table
            % sort by names for comparison
            ChassisData = sortrows(varargin{1, i}.(FieldNames(ii)).Cost.Group_Cost.Chassis, 1);
            % get category
            ChassisCost.Kategorie = ChassisData.Name;
            % get total
            ChassisCost.(FieldNames(ii))= ChassisData.Total;
            
            %% PSM Table
            % get needs and costs for each motor component
            PSMCost.EM_Front1_Needs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.VehicleNeeds(1);
            PSMCost.EM_Front1_Costs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.TotalPrice(1);
            PSMCost.EM_Front2_Needs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.VehicleNeeds(2);
            PSMCost.EM_Front2_Costs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.TotalPrice(2);
            PSMCost.EM_Rear1_Needs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.VehicleNeeds(3);
            PSMCost.EM_Rear1_Costs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.TotalPrice(3);
            PSMCost.EM_Rear2_Needs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.VehicleNeeds(4);
            PSMCost.EM_Rear2_Costs(index) = varargin{1, i}.(FieldNames(ii)).Cost.Powertrain_Cost.CostView.TotalPrice(4);

            
            % add index by one
            index = index + 1;
        end
    end
    % store all cost tables in a single struct
    Cost.CostTable = CostTable;
    Cost.PowertrainCost = PowertrainCost;
    Cost.ChassisCost = ChassisCost;
    Cost.PSMCost = PSMCost;
    
    % get group names for cost table of group costs
    CostTable.CostGroup = varargin{1, i}.(FieldNames(ii)).Cost.Component_Groups.Names;
    % move group names to first column
    CostTable = movevars(CostTable, 'CostGroup', 'Before', 1);
    
    % write all costs table in Result.xlsx
    writetable(CostTable, "Result.xlsx", "Sheet", "Überblick Kosten");
    writetable(PowertrainCost, "Result.xlsx", "Sheet", "Kosten Powertrain");
    writetable(ChassisCost, "Result.xlsx", "Sheet", "Kosten Chassis");
    writetable(PSMCost, "Result.xlsx", "Sheet", "Kosten Motor");
    

end