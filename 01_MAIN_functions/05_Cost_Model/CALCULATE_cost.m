function [Manufacturing_Cost,Component_Groups] = CALCULATE_cost(vehicle,Time_zone,Databank,plot,update)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Dila Akguel (Technical University of Munich), Fabian Liemawan Adji (Technical University of Munich)
%-------------
% Created on: 01.01.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function estimates the manufacturing cost of a vehicle based its technical data generated with function "cost". 
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
%           [2] Dila Akguel, "Development of a Tool for the Cost Estimation of Autonomous Vehicles", Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2020
%           [3] Fabian Liemawan Adji, "Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen", Semester Thesis, Technical University of Munich, Institute of Automotive Technology, 2020
% ------------
% Input:    - Parameters: struct with input and constant values
%           - Time Zone : 1-2-3 -> 1: Present time 2: 2025 3: 2030
%           - Databank: The Cost Databank in the Workspace
%           - plot: int, 1-0 -> 1: plot on  0:plot off
%           - update: int, 1-0 -> 1:updates the databank by reading the Excel table, 0:no update
% ------------
% Output:   - Manufacturing_cost: Manufacturing cost of the vehicle
%           - Component_Groups: shows the manufacturing cost for each
%             component group (powertrain, chassis, structure(BiW), closures,
%             sensors, assembly)
% ------------

%% Implementation
% 1) Update databank
% 2) Read technical data for given year
% 3) Calculate cost
% 4) Optional: Pie chart with cost

%% 1) Update databank
if update==1
    Databank=databank_update('Cost_Database.xlsx');    
end

%% 2) Read technical for given year
%Creates the technical data vector from the vehicle structure
[technical_data_vector,mot_data]=READ_techdata(vehicle);

%Chooses the right column from the exported databank for the chosen time
%zone. The second column is for the present values, the third for 2025 and
%the forth for 2030.
switch Time_zone
    case 1
        v_cost=cell2mat(Databank(:,2));
    case 2
        v_cost=cell2mat(Databank(:,3));
    case 3
        v_cost=cell2mat(Databank(:,4));
end
%----------------------------------------------------------------

%% 3) Calculate cost
%Separate cost functions for motor calculation
v_cost_mot=v_cost(1:4,:);
v_cost=v_cost(5:end,:);

%Read motors
mot_data_IM=mot_data(1:2:3,:); %IM motors
mot_data_IM=mot_data_IM(mot_data_IM(:)~=0); %filter for motors
mot_data_PSM=mot_data(2:2:4,:); %PSM motors
mot_data_PSM=mot_data_PSM(mot_data_PSM(:)~=0); %filter for motors

%initialize motor costs
cost_mot_IM=zeros(length(mot_data_IM));
cost_mot_PSM=zeros(length(mot_data_PSM));

if ~isempty(mot_data_IM)
    cost_mot_IM=mot_data_IM.*v_cost_mot(2,1)+v_cost_mot(1,1);
    cost_mot_IM=sum(cost_mot_IM); % cpst of all IM motors
else
    cost_mot_IM=0;
end

if ~isempty(mot_data_PSM)
    cost_mot_PSM=mot_data_PSM.*v_cost_mot(4,1)+v_cost_mot(3,1);
    cost_mot_PSM=sum(cost_mot_PSM); %cost of all IM motors
else
    cost_mot_PSM=0;
end

cost_mot_tot=cost_mot_IM+cost_mot_PSM; %total motor costs
%multiplies the technical data vector with the chosen column of the
%databank and creates co-> the vector containing manufacturing cost of
%every component
cost_total=technical_data_vector.*v_cost; %cost without motors
cost_total=[cost_mot_tot;cost_total]; %cost with motors

%----------------------------------------------------------------

%Sums up the rows of the cost vector and gives the manufacturing cost for
%each component group in an arry
Cost_Powertrain=sum(cost_total(1:10))-cost_total(3);
Cost_Battery =cost_total(3);
Cost_Chassis=sum(cost_total(11:18));
Cost_Electronic=sum(cost_total(19:23));
Cost_Interior=sum(cost_total(24:40));
Cost_Exterior=sum(cost_total(41:48));
Cost_Structure=sum(cost_total(49:59));
Cost_Tech=sum(cost_total(60:72));
Cost_Assembly=cost_total(73);

Component_Groups=[ Cost_Powertrain Cost_Battery Cost_Chassis Cost_Electronic Cost_Interior Cost_Exterior Cost_Structure Cost_Tech Cost_Assembly];
Manufacturing_Cost=Cost_Powertrain+Cost_Battery+Cost_Chassis+Cost_Electronic+Cost_Interior+Cost_Exterior+Cost_Structure+Cost_Tech+Cost_Assembly;

%------------------------------------------------------------------
%% 4) Optional: Pie chart with cost
%Pie chart
%display(Component_Groups);
%display(Manufacturing_Cost);
if plot==1
    labels={'Powertrain:','Battery:','Chassis:','Electronic:','Interior:','Exterior:','Structure:','Tech:','Assembly:'};
    per=percentages(Component_Groups);
    per=string(per);
    
    a={'%','%','%','%','%','%','%','%','%',};
    
    percentage_values=strcat(labels,per,a);
    %Percentage_values;
    figure(2)
    pie_chart= pie(Component_Groups,percentage_values);
    %Legend(labels);
else
end
%-----------------------------------------------------------------
end

function [per] = percentages(comp)
%Calculates the percentage value for each component group

total_veh_cost=sum(comp);
i=1;
a=zeros(size(comp));
format short
while i<=length(comp)
    a(:,i)=(comp(:,i)/total_veh_cost)*100;
    format short
    i=i+1;
end
per=a;
format short
end


