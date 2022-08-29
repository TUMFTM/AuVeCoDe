function [optimum_check]=CHECK_optimum(state)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 08.05.2022
% ------------
% Version: Matlab2020b
%-------------
% Description: This function checks if optimum is plausible or by variation of one objective, the
%              result can still be improved
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Parameters: struct with input and constant values
%           - Var: objectives from optimizer for optimal vehicle
% ------------
% Output:   - Message if it is plausible, that vehicle was optimal
% ------------

%% Implementation
% 1) Load objectives
% 2) Calculate vehicle and change objectives

%% 1) Load objectives
Parameters=state.Parameters;
best_obj=zeros(2,2);
set_best=state.result.bestPosition(1,:);
result_obj1=sortrows(state.result.bestFitness,1);
best_obj(1,:)=result_obj1(1,:);
result_obj2=sortrows(state.result.bestFitness,2);
best_obj(2,:)=result_obj2(1,:);

%% 2) Calculate vehicle and change objectives
obj=zeros(length(set_best),2);

for i=1:length(set_best)
    fprintf('Objective %d out of %d is checked...\n',i,length(set_best))
    % Differentiate between objective type (1=float, 2=integer) and
    % number of possible values (if integer)
    switch i
        case 1 %Powertrain
            type=2;
            n_ob=8;
        case {2,14,17,26,27}
            type=2;
            n_ob=2;
        case {3,4,6}
            type=2;
            n_ob=3;
        case 11
            type=2;
            n_ob=4;
        otherwise
            type=1; %standard: 1=float, 2=int
            n_ob=0;
    end
    
    if type==1 %float
        value_best=set_best(i); %read value of best set        
        value_var(1)=max(value_best*0.95,state.options.lb(i)); %reduce 5% (only within boundaries)
        value_var(2)=min(value_best*1.05,state.options.ub(i)); %add 5% (only within boundaries)
        set_best_temp=set_best; %set temporary objective set
        for j=1:2
            set_best_temp(i)=value_var(j); % Set objective to new value
            vehicle = MAIN_AuVeCoDe(Parameters,set_best_temp); %Recalculate
            if vehicle.feasible==1
                obj_temp(1)=eval(Parameters.optimization.goal_1);
                obj_temp(2)=eval(Parameters.optimization.goal_2);
            else
                obj_temp(1)=100000;
                obj_temp(2)=100000;
            end
            if j>1 % Check smallest value for objectiv setting
                Comp=sum(obj(i,:)./obj_temp(1,:));
                if Comp>2 %Temp is in sum better then the other values of the objective => Change to better values
                    obj(i,1)=obj_temp(1);
                    obj(i,2)=obj_temp(2);
                end
            else
                obj(i,1)=obj_temp(1);
                obj(i,2)=obj_temp(2);
            end
        end
    elseif type==2 %integer
        set_best_temp=set_best; %set temporary objective set
        for j=1:n_ob
            set_best_temp(i)=j; % Set objective to new value
            vehicle = MAIN_AuVeCoDe(Parameters,set_best_temp); %Recalculate
            if vehicle.feasible==1
                obj_temp(1)=eval(Parameters.optimization.goal_1);
                obj_temp(2)=eval(Parameters.optimization.goal_2);
            else
                obj_temp(1)=100000;
                obj_temp(2)=100000;
            end
            if j>1 % Check smallest value for objectiv setting
                Comp=sum(obj(i,:)./obj_temp(1,:));
                if Comp>2 %Temp is in sum better then the other values of the objective => Change to better values
                    obj(i,1)=obj_temp(1);
                    obj(i,2)=obj_temp(2);
                end
            else
                obj(i,1)=obj_temp(1);
                obj(i,2)=obj_temp(2);
            end
        end
    end    
    
end

%% 3) Compare results
%Search for best results (for each objective)
min_obj1=sortrows(obj,1);
min_obj1=min_obj1(1,:);
min_obj2=sortrows(obj,2);
min_obj2=min_obj2(1,:);

%Check if one or both objectives could be improved
if (min_obj1(1)/best_obj(1,1))<1 &&(min_obj2(2)/best_obj(2,2))<1 
    sprintf('Optimas was probably not reached - both objectives could be improved')
    optimum_check=-1;
elseif (min_obj1(1)/best_obj(1,1))<1 ||(min_obj2(2)/best_obj(2,2))<1 
   sprintf('Optimas was probably not reached - one objective could be improved')
   optimum_check=0;
else
    sprintf('Optimas probably reached')
    optimum_check=1;
end


end

