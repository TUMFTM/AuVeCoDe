function [Result] = analyze_different_Parameters (Parameters, varargin)

    %% Description
    % Function to analyze input variables with different parameters values
    % Current status only for two optimization objectives
    
    % Author: Fabian Liemawan Adji
    % Date: September 2021
    
    %% Input:
    % Parameters: vehicle Parameters per default
    % varargin: input variables value for MAIN_AuVeCoDe
    %       -struct: e.g. veh with fields containing input variables in double:
    %               -veh.Pos_GM_GM1    -veh.Pos_GM_GM2     -veh.Pos_GM_GM3
    %               -veh.Pos_X_GM1     -veh.Pos_X_GM2      -veh.Pos_GM_X1
    %       -array: e.g. Positions with 6 x 22 double which represents the
    %       input variables
    
    %% Output
    % Result: 
    %       -struct: consisting fields of vehicle concepts with
    %       manipulated parameter values
    %       -array: containing fitness of concepts with the new parameter
    %       values
    
    % example of Parameters manipulation
    Parameters.timeframe = 2;
    Parameters.input.Gearboxlossmod = 1;
    
    % if input is a struct
    if isstruct(varargin{1})
        % get fieldnames in string
        FieldNames = string(fieldnames(varargin{1}));
        for i = 1:length(FieldNames)
            % get vehicle concept with manipulated Parameters
            veh.(FieldNames(i)) = MAIN_AuVeCoDe(Parameters, varargin{1}.(FieldNames(i)));
        end
        % save all vehicle concepts in Result
        Result = veh;
    else
        % preallocate cells for optimization objetives
        GoalFields = cell(2,1);
        for i = 1:2
            % split Parameters.optimization.goal at i
            GoalFields{i} = string(split(Parameters.optimization.("goal_"+i),"."));
            % get the fields suffix behind vehicle.
            GoalFields{i} = GoalFields{i}(2:end);
        end
        % get array containing input variables
        Positions = varargin{1};
        % preallocate fitness array
        Fitness = zeros(height(Positions), 2);
        for i = 1:height(Positions)
            % get concept with manipulated Parameters values
            veh_new = MAIN_AuVeCoDe(Parameters, Positions(i,:));
            for ii = 1:2
                % get the optimization goal ii for current concept
                Temp = veh_new;
                for iii = 1:length(GoalFields{ii})
                    % method to get inside the struct until finding the
                    % value of optimization goal ii of current concept
                    try % for cost if new concept infeasible
                        % if concept feasible
                        Temp = Temp.(GoalFields{ii}(iii));
                    catch
                        % if unfeasible then set at a million
                        Temp = 1e6;
                    end
                end
               % if element length after round higher than 2 (e.g. Costs
               % have around 3-5 element length) then round with 0 decimal,
               % else round with 2 decimals
               if numel(num2str(round(Temp))) > 2
                   Fitness(i,ii) = round(Temp);
               else
                   Fitness(i,ii) = round(Temp,2);
               end
            end
            % store Fitness of new generated concepts in Result
            Result = Fitness;
        end
    
end