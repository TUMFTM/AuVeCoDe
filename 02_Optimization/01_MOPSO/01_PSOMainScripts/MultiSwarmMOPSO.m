function [state] = MultiSwarmMOPSO(options, Parameters)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function to use the multiswarm MOPSO optimizer 
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - Algorithm options and vehicle Parameters
% ------------
% Output:   - Swarm state with all results
% ------------       

    
    %% Implementation
    
    %% Step 1: initialize the algorithmic options
    % get the number of particles for each sub-swarm
    numParticles = options.SwarmSize;
    
    % define the lower and upper bounds for all particles in matrix
    lbMatrix = repmat(options.lb',numParticles,1);
    ubMatrix = repmat(options.ub',numParticles,1);
    
    % derive the vector version of the lower and upper bounds
    lb = lbMatrix(1,:);
    ub = ubMatrix(1,:);
    
    % define the number of grids in each dimension for MOPSO
    ngrid = options.ngrid; % Number of grids in each dimension
    u_mut = options.uniform_mutation; % Uniform mutation percentage
    
    % check the validity of Constant variable Column
    if options.ConstVarCol < 1 || ~(floor(options.ConstVarCol)==options.ConstVarCol)
        error("Constant Variable Column must be a real number");
    end
    
    % set the constant variable column
    options.ConstVarCol = max(0, options.ConstVarCol);
    
    % Create initial state of the swarm
    state = makeState;
    % start the tic identifier for optimization time documentation
    state.StartTime = tic;
    
    % initialize optimization parameter for the stochastic search
    % c1 = c2 = 0. inertia_weight = inertia_weight_0;
    OptimParameter.cSelf = 0;
    OptimParameter.cSocial = 0;
    OptimParameter.adaptiveInertia = options.InertiaWeight0;

    % create a repository to store the history of the Paretos
    ParetoHistory = cell(options.MaxIter,1);
        
    % create a repository to store best particles from all subpopulations
    Repository.BestParticles = [];
    
    % initialize plotresult for plotting optimization results
    plotresult.pops     = repmat(struct, [options.MaxIter, 1]);     % each row is the population of one generation
    plotresult.states   = repmat(struct, [options.MaxIter, 1]);   % each row is the optimizaiton state of one generation
    plotresult.opt      = transform_options_MOPSO(options);     % use for output
    
    %% Step 2: Create sub-swarms for initial state
    for iSwarm = 1: options.SubSwarmNumbers
        % define sub-swarm name
        SwarmField = "SubSwarm" + string(iSwarm);
        % create the state of current sub swarm
        Swarm.(SwarmField) = create_subswarm(lb, ub, lbMatrix, ubMatrix, options, Parameters, options.ConstVarCol, iSwarm);
        
        % update state FunEval
        state.FunEval = state.FunEval + Swarm.(SwarmField).FunEval;
        
        % initialize constraints for MOPSO
        Swarm.(SwarmField).cons = zeros(numParticles,options.numCons);
        
        % find nondominated particles and store the solutions in a
        % Repository
        [~, Swarm.(SwarmField).Repository.Pareto] = ...
            collect_Pareto_solutions(Swarm.(SwarmField));
        
        % update hypercubes
        Swarm.(SwarmField).Repository = updateGrid(Swarm.(SwarmField).Repository,ngrid);
        
        % store the best particles of current subswarm in the global repository
        Repository.BestParticles = [Repository.BestParticles; Swarm.(SwarmField).Repository.Pareto];
        
    end
    
    % sort unique fitnesses to spare memory
    [~, RowsUniqueFitnesses] = unique(Repository.BestParticles.Fitness, "rows");
    Repository.BestParticles = Repository.BestParticles(RowsUniqueFitnesses, :);
    
    % collect the best paretos of all subswarms
    [Repository.BestParticles, Repository.Pareto] = ...
        collect_Pareto_solutions(Repository.BestParticles);

    % convert to table to find the best
    Repository.Pareto = Repository.Pareto(Repository.Pareto.rank == 1,:);
    
    % collect only unique fitnesses to spare memory
    ParetoHistory{state.Iteration+1} = unique(Repository.Pareto.Fitness, "rows");
    
    
    % Verbosity to show current optimization progress
    % create struct
    Verbosity = struct;
    % initialize waitbar
    Verbosity.Object = waitbar(0);
    % update message
    update_Verbosity(Verbosity, state, options);
    
    
    %% Step 3: Iterate until the maximal iteration
    
    % apply a counter for the Stochastic Search Iteration
    SS_counter = 0;
    
    % Run the main loop until maximal iteration reached
    while state.Iteration < options.MaxIter
        
        % set state FunEval to zero, because the FunEval in subswarms are
        % already cumulative
        state.FunEval = 0;
        
        % perform update for every subswarm
        for iSwarm = 1: options.SubSwarmNumbers
            % define sub-swarm name
            SwarmField = "SubSwarm" + string(iSwarm);
            
            % select swarm leader from subswarm repository using roulette 
            % wheel selection
            LeaderIndex = selectLeader(Swarm.(SwarmField).Repository);       
        
            % define r1 and r2
            randSelf = rand(options.SwarmSize, options.numVar);
            randSocial = rand(options.SwarmSize, options.numVar);
            
            % update velocities for MOPSO
            newVelocities = OptimParameter.adaptiveInertia* Swarm.(SwarmField).Velocities + ...
                OptimParameter.cSelf*randSelf.*(Swarm.(SwarmField).IndividualBestPositions-Swarm.(SwarmField).Positions) + ...
                OptimParameter.cSocial*randSocial.*...
                (repmat(Swarm.(SwarmField).Repository.Pareto.Positions(LeaderIndex,:),numParticles,1)-Swarm.(SwarmField).Positions);
            
            % check the validation of the new velocities
            tfValid = all(isfinite(newVelocities), 2);
            % only accept valid velocities
            Swarm.(SwarmField).Velocities(tfValid,:) = newVelocities(tfValid,:);

            % Update positions
            newPopulation = Swarm.(SwarmField).Positions + Swarm.(SwarmField).Velocities;

            % Perform mutation for MOPSO
            newPopulation = mutation(newPopulation, state.Iteration, options.MaxIter,...
                numParticles, ub, lb, size(lb,2), u_mut);
            
            % set the constant column values as constant for current subswarm
            newPopulation(:,options.ConstVarCol) = iSwarm;
            
            % check the validity of the new positions
            tfInvalid = ~isfinite(newPopulation);
            % only accept valid new positions
            newPopulation(tfInvalid) = Swarm.(SwarmField).Positions(tfInvalid);

            % Check the validity of the new positions by the bounds and
            % variable types
            Swarm.(SwarmField).Positions = checkswarm(newPopulation,...
                options.vartype,lbMatrix,ubMatrix);

            % evaluate fitness of all particles in subswarm
            Swarm.(SwarmField) = swarmfitness(Swarm.(SwarmField), Swarm.(SwarmField).Positions,...
            options, numParticles, Parameters);
        
            % update state FunEval
            state.FunEval = state.FunEval + Swarm.(SwarmField).FunEval;
            
            % update Repository
            % collect new nondominated solutions in a temporary Repository
            % NonDomRepository
            [~, NonDomRepository] = collect_Pareto_solutions(Swarm.(SwarmField));

            % add the new nondominated solutions into the pareto repository
            % of current subswarm
            Swarm.(SwarmField).Repository.Pareto = [Swarm.(SwarmField).Repository.Pareto; ...
                NonDomRepository];
            
            % sort unique fitnesses to spare memory
            [~, RowsUniqueFitnesses] = unique(Swarm.(SwarmField).Repository.Pareto, "rows");
            Swarm.(SwarmField).Repository.Pareto = Swarm.(SwarmField).Repository.Pareto...
                (RowsUniqueFitnesses, :);

            % collect the nondominated solutions from the expanded
            % repository of current subswarm
            [~, Swarm.(SwarmField).Repository.Pareto] = ...
                collect_Pareto_solutions(Swarm.(SwarmField).Repository.Pareto);

            % expand the global repository by the new founded non dominated
            % solutions
            Repository.BestParticles = [Repository.BestParticles; Swarm.(SwarmField).Repository.Pareto];
            
            
            % Update the best positions found so far for each particle
            % check which new fitness dominates old best fitness
            pos_best = dominates(Swarm.(SwarmField).Fitness, Swarm.(SwarmField).IndividualBestFitness);
            % check which old best fitness can't dominate new fitness
            best_pos = ~dominates(Swarm.(SwarmField).IndividualBestFitness, Swarm.(SwarmField).Fitness);
            % if random >= 0.5 then set best_pos of the position to zero
            best_pos(rand(numParticles,1)>=0.5) = 0;
            
            % if more pos_best found then replace best fitness & positions
            % by new fitness and positions
            if(sum(pos_best)>1)
                Swarm.(SwarmField).IndividualBestFitness(pos_best,:) = Swarm.(SwarmField).Fitness(pos_best,:);
                Swarm.(SwarmField).IndividualBestPositions(pos_best,:) = Swarm.(SwarmField).Positions(pos_best,:);
            end
            
            % if more best_pos found then replace best fitness & positions
            % by new fitness and positions
            if(sum(best_pos)>1)
                Swarm.(SwarmField).IndividualBestFitness(best_pos,:) = Swarm.(SwarmField).Fitness(best_pos,:);
                Swarm.(SwarmField).IndividualBestPositions(best_pos,:) = Swarm.(SwarmField).Positions(best_pos,:);
            end
        
            % update Grid in Repository
            Swarm.(SwarmField).Repository = updateGrid(Swarm.(SwarmField).Repository,ngrid);
        end
        
        % sort unique fitnesses to spare memory
        [~, RowsUniqueFitnesses] = unique(Repository.BestParticles.Fitness, "rows");
        Repository.BestParticles = Repository.BestParticles(RowsUniqueFitnesses, :);
                
        % collect the best paretos of all subswarms
        [Repository.BestParticles, Repository.Pareto] = ...
        collect_Pareto_solutions(Repository.BestParticles);
    
        % convert to table to find the best
        Repository.Pareto = Repository.Pareto(Repository.Pareto.rank == 1,:);

        % collect unique fitnesses into the historical repository
        ParetoHistory{state.Iteration+1} = unique(Repository.Pareto.Fitness, "rows");
    
        
        % apply adaptive update strategy
        if SS_counter <= options.SS
            % Stochastic Search:
            
            % increase the inertia_weight by predefined rate
            OptimParameter.adaptiveInertia = OptimParameter.adaptiveInertia...
                + options.InertiaWeightrateSS;
            
            % random velocities for maximum exploration
            for iSwarm = 1: options.SubSwarmNumbers
                % define subswarm name
                SwarmField = "SubSwarm" + string(iSwarm);
                % random subswarm velocities
                Swarm.(SwarmField).Velocities = random_velocity(...
                ubMatrix(1,:)-lbMatrix(1,:), options.numVar, options.SwarmSize);
            end
            
            % add SS_counter by one
            SS_counter = SS_counter  + 1;
            
            % if SS_counter after added exceeds maximal iteration of SS
            % then apply Local Search settings
            if SS_counter > options.SS
                % set adaptive inertia weigth to minimum
                OptimParameter.adaptiveInertia = options.InertiaWeightlb;
                % activate c1 and c2 for better local search in good found
                % regions
                OptimParameter.cSelf = options.SelfAdjustment;
                OptimParameter.cSocial = options.SocialAdjustment;
            end
        end
        
        % add iteration by one
        state.Iteration = state.Iteration + 1;
        
        % update message of Verbosity
        update_Verbosity(Verbosity, state, options);
    
        % sort BestParticles by rank for plotting    
        Repository.BestParticles = sortrows(Repository.BestParticles, "rank");
        % translate MOPSO data into plotresult for plotnsga
        [plotresult] = build_plotresult_MOPSO(plotresult, Repository.BestParticles, state);
        % plot pareto results for current iteration
        plotnsga(plotresult, state.Iteration);
        
        
    end
    %% Store Optimization results to be passed into ResultData in outer script.
    % close Verbosity
    close(Verbosity.Object);   
    
    % save total optimization time
    state.totaltime = toc(state.StartTime);
    
    % save Repository and historical repository
    result.Repository = Repository;
    result.ParetoHistory = ParetoHistory;
    % save results
    result.bestFitness =  Repository.Pareto.Fitness;
    result.bestPosition = Repository.Pareto.Positions;
    
    % store results in state
    state.result = result;
    
    % save Parameters and subswarm in state
    state.Parameters = OptimParameter;
    state.Swarms = Swarm;
    

end

function d = dominates(x,y)
    
    %% Description
    % Function that returns 1 if x dominates y and 0 otherwise
    
    % Author: Víctor Martínez-Cagigal
    d = all(x<=y,2) & any(x<y,2);
end

function POS = mutation(POS,gen,maxgen,Np,var_max,var_min,nVar,u_mut)
    
    %% Description
    % Function that performs the mutation of the particles depending on the
    % current generation

    % Author: Víctor Martínez-Cagigal
    
    % Sub-divide the swarm in three parts [2]
    fract     = Np/3 - floor(Np/3);
    if(fract<0.5)
        sub_sizes =[ceil(Np/3) round(Np/3) round(Np/3)];
    else
        sub_sizes =[round(Np/3) round(Np/3) floor(Np/3)];
    end
    cum_sizes = cumsum(sub_sizes);
    
    % First part: no mutation
    % Second part: uniform mutation
    nmut = round(u_mut*sub_sizes(2));
    if(nmut>0)
        idx = cum_sizes(1) + randperm(sub_sizes(2),nmut);
        POS(idx,:) = repmat((var_max-var_min),nmut,1).*rand(nmut,nVar) + repmat(var_min,nmut,1);
    end
    
    % Third part: non-uniform mutation
    per_mut = (1-gen/maxgen)^(5*nVar);     % Percentage of mutation
    nmut    = round(per_mut*sub_sizes(3));
    if(nmut>0)
        idx = cum_sizes(2) + randperm(sub_sizes(3),nmut);
        POS(idx,:) = repmat((var_max-var_min),nmut,1).*rand(nmut,nVar) + repmat(var_min,nmut,1);
    end
end
