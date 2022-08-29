function [state] = MOPSO(options, Parameters)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Runs the Multi-Objective Particle Swarm Optimization
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - algorithm options
%           - Parameters struct
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
    Repository.Pareto = [];
    
    % initialize plotresult for plotting optimization results
    plotresult.pops     = repmat(struct, [options.MaxIter, 1]);     % each row is the population of one generation
    plotresult.states   = repmat(struct, [options.MaxIter, 1]);   % each row is the optimizaiton state of one generation
    plotresult.opt      = transform_options_MOPSO(options);     % use for output
    
    % initialize constraints for MOPSO
    state.cons = zeros(numParticles,options.numCons);
    
    %% Step 2: Initialize swarm for initial state
    
    % generate random swarm positions
    state.Positions = random_SwarmPositions(ub, lb, ubMatrix,lbMatrix,...
        numParticles,options.numVar,options.vartype);
    
    % Enforce bounds
    if any(any(state.Positions < lbMatrix)) || any(any(state.Positions > ubMatrix))
        state.Positions = max(lbMatrix, state.Positions);
        state.Positions = min(ubMatrix, state.Positions);
    end
    
    % define maximum velocity as the difference between the upper and the
    % lower bounds
    vmax = ub-lb;
    % initialize velocity by random
    state.Velocities = random_velocity(vmax, options.numVar, numParticles);
    
    % calculate the particles' fitness of swarm
    state = swarmfitness(state, state.Positions,...
        options, numParticles, Parameters);
    
    % Initialize the individual best fitness and best position for all
    % particles in subswarm
    state.IndividualBestFitness = state.Fitness;
    state.IndividualBestPositions = state.Positions;  
    
    % find nondominated particles and store the solutions in a
    % Repository
    [~, Repository.Pareto] = ...
        collect_Pareto_solutions(state);
    
    % collect only unique fitnesses to spare memory
    ParetoHistory{state.Iteration + 1} = unique(Repository.Pareto.Fitness, "rows");
    
    % update hypercubes
    Repository = updateGrid(Repository,ngrid);
    
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
        
    % Run the main loop until some exit condition becomes true
    while state.Iteration < options.MaxIter
        
        % select swarm leader from subswarm repository using roulette 
        % wheel selection
        LeaderIndex = selectLeader(Repository);       
        
        % define r1 and r2
        randSelf = rand(options.SwarmSize, options.numVar);
        randSocial = rand(options.SwarmSize, options.numVar);
        
        % update velocities for MOPSO
        newVelocities = OptimParameter.adaptiveInertia* state.Velocities + ...
            OptimParameter.cSelf*randSelf.*(state.IndividualBestPositions-state.Positions) + ...
            OptimParameter.cSocial*randSocial.*...
            (repmat(Repository.Pareto.Positions(LeaderIndex,:),numParticles,1)-state.Positions);
        
        % check the validation of the new velocities
        tfValid = all(isfinite(newVelocities), 2);
        % only accept valid velocities
        state.Velocities(tfValid,:) = newVelocities(tfValid,:);
        
        % Update positions
        newPopulation = state.Positions + state.Velocities;
        
        % Perform mutation for MOPSO
        newPopulation = mutation(newPopulation, state.Iteration, options.MaxIter,...
            numParticles, ub, lb, size(lb,2), u_mut);
        
        % check the validity of the new positions
        tfInvalid = ~isfinite(newPopulation);
        % only accept valid new positions
        newPopulation(tfInvalid) = state.Positions(tfInvalid);
        
        % Check the validity of the new positions by the bounds and
        % variable types
        state.Positions = checkswarm(newPopulation,...
            options.vartype,lbMatrix,ubMatrix);
        
        % evaluate fitness of all particles in swarm
        state = swarmfitness(state, state.Positions,...
        options, numParticles, Parameters);

        % update Repository
        % collect new nondominated solutions in a temporary Repository
        % NonDomRepository
        [~, NonDomRepository] = collect_Pareto_solutions(state);

        % add the new nondominated solutions into the pareto repository
        % of current subswarm
        Repository.Pareto = [Repository.Pareto; NonDomRepository];
        
        % sort unique fitnesses to spare memory
        [~, RowsUniqueFitnesses] = unique(Repository.Pareto, "rows");
        Repository.Pareto = Repository.Pareto(RowsUniqueFitnesses, :);
        
        % collect the nondominated solutions from the expanded
        % repository of current subswarm
        [Repository.UncutPareto, Repository.Pareto] = ...
            collect_Pareto_solutions(Repository.Pareto);
        
        % collect only unique fitnesses to spare memory
        ParetoHistory{state.Iteration + 1} = unique(Repository.Pareto.Fitness, "rows");
        
        % Update the best positions found so far for each particle
        % check which new fitness dominates old best fitness
        pos_best = dominates(state.Fitness, state.IndividualBestFitness);
        % check which old best fitness can't dominate new fitness
        best_pos = ~dominates(state.IndividualBestFitness, state.Fitness);
        % if random >= 0.5 then set best_pos of the position to zero
        best_pos(rand(numParticles,1)>=0.5) = 0;
        
        % if more pos_best found then replace best fitness & positions
        % by new fitness and positions
        if(sum(pos_best)>1)
            state.IndividualBestFitness(pos_best,:) = state.Fitness(pos_best,:);
            state.IndividualBestPositions(pos_best,:) = state.Positions(pos_best,:);
        end
        
        % if more best_pos found then replace best fitness & positions
        % by new fitness and positions
        if(sum(best_pos)>1)
            state.IndividualBestFitness(best_pos,:) = state.Fitness(best_pos,:);
            state.IndividualBestPositions(best_pos,:) = state.Positions(best_pos,:);
        end
        
        % update Grid in Repository
        Repository = updateGrid(Repository,ngrid);
        
        % apply adaptive update strategy
        if SS_counter <= options.SS
            % Stochastic Search:
            
            % increase the inertia_weight by predefined rate
            OptimParameter.adaptiveInertia = OptimParameter.adaptiveInertia...
                + options.InertiaWeightrateSS;

            % random velocities for maximum exploration
            state.Velocities = random_velocity(...
                ubMatrix(1,:)-lbMatrix(1,:), options.numVar, options.SwarmSize);
            
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
        
        % sort Uncut Pareto by rows    
        Repository.UncutPareto = sortrows(Repository.UncutPareto, "rank");
        % translate MOPSO data into plotresult for plotnsga
        [plotresult] = build_plotresult_MOPSO(plotresult, Repository.UncutPareto, state);
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
    
    % store results in  state
    state.result = result;
    
    % save Parameters in state
    state.Parameters = Parameters;
    
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
