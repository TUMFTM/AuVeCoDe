function [options,Parameters] = MultiSwarmMOPSO_Options(Parameters)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function defines the optimization options for Multiswarm MOPSO
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - User input stored in Parameters struct
% ------------
% Output:   - Algorithm settings struct
% ------------    
    
	%% General Optimization Options
	
    % FolderName defines the name of the main folder where the results will
    % be saved
    options.FolderName = "Multiswarm_MOPSO";
    % Options Name
	options.Name='AuVeCoDe';
    % Number of optimization objectives
	options.numObj=2;       
    % Name of the algorithm for the result
    options.Algo = "Multiswarm_MOPSO";
    
    % options to use parallel pool for faster optimization
	if Parameters.optimization.parallel
		options.useParallel='yes';
		options.Cluster_Mode=1;
	else
		options.useParallel='no';
    end
    
    % initialize variables
	[Variable_Names,Variable_Type,lb,ub,A,b] = Opt_Init(Parameters);  
    
    % initialize lower bounds
	options.lb=lb;          
    % initialize upper bounds
	options.ub=ub;   
    % Correlation Matrix for constraints
	options.A=A;          
    % Limit Vector for constraints
	options.b=b;                             
    % number of constraints
	options.numCons=length(options.b);
    % variable names
	options.nameVar=Variable_Names;  
    % number of variables
	options.numVar=length(lb);      
    % variables type (1 float, 2 integer)
	options.vartype=Variable_Type';     
    % objectives names
	options.nameObj={Parameters.optimization.goal_1_name,Parameters.optimization.goal_2_name};
    % number of parallel workers
	options.poolsize=0;
    % link to objective function
	options.objfun=@Objfunction;
    % vectorized constraints
	options.vectorized = 'yes';                                                
	options.outputInterval=1;
    % Linear constraint matrix
	Parameters.LinCon.A=A;    
    % Linear constraint vector
	Parameters.LinCon.b=b;                                                     
	
	
	%% General PSO Optimization Options
	
    % Swarm size represents the number of particles as search agents in
    % every sub swarm
    options.SwarmSize = Parameters.optimization.popsize; 
    % maximal number of iteration
    options.MaxIter = Parameters.optimization.maxgen; 
    % Number of grids in each dimension for MOPSO
    options.ngrid = 15;
    % uniform mutation percentage for MOPSO
    options.uniform_mutation = 0.5;
    % ConstVarCol (Constant Variable Column) defines which column will be
    % held constant in subswarms.
    % e.g. ConstVarCol = 1 means motor topology will be held constant in
    % every sub swarm.
    options.ConstVarCol = 1; 
    % number of subswarms
    options.SubSwarmNumbers = 8;
    % link to the Multiswarm MOPSO optimization function
    options.func = @MultiSwarmMOPSO;
    
    %% Specific PSO Optimization Options
    % PSO with EE (Extended Exploration): expanding PSO into two phase.
    % Phase 1: Stochastic Search (SS): use mainly inertia weight and
    % always random velocity for a stochastic exploration. c1 = c2 = 0;
    % Phase 2: Local Search (LS): pull the particles to the best
    % found areas in SS by setting inertia to minimum and using c1
    % and c2. inertia_weight = inertia_weight_lb;
    
    % Parameters for Stochastic Search (Exploration):
    
    %stochastic search iteration
    options.SS = 10; 
    
    % inertia weight parameters
    % initial inertia weigth in the stochastic search
    options.InertiaWeight0 = 2;
    % lower bounds: minimal value of inertia weight in the stochastic
    % search and local search
    options.InertiaWeightlb = 0.001;
    % upper bounds: maximal value of inertia weight in the stochastic
    % search
    options.InertiaWeightub = 10;
    % define the increase/decrease rate of inertia weight in the stochastic
    % search
    % default: inertia weigth reaches upper bound in the last iteration of
    % stochastic search
    options.InertiaWeightrateSS = (options.InertiaWeightub - options.InertiaWeight0) /options.SS;

    % Parameters for Local Search (Exploitation):
    % c1 = self-adjustment
    options.SelfAdjustment = 2;
    % c2 = social-adjustment
    options.SocialAdjustment = 2;    
    
    

end



