function [options,Parameters] = MultiPopNSGAII_Options(Parameters)
%% Description:
% Designed by: Fabian Liemawan Adji, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 01.07.2021
% ------------
% Version: Matlab2020b
%-------------
% Description:  Function defines the optimization options for Multipopulation NSGA-II
% ------------
% Sources:  [1] Fabian Liemawan Adji, “Metaoptimierung von NSGA-II und MOPSO für Fahrzeugkonzeptoptimierung von autonomen elektrischen Fahrzeugen,” Technical University of Munich, Institute of Automotive Technology, 2021
%           [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - User input stored in Parameters struct
% ------------
% Output:   - Algorithm settings struct
% ------------

%% Implementation
    
    %% General Optimization Options
    % Initialize basic optimization options for NSGA-II
    options=nsgaopt();
    % Options Name
	options.Name='AuVeCoDe';
    % Number of optimization objectives
	options.numObj=2;  
    
    % options to use parallel pool for faster optimization
    if Parameters.optimization.parallel
        options.useParallel='yes';
        options.Cluster_Mode=1;
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
    
    % FolderName defines the name of the main folder where the results will
    % be saved
    options.FolderName = "Multipopulation_NSGA_II";
    % link to the Multipopulation NSGA-II optimization function
    options.func = @MultiPopNSGAII;
    % Name of the algorithm for the result
    options.Algo = "Multipopulation_NSGA_II";
    
    % Population size: number of individuals in every subpopulations
    options.popsize=Parameters.optimization.popsize;
    % maximal number of generation
    options.maxGen=Parameters.optimization.maxgen;                            
    
    % ConstVarCol (Constant Variable Column) defines which column will be
    % held constant in subpopulations.
    % e.g. ConstVarCol = 1 means motor topology will be held constant in
    % every subpopulations.
    options.ConstVarCol = 1; 
    % number of subpopulations
    options.SubPopNumbers = 8;

    % NSGA-II Crossover and Mutation Parameters best for AuVeCoDe[2]
    options.mutation = {'gaussian',0.1,0.2};         			% mutation operator (scale, shrink)
    options.crossover = {'intermediate',2};		   			% crossover operator (Ratio)
    options.crossoverFraction = 0.2;                			% crossover fraction of variables of an individual
    options.mutationFraction =  0.2;                   		% mutation fraction of variables of an individual
end

