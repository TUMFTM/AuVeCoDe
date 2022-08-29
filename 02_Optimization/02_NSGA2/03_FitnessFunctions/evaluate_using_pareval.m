function [pop, state] = evaluate_using_pareval(opt, pop, state, varargin)
% Function: [pop, state] = evaluate(opt, pop, state, varargin)
% Description: Evaluate the objective functions of each individual in the
%   population.
%
%         LSSSSWC, NWPU
%    Revision: 1.0  Data: 2011-04-20
%*************************************************************************

N = length(pop);
allTime = zeros(N, 1);  % allTime : use to calculate average evaluation times

%*************************************************************************
% Evaluate objective function in parallel
%*************************************************************************
if( strcmpi(opt.useParallel, 'yes') == 1 )
    
    var_new = gcp('nocreate');
    
    if isempty(var_new)
            curPoolsize = 0;
        else
            curPoolsize = var_new.NumWorkers;
    end

    % There isn't opened worker process
    if(curPoolsize == 0)
        if(opt.poolsize == 0)
            parpool local
            p=gcp;
        else
            if opt.Cluster_Mode==1
                p=parpool(opt.c, opt.poolsize);
            else
                p=parpool(opt.poolsize);
            end
        end
    % Close and recreate worker process
    else
        if(opt.poolsize ~= curPoolsize)
            delete(gcp('nocreate'))
            if opt.Cluster_Mode==1
                p=parpool(opt.c, opt.poolsize);
            else
                parpool local
                p=gcp;
            end
        else
            p=gcp;
        end
    end
    
    %Bugfix-Logging von Mathworks-Support
    basePath = opt.c.JobStorageLocation;
    c_log = parallel.pool.Constant(@()setupLoggingOnWorkers(basePath),@fclose);
    
    %Working-folder abspeichern
    opt.cwd = pwd;
    
     for i = 1:N
        asdf(i)=tic;
        fprintf('Evaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        feval_objects(i) = parfeval(p, @parfeval_evaluate_subfunction, 2, i, opt, pop, state, c_log, varargin{:});
     end
     
    for idx = 1:N
        % fetchNext blocks until next results are available.
        timeout = 800; %seconds
        [fetchout.completedIdx,fetchout.value, fetchout.value2] = fetchNext(feval_objects,timeout); %fetch will be aborted after 'timeout' seconds
        fprintf('Evaluating the objective function    Generation: %d / %d , Individual: %d / %d took %d s \n', state.currentGen, opt.maxGen, fetchout.completedIdx, N, round(toc(asdf(i))));
    end
    
    id_finished = find(strcmp({feval_objects.State},'finished'));
    id_not_finished = find(strcmp({feval_objects.State},'running'));
    
    for i=1:N
        if strcmp({feval_objects(1,i).State},'finished');
            pop(i) =        feval_objects(1, i).OutputArguments{1,1}(i);
            allTime(i) =    feval_objects(1, i).OutputArguments{1,2}(i);            
        elseif strcmp({feval_objects(1,i).State},'running');
            % Fehlerdaten abspeichern
            hanging_input=feval_objects(1, i).InputArguments;
            filename = ['Debugging/error_haenger_detected_', datestr(datetime('now'),'yyyy_mm_dd_HHMM')];
            save(filename);
            % dummy-werte einsetztn
            pop(i) =        feval_objects(1, id_finished(1)).OutputArguments{1,1}(id_finished(1));
            allTime(i) =    feval_objects(1, id_finished(1)).OutputArguments{1,2}(id_finished(1));
            pop(i).var =    feval_objects(1, i).InputArguments{1, 3}(i).var;
            pop(i).cons =   1;       
            fprintf('hang detected in Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        end
    end
    
    %After fetching results, cancel the hanging (running) execution     
    id = strcmp({feval_objects.State},'running');
    feval_objects(id).cancel
            
     
%*************************************************************************
% Evaluate objective function in serial
%*************************************************************************
else
    for i = 1:N
        cwd = pwd;
        % t = getCurrentTask();
        tmpdir = 'temp2';
        % tmpdir = strcat(tmpdir, num2str(t.ID));
        
        addpath(cwd)
        % tmpdir = tempname;
        if not(exist(tmpdir,'dir'))
            mkdir(tmpdir)
        end
        cd(tmpdir)
%         disp(pwd)
        
        fprintf('\nEvaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        asdf=tic;
        [pop(i), allTime(i)] = evalIndividual(pop(i), opt.objfun, state.currentGen, i, varargin{:});
        fprintf('\nEvaluating the objective function    Generation: %d / %d , Individual: %d / %d took %d s \n', state.currentGen, opt.maxGen, i, N, round(toc(asdf)));        cd(cwd)
%         rmdir(tmpdir)
        rmpath(cwd)
    end
end

%*************************************************************************
% Statistics
%*************************************************************************
state.avgEvalTime   = sum(allTime) / length(allTime);
state.evaluateCount = state.evaluateCount + length(pop);
    for i=1:N
        state.nViol = state.nViol+pop(i).nViol;
    end




function [indi, evalTime] = evalIndividual(indi, objfun, gen, varargin)
% Function: [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Description: Evaluate one objective function.
%
%         LSSSSWC, NWPU
%    Revision: 1.1  Data: 2011-07-25
%*************************************************************************

tStart = tic;
[y, cons, extra] = objfun( indi.var, gen, varargin{:} );
evalTime = toc(tStart);

% Save the objective values and constraint violations
indi.obj = y;
indi.Verbrauch = extra{1};
indi.Bewertungswerte = extra{2};
indi.Bewertungsnoten = extra{3};
indi.cons=cons;
if( ~isempty(indi.cons) )
    idx = find( cons );
    if( ~isempty(idx) )
        indi.nViol = length(idx);
        indi.violSum = sum( abs(cons) );
    else
        indi.nViol = 0;
        indi.violSum = 0;
    end
end


