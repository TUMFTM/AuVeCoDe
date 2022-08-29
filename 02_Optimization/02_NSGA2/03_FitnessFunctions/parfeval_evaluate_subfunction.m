function [pop, allTime]=parfeval_evaluate_subfunction(i, opt, pop, state, c_log, varargin)
        
        N = length(pop);
        fprintf(c_log.Value, "%s | Generation %d Iteration %d started \n",datestr(datetime('now')),state.currentGen,i);
        fprintf(c_log.Value, num2str(pop(i).var));
        fprintf(c_log.Value, "\n");
%         fprintf('Evaluating the objective function... Generation: %d / %d , Individual: %d / %d \n', state.currentGen, opt.maxGen, i, N);
        
        cd(opt.cwd)
        cwd = pwd;
        t = getCurrentTask();
        tmpdir = 'temp'
        tmpdir = strcat(tmpdir, num2str(t.ID));
        addpath(cwd)
        if not(exist(tmpdir,'dir'))
            mkdir(tmpdir)
        end
        cd(tmpdir)

        asdf=tic;
        [pop(i), allTime(i)] = evalIndividual(pop(i), opt.objfun, state.currentGen, i, varargin{:});
        
        cd(cwd)
        rmdir(tmpdir,'s')
        rmpath(cwd)
        
%         fprintf('Evaluating the objective function    Generation: %d / %d , Individual: %d / %d took %d s \n', state.currentGen, opt.maxGen, i, N, round(toc(asdf))); 
        fprintf(c_log.Value,"%s | Generation %d Iteration %d ended\n",datestr(datetime('now')),state.currentGen,i);

end

function [indi, evalTime] = evalIndividual(indi, objfun, gen, ind, varargin)
% Function: [indi, evalTime] = evalIndividual(indi, objfun, varargin)
% Description: Evaluate one objective function.
%
%         LSSSSWC, NWPU
%    Revision: 1.1  Data: 2011-07-25
%*************************************************************************

tStart = tic;
[y, cons, extra] = objfun( indi.var, gen, ind, varargin{:} );
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
end
