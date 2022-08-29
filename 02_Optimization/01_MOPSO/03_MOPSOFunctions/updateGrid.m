function Repository = updateGrid(Repository,ngrid)

    %% Description
    % Function that updates the hypercube grid, the hypercube where belongs
    % each particle and its quality based on the number of particles inside it
    
    % Author: Víctor Martínez-Cagigal
    
    %% Input:
    % Repository: repository containing non domination solutions
    % ngrid: Number of grids in each dimension for MOPSO
    
    %% Output:
    % Repository: updated Repository
    
    % Computing the limits of each hypercube
    ndim = size(Repository.Pareto.Fitness,2);
    Repository.hypercube_limits = zeros(ngrid+1,ndim);
    for dim = 1:1:ndim
        Repository.hypercube_limits(:,dim) = linspace(min(Repository.Pareto.Fitness(:,dim)),max(Repository.Pareto.Fitness(:,dim)),ngrid+1)';
    end

    
    % Computing where belongs each particle
    npar = size(Repository.Pareto.Fitness,1);
    Repository.grid_idx = zeros(npar,1);
    Repository.grid_subidx = zeros(npar,ndim);
    for n = 1:1:npar
        idnames = [];
        for d = 1:1:ndim
            Repository.grid_subidx(n,d) = find(Repository.Pareto.Fitness(n,d)<=Repository.hypercube_limits(:,d)',1,'first')-1;
            if(Repository.grid_subidx(n,d)==0), Repository.grid_subidx(n,d) = 1; end
            idnames = [idnames ',' num2str(Repository.grid_subidx(n,d))];
        end
        Repository.grid_idx(n) = eval(['sub2ind(ngrid.*ones(1,ndim)' idnames ');']);
    end
    
    % Quality based on the number of Pareto in each hypercube
    quality = zeros(ngrid,2);
    ids = unique(Repository.grid_idx);
    for i = 1:length(ids)
        quality(i,1) = ids(i);  % First, the hypercube's identifier
        quality(i,2) = 10/sum(Repository.grid_idx==ids(i)); % Next, its quality
    end
    Repository.quality = quality(1:length(ids), :);
end