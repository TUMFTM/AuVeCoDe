function Leader = selectLeader(Repository)

    %% Description
    % Function that selects the leader performing a roulette wheel selection
    % based on the quality of each hypercube
    
    % Author: Víctor Martínez-Cagigal
    
    %% Input:
    % Repository: repository containing non domination solutions
    
    %% Output:
    % Leader: Leader of current swarm/subswarm
    
    % Roulette wheel
    prob    = cumsum(Repository.quality(:,2));     % Cumulated probs
    sel_hyp = Repository.quality(find(rand(1,1)*max(prob)<=prob,1,'first'),1); % Selected hypercube
    
    % Select the index leader as a random selection inside that hypercube
    idx      = 1:1:length(Repository.grid_idx);
    Leader = idx(Repository.grid_idx==sel_hyp);
    Leader = Leader(randi(length(Leader)));
end