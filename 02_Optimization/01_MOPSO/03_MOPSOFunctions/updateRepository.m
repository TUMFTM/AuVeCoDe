function Repository = updateRepository(Repository,POS,POS_fit,ngrid)
    
    %% Description:
    % Function that updates the repository given a new population and its
    % fitness
    
    % Author: Víctor Martínez-Cagigal
    
    %% Input:
    % Repository: repository containing non domination solutions
    % POS: Positions of particles
    % POS_fit: Fitness of particles
    % ngrid: Number of grids in each dimension for MOPSO
    
    %% Output:
    % Repository: updated Repository
    
    % Domination between particles
    DOMINATED  = checkDomination(POS_fit);
    Repository.pos    = [Repository.pos; POS(~DOMINATED,:)];
    Repository.pos_fit= [Repository.pos_fit; POS_fit(~DOMINATED,:)];
    % Domination between nondominated particles and the last repository
    DOMINATED  = checkDomination(Repository.pos_fit);
    Repository.pos_fit= Repository.pos_fit(~DOMINATED,:);
    Repository.pos    = Repository.pos(~DOMINATED,:);
    % Updating the grid
    Repository        = updateGrid(Repository,ngrid);
end