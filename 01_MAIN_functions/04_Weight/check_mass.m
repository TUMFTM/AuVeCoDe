function check_mass(m)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich), Lorenzo Nicoletti (FTM, Technical University of Munich)
%-------------
% Created on: 01.04.2022
% ------------
% Version: Matlab2020b
%-------------
% Description:  The function checks that the input 
%               is greater than zero to ensure that there are not errors in the
%               inputs or in the regressions or find limits of the regression models.
% ------------
% Input:    - mass of component
% ------------
% Output:   - Error message if applicable
% ------------

%% Implementation
s=inputname(1);

if m<0
    error([ 'The value of ''' s ''' is negative. Please check for errors in the inputs or in the mass calculation'])
end

if ~isnumeric(m) || isnan(m)
    error([ 'The value of ''' s ''' is not a number. Please check for errors in the inputs or in the mass calculation'])
end


end

