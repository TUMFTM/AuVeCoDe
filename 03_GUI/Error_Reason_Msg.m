function [Msg] = Error_Reason_Msg(v)
%% Description:
% Designed by:  Adrian König (FTM, Technical University of Munich),Daniel Telschow (Technical University of Munich)
%-------------
% Created on: 01.12.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: Function relates the error code to an understandable error output message in order to output in the userform
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - vehicle
% ------------
% Output:   - Error-Msg Type
%
% ------------

%% Implementation
switch v.Error
    case 2
        Msg='Near-axle configuration has to be modelled as coaxial and 1-speed.';    
    case 3
        Msg='Interior height is lower than required ground clearance';
    case 4
        Msg='Maximal battery capacity does not achieve the required range.';
    case 5
        Msg='Required rim diameter larger than defined tire diameter.';
    case 6
        Msg='Max. Motortorque outside of scaling range.';
    case 7
        Msg='Side Cooler not yet implemented.';
    case 8
        Msg='Cannot compute two identical gear ratios and two-speed gearbox.';
    case 9
        Msg='Interior height under ground clearance';
    case 10
        Msg='Required acceleration time could not be achieved.';
    case 11
        Msg='Required component space could not be achieved ';
    case 12
        Msg='Trunk volume to high, trunk is not feasible';
    case 13
        Msg='Windows are too flat, steeper angle required';
    case 14
        Msg='Seat height is lower then adjustment and thickness!';
    case 99
        try
            Msg=v.Error_Detail;
        catch
            Msg='Loop Iteration Counter = Max. Iteration ';
        end
end
end