function [vehicle] = Package_Interior(vehicle)
%% Description:
% Designed by: Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 11.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: This function calculates the interior boundary surface according to the interior settings
% ------------
% Sources:  [1] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters (Number of Seats (per row, total); Interior type (vis-a-vis, conv., ...); Measurements of interior)
%           - Par:  struct with fixed parameters
% ------------
% Output:   - veh:    updated struct with boundary surfaces of interior
% ------------

%% Calculate Boundary lines
%int_boundary_f/r: Boundary line, defined by seat
%int_boundary_f2/r2: Boundary line, defined by legroom
z_f=0:1:(vehicle.interior.int_z_tot(1));
z_r=0:1:(vehicle.interior.int_z_tot(2));

switch vehicle.Input.int_type %(vav=vis-a-vis, btb=back-to-back, con=conventional, sr=singlerow)
    case 'vav'
        %Calculation of the boundary line of the front row
        vehicle.interior.int_boundary_f(z_f>(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1))& z_f<=vehicle.interior.int_z_tot(1))=0;
        vehicle.interior.int_boundary_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))=-(vehicle.interior.int_x_br(1)/vehicle.interior.int_z_br(1))*(z_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))-(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)));
        vehicle.interior.int_boundary_f(z_f<=vehicle.Input.int_z_leg(1) & z_f>=0)=vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        %for vis-a-vis f2=f (leg room points to the middle)
        vehicle.interior.int_boundary_f2=vehicle.interior.int_boundary_f;
        
        %Calculation of the boundary line of the rear row (calculate, mirror and move)
        vehicle.interior.int_boundary_r(z_r>(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2))& z_r<=vehicle.interior.int_z_tot(2))=0;
        vehicle.interior.int_boundary_r(z_r<=(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)) & z_r>vehicle.Input.int_z_leg(2))=-(vehicle.interior.int_x_br(2)/vehicle.interior.int_z_br(2))*(z_r(z_r<=(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)) & z_r>vehicle.Input.int_z_leg(2))-(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)));
        vehicle.interior.int_boundary_r(z_r<=vehicle.Input.int_z_leg(2) & z_r>=0)=vehicle.interior.int_x_br(2)+vehicle.Input.int_x_seat(2);
        vehicle.interior.int_boundary_r=vehicle.interior.int_boundary_r.*-1+(vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1)+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(2)+vehicle.Input.int_x_seat(2)+vehicle.Input.int_x_leg(2))-vehicle.Input.int_x_overlap;
        %for vis-a-vis r2=r (leg room points to the middle)
        vehicle.interior.int_boundary_r2=vehicle.interior.int_boundary_r;
        
    case 'btb'
        %Calculation of the boundary line of the front row   
        vehicle.interior.int_boundary_f(z_f>(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1))& z_f<=vehicle.interior.int_z_tot(1))=0;
        vehicle.interior.int_boundary_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))=-(vehicle.interior.int_x_br(1)/vehicle.interior.int_z_br(1))*(z_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))-(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)));
        vehicle.interior.int_boundary_f(z_f<=vehicle.Input.int_z_leg(1) & z_f>=0)=vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        vehicle.interior.int_boundary_f=(vehicle.interior.int_boundary_f.*-1)+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        
        vehicle.interior.int_boundary_f2(z_f>=0 & z_f<=vehicle.interior.int_z_tot(1))=0;
         
        vehicle.interior.int_boundary_r(z_r>(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2))& z_r<=vehicle.interior.int_z_tot(2))=0;
        vehicle.interior.int_boundary_r(z_r<=(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)) & z_r>vehicle.Input.int_z_leg(2))=-(vehicle.interior.int_x_br(2)/vehicle.interior.int_z_br(2))*(z_r(z_r<=(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)) & z_r>vehicle.Input.int_z_leg(2))-(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)));
        vehicle.interior.int_boundary_r(z_r<=vehicle.Input.int_z_leg(2) & z_r>=0)=vehicle.interior.int_x_br(2)+vehicle.Input.int_x_seat(2);
        vehicle.interior.int_boundary_r=vehicle.interior.int_boundary_r+vehicle.Input.int_b2b_walldistance+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        
        vehicle.interior.int_boundary_r2(z_r>=0 & z_r<=vehicle.interior.int_z_tot(2))=vehicle.interior.int_x_br(2)+vehicle.Input.int_x_seat(2)+vehicle.Input.int_x_leg(2);
        vehicle.interior.int_boundary_r2=vehicle.interior.int_boundary_r2+vehicle.Input.int_b2b_walldistance+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        
    case 'con'
        %Calculation of the boundary line of the front row
        vehicle.interior.int_boundary_f(z_f>(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1))& z_f<=vehicle.interior.int_z_tot(1))=0;
        vehicle.interior.int_boundary_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))=-(vehicle.interior.int_x_br(1)/vehicle.interior.int_z_br(1))*(z_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))-(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)));
        vehicle.interior.int_boundary_f(z_f<=vehicle.Input.int_z_leg(1) & z_f>=0)=vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        vehicle.interior.int_boundary_f=(vehicle.interior.int_boundary_f.*-1)+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        
        vehicle.interior.int_boundary_f2(z_f>=0 & z_f<=vehicle.interior.int_z_tot(1))=0;
        
        %Calculation of the boundary line of the rear row
        vehicle.interior.int_boundary_r(z_r>(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2))& z_r<=vehicle.interior.int_z_tot(2))=0;
        vehicle.interior.int_boundary_r(z_r<=(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)) & z_r>vehicle.Input.int_z_leg(2))=-(vehicle.interior.int_x_br(2)/vehicle.interior.int_z_br(2))*(z_r(z_r<=(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)) & z_r>vehicle.Input.int_z_leg(2))-(vehicle.Input.int_z_leg(2)+vehicle.interior.int_z_br(2)));
        vehicle.interior.int_boundary_r(z_r<=vehicle.Input.int_z_leg(2) & z_r>=0)=vehicle.interior.int_x_br(2)+vehicle.Input.int_x_seat(2);
        vehicle.interior.int_boundary_r=vehicle.interior.int_boundary_r.*-1+(vehicle.interior.int_x_br(2)+vehicle.Input.int_x_seat(2)+vehicle.Input.int_x_leg(2)-vehicle.Input.int_x_overlap)+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        %for conventional r2=r (leg room points to the middle)
        vehicle.interior.int_boundary_r2=vehicle.interior.int_boundary_r;
        
    case 'sr'
        %Calculation of the boundary line of the front row
        vehicle.interior.int_boundary_f(z_f>(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1))& z_f<=vehicle.interior.int_z_tot(1))=0;
        vehicle.interior.int_boundary_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))=-(vehicle.interior.int_x_br(1)/vehicle.interior.int_z_br(1))*(z_f(z_f<=(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)) & z_f>vehicle.Input.int_z_leg(1))-(vehicle.Input.int_z_leg(1)+vehicle.interior.int_z_br(1)));
        vehicle.interior.int_boundary_f(z_f<=vehicle.Input.int_z_leg(1) & z_f>=0)=vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        vehicle.interior.int_boundary_f=(vehicle.interior.int_boundary_f.*-1)+vehicle.Input.int_x_leg(1)+vehicle.interior.int_x_br(1)+vehicle.Input.int_x_seat(1);
        
        vehicle.interior.int_boundary_f2(z_f>=0 & z_f<=vehicle.interior.int_z_tot(1))=0;
        
        %for single row r2=f=r (no second row, leg room points to the middle)
        vehicle.interior.int_boundary_r=vehicle.interior.int_boundary_f;
        vehicle.interior.int_boundary_r2=vehicle.interior.int_boundary_r;
        
end

%% Optional Plot
% Create 3D Surface over width
        %plot 2D Boundary Surface
%         figure
%         hold on
%         plot(vehicle.interior.int_boundary_f,z_f)
%         plot(vehicle.interior.int_boundary_r,z_r)
%         axis([0 2200 0 max(length(z_f),length(z_r))])
%         hold off
%         %Plot 3D figures
%         figure
%         hold on
%         surf([vehicle.interior.int_boundary_f;vehicle.interior.int_boundary_f],[-0.5*vehicle.interior.int_y_tot(1)*ones(size(vehicle.interior.int_boundary_f));0.5*vehicle.interior.int_y_tot(1)*ones(size(vehicle.interior.int_boundary_f))],[z_f;z_f])
%         if strcmp(Parameters.input.seating_layout,'sr')
%             surf([vehicle.interior.int_boundary_r;vehicle.interior.int_boundary_r],[-0.5*vehicle.interior.int_y_tot(2)*ones(size(vehicle.interior.int_boundary_r));0.5*vehicle.interior.int_y_tot(2)*ones(size(vehicle.interior.int_boundary_r))],[z_f;z_f])
%         else
%             surf([vehicle.interior.int_boundary_r;vehicle.interior.int_boundary_r],[-0.5*vehicle.interior.int_y_tot(2)*ones(size(vehicle.interior.int_boundary_r));0.5*vehicle.interior.int_y_tot(2)*ones(size(vehicle.interior.int_boundary_r))],[z_r;z_r])
%         end
%         hold off
%         axis([0 3000 -1000 1000 0 3000])
%         view(3)

end



