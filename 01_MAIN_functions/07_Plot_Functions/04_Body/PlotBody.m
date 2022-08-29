function PlotBody(axis,v)
%% Description:
% Designed by: Michael Mast, Adrian König (FTM, Technical University of Munich)
%-------------
% Created on: 18.11.2021
% ------------
% Version: Matlab2020b
%-------------
% Description: %This function plots the structure and silhouette
% ------------
% Sources:        [1] Michael Mast, “Karosseriemodellierung autonomer Elektrofahrzeuge,” Master Thesis, Technical University of Munich, Institute of Automotive Technology, 2022
%                 [2] Adrian König, "Methodik zur Auslegung von autonomen Fahrzeugkonzepten", Technical University of Munich, Institute of Automotive Technology, 2022
% ------------
% Input:    - veh:  struct with the vehicle parameters
%           - axis: figure where vehicle is plotted
% ------------
% Output:   - Plotted body structure and silhouette

%% Implementation
% 1) Plot front structure
% 2) Plot front silhouette
% 3) Plot rear structure
% 4) Plot rear silhouette

    %% 1) Plot front structure

            % longitudinal
            eu_l=[0,0,0];
            for l=1:3
            [F_long,V_long]=Object3d_Block(v.body.L_ori(l,:)',eu_l,v.body.L_dim(l,1,1),v.body.L_dim(l,2,1),v.body.L_dim(l,3,1));
                    patch(axis,'Faces', F_long, 'Vertices', V_long, 'FaceColor', [200 200 200]./255);
            end
            for lr=1:3
            [F_long,V_long]=Object3d_Block(v.body.L_ori_R(lr,:)',eu_l,v.body.L_dim_R(lr,1,1),v.body.L_dim_R(lr,2,1),v.body.L_dim_R(lr,3,1));
                    patch(axis,'Faces', F_long, 'Vertices', V_long, 'FaceColor', [200 200 200]./255);
            end
            %angled

            for a=1:3 %select 1:4 to plot connection to crossbeam
             eu_a=[0,v.body.A_ang(a,:)];
            [F_ang,V_ang]=Object3d_Block(v.body.A_ori(a,:)',eu_a,v.body.A_dim(a,1),v.body.A_dim(a,2),v.body.A_dim(a,3));
                    patch(axis,'Faces', F_ang, 'Vertices', V_ang, 'FaceColor', [150 150 150]./255);
            end
            for ar=1:3 %select 1:4 to plot connection to crossbeam
             eu_a_R=[0,v.body.A_ang_R(ar,:)];
            [F_ang,V_ang]=Object3d_Block(v.body.A_ori_R(ar,:)',eu_a_R,v.body.A_dim_R(ar,1),v.body.A_dim_R(ar,2),v.body.A_dim_R(ar,3));
                    patch(axis,'Faces', F_ang, 'Vertices', V_ang, 'FaceColor', [150 150 150]./255);
            end

            %crossmember
            for q=1:3
             eu_q=[0,v.body.Q_ang(q),0];
            [F_quer,V_quer]=Object3d_Block(v.body.Q_ori(q,:)',eu_q,v.body.Q_dim(q,1,1),v.body.Q_dim(q,2,1),v.body.Q_dim(q,3,1));
                    patch(axis,'Faces', F_quer, 'Vertices', V_quer, 'FaceColor', [100 100 100]./255);
            end
        %% 2) Plot front silhouette
        plot3(axis,v.body.p_pane_f(:,1),v.body.p_pane_f(:,2),v.body.p_pane_f(:,3),'-','Color','k');
        plot3(axis,v.body.p_hood_f(:,1),v.body.p_hood_f(:,2),v.body.p_hood_f(:,3),'-','Color','k');
        plot3(axis,v.body.p_grill_f(:,1),v.body.p_grill_f(:,2),v.body.p_grill_f(:,3),'-','Color','k');
%% Only possible if sight evaluation succesfull         
%         %plot sight points
%         plot3(axis,v.manikin.AS_point(1),v.manikin.AS_point(2),v.manikin.AS_point(3),'*')
%         plot3(axis,v.manikin.V_points(:,1),v.manikin.V_points(:,2),v.manikin.V_points(:,3),'o')
%         plot3(axis,v.manikin.SgRP(1),v.manikin.SgRP(2),v.manikin.SgRP(3),'*')
%         %plot sight lines
%         plot3(axis,[v.body.down(1),v.manikin.V_points(2,1)],[v.body.down(2),v.manikin.V_points(2,2)],[v.body.down(3),v.manikin.V_points(2,3)])
%         plot3(axis,[v.body.left(1),v.manikin.V_points(1,1)],[v.body.left(2),v.manikin.V_points(1,2)],[v.body.left(3),v.manikin.V_points(1,3)])
%         plot3(axis,[v.body.roof(1),v.manikin.V_points(1,1)],[v.body.roof(2),v.manikin.V_points(1,2)],[v.body.roof(3),v.manikin.V_points(1,3)])

        
    %% 3) Plot rear structure

            %longitudinal
            eu_l_re=[0,0,0];
            for l=2:3
            [F_long,V_long]=Object3d_Block(v.body.L_ori_re(l,:)',eu_l_re,v.body.L_dim_re(l,1,1),v.body.L_dim_re(l,2,1),v.body.L_dim_re(l,3,1));
                    patch(axis,'Faces', F_long, 'Vertices', V_long, 'FaceColor', [200 200 200]./255);
            end
            for lr=2:3
            [F_long,V_long]=Object3d_Block(v.body.L_ori_R_re(lr,:)',eu_l_re,v.body.L_dim_R_re(lr,1,1),v.body.L_dim_R(lr,2,1),v.body.L_dim_R_re(lr,3,1));
                    patch(axis,'Faces', F_long, 'Vertices', V_long, 'FaceColor', [200 200 200]./255);
            end
            %angled

            for a=1:3 %select 1:4 to plot connection to crossbeam
             eu_a_re=[0,v.body.A_ang_re(a,:)];
            [F_ang,V_ang]=Object3d_Block(v.body.A_ori_re(a,:)',eu_a_re,v.body.A_dim_re(a,1),v.body.A_dim_re(a,2),v.body.A_dim_re(a,3));
                    patch(axis,'Faces', F_ang, 'Vertices', V_ang, 'FaceColor', [150 150 150]./255);
            end
            for ar=1:3 %select 1:4 to plot connection to crossbeam
             eu_a_R_re=[0,v.body.A_ang_R_re(ar,:)];
            [F_ang,V_ang]=Object3d_Block(v.body.A_ori_R_re(ar,:)',eu_a_R_re,v.body.A_dim_R_re(ar,1),v.body.A_dim_R_re(ar,2),v.body.A_dim_R_re(ar,3));
                    patch(axis,'Faces', F_ang, 'Vertices', V_ang, 'FaceColor', [150 150 150]./255);
            end

            %crossmember
            for q=1:3
            eu_q_re=[0,v.body.Q_ang_re(q),0];
            [F_quer,V_quer]=Object3d_Block(v.body.Q_ori_re(q,:)',eu_q_re,v.body.Q_dim_re(q,1,1),v.body.Q_dim_re(q,2,1),v.body.Q_dim_re(q,3,1));
                    patch(axis,'Faces', F_quer, 'Vertices', V_quer, 'FaceColor', [100 100 100]./255);
            end
        %% 4) Plot rear silhouette
        plot3(axis,v.body.p_pane_r(:,1),v.body.p_pane_r(:,2),v.body.p_pane_r(:,3),'-','Color','k');
        plot3(axis,v.body.p_hood_r(:,1),v.body.p_hood_r(:,2),v.body.p_hood_r(:,3),'-','Color','k');
        plot3(axis,v.body.p_grill_r(:,1),v.body.p_grill_r(:,2),v.body.p_grill_r(:,3),'-','Color','k');
        for corner=1:4
        plot3(axis,[v.body.p_pane_f(corner,1);v.body.p_pane_r(corner,1)],[v.body.p_pane_f(corner,2);v.body.p_pane_r(corner,2)],[v.body.p_pane_f(corner,3);v.body.p_pane_r(corner,3)],'-','Color','k');%roof and window sill
        end


end
