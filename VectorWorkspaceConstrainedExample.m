clear; close all; clc;

import gtsam.*
import gpmp2.*

addpath(genpath('util'));

%% Problem Setup

% rotation to fix align gpmp2 with actual robot
q_correction = [0.5000   -0.5000    0.5000   -0.5000];

% arm info
arm_name = 'JACO2_7';
dof = 7;   % degrees of freedom

% cartesian constraints info
cart_file_path = 'data/workspace/traj2.txt';
traj_cart = load(cart_file_path); % load cartesian trajectory
num_pts = size(traj_cart,1);   % number of cartesian constraints

% initialization configurations 
% start_conf =  [-2.4248465763502187,1.666398581192087,0.4162146405579079,0.5865603214228519,0.635006139111757,2.536397411828154,-0.18952164797494242]';%[-1.0, 2.2, 0.53, -0.4, -5.0, 1.2, 0.2]';
% end_conf = [0.320874098378337,2.0759001167948945,-0.13459777346347007,0.7503480781098963,-0.24743206495916928,1.5025751787597466,1.4562600094699716]';

% start_conf = [-1.8299938654935133, 1.7499940643239729, -0.6500227271703558, 1.3099082929660426, 0.2600128284268717, 4.479913410603431, -0.8200239873213118]';
% end_conf =  [-2.4248465763502187,1.666398581192087,0.4162146405579079,0.5865603214228519,0.635006139111757,2.536397411828154,-0.18952164797494242]';%[-1.0, 2.2, 0.53, -0.4, -5.0, 1.2, 0.2]';
% start_conf= [4.464453476588275, 1.6691604479730822, 2.468425691166573, 4.921172287731413, 3.5821319621623737, 4.609402410502454, 2.3594201109046486]';
% end_conf = [4.4663163577707845, 2.563444083067292, 2.7552374879237465, 4.212270763485401, 3.550721042369325, 4.3032973432282, 2.4455557953055176]';

start_conf = [3.68832551063 2.92795999621 2.84754185236 4.60666095248 5.44851386993 4.68181322791 2.97121479232]';
end_conf = [3.28416645955 2.1060401307 2.64423983216 4.41771577228 4.51510528407 5.19048074673 2.12904533694]';

% start_conf = [3.7159832375 3.03765666678 2.1303566987 4.01823337909 5.25400404076 5.0563847047 2.52634944488]';
% end_conf = [2.99085990513 2.28935588801 2.34732976998 4.35067821862 5.01780896049 5.52897853711 2.23009761246]';

% 
% start_conf= [4.46609797856 1.64847900474 2.63214405181 5.04885541567 3.5680355842 4.48530016897 2.32341523823]';
% end_conf = [4.53053814044 2.68652484257 2.66635343175 3.92658543581 3.59097511518 4.4579247991 2.50756555985]';

% smoothness param
Qc = 1;

% constraint params
fix_pose_sigma = 0.0001;
fix_vel_sigma = 0.0001;
fix_cartpos_sigma = 0.0001;
fix_cartorient_sigma = 0.001;

% timing information
total_time_step = num_pts-1;
total_time_sec = 10;
total_check_step = 100;

% obtsacle avoaidance params
cost_sigma = 0.02;
epsilon_dist = 0.01;

% obstacle environmen information
obs_info = [0.8, 0, 0.73, 0.5, 1.5, 0.03; ...
    0.15, 0, 1, 0.3, 0.8, 2.0];

% obs_info = [];
% generate sdf
[obs_sdf, obs_dataset] = setupEnvironment3D(obs_info);

%% Generate Arm Model
arm_origin =  [0.289, 0.000, 1.078]';
arm_model = generateVectorArm(arm_origin, arm_name);

start_conf(1) = -start_conf(1);
end_conf(1) = -end_conf(1);
start_vel = zeros(7,1);
end_vel = zeros(7,1);

%% Plot planning problem

% plot environment and robot
h = figure(1); clf(1);
set(h, 'Position', [-1200, 100, 1100, 1200]);
hold on; axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
% plotSignedDistanceField3D(field, origin, cell_size);
hcp = plotRobotModel(arm_model, end_conf);
plotArm(arm_model.fk_model(), end_conf, 'b', 2);
grid on;
view(10,60);
zoom(1);
plotEnvironment(obs_info);
axis([-1 1 -1 1 0 2.5]);
hold off;

%% setting up the problem

delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% GP
Qc_model = noiseModel.Gaussian.Covariance(Qc*eye(dof)); 

pose_fix_model = noiseModel.Isotropic.Sigma(dof, fix_pose_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(dof, fix_vel_sigma);

fix_cartpos_model = noiseModel.Isotropic.Sigma(3, fix_cartpos_sigma);
fix_cartorient_model = noiseModel.Isotropic.Sigma(3, fix_cartorient_sigma);

% plot settings
plot_inter_traj =false;
plot_inter = 10;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;

%% initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

%% init optimization
graph = NonlinearFactorGraph;
graph_obs = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix_model));
    elseif i==total_time_step
        %graph.add(PriorFactorVector(key_pos, end_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix_model));
    else
        
    end
    
    pos = traj_cart(i+1,1:3);
    orient = [traj_cart(i+1,7), traj_cart(i+1,4), traj_cart(i+1,5),traj_cart(i+1,6)];
    orient_corrected = quatmultiply(orient, q_correction);
    % Workspace constraints
    graph.add(GaussianPriorWorkspacePositionArm(key_pos, arm_model, dof-1, Point3(pos'), fix_cartpos_model));
    graph.add(GaussianPriorWorkspaceOrientationArm(key_pos, arm_model, dof-1, Rot3.Quaternion(orient_corrected(1), orient_corrected(2), orient_corrected(3), orient_corrected(4)), fix_cartorient_model));
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        % cost factor
        graph.add(ObstacleSDFFactorArm(...
            key_pos, arm_model, obs_sdf, cost_sigma, epsilon_dist));
        graph_obs.add(ObstacleSDFFactorArm(...
            key_pos, arm_model, obs_sdf, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm_model, obs_sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                graph_obs.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm_model, obs_sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end


%% optimize!
use_LM = true;
use_trustregion_opt = false;

if use_LM
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
    parameters.setVerbosityLM('LAMBDA');
%   parameters.setlambdaUpperBound(1e10);
    parameters.setlambdaInitial(1200.0);
    parameters.setMaxIterations(500);
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
elseif use_trustregion_opt
    parameters = DoglegParams;
    parameters.setMaxIterations(1000);
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

fprintf('Initial Error = %d\n', graph.error(init_values))
fprintf('Initial Collision Cost: %d\n', graph_obs.error(init_values))

optimizer.optimize();

result = optimizer.values();
% result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', graph_obs.error(result))

%% plot results
if plot_inter_traj
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter);
else
    plot_values = result;
end

% plot final values
% figure(4),
% clf(4), hold on
% title('Result Values')
% plot world
% plotEnvironment(obs_info);
% for i=0:total_plot_step
%     % plot arm
%     conf = plot_values.atVector(symbol('x', i));
%     plotArm(arm_model.fk_model(), conf, 'b', 2);
%     % plot config
%     set3DPlotRange(obs_dataset);
%     grid on, view(50, 80);
%     pause(pause_time)
% end
% hold off

% plot final values

% output_traj = [];
% output_cart_traj = [];
% h = figure(5), hold on; set(h, 'Position', [-1200, 100, 1100, 1200]);
% fk_model = arm_model.fk_model;
% for i=0:total_plot_step
%     conf = plot_values.atVector(symbol('x', i));
%     output_traj = [output_traj; conf'];
%     fk = fk_model.forwardKinematicsPose(conf);
%     output_cart_traj =  [output_cart_traj; fk(:,end).'];
% end


% 
% 
output_traj = [];
output_cart_traj = [];
h = figure(5), hold on; set(h, 'Position', [-1200, 100, 1100, 1200]);
for i=0:total_plot_step
    clf(5)
    hold on, view(27, 1)
    title('Result Values')
    % plot world
    plotEnvironment(obs_info);
    % plot arm
    conf = plot_values.atVector(symbol('x', i));
    output_traj = [output_traj; conf'];
    plotRobotModel(arm_model, conf);
    % plot config
    set3DPlotRange(obs_dataset);
    grid on;
    pause(pause_time);
end
hold off

%% save SDF
disp('saving sdf to .bin file...');
obs_sdf.saveSDF('vectorTableDataset.bin'); 