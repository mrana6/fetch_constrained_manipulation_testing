close all; clear; clc;

%%

arm_origin =  [0.03265, 0, 0.72601]';
arm_model = generateFetchArm(arm_origin);

%%

disp('FK for zero conf:');
arm_model.fk_model().forwardKinematicsPose(zeros(7,1))
