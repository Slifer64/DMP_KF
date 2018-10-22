clc;
close all;
clear;

%% load and process training data
disp('Processing training data...');
ind = [1];
v_start = 0.01;
v_stop = 0.01;
processTrainingData(ind, v_start, v_stop);

pause_program();
clear_all();

%% train DMP
disp('Training dmp...');
train_DMP();

pause_program();
clear_all();

%% test scaling
disp('Running DMP with EKF...');
sim_DMP_srff_EKF_disc();

pause_program();
clear_all();
