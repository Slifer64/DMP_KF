clc;
close all;
clear;

set_matlab_utils_path();

N_demos = 1;
Ts = 0.02;
N_kernels = 7;
dt = 0.005;

Data = record_demo(N_demos, Ts, N_kernels, dt);

save('data/demo_data.mat','Data');