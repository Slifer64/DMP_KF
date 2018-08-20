clc;
close all;
clear;

set_matlab_utils_path();

N_demos = 1;
Ts = 0.01;
N_kernels = 7;
dt = 0.005;

record_demo(N_demos, Ts, N_kernels, dt);