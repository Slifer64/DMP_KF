clc;
close all
clear;

addpath('utils/');

load_logged_data();

clc;
close all
clear;

global N_JOINTS fontsize interpreter lineWidth

save('logged_data.mat', ...
     'Time_data','Y_data','dY_data','ddY_data', ...
     'Fext_data','Fext_filt_data', ...
     'g_hat_data', 'Sigma_g_hat_data', ...
     'tau_hat_data', 'sigma_tau_hat_data', 'mf_data', ...
     'Timed','Yd_data','dYd_data','ddYd_data');

g = Y_data(:end);
tau = Time_data(end);
plot_1sigma = true;
plot_estimation_results(Time_data, g, g_hat_data, tau, tau_data, P_data, Fext_filt_data, mf_data, plot_1sigma);

Demo_Data = struct('Time',Timed, 'Y',Yd_data, 'dY',dYd_data, 'ddY',ddYd_data);
plotData(Demo_Data);

Exec_Data = struct('Time',Time_data, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
plotData(Exec_Data);
