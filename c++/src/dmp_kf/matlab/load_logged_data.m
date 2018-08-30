clc;
close all;
clear;

addpath('utils/');

binary = true;
filename = '../data/out_data_Sat_Jun_16_00-20-17_2018.bin';
fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end

Time data = read_mat(fid, binary);
Y_data = read_mat(fid, binary);
dY_data = read_mat(fid, binary);
ddY_data = read_mat(fid, binary);

Fext_data = read_mat(fid, binary);
Fext_filt_data = read_mat(fid, binary);

g_hat_data = read_mat(fid, binary);
Sigma_g_hat_data = read_mat(fid, binary);
tau_hat_data = read_mat(fid, binary);
sigma_tau_hat_data = read_mat(fid, binary);
mf_data = read_mat(fid, binary);

Timed = read_mat(fid, binary);
Yd_data = read_mat(fid, binary);
dYd_data = read_mat(fid, binary);
ddYd_data = read_mat(fid, binary);

% Time_data = (0:(size(S_r_data,2)-1))*0.008;

save('logged_data.mat', ...
     'Time_data','Y_data','dY_data','ddY_data', ...
     'Fext_data','Fext_filt_data', ...
     'g_hat_data', 'Sigma_g_hat_data', ...
     'tau_hat_data', 'sigma_tau_hat_data', 'mf_data', ...
     'Timed','Yd_data','dYd_data','ddYd_data');
