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

S_r_data = read_mat(fid, binary);
dS_r_data = read_mat(fid, binary);
ddS_r_data = read_mat(fid, binary);
Fext_data = read_mat(fid, binary);
F_c_data = read_mat(fid, binary);
F_c_d_data = read_mat(fid, binary);

Time_data = (0:(size(S_r_data,2)-1))*0.008;

save('logged_data.mat','Time_data','S_r_data','dS_r_data','ddS_r_data','Fext_data','F_c_data','F_c_d_data');
