clc;
close all
clear;

addpath('utils/');

load_logged_data();

clc;
close all
clear;

global N_JOINTS fontsize interpreter lineWidth

load('logged_data.mat', ...
     'Time_data','Y_data','dY_data','ddY_data', ...
     'Fext_data','Fext_filt_data', ...
     'g_hat_data', 'Sigma_g_hat_data', ...
     'tau_hat_data', 'sigma_tau_hat_data', 'mf_data', ...
     'Timed','Yd_data','dYd_data','ddYd_data');

% Ts = 0.008;
% 
% dYd_data2 = zeros(size(dYd_data));
% ddYd_data2 = zeros(size(ddYd_data));
% 
% for i=1:size(Yd_data,1)
%     dYd_data2(i,2:end) = diff(Yd_data(i,:))/Ts;
%     ddYd_data2(i,2:end) = diff(dYd_data2(i,:))/Ts;
% end
% 
% figure;
% hold on;
% plot(Yd_data');
% hold off;
% 
% figure;
% hold on;
% plot(Timed, dYd_data(1,:));
% plot(Timed, dYd_data2(1,:));
% hold off;
% 
% figure;
% hold on;
% plot(Timed, ddYd_data(1,:));
% plot(Timed, ddYd_data2(1,:));
% hold off;
% return;

g = Y_data(:,end);
tau = Time_data(end);
plot_1sigma = true;
P_data = [Sigma_g_hat_data; sigma_tau_hat_data];
plot_estimation_results(Time_data, g, g_hat_data, tau, tau_hat_data, P_data, Fext_filt_data, mf_data, plot_1sigma);

Demo_Data{1} = struct('Time',Timed, 'Y',Yd_data, 'dY',dYd_data, 'ddY',ddYd_data);
plotData(Demo_Data);

Exec_Data{1} = struct('Time',Time_data, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
plotData(Exec_Data);
