function plotExecutionData()

addpath('utils/');

binary = true;
filename = '../data/execution_data.bin';
fid = fopen(filename);
if (fid < 0)
    error('Could not load %s\n', filename);
end
    
Time = read_mat(fid, binary);
Y_data = read_mat(fid, binary);
dY_data = read_mat(fid, binary);
ddY_data = read_mat(fid, binary);
mf_data = read_mat(fid, binary);
Fext_data = read_mat(fid, binary);
Fext_filt_data = read_mat(fid, binary);
theta_data = read_mat(fid, binary);
Sigma_theta_data = read_mat(fid, binary);

figure;
plot(Fext_data')

fclose(fid);

if (isempty(Time))
    error('The loaded data are empty %s\n', filename);
end

g = Y_data(:,end);
tau = Time(end);
plot_1sigma = false;
g_hat_data = theta_data(1:3,:);
tau_hat_data = theta_data(4,:);
plot_estimation_results(Time, g, g_hat_data, tau, tau_hat_data, Sigma_theta_data, Fext_filt_data, mf_data, plot_1sigma);


Exec_Data{1} = struct('Time',Time, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
plotData(Exec_Data);


end