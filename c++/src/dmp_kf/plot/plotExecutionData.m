function plotExecutionData(filename)

if (nargin<1), filename = 'execution_data'; end

addpath('utils/');

binary = true;
filename = ['../data/' filename '.bin'];
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
g = read_mat(fid, binary);

norm_dY = zeros(length(Time),1);
for i=1:length(norm_dY)
    norm_dY(i) = norm(dY_data(:,i));
end

v_start = 0.005;
v_stop = 0.008;
i1 = find(norm_dY>=v_start, 1, 'first');
i2 = find(norm_dY>=v_stop, 1, 'last');
ind = i1:i2;

dt = Time(2)-Time(1);
Time = (0:length(ind)-1)*dt;
Y_data = Y_data(:,ind);
dY_data = dY_data(:,ind);
ddY_data = ddY_data(:,ind);
mf_data = mf_data(:,ind);
Fext_data = Fext_data(:,ind);
Fext_filt_data = Fext_filt_data(:,ind);
theta_data = theta_data(:,ind);
Sigma_theta_data = Sigma_theta_data(:,ind);

% figure;
% plot(Fext_data')

fclose(fid);

if (isempty(Time))
    error('The loaded data are empty %s\n', filename);
end

tau = {}; % Time(end);
plot_1sigma = false;
g_hat_data = theta_data(1:3,:);
tau_hat_data = theta_data(4,:);

if (isempty(g)), g = Y_data(:,end); end
    
plot_estimation_results(Time, g, g_hat_data, tau, tau_hat_data, Sigma_theta_data, Fext_filt_data, mf_data, plot_1sigma, Y_data, dY_data);


% Exec_Data{1} = struct('Time',Time, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
% plotData(Exec_Data);


end