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

dt = Time(2)-Time(1);

effort = 0;
P = [];
F_norm = [];
for i=1:size(Fext_filt_data,2)
    effort = effort + (Fext_filt_data(:,i)'*dY_data(:,i)*dt);
    P = [P Fext_filt_data(:,i)'*dY_data(:,i)];
    F_norm = [F_norm norm(Fext_filt_data(:,i))];
end

effort

fontsize = 14;
figure;
plot(Time, P, 'LineWidth',2.0)
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
ylabel('Power [$Watt$]', 'interpreter','latex', 'fontsize',fontsize);
axis tight;

fontsize = 14;
figure;
plot(Time, F_norm, 'LineWidth',2.0)
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',fontsize);
ylabel('$||\mathbf{f}_{ext}||$ [$N$]', 'interpreter','latex', 'fontsize',fontsize);
axis tight;

% figure;
% plot(Fext_data')

fclose(fid);

if (isempty(Time))
    error('The loaded data are empty %s\n', filename);
end

tau = Time(end);
plot_1sigma = false;
g_hat_data = theta_data(1:3,:);
tau_hat_data = theta_data(4,:);

if (isempty(g)), g = Y_data(:,end); end
    
plot_estimation_results(Time, g, g_hat_data, tau, tau_hat_data, Sigma_theta_data, Fext_filt_data, mf_data, plot_1sigma, Y_data);

% g_norm_data = zeros(length(Time),1);
% for i=1:length(g_norm_data)
%     g_norm_data(i) = norm(g_hat_data(:,i));
% end
% figure;
% plot(Time, g_norm_data);


% Exec_Data{1} = struct('Time',Time, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
% plotData(Exec_Data);


end