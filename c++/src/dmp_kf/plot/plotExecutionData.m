function plotExecutionData(filename)

if (nargin<1), filename = 'execution_data'; end

set_matlab_utils_path();

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
Y_ref_data = read_mat(fid, binary);
dY_ref_data = read_mat(fid, binary);
ddY_ref_data = read_mat(fid, binary);
Fext_data = read_mat(fid, binary);
Fext_filt_data = read_mat(fid, binary);
theta_data = read_mat(fid, binary);
Sigma_theta_data = read_mat(fid, binary);
g = read_mat(fid, binary);

fclose(fid);

norm_dY = zeros(length(Time),1);
for i=1:length(norm_dY)
    norm_dY(i) = norm(dY_data(:,i));
end

v_start = 0.02;
v_stop = 0.01;
i1 = find(norm_dY>=v_start, 1, 'first');
i2 = find(norm_dY>=v_stop, 1, 'last');
ind = i1:i2;

dt = Time(2)-Time(1);
Time = (0:length(ind)-1)*dt;
Y_data = Y_data(:,ind);
dY_data = dY_data(:,ind);
ddY_data = ddY_data(:,ind);
Y_ref_data = Y_ref_data(:,ind);
dY_ref_data = dY_ref_data(:,ind);
ddY_ref_data = ddY_ref_data(:,ind);
Fext_data = Fext_data(:,ind);
Fext_filt_data = Fext_filt_data(:,ind);
theta_data = theta_data(:,ind);
Sigma_theta_data = Sigma_theta_data(:,ind);


if (isempty(Time))
    error('The loaded data are empty %s\n', filename);
end

tau = Time(end);
plot_1sigma = 0;
g_hat_data = theta_data(1:3,:);
tau_hat_data = theta_data(4,:);

if (isempty(g)), g = Y_data(:,end); end

f_norm = zeros(size(Fext_data,2),1);
for i=1:length(f_norm)
    f_norm(i) = norm(Fext_data(:,i));
end

f_norm_filt = zeros(size(Fext_filt_data,2),1);
for i=1:length(f_norm)
    f_norm_filt(i) = norm(Fext_filt_data(:,i));
end

figure;
hold on;
plot(Time, f_norm,'b-', 'LineWidth',1.5);
plot(Time, f_norm_filt,'g-', 'LineWidth',1.5);
ylabel(['$||\mathbf{f}_{ext}||$ [$N$]'],'interpreter','latex','fontsize',15);
xlabel('time [$s$]','interpreter','latex','fontsize',15);
legend({'raw','filt'},'interpreter','latex','fontsize',15);
axis tight;
hold off;
    
plot_estimation_results(Time, g, g_hat_data, tau, tau_hat_data, Sigma_theta_data, Fext_filt_data, plot_1sigma, Y_data, dY_data);


% Exec_Data{1} = struct('Time',Time, 'Y',Y_data, 'dY',dY_data, 'ddY',ddY_data);
% plotData(Exec_Data);

% fig = figure;
% D = 3;
% ax_cell = cell(D,1);
% titles = {'position', 'velocity','acceleration'};
% for i=1:D, ax_cell{i} = subplot(3,1,i); end
% for i=1:D
%     ax = ax_cell{i};
%     hold(ax,'on');
%     plot(Time,Y_data(i,:), 'LineWidth',1.2, 'Color','blue', 'Parent',ax);
%     plot(Time,Y_ref_data(i,:), 'LineWidth',1.2, 'Color','green', 'Parent',ax);
%     xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
%     legend(ax, {'robot','dmp'}, 'interpreter','latex', 'fontsize',15);
%     title(ax, titles{i}, 'interpreter','latex', 'fontsize',15);
%     hold(ax,'off');
% end

end
