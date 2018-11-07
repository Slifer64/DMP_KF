function compareExecSim()

format compact;

set_matlab_utils_path();

clear_all();

loadTrainingData('../data/training_data.bin');
% plotTrainData();

loadExecutionData('../data/execution_data_12a.bin');
% plotExecutionData();

exec_DMP_EKF_disc();

compareDMPweights()

delete *.mat;

end

function TrainData = loadTrainingData(filename)
    
    if (nargin<1), filename = '../data/training_data.bin'; end

    binary = true;
    fid = fopen(filename);
    if (fid < 0)
        warning('Could not load %s\n', filename);
        return;
    end

    q_start = read_mat(fid, binary);
    Time = read_mat(fid, binary);
    Y_data = read_mat(fid, binary);
    dY_data = read_mat(fid, binary);
    ddY_data = read_mat(fid, binary);

    fclose(fid);

    if (isempty(Time))
        error('The loaded data are empty %s\n', filename);
    end

    TrainData = struct('Time',Time, 'Y_data',Y_data, 'dY_data',dY_data, 'ddY_data',ddY_data);

    save('training_data.mat','TrainData');

end

function plotTrainData(TrainData)
    
    if (nargin < 1), load('training_data.mat','TrainData'); end
    
    if (~iscell(TrainData)), TrainData = {TrainData}; end
    
    fig = figure;
    ax = axes('Position',[0.12 0.12 0.85 0.85], 'Parent',fig);
    hold(ax, 'on');
    legends = {};
    for i=1:length(TrainData)
        Y_data = TrainData{i}.Y;
        plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0, 'Parent',ax); 
        h_start = plot3(Y_data(1,1), Y_data(2,1), Y_data(3,1), 'Color','blue', 'LineWidth',4.0, 'Marker','o', 'MarkerSize',12, 'HandleVisibility','off', 'Parent',ax); 
        h_end = plot3(Y_data(1,end), Y_data(2,end), Y_data(3,end), 'Color','red', 'LineWidth',4.0, 'Marker','x', 'MarkerSize',12, 'HandleVisibility','off', 'Parent',ax); 
        legends = [legends ['demo ' num2str(i)]];
        if (i==length(TrainData))
            h_start.HandleVisibility = 'on';
            h_end.HandleVisibility = 'on';
            legends = [legends 'start' 'target'];
        end
    end
    legend(ax, legends, 'interpreter','latex', 'fontsize',15);
    xlabel('x [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    ylabel('y [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    zlabel('z [$m$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    hold(ax, 'off');

    % fig = figure;
    % ax = axes('Position',[0.12 0.12 0.85 0.85], 'Parent',fig);
    % hold(ax, 'on');
    % legends = {};
    % for i=1:length(Data)
    %     Y_data = Data{i}.dY;
    %     plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0); 
    %     legends = [legends ['demo ' num2str(i)]];
    % end
    % legend(ax, legends, 'interpreter','latex', 'fontsize',15);
    % xlabel('$\dot{x}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    % ylabel('$\dot{y}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    % zlabel('$\dot{z}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    % hold(ax, 'off');
    % 
    % 
    % fig = figure;
    % ax = axes('Position',[0.12 0.12 0.85 0.85], 'Parent',fig);
    % hold(ax, 'on');
    % legends = {};
    % for i=1:length(Data)
    %     Y_data = Data{i}.ddY;
    %     plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0); 
    %     legends = [legends ['demo ' num2str(i)]];
    % end
    % legend(ax, legends, 'interpreter','latex', 'fontsize',15);
    % xlabel('$\ddot{x}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    % ylabel('$\ddot{y}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    % zlabel('$\ddot{z}$ [$m/s$]', 'interpreter','latex', 'fontsize',15, 'Parent',ax);
    % hold(ax, 'off');

    % plotData(Data);
    
    
end

function ExecData = loadExecutionData(filename)

    if (nargin<1), filename = '../data/execution_data.bin'; end

    fid = fopen(filename);
    if (fid < 0)
        error('Could not load %s\n', filename);
    end
    
    binary = true;

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
    Yg = read_mat(fid, binary);

    norm_dY = zeros(length(Time),1);
    for i=1:length(norm_dY)
        norm_dY(i) = norm(dY_data(:,i));
    end

    v_start = 0.01;
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

    fclose(fid);
    
    if (isempty(Time))
        error('The loaded data are empty %s\n', filename);
    end

    ExecData = struct('Time',Time, 'Y_data',Y_data, 'dY_data',dY_data, 'ddY_data', ddY_data, 'Yg', Yg, ...
            'Fext_data',Fext_filt_data, 'theta_data',theta_data, 'Sigma_theta_data', Sigma_theta_data);
    
    save('execution_data.mat','ExecData');

end

function plotExecutionData(ExecData)
    
    if (nargin < 1), load('execution_data.mat','ExecData'); end

    tau = ExecData.Time(end);
    plot_1sigma = false;
    g_hat_data = ExecData.theta_data(1:3,:);
    tau_hat_data = ExecData.theta_data(4,:);

    if (isempty(ExecData.Yg)), ExecData.Yg = ExecData.Y_data(:,end); end

    plot_estimation_results(ExecData.Time, ExecData.Yg, g_hat_data, tau, tau_hat_data, ExecData.Sigma_theta_data, ...
        ExecData.Fext_data, plot_1sigma, ExecData.Y_data, ExecData.dY_data);
end

function compareDMPweights()

load('training_data.mat','TrainData');
dmp_train_data = train_DMP(TrainData);

load('simulation_data.mat','SimData');
dmp_sim_data = train_DMP(SimData);

load('execution_data.mat','ExecData');
dmp_exec_data = train_DMP(ExecData);

dmp = dmp_train_data.dmp;
dmp_sim = dmp_sim_data.dmp;
dmp_exec = dmp_exec_data.dmp;
dim = length(dmp);

ax = cell(dim,1);
figure;
for i=1:dim
   ax{i} = subplot(dim,1,i);
   hold(ax{i},'on');
   axis(ax{i},'tight');
end

legends = cell(dim,1);
for i=1:dim
    bar(ax{i}, dmp{i}.c, dmp{i}.w, 'BarWidth',0.85, 'FaceColor','green');
    bar(ax{i}, dmp_sim{i}.c, dmp_sim{i}.w, 'BarWidth',0.65, 'FaceColor','blue');
    bar(ax{i}, dmp_exec{i}.c, dmp_exec{i}.w, 'BarWidth',0.45, 'FaceColor',[0.85 0.33 0.1]);
end
legend(ax{1}, {'Training','Simulation','Execution'},'interpreter','latex','fontsize',14);
title(ax{1},'DMP weights','interpreter','latex','fontsize',14);

figure;
ax = axes();
hold(ax,'on');
Y0 = ExecData.Y_data(:,1);
Yg = ExecData.Yg;
[~, Y_data, dY_data, ddY_data] = simulateDMP(dmp, Y0, Yg, ExecData.Time(end), 0.002);
plot3(Y_data(1,:), Y_data(2,:), Y_data(3,:), 'LineWidth',2.0, 'LineStyle','-', 'Color','blue', 'Parent',ax);
plot3(SimData.Y_data(1,:), SimData.Y_data(2,:), SimData.Y_data(3,:), 'LineWidth',2.0, 'LineStyle','-.', 'Color','magenta','Parent',ax);
plot3(ExecData.Y_data(1,:), ExecData.Y_data(2,:), ExecData.Y_data(3,:), 'LineWidth',2.0, 'LineStyle','--', 'Color',[0.85 0.33 0.1], 'Parent',ax);
plot3(Y0(1), Y0(2), Y0(3), 'LineWidth',2.5, 'MarkerSize',10, 'Marker','o', 'Color','green', 'Parent',ax);
plot3(Yg(1), Yg(2), Yg(3), 'LineWidth',2.5, 'MarkerSize',10, 'Marker','x', 'Color','red', 'Parent',ax);
legend(ax, {'Training (scaled)','Simulation','Execution','$\mathbf{y}_0$','$\mathbf{y}_g$'},'interpreter','latex','fontsize',14);
xlabel('$\mathbf{x}$ [$m$]','interpreter','latex','fontsize',14, 'Parent',ax);
ylabel('$\mathbf{y}$ [$m$]','interpreter','latex','fontsize',14, 'Parent',ax);
zlabel('$\mathbf{z}$ [$m$]','interpreter','latex','fontsize',14, 'Parent',ax);
hold(ax,'off');



end

function dmp_data = train_DMP(Data, plot_on)


if (nargin < 1), load('training_data.mat','TrainData'); end
if (nargin < 2), plot_on = false; end

Timed = Data.Time;
Yd_data = Data.Y_data;
dYd_data = Data.dY_data;
ddYd_data = Data.ddY_data;

Dim = size(Yd_data,1);

%% initialize DMP
N_kernels = 20;
a_z = 20;
b_z = a_z/4;
train_method = 'LWR';
can_clock_ptr = CanonicalClock();
dmp = cell(Dim,1);
shapeAttrGatingPtr = SigmoidGatingFunction(1.0, 0.97);
for i=1:Dim 
    dmp{i} = DMP(N_kernels, a_z, b_z, can_clock_ptr, shapeAttrGatingPtr);
end

%% Train the DMP
% disp('DMP training...')
% tic
offline_train_mse = zeros(Dim,1); 
n_data = size(Yd_data,2);
for i=1:Dim
    [offline_train_mse(i), F_train, Fd_train] = dmp{i}.train(train_method, Timed, Yd_data(i,:), dYd_data(i,:), ddYd_data(i,:));      
end
% offline_train_mse
% toc

%% DMP simulation
% disp('DMP simulation...');
% tic
y0 = Yd_data(:,1);
g = Yd_data(:,end); 
T = Timed(end);
dt = 0.005;
[Time, Y_data, dY_data, ddY_data] = simulateDMP(dmp, y0, g, T, dt);
% toc

%% save data
dmp_data = struct('dmp',{dmp}, 'g', g, 'y0',y0, 'tau',T, 'train_method',train_method);
save('dmp_data.mat','dmp_data');

%% plot data
if (plot_on)
    fontsize = 14;
    linewidth = 1.5;
    figure('NumberTitle', 'off', 'Name', ['Demo ' num2str(1)]);
    k = 1;
    ax_cell = cell(3,3);
    for i=1:3
        for j=1:3
            ax_cell{i,j} = subplot(3,3,k);
            hold(ax_cell{i,j}, 'on');
            k = k + 1;
        end
    end
    for i=1:3
        plot(Timed,Yd_data(i,:), 'LineWidth',linewidth, 'Color',[0.85 0.33 0.1], 'Parent',ax_cell{1,i});
        plot(Time,Y_data(i,:), 'LineWidth',linewidth, 'Color','blue', 'Parent',ax_cell{1,i}); 
    end
    for i=1:3
        plot(Timed,dYd_data(i,:), 'LineWidth',linewidth, 'Color',[0.85 0.33 0.1], 'Parent',ax_cell{2,i});
        plot(Time,dY_data(i,:), 'LineWidth',linewidth, 'Color','blue', 'Parent',ax_cell{2,i}); 
    end
    for i=1:3
        plot(Timed,ddYd_data(i,:), 'LineWidth',linewidth, 'Color',[0.85 0.33 0.1], 'Parent',ax_cell{3,i});
        plot(Time,ddY_data(i,:), 'LineWidth',linewidth, 'Color','blue', 'Parent',ax_cell{3,i}); 
    end
    legend(ax_cell{1,3}, {'dmp','demo'},'interpreter','latex','fontsize',fontsize);
    title(ax_cell{1,1}, '$X$','interpreter','latex','fontsize',fontsize);
    title(ax_cell{1,2}, '$Y$','interpreter','latex','fontsize',fontsize);
    title(ax_cell{1,3}, '$Z$','interpreter','latex','fontsize',fontsize);
    ylabel(ax_cell{1,1}, 'position [$m$]','interpreter','latex','fontsize',fontsize);
    ylabel(ax_cell{2,1}, 'velocity [$m/s$]','interpreter','latex','fontsize',fontsize);
    ylabel(ax_cell{3,1}, 'acceleration [$m/s^2$]','interpreter','latex','fontsize',fontsize);
    title(ax_cell{3,1}, 'time [$s$]','interpreter','latex','fontsize',fontsize);
    title(ax_cell{3,2}, 'time [$s$]','interpreter','latex','fontsize',fontsize);
    title(ax_cell{3,3}, 'time [$s$]','interpreter','latex','fontsize',fontsize);
end

end

function exec_DMP_EKF_disc(dmp_data, ExecData)

if (nargin < 1)
    load('training_data.mat','TrainData');
    dmp_data = train_DMP(TrainData);
end

dmp = dmp_data.dmp;
Yg0 = dmp_data.g;
Y0 = dmp_data.y0;
tau0 = dmp_data.tau;
Dim = length(Y0);
can_clock_ptr = dmp{1}.can_clock_ptr;
can_clock_ptr.setTau(tau0);

%% ###################################################################

if (nargin < 2)
    load('execution_data.mat','ExecData');
end

Yg = ExecData.Yg;
tau = ExecData.Time(end);
Y0 = ExecData.Y_data(:,1);

dt = 0.002;

time_offset = tau - tau0;

rng(0);
sigma_f_ext = 0.1;
f_ext_n = 0.0;
a_f_n = 0.7;
f_mag_noise = 0.0;

goal_up_lim = [0.65; 0.8; 0.7];
goal_low_lim = [-0.65; 0.05; 0.15];
tau_low_lim = 1.5;
tau_up_lim = 60.0; %Inf;

process_noise = 0.005; % Q
msr_noise = 1000; % R
init_params_variance = 1.0; % P
a_p = 1.002; % forgetting factor in fading memory EKF

theta_low_lim = [goal_low_lim; tau_low_lim];
theta_up_lim = [goal_up_lim; tau_up_lim];
N_params = length(theta_low_lim);
A_c = [-eye(N_params, N_params); eye(N_params, N_params)];
b_c = [-theta_low_lim; theta_up_lim];
enable_constraints = true*1;

theta_sigma_min = 0.001;
theta_sigma_max = 100000;
apply_cov_sat = false;

ekf_provide_Jacob = true;

plot_1sigma = true*0;

stiff_human = false;

a_py = 150;
a_dpy = 50;
M_r = 10*eye(3,3);
inv_M_r = inv(M_r);
K_r = 300*eye(3,3);
D_r = 2*sqrt(M_r*K_r);

M_h = 3*eye(3,3);
inv_M_h = inv(M_h);
K_h_min = 50*eye(3,3);
K_h_max = 500*eye(3,3);
K_h = K_h_min;
D_h = 2*sqrt(M_h*K_h);
p_e_max = 0.15;
p_e_min = 0.01;
p_e = p_e_max;


inv_M_rh = inv(inv_M_r + inv_M_h);

%% ###################################################################

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
x = 0.0; dx = 0.0;
Y = Y0; dY = zeros(Dim,1); ddY = zeros(Dim,1);
Y_hat = Y0; dY_hat = zeros(Dim,1); ddY_hat = zeros(Dim,1);
Y_r = Y0; dY_r = zeros(Dim,1); ddY_r = zeros(Dim,1);
Y_h = Y0; dY_h = zeros(Dim,1); ddY_h = zeros(Dim,1);
F_ext = zeros(Dim,1);

Time = [];
Y_data = []; dY_data = []; ddY_data = [];
Y_hat_data = []; dY_hat_data = []; ddY_hat_data = [];
Y_r_data = []; dY_r_data = []; ddY_r_data = [];
Y_h_data = []; dY_h_data = []; ddY_h_data = [];
x_data = [];
Yg_data = [];
tau_data = [];
F_data = [];
Sigma_theta_data = [];
K_h_data = [];
p_e_data = [];

t_end = tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
Yg_hat = Yg0;

x_hat = t/tau_hat;
N_out = length(Yg_hat);

theta = [Yg_hat; tau_hat];

N_params = length(theta);
P_theta = eye(N_params, N_params) * init_params_variance;
R = eye(N_out,N_out)*msr_noise;
inv_R = inv(R);
Q = eye(N_params,N_params) * process_noise;

%% Set up EKF object
ekf = EKF(N_params, N_out, @stateTransFun, @msrFun);
ekf.setProcessNoiseCov(Q);
ekf.setMeasureNoiseCov(R);
ekf.setFadingMemoryCoeff(a_p); %exp(a_pc*dt));
ekf.theta = theta;
ekf.P = P_theta;

ekf.enableParamsContraints(enable_constraints);
ekf.setParamsConstraints(A_c, b_c);
ekf.setPartDerivStep(0.001);

if (ekf_provide_Jacob)
    ekf.setStateTransFunJacob(@stateTransFunJacob);
    ekf.setMsrFunJacob(@msrFunJacob);
end

msr_cookie = MsrCookie();

disp('DMP-EKF (discrete) simulation...')
% tic
while (true)

    %% data logging

    Time = [Time t];

    Y_data = [Y_data Y];
    dY_data = [dY_data dY];  
    ddY_data = [ddY_data ddY];

    Y_hat_data = [Y_hat_data Y_hat];
    dY_hat_data = [dY_hat_data dY_hat];  
    ddY_hat_data = [ddY_hat_data ddY_hat];

    Y_r_data = [Y_r_data Y_r];
    dY_r_data = [dY_r_data dY_r];  
    ddY_r_data = [ddY_r_data ddY_r];

    Y_h_data = [Y_h_data Y_h];
    dY_h_data = [dY_h_data dY_h];  
    ddY_h_data = [ddY_h_data ddY_h];

    Yg_data = [Yg_data Yg_hat];
    tau_data = [tau_data tau_hat];

    x_data = [x_data x];

    F_data = [F_data F_ext];

    Sigma_theta_data = [Sigma_theta_data sqrt(diag(P_theta))];
    
    K_h_data = [K_h_data trace(K_h)/3];
    p_e_data = [p_e_data p_e];

    Y_hat = Y_r;
    dY_hat = dY_r;
    Y_c = a_py*(Y_r-Y_hat) + a_dpy*(dY_r-dY_hat);
    Z_c = zeros(Dim,1);
    
%     x1 = 0.3;
%     tau1 = 2.0*t_end;
%     % x1*tau1<t_end to ensure tau2>0
%     tau2 = (t_end - x1*tau1)/(1-x1);
%     if (x <= x1)
%         tau = tau1;
%     else
%        tau = tau2;
%     end
    
    can_clock_ptr.setTau(tau);

    %% DMP simulation
    for i=1:Dim  
        ddY(i) = dmp{i}.getAccel(Y(i), dY(i), Y0(i), 0, 0, x, Yg(i), dmp{i}.getTau());
        ddY_hat(i) = dmp{i}.getAccel(Y_hat(i), dY_hat(i), Y0(i), 0, 0, x_hat, Yg_hat(i), tau_hat);
    end

    p_e = p_e_max + (p_e_min - p_e_max)*(t/t_end)^0.4;
    p_a = norm(Y_h - Y)/p_e;
    % if (p_a > 1), p_a = 1.0; end
    K_h = K_h_min + (K_h_max - K_h_min)*(p_a)^0.6;
    D_h = 2*sqrt(M_h*K_h);
    
    f_ext_n = a_f_n * f_ext_n + (1-a_f_n)*sigma_f_ext*randn(3,1);
%     ddY_n = ddY + f_ext_n;
%     dY_n = dY;
%     Y_n = Y;
    ddY_n = zeros(3,1) + f_ext_n;
    dY_n = zeros(3,1);
    Y_n = Y;
    
    if (stiff_human)
        F_ext = M_r*(ddY_n-ddY_hat) + M_r*( inv_M_r*(D_r*(dY_r-dY_hat)+K_r*(Y_r-Y_hat)) - inv_M_h*(D_h*(dY_h-dY_n)+K_h*(Y_h-Y_n)) );
    else
        F_ext = inv_M_rh*(ddY_n-ddY_hat) + inv_M_rh*( inv_M_r*(D_r*(dY_r-dY_hat)+K_r*(Y_r-Y_hat)) - inv_M_h*(D_h*(dY_h-dY_n)+K_h*(Y_h-Y_n)) );
    end
    
    Y_out_hat = ddY_hat;
    Y_out = ddY_r;
    kf_err = Y_out - Y_out_hat;
    % kf_err = inv(M_r)*F_ext;
    
    %F_ext = F_ext + f_ext_n + sign(F_ext).*abs(f_mag_noise*rand(3,1));

    ddY_r = ddY_hat + inv_M_r*(-D_r*(dY_r-dY_hat) - K_r*(Y_r-Y_hat) + F_ext);
    if (stiff_human)
        ddY_h = ddY_n + inv_M_h*(-D_h*(dY_h-dY_n) - K_h*(Y_h-Y_n));
    else
        ddY_h = ddY_n + inv_M_h*(-D_h*(dY_h-dY_n) - K_h*(Y_h-Y_n) - F_ext);
    end

    if (norm(ddY_h-ddY_r)>1e-12)
        warning(['t=' num2str(t) ' sec: Rigid bond contraint might be violated!']);
    end

    %% Update phase variable
    dx = can_clock_ptr.getPhaseDot(x);

    %% Stopping criteria
    err_p = norm(Yg-Y)/norm(Yg);
    if (err_p <= 0.5e-2 && t>=t_end)
        break; 
    end

    iters = iters + 1;
    if (t>=t_end)
        warning('Time limit reached. Stopping simulation...\n');
        break;
    end

    %% KF update
    % time update
    ekf.predict([]);
    % measurement update
    msr_cookie.set(dmp, t, Y, dY, Y0, 0.0, 0.0, x_hat);
    ekf.correct(Y_out, msr_cookie);

    theta = ekf.theta;
    P_theta = ekf.P;

    Yg_hat = theta(1:end-1);
    tau_hat = theta(end);
    x_hat = t/tau_hat; 

    %% Numerical integration
    t = t + dt;

    x = x + dx*dt;

    Y = Y + dY*dt;
    dY = dY + ddY*dt;

    Y_hat = Y_hat + dY_hat*dt;
    dY_hat = dY_hat + ddY_hat*dt;

    Y_r = Y_r + dY_r*dt;
    dY_r = dY_r + ddY_r*dt;

    Y_h = Y_h + dY_h*dt;
    dY_h = dY_h + ddY_h*dt;


end
% toc

tau = Time(end);

SimData = struct('Time',Time, 'Y_data',Y_r_data, 'dY_data',dY_r_data, 'ddY_data', ddY_r_data, 'Yg', Yg, ...
            'Fext_data',F_data, 'theta_data',[Yg_data; tau_data], 'Sigma_theta_data', Sigma_theta_data);
    
save('simulation_data.mat','SimData');

disp('*** Simulation results ***');
plot_estimation_results(Time, Yg, Yg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, Y_data, dY_data);

figure;
subplot(2,1,1);
hold on;
plot(Time, p_e_data, 'LineWidth',2.0, 'Color','red', 'LineStyle','--');
Y_Yh_norm_data = zeros(1,size(Y_data,2));
for i=1:length(Y_Yh_norm_data)
    Y_Yh_norm_data(i) = norm(Y_data(:,i)-Y_h_data(:,i));
end
plot(Time, Y_Yh_norm_data, 'LineWidth',2.0, 'Color','blue');
legend({'$||\mathbf{y}_r - \mathbf{y}||_{max}$','$||\mathbf{y}_r - \mathbf{y}||$'}, 'interpreter','latex', 'fontsize',14);
%ylabel('$||\mathbf{y}_r - \mathbf{y}||$', 'interpreter','latex', 'fontsize',14);
title('Human allowable error tube around nominal trajectory', 'interpreter','latex', 'fontsize',14);
hold off;
subplot(2,1,2);
plot(Time, K_h_data, 'LineWidth',2.0, 'Color','blue');
legend({'$tr(K_h)/3$'}, 'interpreter','latex', 'fontsize',14);
title('Human stiffness evolution', 'interpreter','latex', 'fontsize',14);
xlabel('time [$s$]', 'interpreter','latex', 'fontsize',14);
ylabel('$Nm$', 'interpreter','latex', 'fontsize',14);

Time = ExecData.Time;
Yg = ExecData.Yg;
Yg_data = ExecData.theta_data(1:end-1,:);
tau = ExecData.Time(end);
tau_data = ExecData.theta_data(end,:);
Sigma_theta_data = ExecData.Sigma_theta_data;
F_data = ExecData.Fext_data;
Y_data = ExecData.Y_data;
dY_data = ExecData.dY_data;

disp('*** Execution results ***');
plot_estimation_results(Time, Yg, Yg_data, tau, tau_data, Sigma_theta_data, F_data, plot_1sigma, Y_data, dY_data);


        
% fig = figure;
% Dim = 3;
% ax_cell = cell(Dim,Dim);
% titles = {'position $[m]$', 'velocity $[m/s]$','acceleration $[m/s^2]$'};
% ylabels = {'$X$', '$Y$', '$Z$'};
% Data = { y_data, dy_data, ddy_data;
%          y_hat_data, dy_hat_data, ddy_hat_data;
%          y_r_data, dy_r_data, ddy_r_data;
%          y_h_data, dy_h_data, ddy_h_data};
% for j=1:Dim
%     Y_data = Data{1,j};
%     Y_hat_data = Data{2,j};
%     Y_r_data = Data{3,j};
%     Y_h_data = Data{4,j};
%     for i=1:Dim
%         ax_cell{i,j} = subplot(Dim,Dim,(i-1)*Dim+j);
% 
%         ax = ax_cell{i,j};
%         hold(ax,'on');
%         plot(Time,Y_data(i,:), 'LineWidth',1.2, 'Color',[0 0 1], 'Parent',ax);
%         plot(Time,Y_hat_data(i,:), 'LineWidth',1.2, 'Color',[0 0.75 0.75], 'LineStyle','--', 'Parent',ax);
%         plot(Time,Y_r_data(i,:), 'LineWidth',1.2, 'Color',[0.85 0.33 0.1], 'LineStyle',':', 'Parent',ax);
%         plot(Time,Y_h_data(i,:), 'LineWidth',1.2, 'Color',[1 0 0], 'LineStyle','-.', 'Parent',ax);
%         xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
%         if (i==1 && j==1)
%             legend(ax, {'$\mathbf{y}$','$\hat{\mathbf{y}}$','$\mathbf{y}_r$','$\mathbf{y}_h$'}, ...
%                 'interpreter','latex', 'fontsize',15, 'orientation','horizontal');
%         end
%         if (i==1), title(ax, titles{j}, 'interpreter','latex', 'fontsize',15); end
%         if (j==1), ylabel(ax, ylabels{i}, 'interpreter','latex', 'fontsize',15); end
%         hold(ax,'off');
%     end
% end

end








