function sim_DMP_EKF_disc()

%% ==============================================================
%% DMP with state reset (y=y_r, y_dot=y_r_dot) and force feedback, i.e. 
%% y_ddot = h(theta, y_r, y_r_dot, t) + f_ext/M

format compact;

set_matlab_utils_path();

%% ###################################################################

dt = 0.002;

goal_scale = [1.0 1.0 1.0]';
time_scale = 1.0; 

goal_offset = [0.34, 0.41, -0.43]';
y0_offset = [0.0 0.0 0.0]';
time_offset = 5.88;

goal_up_lim = 0.5*[1.0 1.0 1.0]';
goal_low_lim = -goal_up_lim;
% goal_up_lim = [0.35 1.0 1.0]';
% goal_low_lim = -[-0.2 1.0 1.0]';
tau_low_lim = 1.0;
tau_up_lim = 20.0; %Inf;

process_noise = 0.0001; % Q
msr_noise = 0.005; % R
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

M_h = 4*eye(3,3);
inv_M_h = inv(M_h);
K_h = 500*eye(3,3);
D_h = 2*sqrt(M_h*K_h);

inv_M_rh = inv(inv_M_r + inv_M_h);

%% ###################################################################

set_matlab_utils_path();

load('data/dmp_data.mat', 'dmp_data');

dmp = dmp_data.dmp;
Yg0 = dmp_data.g;
Y0 = dmp_data.y0;
tau0 = 5; %dmp_data.tau;
Dim = length(Y0);
can_clock_ptr = dmp{1}.can_clock_ptr;
can_clock_ptr.setTau(tau0);

%% DMP simulation
% set initial values
t = 0.0;
iters = 0;
Yg = Yg0;
x = 0.0; dx = 0.0;
Y0 = Y0 + y0_offset;
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

Yg = goal_scale.*Yg + goal_offset;

t_end = time_scale*tau0 + time_offset;
tau = t_end;
can_clock_ptr.setTau(tau);

tau_hat = tau0;
Yg_hat = Yg0;

x_hat = t/tau_hat;
N_out = length(Yg_hat);
Y_out_hat = zeros(N_out,1);
Y_out = zeros(N_out,1);

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
tic
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

    Y_hat = Y_r;
    dY_hat = dY_r;
    Y_c = a_py*(Y_r-Y_hat) + a_dpy*(dY_r-dY_hat);
    Z_c = zeros(Dim,1);

    %% DMP simulation
    for i=1:Dim  
        ddY(i) = dmp{i}.getAccel(Y(i), dY(i), Y0(i), 0, 0, x, Yg(i), dmp{i}.getTau());
        ddY_hat(i) = dmp{i}.getAccel(Y_hat(i), dY_hat(i), Y0(i), 0, 0, x_hat, Yg_hat(i), tau_hat);
    end

    if (stiff_human)
        F_ext = M_r*(ddY-ddY_hat) + M_r*( inv_M_r*(D_r*(dY_r-dY_hat)+K_r*(Y_r-Y_hat)) - inv_M_h*(D_h*(dY_h-dY)+K_h*(Y_h-Y)) );
    else
        F_ext = inv_M_rh*(ddY-ddY_hat) + inv_M_rh*( inv_M_r*(D_r*(dY_r-dY_hat)+K_r*(Y_r-Y_hat)) - inv_M_h*(D_h*(dY_h-dY)+K_h*(Y_h-Y)) );
    end
    
    Y_out_hat = ddY_hat;
    Y_out = ddY_r;
    kf_err = Y_out - Y_out_hat;
    % kf_err = inv(M_r)*F_ext;

    ddY_r = ddY_hat + inv_M_r*(-D_r*(dY_r-dY_hat) - K_r*(Y_r-Y_hat) + F_ext);
    if (stiff_human)
        ddY_h = ddY + inv_M_h*(-D_h*(dY_h-dY) - K_h*(Y_h-Y));
    else
        ddY_h = ddY + inv_M_h*(-D_h*(dY_h-dY) - K_h*(Y_h-Y) - F_ext);
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
toc

plot_estimation_results(Time, Yg, Yg_data, tau, tau_data, Sigma_theta_data, plot_1sigma, F_data, Y_data, dY_data);

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






