%% ==============================================================
clc;
close all;
clear;

format compact;

set_matlab_utils_path();

%% ###################################################################

dt = 0.002;

goal_scale = [1.5 -1.4 -1.6]';
time_scale = 0.8; 

process_noise = 0.001*dt; % Q
msr_noise = 0.01/dt; % R
init_params_variance = 1.0; % P
a_p = 0.6; % forgetting factor in fading memory EKF

p1 = 0.01 * 1e-280;
p2 = 50.0 * 1e280;
p_r = 6.0;
tau_e = 0.1;

plot_1sigma = true;

stiff_human = false;

a_py = 150;
a_dpy = 50;
M_r = 2*eye(3,3);
inv_M_r = inv(M_r);
D_r = 30*eye(3,3);
K_r = 150*eye(3,3);

M_h = 4*eye(3,3);
inv_M_h = inv(M_h);
D_h = 80*eye(3,3);
K_h = 350*eye(3,3);

inv_M_rh = inv(inv_M_r + inv_M_h);

f1_ = 1.0;
f2_ = 2.0;
p_5th = get5thOrderParams(f1_, f2_, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
m2_fun = @(x) (x<=f1_)*1 + ((x>f1_) & (x<f2_)).*(p_5th(1) + p_5th(2)*x + p_5th(3)*x.^2 + p_5th(4)*x.^3 + p_5th(5)*x.^4 + p_5th(6)*x.^5) + (x>=f2_)*0;

m_fun = m2_fun;


% x = 0:0.002:5;
% y = m_fun(x);
% figure;
% plot(x,y);
% 
% return

%% ###################################################################

set_matlab_utils_path();

load('data/dmp_nss_data.mat', 'DMP_data');

dmp = DMP_data{1};
can_clock_ptr = dmp{1}.can_clock_ptr;

load('data/training_data.mat','Data');

N = length(Data);

Data_sim = cell(N,1);

for n=1:1
    
    data = Data{n};
    
    Timed = data.Time;
    yd_data = data.Y;
    dyd_data = data.dY;
    ddyd_data = data.ddY;
    
    D = size(yd_data,1);

    %% DMP simulation
    % set initial values
    t = 0.0;
    iters = 0;
    y0 = yd_data(:,1);
    g0 = yd_data(:,end); 
    g = g0;
    x = 0.0; dx = 0.0;
    y = y0; dy = zeros(D,1); ddy = zeros(D,1);
    y_hat = y0; dy_hat = zeros(D,1); ddy_hat = zeros(D,1);
    y_r = y0; dy_r = zeros(D,1); ddy_r = zeros(D,1);
    y_h = y0; dy_h = zeros(D,1); ddy_h = zeros(D,1);
    z = zeros(D,1); dz = zeros(D,1);
    F_ext = zeros(D,1);
    mf = 1;

    Time = [];
    y_data = []; dy_data = []; ddy_data = [];
    y_hat_data = []; dy_hat_data = []; ddy_hat_data = [];
    y_r_data = []; dy_r_data = []; ddy_r_data = [];
    y_h_data = []; dy_h_data = []; ddy_h_data = [];
    x_data = [];
    g_data = [];
    tau_data = [];
    F_data = [];
    mf_data = [];
    P_data = [];
    theta_data = [];
    Ptheta_data = [];

    N_data = size(yd_data,2);

    g = goal_scale.*g;
    
    t_end = time_scale*Timed(end);
    tau = t_end;
    can_clock_ptr.setTau(tau);
    
    tau_hat = Timed(end);
    g_hat = yd_data(:,end);
    
    x_hat = t/tau_hat;
    N_out = length(g_hat);
    y_out_hat = zeros(N_out,1);
    y_out = zeros(N_out,1);

    N_kernels = length(dmp{1}.w);
    N_params1 = N_kernels + 1;
    N_params = N_out*N_params1;
    theta = [dmp{1}.w; g_hat(1); dmp{2}.w; g_hat(2); dmp{3}.w; g_hat(3)];
    ig1 = N_params1;
    ig2 = 2*N_params1;
    ig3 = 3*N_params1;
    ig = [ig1 ig2 ig3];
    P_theta = eye(N_params, N_params) * init_params_variance;
    for i=1:length(ig), P_theta(ig(i),ig(i)) = init_params_variance*0.1; end
    R = eye(N_out,N_out)*msr_noise;
    inv_R = inv(R);
    Q = eye(N_params,N_params) * process_noise;
    
    ekf = struct('F_k',eye(N_params,N_params), 'H_k',zeros(N_out,N_params) , 'Q',Q, 'R',R, 'a_p',exp(a_p*dt) ,'theta',theta, 'P',P_theta);

    disp('DMP-IP simulation...')
    tic
    while (true)
        
        %% data logging

        Time = [Time t];

        y_data = [y_data y];
        dy_data = [dy_data dy];  
        ddy_data = [ddy_data ddy];
        
        y_hat_data = [y_hat_data y_hat];
        dy_hat_data = [dy_hat_data dy_hat];  
        ddy_hat_data = [ddy_hat_data ddy_hat];
        
        y_r_data = [y_r_data y_r];
        dy_r_data = [dy_r_data dy_r];  
        ddy_r_data = [ddy_r_data ddy_r];
        
        y_h_data = [y_h_data y_h];
        dy_h_data = [dy_h_data dy_h];  
        ddy_h_data = [ddy_h_data ddy_h];
        
        g_data = [g_data g_hat];
        tau_data = [tau_data tau_hat];

        x_data = [x_data x];
        
        F_data = [F_data F_ext];
        mf_data = [mf_data mf];

        P_data = [P_data sqrt([P_theta(ig1,ig1), P_theta(ig2,ig2), P_theta(ig3,ig3), 0.01]')];
        
        theta_data = [theta_data theta];
        Ptheta_data = [Ptheta_data sqrt(diag(P_theta))];
        
        y_hat = y_r;
        dy_hat = dy_r;
        Y_c = a_py*(y_r-y_hat) + a_dpy*(dy_r-dy_hat);
        Z_c = zeros(D,1);

        %% DMP simulation
        for i=1:D  
            ddy(i) = dmp{i}.getAccel(y(i), dy(i), y0(i), 0, 0, x, g(i), dmp{i}.getTau());
            ddy_hat(i) = dmp{i}.getAccel(y_hat(i), dy_hat(i), y0(i), 0, 0, x_hat, g_hat(i), tau_hat);
        end

        if (stiff_human)
            F_ext = M_r*(ddy-ddy_hat) + M_r*( inv_M_r*(D_r*(dy_r-dy_hat)+K_r*(y_r-y_hat)) - inv_M_h*(D_h*(dy_h-dy)+K_h*(y_h-y)) );
        else
            F_ext = inv_M_rh*(ddy-ddy_hat) + inv_M_rh*( inv_M_r*(D_r*(dy_r-dy_hat)+K_r*(y_r-y_hat)) - inv_M_h*(D_h*(dy_h-dy)+K_h*(y_h-y)) );
        end

        a_z = dmp{1}.a_z;
        b_z = dmp{1}.b_z;
        y_out_hat = (ddy_hat + a_z*dy_hat + a_z*b_z*y_hat)/tau_hat^2;
        y_out = (ddy_r + a_z*dy_r + a_z*b_z*y_r)/tau^2;
        kf_err = y_out - y_out_hat;
        % kf_err = inv(M_r)*F_ext;
        
        ddy_r = ddy_hat + inv_M_r*(-D_r*(dy_r-dy_hat) - K_r*(y_r-y_hat) + F_ext);
        if (stiff_human)
            ddy_h = ddy + inv_M_h*(-D_h*(dy_h-dy) - K_h*(y_h-y));
        else
            ddy_h = ddy + inv_M_h*(-D_h*(dy_h-dy) - K_h*(y_h-y) - F_ext);
        end
        
        if (norm(ddy_h-ddy_r)>1e-14)
            warning(['t=' num2str(t) ' sec: Rigid bond contraint might be violated!']);
        end

        mf = m_fun(norm(F_ext));

        %% Update phase variable
        dx = can_clock_ptr.getPhaseDot(x);

        %% Stopping criteria
        err_p = norm(g-y)/norm(g);
        if (err_p <= 0.5e-2 && t>=t_end)
            break; 
        end

        iters = iters + 1;
        if (t>=t_end)
            warning('Time limit reached. Stopping simulation...\n');
            break;
        end
        
        %% KF update
        for i=1:length(dmp)
            a_z = dmp{i}.a_z;
            b_z = dmp{i}.b_z;
            psi = dmp{i}.kernelFunction(x_hat);
            psi = psi/(sum(psi)+dmp{i}.zero_tol);
            ekf.H_k(i,(i-1)*N_params1+1:i*N_params1) = [psi' a_z*b_z];
        end
        % calc Kalman gain
        K_kf = ekf.P*ekf.H_k'/(ekf.H_k*ekf.P*ekf.H_k' + ekf.R);
        % measurement update
        ekf.theta = ekf.theta + K_kf * kf_err;
        ekf.P = ekf.P - K_kf*ekf.H_k*ekf.P;
        % time update
%         ekf.theta = ekf.F_k*ekf.theta;
%         ekf.P = ekf.a_p^2*ekf.F_k*ekf.P*ekf.F_k' + ekf.Q;
        
%         P_theta
        
        P_theta = ekf.P;
        
%         P_theta
        
%         pause
        
        i1 = 0;
        for i=1:length(dmp)
           dmp{i}.w = theta(i1+1:i1+N_kernels);
           i1 = i1 + N_kernels + 1;
           g_hat(i) = theta(i1);
        end
        
        
        x_hat = t/tau_hat; 
        
        %% Numerical integration
        t = t + dt;

        x = x + dx*dt;

        y = y + dy*dt;
        dy = dy + ddy*dt;
        
        y_hat = y_hat + dy_hat*dt;
        dy_hat = dy_hat + ddy_hat*dt;
        
        y_r = y_r + dy_r*dt;
        dy_r = dy_r + ddy_r*dt;
        
        y_h = y_h + dy_h*dt;
        dy_h = dy_h + ddy_h*dt;

        
    end
    toc
    
    Data_sim{n} = struct('Time',Time, 'Y',y_data, 'dY',dy_data, 'ddY',ddy_data);
    
    plot_estimation_results(Time, g, g_data, tau, tau_data, P_data, F_data, mf_data, plot_1sigma, y_r_data);
    
    for i=1:length(dmp)
        i1 = (i-1)*N_params1+1;
        i2 = i*N_params1;
        linewidth = 1.3;
        figure;
        hold on;
        for j=i1:i2
            plot(Time,theta_data(j,:), 'LineStyle','-', 'LineWidth',linewidth);
            plot(Time,theta_data(j,:)+Ptheta_data(j,:),'c-.', 'LineWidth',linewidth);
            plot(Time,theta_data(j,:)-Ptheta_data(j,:),'c-.', 'LineWidth',linewidth);
        end
        axis tight;
    end
    
%     plotData(Data_sim);
    
    fig = figure;
    D = 3;
    ax_cell = cell(D,D);
    titles = {'position $[m]$', 'velocity $[m/s]$','acceleration $[m/s^2]$'};
    ylabels = {'$X$', '$Y$', '$Z$'};
    Data = { y_data, dy_data, ddy_data;
             y_hat_data, dy_hat_data, ddy_hat_data;
             y_r_data, dy_r_data, ddy_r_data;
             y_h_data, dy_h_data, ddy_h_data};
    for j=1:D
        Y_data = Data{1,j};
        Y_hat_data = Data{2,j};
        Y_r_data = Data{3,j};
        Y_h_data = Data{4,j};
        for i=1:D
            ax_cell{i,j} = subplot(D,D,(i-1)*D+j);
            
            ax = ax_cell{i,j};
            hold(ax,'on');
            plot(Time,Y_data(i,:), 'LineWidth',1.2, 'Color',[0 0 1], 'Parent',ax);
            plot(Time,Y_hat_data(i,:), 'LineWidth',1.2, 'Color',[0 0.75 0.75], 'LineStyle','--', 'Parent',ax);
            plot(Time,Y_r_data(i,:), 'LineWidth',1.2, 'Color',[0.85 0.33 0.1], 'LineStyle',':', 'Parent',ax);
            plot(Time,Y_h_data(i,:), 'LineWidth',1.2, 'Color',[1 0 0], 'LineStyle','-.', 'Parent',ax);
            xlabel('time [$s$]', 'interpreter','latex', 'fontsize',15);
            if (i==1 && j==1)
                legend(ax, {'$\mathbf{y}$','$\hat{\mathbf{y}}$','$\mathbf{y}_r$','$\mathbf{y}_h$'}, ...
                    'interpreter','latex', 'fontsize',15, 'orientation','horizontal');
            end
            if (i==1), title(ax, titles{j}, 'interpreter','latex', 'fontsize',15); end
            if (j==1), ylabel(ax, ylabels{i}, 'interpreter','latex', 'fontsize',15); end
            hold(ax,'off');
        end
    end

end




