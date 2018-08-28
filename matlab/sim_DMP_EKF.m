clc;
close all;
clear;

%% ###################################################################

est_g = true;
est_tau = true;

goal_scale = 0.5*[1.5 1.5 1.5]';
time_scale = 0.5; 

process_noise = 0.0; % Q
msr_noise = 0.005; % R
init_params_variance = 1; % P
a_p = 0.0; % instability term in modified EKF

plot_1sigma = false;

dt = 0.002;

%% ###################################################################

set_matlab_utils_path();

load('data/dmp_data.mat', 'DMP_data');

dmp = DMP_data{1};
canClockPtr = dmp{1}.canClockPtr;

load('data/demo_data.mat','Data');

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
    z = zeros(D,1); dz = zeros(D,1);
    F = zeros(D,1);
    mf = 1;

    Time = [];
    y_data = []; dy_data = []; ddy_data = [];
    x_data = [];
    g_data = [];
    tau_data = [];
    F_data = [];
    mf_data = [];
    P_data = [];
    
    mr = 1;   
    Mr = eye(3,3) * mr;

    N_data = size(yd_data,2);

    g = goal_scale.*g;
    
    t_end = time_scale*Timed(end);
    tau = t_end;
    canClockPtr.setTau(tau);
    
    tau_hat = Timed(end);
    if (est_tau == false), tau_hat=tau; end
    
    g_hat = yd_data(:,end);
    if (est_g == false), g_hat=g; end
    
    x_hat = t/tau_hat;
    N_out = length(g_hat);
    y_out_hat = zeros(N_out,1);
    y_out = zeros(N_out,1);
    
    theta = params2theta(tau_hat, g_hat, est_tau, est_g);
    
    N_params = length(theta);
    P_theta = eye(N_params, N_params) * init_params_variance;
    R = eye(N_out,N_out)*msr_noise;
    inv_R = inv(R);
    Q = eye(N_params,N_params) * process_noise;

    disp('DMP-EKF (continuous) simulation...')
    tic
    while (true)
        
        %% data logging

        Time = [Time t];

        y_data = [y_data y];
        dy_data = [dy_data dy];  
        ddy_data = [ddy_data ddy];
        
        g_data = [g_data g_hat];
        tau_data = [tau_data tau_hat];

        x_data = [x_data x];
        
        F_data = [F_data F];
        mf_data = [mf_data mf];
        
        P_temp = zeros(length(g)+1, 1);
        p_diag = diag(P_theta);
        if (est_tau), P_temp(end) = p_diag(end); end
        if (est_g)
            if (est_tau)
                P_temp(1:end-1) = p_diag(1:end-1);
            else
                P_temp(1:end-1) = p_diag(1:end);
            end
        end
            
        P_data = [P_data sqrt(P_temp)];
        
        dC_dtheta = zeros(D, length(theta));

        %% DMP simulation
        for i=1:D  

            y_c = 0.0;
            z_c = 0.0;

            [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);
            ddy(i) = dz(i)/dmp{i}.getTau();
            
            y_out(i) = dmp{i}.getAccel(y(i), z(i), y0(i), y_c, z_c, x, g(i), dmp{i}.getTau());
            y_out_hat(i) = dmp{i}.getAccel(y(i), z(i), y0(i), y_c, z_c, x_hat, g_hat(i), tau_hat);

            dC_dtheta_i = dmp{i}.getAcellPartDev_g_tau(t, y(i), dy(i), y0(i), x_hat, g_hat(i), tau_hat);
            
            if (est_tau), dC_dtheta(i,end) = dC_dtheta_i(2); end
            if (est_g), dC_dtheta(i,i) = dC_dtheta_i(1); end        

        end
        
        %% Interaction force calculation
        F = Mr*(y_out-y_out_hat);
        mf = 1 / (1 + exp(4*(norm(F)-1)));

        %% Update phase variable
        dx = canClockPtr.getPhaseDot(x);

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
        K_kf = P_theta*dC_dtheta'*inv_R;
        theta_dot = K_kf * (y_out - y_out_hat);
        P_dot = Q - K_kf*dC_dtheta*P_theta + 2*a_p*P_theta;
        
        theta = theta + theta_dot*dt;
        P_theta = P_theta + P_dot*dt;
        
        [tau_hat, g_hat] = theta2params(theta, tau, g, est_tau, est_g);
        x_hat = t/tau_hat; 
        
        %% Numerical integration
        t = t + dt;

        x = x + dx*dt;

        y = y + dy*dt;
        z = z + dz*dt;
        
    end
    toc
    
    Data_sim{n} = struct('Time',Time, 'Y',y_data, 'dY',dy_data, 'ddY',ddy_data);
    
    plot_estimation_results(Time, g, g_data, tau, tau_data, P_data, F_data, mf_data, plot_1sigma);
    
    plotData(Data_sim);

end



