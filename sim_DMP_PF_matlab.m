clc;
close all;
clear;

%% ###################################################################

est_g = true;
est_tau = true;

goal_scale = 0.6;
time_scale = 0.7; 

process_noise = 0.002; % Q
msr_noise = 0.01; % R
init_params_variance = 1.0; % P
N_particles = 100;

plot_1sigma = false;

dt = 0.005;

rng(1);

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
    
    g = goal_scale*g;
    
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
    
    %% ###################################################################
    
    N_params = length(theta);
    P_theta = eye(N_params, N_params)*init_params_variance;
    
    Q_w = eye(N_params, N_params)*process_noise;
    sqrt_Q = sqrt(Q_w);
    R_v = eye(N_out, N_out)*msr_noise;
    
    %% === particle filter ===
    pf = robotics.ParticleFilter;
    pf.StateTransitionFcn = @stateTransParticleFilterFun;
    pf.MeasurementLikelihoodFcn = @particleFilterMsrLikelihoodFun;
    pf.StateEstimationMethod = 'mean';
    pf.ResamplingMethod = 'systematic';
    pf.initialize(N_particles, theta, P_theta);
    
    %% ###################################################################

    disp('DMP-PF matlab simulation...')
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

        %% DMP simulation
        for i=1:D  

            y_c = 0.0;
            z_c = 0.0;

            [dy(i), dz(i)] = dmp{i}.getStatesDot(x, y(i), z(i), y0(i), g(i), y_c, z_c);

            ddy(i) = dz(i)/dmp{i}.getTau();
            
            y_out(i) = dmp{i}.getAccel(y(i), dy(i), y0(i), y_c, z_c, x, g(i), dmp{i}.getTau());
            y_out_hat(i) = dmp{i}.getAccel(y(i), dy(i), y0(i), y_c, z_c, x_hat, g_hat(i), tau_hat);
            
        end
        
        msr_cookie = struct('dmp',{dmp}, 't',t, 'y',y, 'dy',dy, 'y0',y0, 'y_c',y_c, 'z_c',z_c, 'x_hat',x_hat, 'g', g, 'tau', tau, 'est_g',est_g, 'est_tau',est_tau);
        
        %% ###################################################################
        %% === particle filter update ===
        pf.correct(y_out, msr_cookie);
        [pred_state, P_state] = pf.predict(sqrt_Q);
        theta = pred_state';
        P_theta = P_state;
        
        [tau_hat, g_hat] = theta2params(theta, tau, g, est_tau, est_g);
        x_hat = t/tau_hat;
        %% ###################################################################

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

%% ###################################################################
%% ###################################################################

function particles = stateTransParticleFilterFun(obj, particles, sqrt_Q)

    [N_particles, numberOfStates] = size(particles);
    
    % Time-propagate each particle
    for i=1:N_particles
        particles(i,:) = stateTransFun(particles(i,:));
    end

    % Add Gaussian noise with variance 'Q' on each state variable
    processNoise = sqrt_Q(1,1)*ones(N_particles, numberOfStates);
    particles = particles + processNoise .* randn(N_particles, numberOfStates);

end

function y = stateTransFun(x)

    y = x;
    
end

%% ###################################################################
%% ###################################################################

function likelihood = particleFilterMsrLikelihoodFun(obj, particles, measurement, msr_cookie)

    % The measurement is first state. Get all measurement hypotheses from particles
    N_particles = size(particles,1);
    
    numberOfMeasurements = length(msr_cookie.dmp); % dim of output
    
    predictedMeasurement = zeros(numberOfMeasurements, N_particles);
    for i=1:N_particles
        theta = particles(i,:)';
        predictedMeasurement(:,i) = msrFun(theta, msr_cookie);
    end

    % Assume the ratio of the error between predicted and actual measurements
    % follow a Gaussian distribution with zero mean, variance 0.2
    mu = zeros(numberOfMeasurements,1); % mean
    sigma = 0.2 * eye(numberOfMeasurements); % variance

    % Use multivariate Gaussian probability density function, calculate
    % likelihood of each particle
    likelihood = zeros(N_particles,1);
    C = det(2*pi*sigma) ^ (-0.5);
    for i=1:N_particles
        errorRatio = (predictedMeasurement(:,i)-measurement)./predictedMeasurement(:,i);
        v = errorRatio-mu;
        likelihood(i) = C * exp(-0.5 * (v' / sigma * v) );
    end    

end

function y_out = msrFun(theta, msr_cookie)

    [tau_hat, g_hat] = theta2params(theta, msr_cookie.tau, msr_cookie.g, msr_cookie.est_tau, msr_cookie.est_g);
    
    D = length(msr_cookie.dmp);
    y_out = zeros(D,1);
    
    for i=1:D
        y_out(i) = msr_cookie.dmp{i}.getAccel(msr_cookie.y(i), msr_cookie.dy(i), msr_cookie.y0(i), msr_cookie.y_c, msr_cookie.z_c, msr_cookie.x_hat, g_hat(i), tau_hat);
    end
    
end
