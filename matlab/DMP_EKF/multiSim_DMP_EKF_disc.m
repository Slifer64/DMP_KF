function multiSim_DMP_EKF_disc()

%% ==============================================================
%% DMP with state reset (y=y_r, y_dot=y_r_dot) and force feedback, i.e. 
%% y_ddot = h(theta, y_r, y_r_dot, t) + f_ext/M

format compact;

set_matlab_utils_path();

%% ###################################################################

dg_x = [-0.4 0.3 0.5];
dg_y = [-0.5 -0.3 0.4];
dg_z = [-0.45 -0.25 0.25 0.5];
dtau = [-2.0 2.0 4.0 8.0 10.0];

Data = cell(length(dg_x)*length(dg_y)*length(dg_z)*length(dtau), 1);

id = 1;

n_total = length(Data);

fprintf('Progress: %.1f %%\n', 0*100.0/n_total);

for ix=1:length(dg_x)
    for iy=1:length(dg_y)
        for iz=1:length(dg_z)
            for it=1:length(dtau)
            
            dt = 0.002;

            goal_scale = [1.0 1.0 1.0]';
            time_scale = 1.0; 

            goal_offset = [dg_x(ix) dg_y(iy) dg_z(iz)]';
            y0_offset = [0.0 0.0 0.0]';
            time_offset = dtau(it);

            process_noise = 0.01*dt; % Q
            msr_noise = 0.001/dt; % R
            init_params_variance = 1.0; % P
            a_p = 2.0; % forgetting factor in fading memory EKF

            goal_up_lim = [0.6 0.6 0.6];
            goal_low_lim = -[0.6 0.6 0.6];
            tau_low_lim = 1.0;
            tau_up_lim = 20.0; %Inf;
            theta_low_lim = [goal_low_lim tau_low_lim];
            theta_up_lim = [goal_up_lim tau_up_lim];
            enable_constraints = true*1;

            theta_sigma_min = 0.001;
            theta_sigma_max = 100000;
            apply_cov_sat = true*1;


            plot_1sigma = false;

            stiff_human = false;

            a_py = 150;
            a_dpy = 50;
            M_r = 5*eye(3,3);
            inv_M_r = inv(M_r);
            K_r = 300*eye(3,3);
            D_r = 2*sqrt(M_r*K_r);

            M_h = 4*eye(3,3);
            inv_M_h = inv(M_h);
            K_h = 500*eye(3,3);
            D_h = 2*sqrt(M_h*K_h);

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

            load('data/dmp_data.mat', 'dmp_data');

            dmp = dmp_data.dmp;
            g0 = dmp_data.g;
            y0 = dmp_data.y0;
            tau0 = 5; %dmp_data.tau;
            Dim = length(y0);
            can_clock_ptr = dmp{1}.can_clock_ptr;
            can_clock_ptr.setTau(tau0);

            %% DMP simulation
            % set initial values
            t = 0.0;
            iters = 0;
            g = g0;
            x = 0.0; dx = 0.0;
            y0 = y0 + y0_offset;
            y = y0; dy = zeros(Dim,1); ddy = zeros(Dim,1);
            y_hat = y0; dy_hat = zeros(Dim,1); ddy_hat = zeros(Dim,1);
            y_r = y0; dy_r = zeros(Dim,1); ddy_r = zeros(Dim,1);
            y_h = y0; dy_h = zeros(Dim,1); ddy_h = zeros(Dim,1);
            z = zeros(Dim,1); dz = zeros(Dim,1);
            F_ext = zeros(Dim,1);
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

            g = goal_scale.*g + goal_offset;

            t_end = time_scale*tau0 + time_offset;
            tau = t_end;
            can_clock_ptr.setTau(tau);

            tau_hat = tau0;
            g_hat = g0;

            x_hat = t/tau_hat;
            N_out = length(g_hat);
            y_out_hat = zeros(N_out,1);
            y_out = zeros(N_out,1);

            theta = [g_hat; tau_hat];

            N_params = length(theta);
            P_theta = eye(N_params, N_params) * init_params_variance;
            R = eye(N_out,N_out)*msr_noise;
            inv_R = inv(R);
            Q = eye(N_params,N_params) * process_noise;

            %% Set up EKF object
            ekf = EKF(N_params, N_out);
            ekf.setProcessNoiseCov(Q);
            ekf.setMeasureNoiseCov(R);
            ekf.setFadingMemoryCoeff(exp(a_p*dt));
            ekf.theta = theta;
            ekf.P = P_theta;

            ekf.enableParamsContraints(enable_constraints);
            ekf.setParamsConstraints(eye(N_params, N_params),theta_up_lim, eye(N_params, N_params),theta_low_lim);

            F_k = eye(N_params,N_params);

            disp('DMP-EKF (discrete) simulation...');
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

                P_data = [P_data sqrt(diag(P_theta))];

                dC_dtheta = zeros(Dim, length(theta));

                y_hat = y_r;
                dy_hat = dy_r;
                Y_c = a_py*(y_r-y_hat) + a_dpy*(dy_r-dy_hat);
                Z_c = zeros(Dim,1);

                %% DMP simulation
                for i=1:Dim  

                    ddy(i) = dmp{i}.getAccel(y(i), dy(i), y0(i), 0, 0, x, g(i), dmp{i}.getTau());
                    ddy_hat(i) = dmp{i}.getAccel(y_hat(i), dy_hat(i), y0(i), 0, 0, x_hat, g_hat(i), tau_hat);

                    dC_dtheta_i = dmp{i}.getAcellPartDev_g_tau(t, y_hat(i), dy_hat(i), y0(i), x_hat, g_hat(i), tau_hat);

                    dC_dtheta(i,end) = dC_dtheta_i(2);
                    dC_dtheta(i,i) = dC_dtheta_i(1);     

                end

                if (stiff_human)
                    F_ext = M_r*(ddy-ddy_hat) + M_r*( inv_M_r*(D_r*(dy_r-dy_hat)+K_r*(y_r-y_hat)) - inv_M_h*(D_h*(dy_h-dy)+K_h*(y_h-y)) );
                else
                    F_ext = inv_M_rh*(ddy-ddy_hat) + inv_M_rh*( inv_M_r*(D_r*(dy_r-dy_hat)+K_r*(y_r-y_hat)) - inv_M_h*(D_h*(dy_h-dy)+K_h*(y_h-y)) );
                end

                y_out_hat = ddy_hat;
                y_out = ddy_r;
                kf_err = y_out - y_out_hat;
                % kf_err = inv(M_r)*F_ext;

                ddy_r = ddy_hat + inv_M_r*(-D_r*(dy_r-dy_hat) - K_r*(y_r-y_hat) + F_ext);
                if (stiff_human)
                    ddy_h = ddy + inv_M_h*(-D_h*(dy_h-dy) - K_h*(y_h-y));
                else
                    ddy_h = ddy + inv_M_h*(-D_h*(dy_h-dy) - K_h*(y_h-y) - F_ext);
                end

                if (norm(ddy_h-ddy_r)>1e-12)
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
                % time update
                ekf.predict(F_k);
                % measurement update
                ekf.correct(y_out, y_out_hat, dC_dtheta);

                if (apply_cov_sat)
                    ekf.P = svSat(ekf.P, theta_sigma_min, theta_sigma_max);
                end

                theta = ekf.theta;
                P_theta = ekf.P;

                g_hat = theta(1:end-1);
                tau_hat = theta(end);
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

            Data{id} = struct('Time',Time, 'Y_data',y_r_data, 'Yg_data',g_data, 'Yg',g, ...
                              'tau_data', tau_data, 'tau',tau, ...
                              'Yg_offset',goal_offset, 'tau_offset',time_offset);
            
            fprintf('Progress: %.1f %%\n', id*100.0/n_total);
            
            id = id + 1;

            end
        end
    end
end

save('data/multisim_data.mat', 'Data');

plotMultiEstSettlings(Data);






