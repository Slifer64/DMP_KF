function multiSim_DMP_EKF_disc()

%% ==============================================================
%% DMP with state reset (y=y_r, y_dot=y_r_dot) and force feedback, i.e. 
%% y_ddot = h(theta, y_r, y_r_dot, t) + f_ext/M

format compact;

set_matlab_utils_path();

%% ###################################################################

rng(0);

dg_x = [-0.4 -0.2 0.2 0.5];
dg_y = [-0.5 -0.25 0.25 0.45];
dg_z = [-0.45 -0.2 0.2 0.5];
dtau = [-2.0 -1.0 2.0 4.0 6.0 8.0 10.0 12.0];

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

            goal_offset = [dg_x(ix) dg_y(iy) dg_z(iz)]' + 0.1*rand(3,1);
            y0_offset = [0.0 0.0 0.0]';
            time_offset = dtau(it) + 1*rand(1);

            process_noise = 0.0001; % Q
            msr_noise = 0.005; % R
            init_params_variance = 1.0; % P
            a_p = 1.002; % forgetting factor in fading memory EKF

            goal_up_lim = 0.6*[1.0 1.0 1.0]';
            goal_low_lim = -goal_up_lim;
            tau_low_lim = 1.0;
            tau_up_lim = 20.0; %Inf;
            theta_low_lim = [goal_low_lim; tau_low_lim];
            theta_up_lim = [goal_up_lim; tau_up_lim];
            N_params = length(theta_low_lim);
            A_c = [-eye(N_params, N_params); eye(N_params, N_params)];
            b_c = [-theta_low_lim; theta_up_lim];
            enable_constraints = true*1;

            theta_sigma_min = 0.001;
            theta_sigma_max = 100000;
            apply_cov_sat = true*0;
            
            ekf_provide_Jacob = true;

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

            %% ###################################################################

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

            disp('DMP-EKF (discrete) simulation...');
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

                dC_dtheta = zeros(Dim, length(theta));

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

                if (apply_cov_sat)
                    ekf.P = svSat(ekf.P, theta_sigma_min, theta_sigma_max);
                end

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

            Data{id} = struct('Time',Time, 'Y_data',Y_r_data, 'Yg_data',Yg_data, 'Yg',Yg, ...
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






