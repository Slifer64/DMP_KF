%% EKF class
% Implementation of the discrete Extended Kalman Filter algorithm.
% - The Jacobian of the state transition and measurement functions can either
% be provided or approximated numerically.
% - A fading memory coefficient can be defined.
% - Enforcement of linear constraints can be imposed on the estimated parameteres.
%


classdef EKF < handle
    properties
        F_k % forward model
        H_k % measurement model
        K % Kalman gain
        Q % process covariance
        R % measurement covariance
        
        a_p % fading memory coefficient
        theta % params estimation
        P % params estimation covariance
        
        % Apply projection so that:
        % D_up * theta <= d_up
        % D_low * theta >= d_low
        enable_constraints
        D_up % upper bound constraint matrix
        d_up % upper bound constraints
        D_low % lower bound constraint matrix
        d_low % lower bound constraints
        
        apply_cov_sat
        theta_sigma_min
        theta_sigma_max
       
        dtheta % Parameters step size for numerical approximation of transition and measurement function Jacobian
    end

    methods
        %% Extended Kalman Filter constructor
        %  @param[in] N_params: Number of states to estimate.
        %  @param[in] N_out: Number of outputs.
        function this = EKF(N_params, N_out)
            
            this.theta = zeros(N_params);
            this.P = zeros(N_params, N_params);
            
            this.setProcessNoiseCov(eye(N_params,N_params)*1e-5);
            this.setMeasureNoiseCov(eye(N_out,N_out)*0.05);
            
            this.setFadingMemoryCoeff(1.0);
            
            this.enableParamsContraints(false);
            this.setParamsConstraints([],[],[],[]);
            
            this.setPartDerivStep(0.001);

        end
        
        %% Sets the fading memory coefficient. 
        %  Note if the continuous time fading memory coefficient is 'a' then 
        %  the discrete one is 'a_p = exp(a*Ts)' where Ts is the sampling period.
        %  @param[in] a_p: Fading memory coefficient
        function setFadingMemoryCoeff(this, a_p)
            
            this.a_p = a_p;
            
        end
        
        %% Enables/Disables constraints in the estimation parameters.
        %  Note that the constraints must be set with 'setParamsConstraints' first.
        %  @param[in] enable_contraints: Flag that is true/false for enabling/disabling the constraints.
        function enableParamsContraints(this, enable_contraints)
           
            this.enable_constraints = enable_contraints;
            
        end
           
        %% Sets linear constraints in the estimation parameters.
        %  The constraints are such that D_up*theta <= d_up and D_low*theta >= d_low
        %  @param[in] D_up: Constraint matrix for upper bound constraints.
        %  @param[in] d_up: Limits for upper bound constraints.
        %  @param[in] D_low: Constraint matrix for lower bound constraints.
        %  @param[in] d_low: Limits for lower bound constraints.
        function setParamsConstraints(this, D_up, d_up, D_low, d_low)
            
            this.D_up = D_up;
            this.d_up = d_up;
            
            this.D_low = D_low;
            this.d_low = d_low;
            
        end

        %% Sets the covariance matrix of the process noise.
        %  Note that if the continuous time process noise is R_c the discrete
        %  one is 'R = Q_c/Ts' where Ts is the sampling period.
        %  @param[in] Q: Process noise covariance matrix.
        function setProcessNoiseCov(this, Q)
            
            this.Q = Q;
            
        end
        
        %% Sets the covariance matrix of the measurement noise.
        %% Note that if the continuous time process noise is Q_c the discrete
        %% one is 'Q = Q_c*Ts' where Ts is the sampling period.
        %  @param[in] R: measurement noise covariance matrix.
        function setMeasureNoiseCov(this, R)
            
            this.R = R;
            
        end
            
        %% Sets the step for computing the partial derivatives of the state
        %% transition and/or measurement function w.r.t. the estimation parameters.
        %  @param[in] dtheta: Parameters step size
        function setPartDerivStep(this, dtheta)
            
            this.dtheta = dtheta;
            
        end
        
        %% Performs the EKF prediction (time update).
        %  @params[in] F_k: Jacobian matrix of the state transition function w.r.t to the estimation parameters.
        function predict(this, F_k)
            
            this.F_k = F_k;
            this.theta = this.F_k*this.theta;
            this.P = this.a_p^2*this.F_k*this.P*this.F_k' + this.Q;
    
        end
        
        %% Performs the EKF prediction (time update) using numerical differentiation
        %% for approximating the Jacobian of the state transition matrix w.r.t. the estimation parameters.
        %  The step size for numerical differentiation can be set from 'setPartDerivStep'.
        %  @param[in] state_trans_fun_ptr: Pointer to state transition function. It should accept a 
        %                                  vector (the parameters) and null pointer (cookie).
        %  @param[in] cookie: pointer that is passed to 'state_trans_fun_ptr' and can contain extra 
        %                     arguments needed. If not needed it can be set to empty '[]'.
        function predictApprox(this, state_trans_fun_ptr, cookie)
            
            theta2 = state_trans_fun_ptr(this.theta + this.dtheta, cookie);
            theta1 = state_trans_fun_ptr(this.theta - this.dtheta, cookie);
            F_k_approx = (theta2 - theta1) / 2*dtheta;
            
            this.predict(F_k_approx);
    
        end
        
        %% Performs the EKF correction (measurement update).
        %  @param[in] z: Groundtruth measurements.
        %  @param[in] z_hat: Estimated measurements.
        %  @param[in] H_k: Jacobian of the measurement function w.r.t to the estimation parameters.
        function correct(this, z, z_hat, H_k)
            
            this.H_k = H_k;

            N_params = length(this.theta);
            D = [];
            d = [];

            proj_flag = false;

            K_kf = this.P*this.H_k'/(this.H_k*this.P*this.H_k' + this.R);
            this.theta = this.theta + K_kf * (z - z_hat);

            if (this.enable_constraints)
                N_constraints = size(this.D_up,1);
                for i=1:N_constraints
                    if (dot(this.D_up(i,:),this.theta) > this.d_up(i))
                        d = [d; this.d_up(i)];
                        D = [D; this.D_up(i,:)];
                        proj_flag = true;
                    end
                end
                N_constraints = size(this.D_low,1);
                for i=1:N_constraints
                    if (dot(this.D_low(i,:),this.theta) < this.d_low(i))
                        d = [d; this.d_low(i)];
                        D = [D; this.D_low(i,:)];
                        proj_flag = true;
                    end
                end
            end

            I = eye(N_params, N_params);

            if (proj_flag)
                % K_kf = ( I - this.P*D'/(D*this.P*D')*D ) * K_kf;
                % this.theta = this.theta - this.P*D'/(D*this.P*D')*(D*this.theta-d); 
                K_kf = ( I - D'/(D*D')*D ) * K_kf;
                this.theta = this.theta - D'/(D*D')*(D*this.theta-d); 
            end

            this.P = (I - K_kf*this.H_k) * this.P * (I - K_kf*this.H_k)' + K_kf*this.R*K_kf';
            
            this.K = K_kf;
            
        end

        %% Performs the EKF prediction (time update) using numerical differentiation
        %% for approximating the Jacobian of the state transition matrix w.r.t. the estimation parameters.
        %  The step size for numerical differentiation can be set from 'setPartDerivStep'.
        %  @param[in] state_trans_fun_ptr: Pointer to state transition function. It should accept a 
        %                                  vector (the parameters) and null pointer (cookie).
        %  @param[in] cookie: pointer that is passed to 'state_trans_fun_ptr' and can contain extra 
        %                     arguments needed. If not needed it can be set to empty '[]'.
        function correctApprox(this, measure_trans_fun_ptr, cookie)
            
            theta2 = measure_trans_fun_ptr(this.theta + this.dtheta, cookie);
            theta1 = measure_trans_fun_ptr(this.theta - this.dtheta, cookie);
            H_k_approx = (theta2 - theta1) / 2*dtheta;
            
            this.correct(H_k_approx);
    
        end

    end
end
