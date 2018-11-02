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
        A_c % constraint matrix
        b_c % constraints bounds
        
        apply_cov_sat
        theta_sigma_min
        theta_sigma_max
        
        msr_fun_ptr % measurement function pointer
        state_trans_fun_ptr % state transition function pointer
        
        msr_fun_Jacob_ptr % Jacobian of measurement function function pointer
        state_trans_fun_Jacob_ptr % Jacobian state transition function function pointer
       
        dtheta % Parameters step size for numerical approximation of transition and measurement function Jacobian
    end

    methods
        %% Extended Kalman Filter constructor
        %  @param[in] N_params: Number of states to estimate.
        %  @param[in] N_out: Number of outputs.
        %  @param[in] state_trans_fun_ptr: Pointer to state transition function. It should accept a vector (the parameters) and 
        %                                  a void pointer (cookie) to pass extra arguments to the state transition function.
        %  @param[in] msr_fun_ptr: Pointer to measurement function. It should accept a vector (the parameters) and 
        %                                  a void pointer (cookie) to pass extra arguments to the measurement function.
        function this = EKF(N_params, N_out, state_trans_fun_ptr, msr_fun_ptr)
            
            this.theta = zeros(N_params);
            this.P = zeros(N_params, N_params);
            
            this.setProcessNoiseCov(eye(N_params,N_params)*1e-5);
            this.setMeasureNoiseCov(eye(N_out,N_out)*0.05);
            
            this.setFadingMemoryCoeff(1.0);
            
            this.enableParamsContraints(false);
            this.setParamsConstraints([],[]);
            
            this.setPartDerivStep(0.001);
            
            this.state_trans_fun_ptr = state_trans_fun_ptr;
            this.msr_fun_ptr = msr_fun_ptr;
            
            this.msr_fun_Jacob_ptr = [];
            this.state_trans_fun_Jacob_ptr = [];

        end
        
        %% Sets the fading memory coefficient. 
        %  Note if the continuous time fading memory coefficient is 'a' then 
        %  the discrete one is 'a_p = exp(a*Ts)' where Ts is the sampling period.
        %  @param[in] a_p: Fading memory coefficient.
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
        %  The constraints are such that D*theta <= d
        %  @param[in] A_c: Constraint matrix.
        %  @param[in] b_c: Constraints bounds.
        function setParamsConstraints(this, A_c, b_c)
            
            this.A_c = A_c;
            this.b_c = b_c;
            
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
        %  @param[in] R: Measurement noise covariance matrix.
        function setMeasureNoiseCov(this, R)
            
            this.R = R;
            
        end
            
        %% Sets the state transition function Jacobian.
        %  @param[in] state_trans_fun_Jacob_ptr: Pointer to the state transition function Jacobian. It should accept a vector (the parameters)
        %                                        and a void pointer (cookie) to pass extra arguments to the function.
        function setStateTransFunJacob(this, state_trans_fun_Jacob_ptr)
            
            this.state_trans_fun_Jacob_ptr = state_trans_fun_Jacob_ptr;
            
        end
        
        %% Sets the measurement function Jacobian.
        %  @param[in] msr_fun_Jacob_ptr: Pointer to the measurement function Jacobian. It should accept a vector (the parameters)
        %                                        and a void pointer (cookie) to pass extra arguments to the function.
        function setMsrFunJacob(this, msr_fun_Jacob_ptr)
            
            this.msr_fun_Jacob_ptr = msr_fun_Jacob_ptr;
            
        end
        
        %% Sets the step for computing the partial derivatives of the state
        %% transition and/or measurement function w.r.t. the estimation parameters.
        %  @param[in] dtheta: Scalar or vector of parameters step size.
        function setPartDerivStep(this, dtheta)
            
            if (isscalar(dtheta))
                this.dtheta = ones(length(this.theta),1)*dtheta;
            else
                this.dtheta = dtheta;
            end

        end
        
        %% Performs the EKF prediction (time update).
        %  @params[in] cookie: Void pointer to additional arguments needed by the state transition function
        %                      and its Jacobian (if it was set by 'setStateTransFunJacob').
        %                      If not needed set to empty [] (null).
        function predict(this, cookie)
            
            N_params = length(this.theta);
            this.F_k = zeros(N_params,N_params);
            
            if (isempty(this.state_trans_fun_Jacob_ptr))
                % compute Jacobian numerically
                dtheta_j = zeros(N_params,1);
                for j=1:N_params
                    dtheta_j(j) = this.dtheta(j);
                    Ftheta2 = this.state_trans_fun_ptr(this.theta + dtheta_j, cookie);
                    Ftheta1 = this.state_trans_fun_ptr(this.theta - dtheta_j, cookie);
                    this.F_k(:,j) = (Ftheta2 - Ftheta1) / (2*this.dtheta(j));
                    dtheta_j(j) = 0.0;
                end
            else
                this.F_k = this.state_trans_fun_Jacob_ptr(this.theta, cookie);
            end
                           
            this.theta = this.state_trans_fun_ptr(this.theta);
            this.P = this.a_p^2*this.F_k*this.P*this.F_k' + this.Q;
    
        end
        
        %% Performs the EKF correction (measurement update).
        %  @param[in] z: Groundtruth measurement.
        %  @params[in] cookie: Void pointer to additional arguments needed by the measurement function
        %                      and its Jacobian (if it was set by 'setMsrFunJacob'). 
        %                      If not needed set to empty [] (null).
        function correct(this, z, cookie)
            
            N_params = length(this.theta);
            N_out = length(z);
            
            % =====  Retrive the measurement function Jacobian  ===== 
            this.H_k = zeros(N_out,N_params);
            
            if (isempty(this.msr_fun_Jacob_ptr))
                % compute Jacobian numerically
                dtheta_j = zeros(N_params,1);
                for j=1:N_params
                    dtheta_j(j) = this.dtheta(j);
                    Htheta2 = this.msr_fun_ptr(this.theta + dtheta_j, cookie);
                    Htheta1 = this.msr_fun_ptr(this.theta - dtheta_j, cookie);
                    this.H_k(:,j) = (Htheta2 - Htheta1) / (2*this.dtheta(j));
                    dtheta_j(j) = 0.0;
                end
            else
                this.H_k = this.msr_fun_Jacob_ptr(this.theta, cookie);
            end
            
            % =====  Correction estimates ===== 
            z_hat = this.msr_fun_ptr(this.theta, cookie);
            K_kf = this.P*this.H_k'/(this.H_k*this.P*this.H_k' + this.R);
            this.theta = this.theta + K_kf * (z - z_hat);
            
            % =====  Apply projection if enabled  ===== 
            D = []; % active contraints
            d = [];

            proj_flag = false;
            
            if (this.enable_constraints)
                N_constraints = size(this.A_c,1);
                for i=1:N_constraints
                    if (dot(this.A_c(i,:),this.theta) > this.b_c(i))
                        d = [d; this.b_c(i)];
                        D = [D; this.A_c(i,:)];
                        proj_flag = true;
                    end
                end
            end

            I = eye(N_params, N_params);
            
            if (proj_flag)
                % K_kf = ( I - this.P*D_a'/(D_a*this.P*D_a')*D_a ) * K_kf;
                K_kf = ( I - D'/(D*D')*D ) * K_kf;
                this.theta = this.theta - D'/(D*D')*(D*this.theta-d); 
            end

            % =====  Calculate new covariance  =====
            this.P = (I - K_kf*this.H_k) * this.P * (I - K_kf*this.H_k)' + K_kf*this.R*K_kf';
            
            this.K = K_kf;

        end


    end
end
