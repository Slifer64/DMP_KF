%% UKF class
% Implementation of the Unscented Kalman Filter algorithm.
% Stores the high level parameters of UKF concerning the weights of sigma
% points. Implements the prediction (time update) and correction (measurement 
% update) given the current state estimate, state covariance, noise
% covariance and state transition and measurement model respectively.
%
% The spread of the sigma points around the mean state value is controlled by 
% two parameters Alpha and Kappa. A third parameter, Beta, impacts the weights 
% of the transformed points during state and measurement covariance calculations:
%
%    Alpha — Determines the spread of the sigma points around the mean state value. 
%            It is usually a small positive value. The spread of sigma points is 
%            proportional to Alpha. Smaller values correspond to sigma points closer 
%            to the mean state.
%
%    Kappa — A second scaling parameter that takes values in [0 3]. It is usually 
%            set to 0. Smaller values correspond to sigma points closer to the mean 
%            state. The spread is proportional to the square-root of Kappa.
%
%    Beta — Incorporates prior knowledge of the distribution of the state. 
%           For Gaussian distributions, Beta = 2 is optimal.
%
% If you know the distribution of state and state covariance, you can adjust 
% these parameters to capture the transformation of higher-order moments of 
% the distribution. The algorithm can track only a single peak in the probability 
% distribution of the state. If there are multiple peaks in the state distribution 
% of your system, you can adjust these parameters so that the sigma points stay 
% around a single peak. For example, choose a small Alpha to generate sigma points 
% close to the mean state value.
%
% Sample usage:
%{
    x = [1.0; 0.4; 2.0];                    % actual state
    x_hat = [0.2; 1; 0.5];                  % state estimate
    P = eye(3,3)*100;                       % state estimate covariance

    stateFun = @(x) sin(x).*x + 0.5*x.^2;    % state transition function
    msrFun = @(x) [x(1)^2+x(2); cos(x(3))]; % measurement function

    alpha = 1e-3;
    beta = 2;
    k = 0;
    N_params = 3;

    ukf = UKF(alpha, beta, k, N_params);    % construct UKF object

    R = eye(2,2)*0.5;                       % measurement noise covariance
    Q = eye(3,3)*0.1;                       % process noise covariance
    R_sqrt = sqrt(R);
    Q_sqrt = sqrt(Q);

    x_data = [];                            % for logging actual state
    x_hat_data = [];                        % for logging estimated state
    iter_limit = 400;                       % number of iterations to run the simulation
    iters = 0;

    while (true)
        
        % logging
        x_data = [x_data x];
        x_data = [x_hat_data x_hat];

        [x_hat, P] = ukf.prediction(stateFun, x_hat, P, Q);

        x = stateFun(x);
        % x = x + Q_sqrt*randn(3,1);
        y = msrFun(x);
        y_n = y + R_sqrt*randn(2,1);

        [x_hat, P] = ukf.correct(msrFun, x_hat, P, y, R);

        iters = iters + 1;
        if (iters == iter_limit), break; end
        
    end

    figure;
    T = 1:iters;
    for i=1:3
        subplot(T,x_data(i,:), T,x_hat_data(i,:));
        if (i==1), legend({'x','$\hat{x}$'}, 'interpreter','latex', 'fontsize',14); end
        if (i==1), title('UKF states estimation', 'interpreter','latex', 'fontsize',14); end
        if (i==3), xlabel('# iter', 'interpreter','latex', 'fontsize',14); end
        ylabel(['Dim $' num2str(i) '$'], 'interpreter','latex', 'fontsize',14);
    end

%}
%


classdef UKF < handle
    properties
        alpha % alpha parameter of UKF
        beta % beta parameter of UKF
        k % k parameter of UKF
        L % number of states to estimate
        
        lambda %
        
        W_s % weights for state update
        W_c % weights for state covariance update

    end

    methods
        %% Unscented Kalman Filter constructor
        %  @param[in] alpha: alpha parameter of UKF.
        %  @param[in] beta: beta parameter of UKF.
        %  @param[in] k: k parameter of UKF.
        %  @param[in] N_params: number of states to estimate.
        function this = UKF(alpha, beta, k, N_params)
            
            this.alpha = alpha;
            this.beta = beta;
            this.k = k;
            
            this.L = N_params;
            
            this.lambda = alpha^2*(this.L - this.k) - this.L;
            
            N_sigma = 2*this.L + 1;
    
            this.W_s = zeros(N_sigma,1);
            this.W_s(1) = this.lambda/(this.L + this.lambda);
            this.W_s(2:end) = 0.5/(this.L + this.lambda);

            this.W_c = zeros(N_sigma,1);
            this.W_c(1) = this.lambda/(this.L + this.lambda) + (1 - this.alpha^2 + this.beta);
            this.W_c(2:end) = 0.5/(this.L + this.lambda);
        end


        %% Performs the UKF prediction (time update).
        %  @param[in] stateFun: state transition function pointer.
        %  @param[in] x: current state.
        %  @param[in] P: state covariance.
        %  @param[in] Q: process noise covariance.
        function [x, P] = predict(this, stateFun, x, P, Q)

            X = this.genSigmaPoints(x, P);

            [x, P] = this.unscentedTransform(stateFun, X, Q);
            
        end
        
        %% Performs the UKF correction (measurement update).
        %  @param[in] msrFun: measurement function pointer.
        %  @param[in] x: current state.
        %  @param[in] P: state covariance.
        %  @param[in] y_out: Groundtruth measurements.
        %  @param[in] R: measurement noise covariance.
        function [x, P] = correct(this, msrFun, x, P, y_out, R)

            X = this.genSigmaPoints(x, P);
            N_sigma = size(X,2);
            
            [y, Py, ~, Ydiff] = this.unscentedTransform(msrFun, X, R);

            Xdiff = X - repmat(x,1,N_sigma);
%             Ydiff = Y - repmat(y,1,N_sigma);
            Pxy = (repmat(this.W_c',size(Xdiff,1),1).*Xdiff)*Ydiff';

            K = Pxy/Py;

            x = x + K*(y_out - y);
            P = P - K*Py*K';
            
        end
        
        %% Performs the unscented transform of the points in X throught the nonlinear mapping 'fmap'
        %  @param[in] fmap: nonlinear mapping function from x to y.
        %  @param[in] X: points arranged in columns.
        %  @param[in] Sigma_n: additive noise covariance.
        %  @param[out] y: average of transformed points.
        %  @param[out] Y: transformed points arranged in columns.
        %  @param[out] Ydiff: difference of transformed points arranged in columns from the mean y.
        function [y, Py, Y, Ydiff] = unscentedTransform(this, fmap, X, Sigma_n)
            
            N_sigma = size(X,2);
            N_out = length(fmap(X(:,1)) );

            % pass the points in X through the mapping 'fmap'
            Y = zeros(N_out, N_sigma);
            for j=1:size(X,2)
                Y(:,j)  = fmap(X(:,j));
            end

            % calculate average
            y = Y*this.W_s;
            
            % calculate covariance
            Ydiff = Y - repmat(y,1,N_sigma);
            Py = (repmat(this.W_c',size(Ydiff,1),1).*Ydiff)*Ydiff' + Sigma_n;
            
        end
        
        %% Generates sigma points given x and its covariance P
        %  @param[in] x: column vector of point around which sigma points are generated.
        %  @param[in] P: covariance around x.
        %  @param[out] X: generated sigma points arranged in columns.
        function X = genSigmaPoints(this, x, P)
            
            X = zeros(this.L, 2*this.L+1);

            P_sqrt = chol(P,'lower');

            X(:,1) = x;
            X(:,2:this.L+1) = repmat(x,1,this.L) + sqrt(this.L + this.lambda)*P_sqrt;
            X(:,this.L+2:end) = repmat(x,1,this.L) - sqrt(this.L + this.lambda)*P_sqrt;
            
        end


    end
end
