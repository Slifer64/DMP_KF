%% DMP class
%  Implements an 1-D this.
%  The DMP is driven by a canonical clock. It outputs the phase varialbe 
%  'x' which serves as a substitute for time. Typically, it evolves from 
%  x0=0 at t=0 to x_end=1, at t=tau, where tau is the total movement's 
%  duration. An example of a linear canonical clock is:
%     dx = -ax/tau
%  where x is the phase variable and ax the evolution factor. Other types 
%  of canonical clocks, such as exponential, can be used. However, keeping
%  a linear mapping between the phase variable 'x' and time 't' is more
%  intuitive.
%
%  The DMP has the in general the following form:
%
%     tau*dz = g1(x)*( a_z*(b_z*(g-y) - z ) + g2(x)*fs*f(x) + z_c
%     tau*dy = z + y_c;
%
%  Assuming y_c=z_c=0, we can write equivalently:
%     ddy = g1(x)*( a_z*(b_z*(g-y)-dy*tau) + 2(x)*fs*f(x) ) / tau^2;
%
%  where
%     tau: is scaling factor defining the duration of the motion
%     a_z, b_z: constants relating to a spring-damper system
%     fs: scaling of the forcing term (typically fs = g0-y0)
%     g: the goal-final position
%     y0: the initial position
%     x: the phase variable
%     y,dy,ddy: the position, velocity and accelaration of the motion
%     f(x): the forcing term defined by the normalized weighted sum of the 
%        kernel functions (gaussian kernels), i.e.:
%        f(x) = w'*Psi(x)/ sum(Psi(x));
%     g1(x): the gating factor of the spring-damper term
%     g2(x): the gating factor of non-linear forcing term
%

classdef DMP_smoother < handle % : public DMP_
    properties
        N_kernels % number of kernels (basis functions)

        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        canClockPtr % handle (pointer) to the canonical clock
        shapeAttrGatingPtr % pointer to gating function for the shape attractor

        w % N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP

        zero_tol % tolerance value used to avoid divisions with very small numbers

    end

    methods
        %% DMP constructor
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] canClockPtr: Pointer to a DMP canonical system object.
        function this = DMP_smoother(N_kernels, a_z, b_z, canClockPtr)

            this.shapeAttrGatingPtr = struct('u0',1.0, 'u_end',0.99, 'a_u', 800, 'c',0);
            this.shapeAttrGatingPtr.c = 1.0 - (1.0/this.shapeAttrGatingPtr.a_u)*log((this.shapeAttrGatingPtr.u0-this.shapeAttrGatingPtr.u_end)/this.shapeAttrGatingPtr.u_end);

            this.init(N_kernels, a_z, b_z, canClockPtr);

        end


        %% Initializes the DMP
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] canClockPtr: Pointer to a DMP canonical system object.
        function init(this, N_kernels, a_z, b_z, canClockPtr)

            this.zero_tol = 1e-30;%realmin;

            kernelStdScaling = 1.0;

            this.N_kernels = N_kernels;
            this.a_z = a_z;
            this.b_z = b_z;
            this.canClockPtr = canClockPtr;

            this.w = zeros(this.N_kernels,1); %rand(this.N_kernels,1);
            this.setCenters();
            this.setStds(kernelStdScaling);

        end


        %% Sets the centers for the kernel functions of the DMP according to the canonical system
        function setCenters(this)

            t = ((1:this.N_kernels)-1)/(this.N_kernels-1);
            x = this.phase(t*this.getTau());
            this.c = x(1,:)';

        end


        %% Sets the standard deviations for the kernel functions  of the DMP
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] kernelStdScaling: Scales the std of each kernel by 'kernelStdScaling' (optional, default = 1.0).
        function setStds(this, kernelStdScaling)
            
            if (nargin < 2), kernelStdScaling=1.0; end

            this.h = 1./(kernelStdScaling*(this.c(2:end)-this.c(1:end-1))).^2;
            this.h = [this.h; this.h(end)];

        end


        %% Trains the DMP
        %  @param[in] Time: Row vector with the timestamps of the training data points.
        %  @param[in] yd_data: Row vector with the desired potition.
        %  @param[in] dyd_data: Row vector with the desired velocity.
        %  @param[in] ddyd_data: Row vector with the desired accelaration.
        %  @param[in] y0: Initial position.
        %  @param[in] g: Target-goal position.
        %
        %  \note The timestamps in \a Time and the corresponding position,
        %  velocity and acceleration data in \a yd_data, \a dyd_data and \a
        %  ddyd_data need not be sequantial in time.
        function [train_error, F, Fd] = train(this, train_method, Time, yd_data, dyd_data, ddyd_data)

            n_data = length(Time);
            
            tau0 = this.canClockPtr.getTau();
            
            tau = Time(end);
            y0 = yd_data(:,1);
            g = yd_data(:,end);
            
            this.canClockPtr.setTau(tau);
    
            x = zeros(1, n_data);
            s = zeros(1, n_data);
            Fd = zeros(1,n_data);
            Psi = zeros(this.N_kernels, n_data);
            for i=1:n_data
                x(i) = this.phase(Time(i));
                s(i) = this.forcingTermScaling(y0, g) * this.shapeAttrGating(x(i));
                Fd(i) = this.calcFd(x(i), yd_data(i), dyd_data(i), ddyd_data(i), y0, g);
                Psi(:,i) = this.kernelFunction(x(i));
            end

            if (strcmpi(train_method,'LWR'))
                this.w = this.LWR(Psi, s, Fd, this.zero_tol);
            else
                error('Unsopported training method ''%s''', train_method);
            end

            F = zeros(size(Fd));
            for i=1:size(F,2)
                F(i) = this.shapeAttractor(x(i), y0, g);
            end

            train_error = norm(F-Fd)/length(F);
            
%             this.canClockPtr.setTau(tau0);

        end
        

        %% Calculates the desired values of the scaled forcing term.
        %  @param[in] x: The phase variable.
        %  @param[in] y: Position.
        %  @param[in] dy: Velocity.
        %  @param[in] ddy: Acceleration.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] Fd: Desired value of the scaled forcing term.
        function Fd = calcFd(this, x, y, dy, ddy, y0, g)

            tau = this.getTau();
            Fd = (ddy*tau^2 - this.goalAttractor(x, y, tau*dy, g));

        end


        %% Returns the forcing term of the this.
        %  @param[in] x: The phase variable.
        %  @param[out] f: The normalized weighted sum of Gaussians.
        function f = forcingTerm(this,x)

            Psi = this.kernelFunction(x);
    
            f = dot(Psi,this.w) / (sum(Psi)+this.zero_tol); % add 'zero_tol' to avoid numerical issues

        end


        %% Returns the scaling factor of the forcing term.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] f_scale: The scaling factor of the forcing term.
        function f_scale = forcingTermScaling(this, y0, g)

            f_scale = (g-y0);

        end
        
        
        %% Returns the shape attractor gating factor.
        %  @param[in] x: The phase variable.
        function sAttrGat = shapeAttrGating(this, x)

            exp_t = exp((this.shapeAttrGatingPtr.a_u)*(x-this.shapeAttrGatingPtr.c));
            sAttrGat = this.shapeAttrGatingPtr.u0 * 1.0 ./ (1.0 + exp_t);
          
            sAttrGat(sAttrGat<0) = 0.0;

        end
        
        
        %% Returns the goal attractor gating factor.
        %  @param[in] x: The phase variable.
        function gAttrGat = goalAttrGating(this, x)

            gAttrGat = 1.0;

        end


        %% Returns the goal attractor of the this.
        %  @param[in] x: The phase variable.
        %  @param[in] y: \a y state of the this.
        %  @param[in] z: \a z state of the this.
        %  @param[in] g: Goal position.
        %  @param[out] goal_attr: The goal attractor of the this.
        function goal_attr = goalAttractor(this, x, y, z, g)

            g_attr_gating = this.goalAttrGating(x);
            goal_attr = g_attr_gating * this.a_z*(this.b_z*(g-y)-z);

        end


        %% Returns the shape attractor of the this.
        %  @param[in] x: The phase variable.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[out] shape_attr: The shape_attr of the this.
        function shape_attr = shapeAttractor(this, x, y0, g)
            
            s_attr_gating = this.shapeAttrGating(x);
            f = this.forcingTerm(x);
            f_scale = this.forcingTermScaling(y0, g);
            shape_attr = s_attr_gating * f * f_scale;
            
        end
        
        
        %% Returns the phase variable.
        %  @param[in] t: The time instant.
        %  @param[out] x: The phase variable for time 't'.
        function x = phase(this, t)
            
            x = this.canClockPtr.getPhase(t);

        end
        
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @param[out] dx: The derivative of the phase variable.
        function dx = phaseDot(this, x)
            
            dx = this.canClockPtr.getPhaseDot(x);

        end


        %% Returns the derivatives of the DMP states
        %  @param[in] x: phase variable.
        %  @param[in] y: \a y state of the this.
        %  @param[in] z: \a z state of the this.
        %  @param[in] y0: initial position.
        %  @param[in] g: Goal position.
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        %  @param[out] dy: derivative of the \a y state of the this.
        %  @param[out] dz: derivative of the \a z state of the this.
        %  @param[out] dx: derivative of the phase variable of the this.
        function [dy, dz, dx] = getStatesDot(this, x, y, z, y0, g, y_c, z_c)

            if (nargin < 8), y_c=0; end
            if (nargin < 7), z_c=0; end

            tau = this.getTau();

            shape_attr = this.shapeAttractor(x, y0, g);
            goal_attr = this.goalAttractor(x, y, z, g);

            dz = ( goal_attr + shape_attr + z_c) / tau;

            dy = ( z + y_c) / tau;

            dx = this.phaseDot(x);

        end


        %% Returns a column vector with the values of the kernel functions of the DMP
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the kernel functions of the DMP
        function psi = kernelFunction(this,x)

            n = length(x);
            psi = zeros(this.N_kernels, n);

            for j=1:n
                psi(:,j) = exp(-this.h.*((x(j)-this.c).^2));
            end 

        end

        
        %% Returns the time cycle of the DMP
        %  @param[out] tau: The time duration of the this.
        function tau = getTau(this)

            tau = this.canClockPtr.getTau();

        end
        
        function w = LWR(this, Psi, X, Fd, zero_tol)
  
            if (nargin < 4), zero_tol = 0.0; end

            N_kernels = size(Psi,1);
            w_dim = size(X,1);

            w = zeros(N_kernels, w_dim);

            for k=1:N_kernels
              X_Psi = X .* repmat(Psi(k,:), size(X,1), 1);
              w(k,:) = ((X_Psi*X' + zero_tol) \ X_Psi*Fd')';

            %       w(k,:) = (pinv(X_Psi*X' + zero_tol) * X_Psi*Fd')';
            end

        end

    end
end
