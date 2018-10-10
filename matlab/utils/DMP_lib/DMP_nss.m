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
%     ddy = g1(x)*( a_z*(b_z*(g-y)-dy*tau) + g2(x)*f(x) ) / tau^2;
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

classdef DMP_nss < handle % : public DMP_
    properties
        N_kernels % number of kernels (basis functions)

        a_z % parameter 'a_z' relating to the spring-damper system
        b_z % parameter 'b_z' relating to the spring-damper system

        can_clock_ptr % handle (pointer) to the canonical clock
        shape_attr_gating_ptr % pointer to gating function for the shape attractor
        
        w % N_kernelsx1 vector with the weights of the DMP
        c % N_kernelsx1 vector with the kernel centers of the DMP
        h % N_kernelsx1 vector with the kernel stds of the DMP

        zero_tol % tolerance value used to avoid divisions with very small numbers

    end

    methods
        %% DMP constructor.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        function this = DMP_nss(N_kernels, a_z, b_z, can_clock_ptr, s_gat_ptr)

            if (nargin < 5)
                this.shape_attr_gating_ptr = SigmoidGatingFunction(1.0, 0.99);
            else
                this.shape_attr_gating_ptr = s_gat_ptr;
            end
%             this.shape_attr_gating_ptr = SigmoidGatingFunction(1.0, 0.99);
%             this.shape_attr_gating_ptr = LinGatingFunction(1.0, 0.0);
%             this.shape_attr_gating_ptr = ExpGatingFunction(1.0, 0.05);

            this.init(N_kernels, a_z, b_z, can_clock_ptr);

        end


        %% Initializes the DMP.
        %  @param[in] N_kernels: the number of kernels
        %  @param[in] a_z: Parameter 'a_z' relating to the spring-damper system.
        %  @param[in] b_z: Parameter 'b_z' relating to the spring-damper system.
        %  @param[in] can_clock_ptr: Pointer to a DMP canonical system object.
        function init(this, N_kernels, a_z, b_z, can_clock_ptr)

            this.zero_tol = 1e-30; %realmin;

            this.N_kernels = N_kernels;
            this.a_z = a_z;
            this.b_z = b_z;
            this.can_clock_ptr = can_clock_ptr;

            this.w = zeros(this.N_kernels,1);
            this.setCenters();
            kernel_std_scaling = 1.0;
            this.setStds(kernel_std_scaling);

        end
        
        function n_ker = getNumOfKernels(this)
            
            n_ker = length(this.w);
            
        end
        
        %% Trains the DMP.
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
            
            tau0 = this.getTau();
            
            tau = Time(end);
            y0 = yd_data(:,1);
            g = yd_data(:,end);
            
            this.setTau(tau);
    
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

                this.w = LWR(Psi, s, Fd, this.zero_tol);

            elseif (strcmpi(train_method,'LS'))

                this.w = leastSquares(Psi, s, Fd, this.zero_tol);

            else
                error('Unsopported training method ''%s''', train_method);
            end

            F = zeros(size(Fd));
            for i=1:size(F,2)
                F(i) = this.shapeAttractor(x(i), y0, g);
            end

            train_error = norm(F-Fd)/length(F);
            
%             this.setTau(tau0);

        end
        
        
        %% Returns the derivatives of the DMP states.
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

        
        %% Returns the time scale of the DMP.
        %  @param[out] tau: The time scale of the this.
        function tau = getTau(this)

            tau = this.can_clock_ptr.getTau();

        end
        
        
        %% Sets the time scale of the DMP.
        function tau = setTau(this, tau)

            this.can_clock_ptr.setTau(tau);

        end
        
        
        %% Returns the phase variable.
        %  @param[in] t: The time instant.
        %  @param[out] x: The phase variable for time 't'.
        function x = phase(this, t)
            
            x = this.can_clock_ptr.getPhase(t);

        end
        
        
        %% Returns the derivative of the phase variable.
        %  @param[in] x: The phase variable.
        %  @param[out] dx: The derivative of the phase variable.
        function dx = phaseDot(this, x)
            
            dx = this.can_clock_ptr.getPhaseDot(x);

        end


        %% Returns a column vector with the values of the kernel functions of the DMP.
        %  @param[in] x: phase variable
        %  @param[out] psi: column vector with the values of the kernel functions of the DMP
        function psi = kernelFunction(this,x)

            n = length(x);
            psi = zeros(this.N_kernels, n);

            for j=1:n
                psi(:,j) = exp(-this.h.*((x(j)-this.c).^2));
            end 

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
        
        
        %% Returns the shape attractor gating factor.
        %  @param[in] x: The phase variable.
        function sAttrGat = shapeAttrGating(this, x)

            sAttrGat = this.shape_attr_gating_ptr.getOutput(x);
            sAttrGat(sAttrGat<0) = 0.0;

        end
        
        
        %% Returns the goal attractor gating factor.
        %  @param[in] x: The phase variable.
        function gAttrGat = goalAttrGating(this, x)
            
            gAttrGat = 1.0;

        end

        
        %% Returns the forcing term of the dmp.
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

            f_scale = 1.0;

        end
        
        
        %% Sets the centers for the kernel functions of the DMP according to the canonical system.
        function setCenters(this)

            t = ((1:this.N_kernels)-1)/(this.N_kernels-1);
            x = this.phase(t*this.getTau());
            this.c = x(1,:)';

        end


        %% Sets the standard deviations for the kernel functions  of the DMP.
        %  Sets the variance of each kernel equal to squared difference between the current and the next kernel.
        %  @param[in] kernelStdScaling: Scales the std of each kernel by 'kernelStdScaling' (optional, default = 1.0).
        function setStds(this, kernelStdScaling)
            
            if (nargin < 2), kernelStdScaling=1.0; end

            this.h = 1./(kernelStdScaling*(this.c(2:end)-this.c(1:end-1))).^2;
            this.h = [this.h; this.h(end)];

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
        
        
        %% Returns the partial derivative of the DMP's acceleration wrt to the goal and tau.
        %  @param[in] t: current timestamp.
        %  @param[in] y: position.
        %  @param[in] dy: velocity.
        %  @param[in] y0: initial position.
        %  @param[in] x_hat: phase variable estimate.
        %  @param[in] g_hat: goal estimate.
        %  @param[in] tau_hat: time scale estimate.
        %  @param[out] dC_dtheta: partial derivative of the DMP's acceleration wrt to the goal and tau.
        function dC_dtheta = getAcellPartDev_g_tau(this, t, y, dy, y0, x_hat, g_hat, tau_hat)

            dC_dtheta = zeros(2,1);
            
            K_dmp = this.a_z*this.b_z;
            D_dmp = this.a_z;
            psi = this.kernelFunction(x_hat);
            sum_psi = sum(psi) + this.zero_tol;
            sum_w_psi = psi'*this.w;
            shape_attr_gat = this.shapeAttrGating(x_hat);

            theta1 = g_hat;
            theta2 = 1/tau_hat;
            
            dshape_attr_gat_dtheta2 = this.shape_attr_gating_ptr.getPartDev_1oTau(t,x_hat);
            
            dPsidtheta2 = -2*t*this.h.*(theta2*t-this.c).*psi;
            sum_w_dPsidtheta2 = this.w'*dPsidtheta2;
            dSumWPsi_dtheta2 = (sum_w_dPsidtheta2*sum_psi - sum_w_psi*sum(dPsidtheta2) ) / sum_psi^2;
            
            dC_dtheta(1) = (K_dmp + shape_attr_gat*sum_w_psi/sum_psi)*theta2^2;
            
            dC_dtheta(2) = 2*theta2* (K_dmp*(theta1-y) + shape_attr_gat*(theta1-y0)*sum_w_psi/sum_psi) + ...
                -D_dmp*dy + theta2^2*(theta1-y0)*( dshape_attr_gat_dtheta2*sum_w_psi/sum_psi + shape_attr_gat*dSumWPsi_dtheta2 );
            dC_dtheta(2) = dC_dtheta(2)*(-1/tau_hat^2);

        end
        
        
        %% Returns the DMP's acceleration.
        %  @param[in] y: position.
        %  @param[in] dy: velocity.
        %  @param[in] y0: initial position.
        %  @param[in] y_c: coupling term for the dynamical equation of the \a y state.
        %  @param[in] z_c: coupling term for the dynamical equation of the \a z state.
        %  @param[in] x_hat: phase variable estimate.
        %  @param[in] g_hat: goal estimate.
        %  @param[in] tau_hat: time scale estimate.
        %  @param[out] ddy: DMP's acceleration.
        function ddy = getAccel(this, y, dy, y0, y_c, z_c, x_hat, g_hat, tau_hat)
            
            z = dy*tau_hat;
            [~, dz] = this.getStatesDot(x_hat, y, z, y0, g_hat, y_c, z_c);
            ddy = dz/tau_hat;
            
        end

    end
end
