%% Exponential Gating Function class
%  Implements an exponential gating function, u=f(x), x:[0 1]->u:[u0 u_end],
%  where u0 is the initial and u_end the final value.
%  The output of the gating function is:
%     u = u0*exp(-a_u*x);
%    du = -a_u*u0*exp(-a_u*x);
%

classdef ExpGatingFunction < handle
   properties
       u0 % initial value of the gating function
       a_u % the rate of evolution of the gating function
       dec_flag % true of the gating is decreasing, false otherwise
   end

   methods
      %% Exponential Gating Function Constructor.
      %  @param[in] u0: Initial value of the gating function (optional, default = 1.0).
      %  @param[in] u_end: Value of the gating function at the end of the motion (optional, default = 0.005).
      %  @param[out] gating_fun: Gating function object.
      function this = ExpGatingFunction(u0, u_end)

          if (nargin < 1), u0 = 1.0; end
          if (nargin < 2), u_end = 0.005; end

          this.init(u0, u_end);

      end

      %% Initializes the gating function.
      %  @param[in] u0: Initial value of the phase variable.
      %  @param[in] u_end: Value of phase variable at the end of the motion.
      function init(this, u0, u_end)

          this.u0 = u0;
          
          dec_flag = (u0 > u_end);
 
          x = u_end/this.u0;

          if (u0 == 0)
              error('ExpGatingFunction: setGatingFunParams: u0 must be != 0');
          end

          if (x <= 0)
              error('ExpGatingFunction: setGatingFunParams: u0 and u_end must be both positive or negative.');
          end

          this.a_u = -log(x);

      end

      %% Returns the gating function's output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's output.
      function u = getOutput(this, x)

          u = this.u0*exp(-this.a_u*x);

      end

      %% Returns the gating function's derivated output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's derivated output.
      function du = getOutputDot(this, x)

          du = -this.a_u*this.u0*exp(-this.a_u*x);

      end
      
      %% Returns the partial derivative of the gating output wrt 1/tau
      %  @param[in] t: timestamp
      %  @param[in] x: phase variable
      %  @param[out] u: partial derivative of the gating wrt 1/tau.
      function u = getPartDev_1oTau(this, t, x)

          u = -this.a_u * t * this.getOutput(x);

      end

   end
end
