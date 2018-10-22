%% Sigmoid Gating Function class
%  Implements a sigmoidal gating function, u=f(x), x:[0 1]->u:[u0 u_end],
%  where u0 is the initial and u_end the final value.
%  The output of the gating function is:
%     u = u0 * ( 1 / (1 + exp(-a_u*(x-c)) ) );
%    du = -a_u*u0 * ( exp(-a_u*(x-c)) / (1 + exp(-a_u*(x-c)) )^2 );
%

classdef SigmoidGatingFunction < handle
   properties
       u0 % initial value of the gating function
       a_u % the rate of evolution of the gating function
       c % center of the exponential in the sigmoid
   end

   methods
      %% Sigmoid Gating Function Constructor.
      %  @param[in] u0: Initial value of the gating function (optional, default = 1.0).
      %  @param[in] u_end: Final value of the gating function (optional, default = 0.99).
      %  @param[out] gating_fun: Gating function object.
      function gating_fun = SigmoidGatingFunction(u0, u_end)

          if (nargin < 1), u0 = 1.0; end
          if (nargin < 2), u_end = 0.99; end

          gating_fun.init(u0, u_end);

      end

      %% Initializes the gating function.
      %  @param[in] u0: Initial value of the gating function.
      %  @param[in] u_end: Final value of the gating function.
      function init(gating_fun, u0, u_end)

          gating_fun.a_u = 700.0;
          gating_fun.u0 = u0;
          gating_fun.c = 1.0 - (1.0/gating_fun.a_u)*log((gating_fun.u0-u_end)/u_end);

      end

      %% Returns the gating function's output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's output.
      function u = getOutput(gating_fun, x)

          exp_t = exp((gating_fun.a_u)*(x-gating_fun.c));
          u = gating_fun.u0 * 1.0 ./ (1.0 + exp_t);

      end

      %% Returns the gating function's derivated output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's derivated output.
      function du = getOutputDot(gating_fun, x)

          exp_t = exp((gating_fun.a_u/gating_fun.tau)*(x-gating_fun.c));
          du = -gating_fun.u0 * (gating_fun.a_u) * exp_t ./ (1.0 + exp_t).^2;

      end
      
      %% Returns the partial derivative of the gating output wrt 1/tau
      %  @param[in] t: timestamp
      %  @param[in] x: phase variable
      %  @param[out] u: partial derivative of the gating wrt 1/tau.
      function u = getPartDev_1oTau(this, t, x)

          h = this.getOutput(x);
          
          u = -this.a_u * t * h * (1-h);

      end


   end
end
