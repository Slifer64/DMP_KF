%% Linear Gating Function class
%  Implements a linear gating function, u=f(x), x:[0 1]->u:[u0 u_end],
%  where u0 is the initial and u_end the final value.
%  The output of the gating function is:
%     u = u0 - a_u*x;
%    du = -a_u;
%

classdef LinGatingFunction < handle
   properties
       u0 % initial value of the gating function
       a_u % the rate of evolution of the gating function
   end

   methods
      %% Linear Gating Function Constructor.
      %  @param[in] u0: Initial value of the gating function (optional, default = 1.0).
      %  @param[in] u_end: Final value of the gating function (optional, default = 0.005).
      %  @param[out] gating_fun: Gating function object.
      function gating_fun = LinGatingFunction(u0, u_end)

          if (nargin < 1), u0 = 1.0; end
          if (nargin < 2), u_end = 0.005; end

          gating_fun.init(u0, u_end);

      end

      %% Initializes the gating function.
      %  @param[in] u0: Initial value of the gating function.
      %  @param[in] u_end: Final value of the gating function.
      function init(gating_fun, u0, u_end)

          gating_fun.u0 = u0;
          gating_fun.a_u = gating_fun.u0 - u_end;

      end

      %% Returns the gating function's output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's output.
      function u = getOutput(gating_fun, x)

          u = gating_fun.u0 - gating_fun.a_u*x;

      end

      %% Returns the gating function's derivated output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's derivated output.
      function du = getOutputDot(gating_fun, x)

          du = -gating_fun.a_u * ones(size(x));

      end
      
      %% Returns the partial derivative of the gating output wrt 1/tau
      %  @param[in] t: timestamp
      %  @param[in] x: phase variable
      %  @param[out] u: partial derivative of the gating wrt 1/tau.
      function u = getPartDev_1oTau(this, t, x)

          u = -this.a_u * t;
          
          if (x >= 1), u=0; end

      end


   end
end
