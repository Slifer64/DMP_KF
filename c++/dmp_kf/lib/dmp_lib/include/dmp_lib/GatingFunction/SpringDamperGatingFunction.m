%% Spring-Damper Gating Function class
%  Implements a spring-damper gating function, u=f(x), x:[0 1]->u:[u0 u_end],
%  where u0 is the initial and u_end the final value.
%  The output of the gating function is:
%     u = u0*(x*exp(-a_u*x));
%    du = u0*exp(-a_u*x)*(1-a_u*x);
%

classdef SpringDamperGatingFunction < handle
   properties
       u0 % scaling of the gating function's output
       a_u % the rate of evolution the gating function
   end

   methods
      %% Spring-Damper Gating Function Constructor.
      %  @param[in] u0: Initial value of the gating function (optional, default = 1.0).
      %  @param[in] u_end: Final value of the gating function (optional, default = 0.005).
      %  @param[out] gating_fun: Gating function object.
      function gating_fun = SpringDamperGatingFunction(u0, u_end)

          if (nargin < 1), u0 = 1.0; end
          if (nargin < 2), u_end = 0.005; end

          gating_fun.init(u0, u_end);

      end

      %% Initializes the gating function.
      %  @param[in] u0: Initial value of the gating function.
      %  @param[in] u_end: Final value of the gating function.
      function init(gating_fun, u0, u_end)

          if (u0*u_end <= 0)
              error('SpringDamperGatingFunction: init: u_end and u0 must have the same sign');
          end

          if (abs(u_end) > abs(u0))
              error('SpringDamperGatingFunction: init: |u_end| must be less than |u0|');
          end

          gating_fun.setGatingFunParams(u0, u_end);

      end

      %% Sets the gating function's time constants based on the value of
      %% the phase variable at the end of the movement.
      %  @param[in] u0: Initial value of the gating function.
      %  @param[in] u_end: Final value of the gating function.
      function setGatingFunParams(gating_fun, u0, u_end)

          if (u_end == 0)
              error('SpringDamperGatingFunction: setGatingFunParams: u_end must be != 0');
          end

          gating_fun.u0 = u0;

          gating_fun.a_u = -log(u_end/gating_fun.u0);

      end

      %% Returns the gating function's output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's output.
      function u = getOutput(gating_fun, x)

          exp_at = exp(-gating_fun.a_u * x);
          s = 1 / (exp(-1)/gating_fun.a_u);
          u = gating_fun.u0*s*(x.*exp_at );

      end

      %% Returns the gating function's derivated output for the specified timestamps.
      %  @param[in] x: Vector of timestamps.
      %  @param[out] u: Vector of values of the gating function's derivated output.
      function du = getOutput_dot(gating_fun, x)

          exp_at = exp(-gating_fun.a_u * x);
          du = gating_fun.u0*exp_at .* (1 - gating_fun.a_u*x );

      end


   end
end
