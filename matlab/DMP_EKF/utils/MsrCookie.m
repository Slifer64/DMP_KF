
classdef MsrCookie < handle
    properties
        dmp % cell array with dmp objects
        t % timestamp
        Y % Cartesian position
        dY % Cartesian velocity
        Y0 % initial Cartesian position
        Y_c % coupling term for dmp velocity
        Z_c % coupling term for dmp acceleration
        x_hat % estimate of phase variable
    end

    methods
        %% Cookie constructor
        function this = MsrCookie()

        end
        
        %% Extended Kalman Filter constructor
        %  @param[in] dmp: Cell array with dmp objects.
        %  @param[in] t: Timestamp.
        %  @param[in] Y: Cartesian position.
        %  @param[in] dY: Cartesian velocity.
        %  @param[in] Y0: Initial Cartesian position.
        %  @param[in] Y_c: Coupling term for dmp velocity.
        %  @param[in] Z_c: Coupling term for dmp acceleration.
        %  @param[in] x_hat: Estimate of phase variable.
        function set(this, dmp, t, Y, dY, Y0, Y_c, Z_c, x_hat)
            
            this.dmp = dmp;
            this.t = t;
            this.Y = Y;
            this.dY = dY;
            this.Y0 = Y0;
            this.Y_c = Y_c;
            this.Z_c = Z_c;
            this.x_hat = x_hat;
            
        end
            
            


    end
end
