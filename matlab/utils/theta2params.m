function [tau_hat, g_hat] = theta2params(theta, tau, g, est_tau, est_g)
    
    tau_hat = tau;
    g_hat = g;
    
    if (est_tau), tau_hat = theta(end); end
    if (est_g)
        if (est_tau) 
            g_hat = theta(1:end-1);
        else
            g_hat = theta;
        end
    end
        
end