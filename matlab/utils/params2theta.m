function theta = params2theta(tau_hat, g_hat, est_tau, est_g)
    
    theta = [];
    if (est_g), theta=[theta; g_hat]; end
    if (est_tau), theta=[theta; tau_hat]; end
        
end