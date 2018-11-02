function J = msrFunJacob(theta, cookie)
    
    g_hat = theta(1:end-1);
    tau_hat = theta(end);
    
    Dim = length(cookie.dmp);
    
    dC_dtheta = zeros(Dim, length(theta));
    
    for i=1:Dim
        dC_dtheta_i = cookie.dmp{i}.getAcellPartDev_g_tau(cookie.t, cookie.y(i), cookie.dy(i), cookie.y0(i), cookie.x_hat, g_hat(i), tau_hat);

        dC_dtheta(i,end) = dC_dtheta_i(2);
        dC_dtheta(i,i) = dC_dtheta_i(1);
    end
    
    J = dC_dtheta;

end