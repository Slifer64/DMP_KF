function J = msrFunJacob(theta, cookie)
    
    Yg_hat = theta(1:end-1);
    tau_hat = theta(end);
    
    Dim = length(cookie.dmp);
    
    J = zeros(Dim, length(theta));
    
    for i=1:Dim
        dC_dtheta_i = cookie.dmp{i}.getAcellPartDev_g_tau(cookie.t, cookie.Y(i), cookie.dY(i), cookie.Y0(i), cookie.x_hat, Yg_hat(i), tau_hat);

        J(i,end) = dC_dtheta_i(2);
        J(i,i) = dC_dtheta_i(1);
    end

end