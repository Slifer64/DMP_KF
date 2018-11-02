function y_out = msrFun(theta, cookie)

    g_hat = theta(1:end-1);
    tau_hat = theta(end);
    
    Dim = length(cookie.dmp);
    y_out = zeros(Dim,1);
    
    for i=1:Dim
        y_out(i) = cookie.dmp{i}.getAccel(cookie.y(i), cookie.dy(i), cookie.y0(i), cookie.y_c, cookie.z_c, cookie.x_hat, g_hat(i), tau_hat);
    end
    
end