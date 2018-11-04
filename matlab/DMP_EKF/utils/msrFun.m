function y_out = msrFun(theta, cookie)

    Yg_hat = theta(1:end-1);
    tau_hat = theta(end);
    
    Dim = length(cookie.dmp);
    y_out = zeros(Dim,1);
    
    for i=1:Dim
        y_out(i) = cookie.dmp{i}.getAccel(cookie.Y(i), cookie.dY(i), cookie.Y0(i), cookie.Y_c, cookie.Z_c, cookie.x_hat, Yg_hat(i), tau_hat);
    end
    
end