function theta_proj = paramsProj(theta, P, theta_low_lim, theta_up_lim)

n = length(theta);
D = [];
d = [];

proj_flag = false;

for i=1:length(theta)
    if (theta(i) < theta_low_lim(i))
        d_i = theta_low_lim(i);
        d = [d; d_i];
        D_i = zeros(1,n);
        D_i(i) = 1.0;
        D = [D; D_i];
        proj_flag = true;
    elseif (theta(i) > theta_up_lim(i))
        d_i = theta_up_lim(i);
        d = [d; d_i];
        D_i = zeros(1,n);
        D_i(i) = 1.0;
        D = [D; D_i];
        proj_flag = true;
    end
end

if (proj_flag)
    theta_proj = theta - P*D'/(D*P*D')*(D*theta-d);
else
    theta_proj = theta;
end

end

