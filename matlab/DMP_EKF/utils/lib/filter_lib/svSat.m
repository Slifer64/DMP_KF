function [Sigma_f, inv_Sigma_f] = svSat(Sigma, sigma_min, sigma_max)

[V, D] = eig(Sigma);

sigma = diag(D);

for i=1:length(sigma)
    if (sigma(i) > sigma_max)
        sigma(i) = sigma_max;
    elseif (sigma(i) < sigma_min)
        sigma(i) = sigma_min;
    end
end

Sigma_f = V * diag(sigma) * V';

if (nargout > 1)
    inv_Sigma_f = V * diag(1./sigma) * V';
end

end

