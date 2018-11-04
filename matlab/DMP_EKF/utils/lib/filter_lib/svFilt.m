function [Sigma_f, inv_Sigma_f] = svFilt(Sigma, sigma0, v)

[V, D] = eig(Sigma);

sigma = diag(D);

for i=1:length(sigma)
    sigma(i) = (sigma(i)^3 + v*sigma(i)^2 + 2*sigma(i) + 2*sigma0) / (sigma(i)^2 + v*sigma(i) + 2);
end

Sigma_f = V * diag(sigma) * V';

if (nargout > 1)
    inv_Sigma_f = V * diag(1./sigma) * V';
end

end

