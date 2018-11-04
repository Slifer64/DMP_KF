function a = get5thOrderParams(x1, x2, y1, y2, y1_dot, y2_dot, y1_ddot, y2_ddot)

a = zeros(6,1);

b = [y1; y2; y1_dot; y2_dot;  y1_ddot;  y2_ddot];
A = [ 1  x1    x1^2    x1^3     x1^4      x1^5
      1  x2    x2^2    x2^3     x2^4      x2^5
      0   1    2*x1   3*x1^2   4*x1^3    5*x1^4
      0   1    2*x2   3*x2^2   4*x2^3    5*x2^4
      0   0     2      6*x1    12*x1^2   20*x1^3
      0   0     2      6*x2    12*x2^2   20*x2^3
     ];
 
a = A\b;

end