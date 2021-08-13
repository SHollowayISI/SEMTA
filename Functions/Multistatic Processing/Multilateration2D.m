function [r] = Multilateration2D(v1,r1,v2,r2)
%2-D MULTILATERATION Performs 2-dimensional multilateration
%   Takes as inputs [x; y] location and range-to-target for two detectors, 
%   gives as output estimated [x; y] of target.

% Translate v1 to (0,0)
w = v2-v1;

% Calculate target position if v2 were on x-axis
d = sqrt(sum(w.^2));
x_raw = (d^2 + r1^2 - r2^2)/(2*d);
y_raw = sqrt(abs(r1^2 - x_raw^2));


% Inverse rotation of v2
T = (1/d)*[w(1), w(2); w(2), -w(1)];
r = T*[x_raw; y_raw];

% Inverse translation of v1
r = r + v1;

end

