function w = win(N,a,wtype)
% used by filt.m for Versa-Filter

% t = linspace(-1,1,N)';    % column vector from -1 to 1
t = 2*(0:(N-1))./(N-1);

switch wtype
case 1  % window is sum of cosines
  w = a(1) + a(2)*cos(pi*t) + a(3)*cos(2*pi*t);
case 2  % window is power series
  w = a(1) + a(2)*t.^2 + a(3)*t.^4 + a(4)*t.^8 + a(5)*t.^10;
case 3  % window is cos + power series
  w = a(1) + a(2)*cos(pi*t) + ...
    a(3)*t.^4 + a(4)*t.^8 + a(5)*t.^10;
case 4  %
  w = a(1) + a(2)*cos(pi*t) + a(3)*t.^8 + a(4)*t.^10 + ...
    a(5)*abs(t).^(4 + 2*atan(a(6)));
end
