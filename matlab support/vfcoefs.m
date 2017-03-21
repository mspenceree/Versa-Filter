function [b, a] = vfcoefs(func,f1,f2,fs,N,q_flag)
% This function computes the filter coefficients as in the Versa-Filter Module
% Call as:
% [b, a] = vfcoefs(func,f1,f2,fs,order,quantize_coefs);
%
%   func    -   function code:
%                   1-AllPass
%                   2-LowPass
%                   3-HighPass
%                   4-BandPass
%                   5-BandStop
%                   6-Notch
%                   7-InvNotch
%   f1      -   cutoff freq of LowPass (etc.) or first band edge of BandPass (etc.)
%   f2      -   second band edge of BandPass (etc.)
%   fs      -   sampling frequency
%   N       -   filter order (3 to 256 for Versa-Filter Module)
%   q_flag  -   quantize flag: set to 0 for ideal coefficients, or 1 for quantized coefficients
%
%   b       -   numerator filter coefficients
%   a       -   denomiator filter coefficients


if(func==1) % AllPass
  b = 1;
  a = 1;
  return
end

if(func==6 | func==7)   % Notch or inverse notch
  k1 = -cos(2*pi*f1/fs);
  k2 = (1 - tan(pi*f2/fs))/(1 + tan(pi*f2/fs));
  d1 = .999;
  d2 = .999;
  c1 = (1 - k1^2)/d1;
  c2 = (1 - k2^2)/d2;
  
  if(q_flag)% perform coef. quantization (rounding):
    nbits = 16;     % number of bits
    k1 = (2^-nbits)*fix((2^nbits)*k1 + 0.5);
    k2 = (2^-nbits)*fix((2^nbits)*k2 + 0.5);
    d1 = (2^-nbits)*fix((2^nbits)*d1 + 0.5);
    d2 = (2^-nbits)*fix((2^nbits)*d2 + 0.5);
    c1 = (2^-nbits)*fix((2^nbits)*c1 + 0.5);
    c2 = (2^-nbits)*fix((2^nbits)*c2 + 0.5);
  end

  b0 = k2;
  b1 = k1*k2 + (c2*d2 + k2^2)*k1;
  b2 = (c2*d2 + k2^2)*(k1^2 + c1*d1);
  a0 = 1;
  a1 = k1 + k1*k2;
  a2 = k2*(k1^2 + c1*d1);
  
  if(func==6)   % Notch
    b = [(a0 + b0), (a1 + b1), (a2 + b2)]./(2*a0);
  else
    b = [(a0 - b0), (a1 - b1), (a2 - b2)]./(2*a0);
  end
  a = [1, a1/a0, a2/a0];
  
  return
end



if(func==2) % LowPass
  g = [1, 0, 0];
  f = [f1, fs/2];
elseif(func==3) % HighPass
  g = [0, 1, 0];
  f = [f1, fs/2];
elseif(func==4) % BandPass
  g = [0, 1, 0, 0];
  f = [f1, f2, fs/2];
elseif(func==5) % BandStop
  g = [1, 0, 1, 0];
  f = [f1, f2, fs/2];
end
  
% design the truncated ideal filter:
Nbands = length(f); % Number of bands
hrect = zeros(1, N);
M = N - 1;      % Filter length - 1
for n = 0:M
  ssum = 0;
  for k = 1:Nbands
    t1 = pi*(n - M/2);
    if(t1==0)
      sincx = (2/fs)*f(k);
    else
      sincx = sin((2/fs)*f(k)*t1)/t1;
    end
    ssum = ssum + (g(k) - g(k+1))*sincx;
  end
  hrect(n+1) = ssum;
end
  
% Compute the window used in Versa-Filter:
poly_opt = 1;
if(poly_opt)    % Compute polynomial window
  % From: fpass = 0.85*127/Nfir, |.|^6
  %aopt = [ -2.82565396446698   3.33625910016795  -1.96146734929453   0.36410950684658  0.11582207244760];
  %orders = [2 4 6 8 10];
  % From: fpass = 0.9*127/Nfir, |.|^6
  aopt = [-3.02516267528867   3.86916964583341  -2.57018413301440   0.74669046798555];
  orders = [2 4 6 8];
  % From: fpass = 0.5*127/Nfir, |.|^6
  %aopt = [-1.60736278478314   2.28982348965903  -3.59598738112930   2.32443579494662];
  %orders = [2 4 6 8];
  x = linspace(-1,1,N)';
  window = ones(N,1) + ((x*ones(1,length(orders))).^(ones(N,1)*orders))*aopt';  % compute poly window
  window = window';
else
  % These are the coefs for the modified-Blackman-window used in the Versa-Filter:
  aopt =[0.48216433063585  -0.48550251793519   0.03233315142896];
  n = ((0:(N-1))./(N-1))';
  window = aopt(1) + aopt(2)*cos(2*pi*n) + aopt(3)*cos(4*pi*n); % compute the window
end


b = hrect.*window;      % window the ideal filter coefs and save results
a = 1;                  % no poles
   
if(q_flag)% perform coef. quantization:
  coef_max = max(abs(b));   
  if(0.4999<coef_max)   
    nbits = 15;     % typical for wide band filters
  else
    nbits = 16;     % typical for narrow band filters
  end
  b = (2^-nbits)*fix((2^nbits)*b);
end
  
return
