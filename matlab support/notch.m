% notch.m

fs = 48000
fnotch = 5000
r = 0.999

theta = 2*pi*fnotch/fs;

% Set notch numerator and denomiator:
b = [1 -2*cos(theta) 1];
b = 1;
a = [1 -2*r*cos(theta) r.^2];
% a = [1 0 ];

% Set inverse notch numerator and denomiator:
if(0)
b = [1 0];
a = [1 -2*r*cos(theta) r.^2];

b = [1 0 -1];
a = [1 -2*r*cos(theta) r.^2];

b = [1 -1];                         % best !
a = [1 -2*r*cos(theta) r.^2];

%b = [1 -2*r*0.99*cos(theta) (r*0.99).^2];
%a = [1 -2*r*cos(theta) r.^2];
end
a
b
zeros = roots(b)
poles = roots(a)

[h,ht] = impz(b,a,200);
figure, plot(ht,h), zoom on;
figure, zplane(b,a), zoom on;
figure, freqz(b,a,16*1024,fs), zoom on;

% for quantized coefs:
b = fix(16384*b)/16384;
a = fix(16384*a)/16384;

a
b
zerosq = roots(b)
polesq = roots(a)

[h,ht] = impz(b,a,200);
figure, plot(ht,h), zoom on;
figure, zplane(b,a), zoom on;
figure, freqz(b,a,16*1024,fs), zoom on;
