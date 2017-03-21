% sine_gen.m Batch mode simulation sing wave generator DDS

fs = 48;    % sampling rate of simulation
fout = 1.33;    % 1.33 first tone freq.
Lsin = 384; % 32 length of sine table

N = 16*1024;    % length of simulations
NFFT = 8*1024;  % 8K FFT size for analysis
nskip = 4*1024 %4000;   % number of samples to skip to avoid edge effects
range = (nskip+1):(nskip+NFFT); % range of values to display

t = (0:(N-1))'; % generate time index

% Generate 20KHz lowpass filter (for wideband tests)
if(1)
h20 = fir1(100,20/(fs/2));
%figure, freqz(h20,1,1024,fs), zoom on  % plot lowpass filter response
%subplot(2,1,1)
%title('LP Filter Response for noise generation')
end

% load cos table
cos_table = cos((2*pi/Lsin)*(0:(Lsin-0)))';
%figure, plot(cos_table)


%**************************** Start signal generation *********************************
% Generate signal:

x = cos((2*pi*fout/fs)*t);

rand('state',0);    % init. random number generator to known state for each run

finc = (fout/fs)*Lsin               % compute increment size
fidx = linspace(0,(N-1)*finc,N)';   % compute unwrapped floating index into cosine table

idx = mod(fix(fidx + 0.5 + 0.0*(rand(size(x))) - 0.5), Lsin);   % compute dithered index (note dithering has no benifit)
xq1 = cos_table(idx + 1);   % generate quantized signal

a = fidx - fix(fidx);
xq2 = (1-a).*cos_table(idx + 1) + a.*cos_table(idx + 2);    % generate quantized signal using linear interpolation

scl = 0.25;
plot_result(scl*x,'ideal x',NFFT,nskip,fs);
plot_result(scl*xq1,'sin table- xq1',NFFT,nskip,fs);
plot_result(scl*xq2,'interpolated sin table- xq2',NFFT,nskip,fs);


dist1 = xq1 - x;    % compute ith order distortion
dist2 = xq2 - x;    % compute ith order distortion
%plot_result(dist,'distortion dist',NFFT,nskip,fs);

xvar = cov(x(range));   % compute variance of x for distortion calculations below
percent_distortion1 = 100*sqrt(cov(dist1(range))/xvar)      % calculate percent distortion
percent_distortion2 = 100*sqrt(cov(dist2(range))/xvar)      % calculate percent distortion
