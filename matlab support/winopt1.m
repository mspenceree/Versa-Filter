% This program designs window experimental parametric windows


N = 256;    % (256) length of window
fs = 1;     % Sampling rate (in Hz)


%*************************************************************************:
% Design the FIR filter window:
% The DSP uses equ. 7.98 on p.461 in Oppenheim and Schafer to compute the
% truncated ideal inpulse reponse of a multi-band filter.

windows = zeros(N,7);
windows = [boxcar(N) bartlett(N) hanning(N) hamming(N) blackman(N) kaiser(N,6.0) chebwin(N,50)];
% figure, plot(windows)
windows = windows./(ones(N,1)*sum(windows));    % normalize DC gain to one

wtype = 3;      % optimal window type
find_opt = 1;   % find optimal window
% fpass_list = [3:20]./(2*pi*N) % list of passband widths
fpass_list = [12]./(2*pi*N) % list of passband widths
aall = zeros(length(fpass_list),4);
for i = 1:length(fpass_list)

if(find_opt)    % Find optimal parameters for window that minimize the sidelobe levels:
  % fpass = 12/(2*pi*N);    % set the main lobe width
  fpass = fpass_list(i);
  switch wtype
  case 1    % window is sum of cosines: w = 1 + a(1)*cos(pi*t) + a(2)*cos(2*pi*t);
    aopt = [-.5, .08]./0.42;    % start with the Blackman coefs
  case 2    % window is power series: w = 1 + a(1)*t.^2 + a(2)*t.^4 + a(3)*t.^6 + a(4)*t.^8;
    aopt = [0 0 0 0];
    aopt = randn(size(aopt));
  case 3    % window: cos + powers series
    aopt = [1.04617566755142   0.86035769077562  -2.11327967833070   1.42923100954384];
    % aopt = randn(size(aopt));
  case 4    % window: w = 1 + a(1)*abs(t).^a(2);
    aopt = [0 0 0 0 0];
    aopt = randn(size(aopt));
  end
  L = 2;    % start with 2-norm to get window in the right ballpark
  aopt = fmins('wincost',aopt,[],[],N,L,fpass); % find aopt that minimize wincost
  cost = wincost(aopt,N,L,fpass);
  L = 100;
  aopt = fmins('wincost',aopt,[],[],N,L,fpass); % find aopt that minimize wincost
  cost = wincost(aopt,N,L,fpass);
  aopt = fmins('wincost',aopt,[],[],N,L,fpass)  % restart
  cost = wincost(aopt,N,L,fpass)
  aall(i,:) = aopt;     % save parameters
  
else    % use predetermined parameters:
  switch wtype
  case 1    % window is sum of cosines
    aopt = [-.5, .08]./0.42;    % start with the Blackman coefs
  case 2    % window is power series
    aopt = [-2.50703119224064   2.30598269372605  -1.86027170715794   1.13006919420658];
  case 3    % window: cos + powers series
    aopt = [1.04617566755142   0.86035769077562  -2.11327967833070   1.42923100954384]; % fpass = 13/(2*pi*N)
    aopt = [1.12794358346409   1.09379540503637  -2.49538061025466   1.64782091703098]; % fpass = 13.5/ -> beats Kaiser window
    aopt = [0.87024591503591   0.40971027852393  -1.46423171571032   1.09789075579087]; % fpass = 12/
  case 4    % window: w = 1 + a(1)*abs(t).^a(2);
    aopt = [0 0 0 0];
  end
end


window = win(N,aopt);       % compute parametric window
window = window./sum(window);   % normalize window so DC gain is 1

% Plot window and response of window:
choice = 6;
figure
subplot(2,1,1)
plot(windows(:,choice),'k'), zoom on, hold on
plot(window), zoom on
title('Optimal Window')
axis([1, N, 0, max([windows(:,choice); window])])
subplot(2,1,2)
[H,f] = freqz(windows(:,choice),[1], 8*1024, fs);
plot(f,20*log10(abs(H)),'k'), hold on
[H,f] = freqz(window,[1], 8*1024, fs);
plot(f,20*log10(abs(H)))
title('Magnitude Response of Optimal Window')
xlabel('Frequency (Hz)')
ylabel('Magnitude Response (dB)')
grid on
zoom on
axis([0, fs/2, -100,10])

end % for fpass


error('end for now')
junk = input('Hit Enter to continue...', 's');

%***************************** DSP ***************************************:
% Plot some sample responses:
M = N - 1;  % Filter length - 1
% specify filter: G =       1 - lowpass,  fc, 0
%                           2 - highpass, fc, 0
%                           3 - bandpass, f1, f2
%                           4 - highpass, f2, f2

G = [];
% fc = linspace(0,fs/2,40)';
fc = linspace(0,fs/2,25)';
fc = [1 1 2 5 10 20 20]';
fc(end) = [];       % delete last element
fc(1) = [];     % delete first element
fwidth = linspace(0,fs/2,26)';
fwidth = [2 2 4 8 10 10]';
fwidth(end) = [];   % delete last element
fwidth(1) = [];     % delete first element
f1 = fs/4 - fwidth/2;
f2 = fs/4 + fwidth/2;
G = [G; [ones(size(fc)), fc, zeros(size(fc))]];
G = [G; [2*ones(size(fc)), fc, zeros(size(fc))]];
G = [G; [3*ones(size(f1)), f1, f2]];
G = [G; [4*ones(size(f1)), f1, f2]];

[mg, ng] = size(G);
maxh = zeros(mg,1);
centerh = zeros(mg,1);
% figure, hold on
for fidx = 1:mg
   if(G(fidx,1)==1)
      g = [1, 0, 0];
        f = [G(fidx,2), fs/2];
   elseif(G(fidx,1)==2)
      g = [0, 1, 0];
        f = [G(fidx,2), fs/2];
   elseif(G(fidx,1)==3)
      g = [0, 1, 0, 0];
        f = [G(fidx,2:3), fs/2];
   elseif(G(fidx,1)==4)
      g = [1, 0, 1, 0];
        f = [G(fidx,2:3), fs/2];
   end
   % design ideal filter:
    Nbands = length(f); % Number of bands
    hrect = zeros(Nfir,1);
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

   h = hrect.*window;       % Weight ideal filter
   maxh(fidx) = max(abs(h));
   centerh(fidx) = h((Nfir+1)/2);
   
   % error(' ')
   
   % Perform coef. quantization:
   nbits = 16;
   h = (2^-nbits)*fix((2^nbits)*h);
   
   
   % Plot results
   plotr = 1;
   if(plotr)
        % figure, plot(h)
       if(0)
       [H,f] = freqz(hrect,[1], 1*1024, fs);
        figure
        plot(f,20*log10(abs(H)))
        title('Magnitude Response of Rectangular Windowed filter')
        xlabel('Frequency (KHz))')
        ylabel('Magnitude Response (dB)')
        grid on
        zoom on
        axis([0, fs/2, -100,10])
        %   print c:\work\sps\filter\filtsoft\ideal.eps -deps -tiff
       end
      
       if(0)
      figure
        plot(h)
        title('Impulse Response')
        xlabel('Sample')
        grid on
      zoom on
      end

       [H,f] = freqz(h,[1], 1*1024, fs);
        figure
        plot(f,20*log10(abs(H)))
        title('Magnitude Response of FIR Filter')
        xlabel('Frequency (KHz))')
        ylabel('Magnitude Response (dB)')
        grid on
        zoom on
        % axis([0, fs/2, -100,10])
        axis([0, 25, -100,10])
      % print c:\work\sps\filter\filtsoft\ideal.eps -deps -tiff
      % print -dbitmap      % paste figure image
   end

end
err = sum(abs(maxh - centerh))
centerh
maxcenterh = max(centerh)
mincenterh = min(centerh)
