% winopt.m  -   This program designs experimental parametric windows of the form:
%
%           window = 1 + c(1)*cos(pi*t) + c(2)*t.^4 + c(3)*t.^8 + c(4)*t.^10
%           with: -1 <= t <= 1


N = 51; % (256) length of window
fs = 1;     % Sampling rate (in Hz)

% fpass_list = [4:2:14]./(2*pi*N)       % list of passband widths for new window design
fpass_list = [8 10 12 14]./(2*pi*N)     % list of passband widths for new window design
coefs = zeros(length(fpass_list),4);    % init. matrix to save optimal coefs

aopt = [0 0 0 0];   % initial guess at parameters

for i = 1:length(fpass_list)
  fpass = fpass_list(i);
  
  % Find optimal coefficients for window:
  L = 2;    % start with 2-norm to get window in the right ball park
  aopt = fmins('wincost',aopt,[],[],N,L,fpass); % find aopt that minimize wincost
  L = 100;  % aproximate the L-infinity norm
  aopt = fmins('wincost',aopt,[],[],N,L,fpass); % find aopt that minimize wincost
  aopt = fmins('wincost',aopt,[],[],N,L,fpass)  % restart
  
  cost = wincost(aopt,N,L,fpass)    % report final cost
  coefs(i,:) = aopt;                % save optimal parameters

  % Compute parametric window:
  window = win(N,aopt);
  window = window./sum(window); % normalize window so DC gain is 1

  % Plot window and response of window:
  % Generate conventional windows for comparison:
  windows = zeros(N,7);
  windows = [boxcar(N) bartlett(N) hanning(N) hamming(N) blackman(N) kaiser(N,6.0) chebwin(N,50)];
  windows = windows./(ones(N,1)*sum(windows));  % normalize DC gains to one

  choice = 4;       % compare to Hamming
  figure
  subplot(2,1,1)
  plot(windows(:,choice),'k'), zoom on, hold on
  plot(window), zoom on
  title('Optimal Window (blue), Hamming (black)')
  axis([1, N, 0, max([windows(:,choice); window])])
  subplot(2,1,2)
  [H,f] = freqz(windows(:,choice),[1], 8*1024, fs);
  plot(f,20*log10(abs(H)),'k'), hold on
  [H,f] = freqz(window,[1], 8*1024, fs);
  plot(f,20*log10(abs(H)))
  title('Magnitude Response of Optimal Window (blue)')
  xlabel('Frequency (Hz)')
  ylabel('Magnitude Response (dB)')
  grid on
  zoom on
  axis([0, fs/2, -100,10])
end
