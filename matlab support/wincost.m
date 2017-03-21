function cost = wincost(aopt,N,L,fpass,wtype)
% computes the cost of blackman type window

window = win(N,aopt,wtype);     % compute parametric window
window = window./sum(window);   % normalize window so DC gain is 1

% Nfft = N*20;
Nfft = 512; % use power of 2 for speed
H = freqz(window./sum(window),[1], Nfft, 1);    % compute frequency response of window
H(1:fix(Nfft*2*fpass)) = [];                    % clear the main lobe points

%cost = max(abs(H));            % minimize peek out-of-band (this error surface is too demanding)

cost = sum(abs(H).^L)^(1/L);    % minimize out-of-band L-norm
