
% wins.m -  This program demonstrates some example windows of the form:
%
%           window = 1 + c(1)*cos(pi*t) + c(2)*t.^4 + c(3)*t.^8 + c(4)*t.^10
%
%           with: t in the range [-1 1]
%

N = 51;     % length of window
fs = 1;     % Sampling rate

% Specify six sets of coefficients:
coefs = 1.0e+002 * ...
 [ 0.01284675466270   0.21083302716264  -1.13515699057810   1.09116177423864; ...
   0.00414930246239   0.03051187538742  -0.18821216264602   0.18276327621030; ...
   0.00422384136954   0.00508618309935  -0.05279014775805   0.05238316550347; ...
   0.00583575594599   0.00034883042889  -0.01843234655351   0.01808247747501; ...
   0.00870244806614   0.00409711795779  -0.01464253956578   0.01097912072531; ...
   0.01269066685089   0.01520814977051  -0.03234500239534   0.02087570575326 ];

t = linspace(-1,1,N)';  % column vector from -1 to 1
for i = 1:6
  window = 1 + coefs(i,1)*cos(pi*t) + coefs(i,2)*t.^4 + coefs(i,3)*t.^8 + coefs(i,4)*t.^10;
  window = window./sum(window);     % normalize window so DC gain is 1
  
  % Plot window responses:
  figure
  subplot(2,1,1)
  plot(window), zoom on
  title('Window')
  axis([1, N, 0, max(window)])
  subplot(2,1,2)
  [H,f] = freqz(window,[1], 8*1024, fs);
  plot(f,20*log10(abs(H)))
  title('Magnitude Response of Window')
  xlabel('Frequency')
  ylabel('Magnitude Response (dB)')
  grid on
  zoom on
  axis([0, fs/2, -100,10])
end
