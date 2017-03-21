% vf.m
% This program plots actual Versa-Filter frequency responses

quantize_flag = 1;  % set to 1 to quantize the filter coefs as in the Versa-Filter
                    % set to 0 for ideal filters
plot_impulse = 0;
manual_axis = 1;
  fig_scale = 1.3

disp('**********************************************************************************************');
disp(' ');
disp('This program computes the frequency responses of the Versa-Filter Module ');
disp(' ');
disp('**********************************************************************************************');
disp(' ');

ans = input('Enter 0 to show a variety of functions, 1 to specify the filter function: ');
if(ans==0)  % Specify variety of Versa-Filter functions and parameters:
  fs =      [   48  48  ];
  
  func_a =  [   4   2   ];
  f1_a =    [   2   10  ];
  f2_a =    [   12  0   ];
  N_a =     [   128 256 ];
  
  func_b =  [   6   5   ];
  f1_b =    [   6   5   ];
  f2_b =    [   .5  10  ];
  N_b =     [   500 256 ];
  
  if(0)
  fs =      [   48  48  48  48  48];
  
  func_a =  [   4   2   4   3   7];
  f1_a =    [   2   10  5   5   8];
  f2_a =    [   12  0   10  0   .01];
  N_a =     [   128 256 256 35  500];
  
  func_b =  [   6   5   1   2   6];
  f1_b =    [   6   5   0   15  9];
  f2_b =    [   .5  10  0   0   .5];
  N_b =     [   500 256 1   128 500];
  end
  
  
else
  disp(' ')
  fs = input('Enter Sampling rate (in KHz, 8 or 48 for Versa-filter): ');
  disp(' ')
  func_a=input('Enter Ch A Function (1-AllPass, 2-LowPass, 3-HighPass, 4-BandPass, 5-BandStop, 6-Notch, 7-InvNotch): ');
  switch func_a
  case 1
    N_a = 1;
  case {2, 3}
    f1_a = input('Enter Ch A, Cutoff frequency (in KHz): ');
    f2_a = 0;
    N_a = input('Enter Ch A, filter Order (3 to 256) : ');
  case {4, 5}
    f1_a = input('Enter Ch A, f1 (in KHz): ');
    f2_a = input('Enter Ch A, f2 (in KHz): ');
    N_a = input('Enter Ch A filter Order (3 to 256) : ');
  case 6
    f1_a = input('Enter Ch A Notch frequency (in KHz): ');
    f2_a = input('Enter Ch A Notch Width (in KHz): ');
    N_a = 500;
  case 7
    f1_a = input('Enter Ch A Center frequency (in KHz): ');
    f2_a = input('Enter Ch A passband Width (in KHz): ');
    N_a = 500;
  end
  
  disp(' ')
  func_b=input('Enter Ch B Function (1-AllPass, 2-LowPass, 3-HighPass, 4-BandPass, 5-BandStop, 6-Notch, 7-InvNotch): ');
  switch func_b
  case 1
    N_b = 1;
  case {2, 3}
    f1_b = input('Enter Ch B, Cutoff frequency (in KHz): ');
    f2_b = 0;
    N_b = input('Enter Ch B, filter Order (3 to 256) : ');
  case {4, 5}
    f1_b = input('Enter Ch B, f1 (in KHz): ');
    f2_b = input('Enter Ch B, f2 (in KHz): ');
    N_b = input('Enter Ch B filter Order (3 to 256) : ');
  case 6
    f1_b = input('Enter Ch B Notch frequency (in KHz): ');
    f2_b = input('Enter Ch B Notch Width (in KHz): ');
    N_b = 500;
  case 7
    f1_b = input('Enter Ch B Center frequency (in KHz): ');
    f2_b = input('Enter Ch B passband Width (in KHz): ');
    N_b = 500;
  end
end


for i = 1:length(func_a)
  
  %*************************************************************************:
  % Get the Versa-Filter coefs:
  [b_a, a_a] = vfcoefs(func_a(i),f1_a(i),f2_a(i),fs(i),N_a(i),quantize_flag);
  [b_b, a_b] = vfcoefs(func_b(i),f1_b(i),f2_b(i),fs(i),N_b(i),quantize_flag);


  %*************************************************************************:
  % Plot all results:
  fplot = fs(i)/2.4;    % Ending plot frequency
  Nfreq_pts = 8*1024;
  
  %--------------------------------------------------------------------------
  % Plot Ch A results
  if(plot_impulse)
    h_a = filter(b_a, a_a, [1, zeros(1,N_a(i)-1)]); % compute impulse response
    figure
    plot((0:(length(h_a)-1))/fs(i), h_a, 'r')
    title('Channel A Impulse Response')
    set(get(gca,'Title'),'FontName','Helvetica','FontSize',16)  % set Title font and size
    xlabel('Time (msec)')
    set(get(gca,'XLabel'),'FontName','Helvetica','FontSize',14) % set x-label font and size
    grid on
    zoom on
  end
  
  [H_a,f] = freqz(b_a, a_a, Nfreq_pts, fs(i));
  figure
  plot(f,20*log10(abs(H_a)), 'r')
  title('Channel A Magnitude Response')
  set(get(gca,'Title'),'FontName','Helvetica','FontSize',fig_scale*16)  % set Title font and size
  xlabel('Frequency (KHz)')
  set(get(gca,'XLabel'),'FontName','Helvetica','FontSize',fig_scale*14) % set x-label font and size
  ylabel('Magnitude (dB)')
  set(get(gca,'YLabel'),'FontName','Helvetica','FontSize',fig_scale*14) % set y-label font and size
  axis([0, fplot, -100,10])     % set axis limits to plot
  if(manual_axis)
    set(gca,'Xtick',[0:5:fplot])    % set x-axis tick label positions
    set(gca,'Ytick',[-100:10:10])   % set y-axis tick label positions
    set(gca,'FontSize',fig_scale*12);       % set font size of axis labels (def. 10)
    set(gca,'LineWidth',fig_scale*1.0); % set width of plot box lines (def. 0.5)
    set(get(gca,'Children'),'LineWidth',fig_scale*1.5); % set width of ploted line (def. 0.5)
    set(gcf,'Position', [100 100 fig_scale*560 fig_scale*420])  % scale plot
  end
  grid on
  zoom on
  % print c:\work\sps\filter\filtsoft\junk.eps -deps -tiff
  % print c:\work\sps\filter\filtsoft\junk.tif -tiff
  % print -dbitmap      % Send figure to clipboard in bitmap format
  
  %--------------------------------------------------------------------------
  % Plot Ch B results
  if(plot_impulse)
    h_b = filter(b_b, a_b, [1, zeros(1,N_b(i)-1)]); % compute impulse response
    figure
    plot((0:(length(h_b)-1))/fs(i), h_b, 'b')
    title('Channel B Impulse Response')
    set(get(gca,'Title'),'FontName','Helvetica','FontSize',16)  % set Title font and size
    xlabel('Time (msec)')
    set(get(gca,'XLabel'),'FontName','Helvetica','FontSize',14) % set x-label font and size
    grid on
    zoom on
  end
  
  [H_b,f] = freqz(b_b, a_b, Nfreq_pts, fs(i));
  figure
  plot(f,20*log10(abs(H_b)), 'b')
  title('Channel B Magnitude Response')
  set(get(gca,'Title'),'FontName','Helvetica','FontSize',fig_scale*16)  % set Title font and size
  xlabel('Frequency (KHz)')
  set(get(gca,'XLabel'),'FontName','Helvetica','FontSize',fig_scale*14) % set x-label font and size
  ylabel('Magnitude (dB)')
  set(get(gca,'YLabel'),'FontName','Helvetica','FontSize',fig_scale*14) % set y-label font and size
  axis([0, fplot, -100,10])     % set axis limits to plot
  if(manual_axis)
    set(gca,'Xtick',[0:5:fplot])    % set x-axis tick label positions
    set(gca,'Ytick',[-100:10:10])   % set y-axis tick label positions
    set(gca,'FontSize',fig_scale*12);       % set font size of axis labels (def. 10)
    set(gca,'LineWidth',fig_scale*1.0); % set width of plot box lines (def. 0.5)
    set(get(gca,'Children'),'LineWidth',fig_scale*1.5); % set width of ploted line (def. 0.5)
    set(gcf,'Position', [100 100 fig_scale*560 fig_scale*420])  % scale plot
  end
  grid on
  zoom on
  % print c:\work\sps\filter\filtsoft\junk.eps -deps -tiff
  % print c:\work\sps\filter\filtsoft\junk.tif -tiff
  % print -dbitmap      % Send figure to clipboard in bitmap format
  
  %--------------------------------------------------------------------------
  % Plot Cascaded Ch A and Ch B results
  b_ab = conv(b_a, b_b);    % convolve the polynomaials to get equivelent cascade
  a_ab = conv(a_a, a_b);
  if(plot_impulse)
    h_ab = filter(b_ab, a_ab, [1, zeros(1, N_a(i) + N_b(i) - 1)]);  % compute impulse response
    figure
    plot((0:(length(h_ab)-1))/fs(i), h_ab, 'g')
    title('Cascaded Impulse Response')
    set(get(gca,'Title'),'FontName','Helvetica','FontSize',16)  % set Title font and size
    xlabel('Time (msec)')
    set(get(gca,'XLabel'),'FontName','Helvetica','FontSize',14) % set x-label font and size
    grid on
    zoom on
  end
  
  [H_ab,f] = freqz(b_ab, a_ab, Nfreq_pts, fs(i));
  figure
  plot(f,20*log10(abs(H_ab)), 'g')
  title('Cascaded Magnitude Response')
  set(get(gca,'Title'),'FontName','Helvetica','FontSize',fig_scale*16)  % set Title font and size
  xlabel('Frequency (KHz)')
  set(get(gca,'XLabel'),'FontName','Helvetica','FontSize',fig_scale*14) % set x-label font and size
  ylabel('Magnitude (dB)')
  set(get(gca,'YLabel'),'FontName','Helvetica','FontSize',fig_scale*14) % set y-label font and size
  axis([0, fplot, -100,10])     % set axis limits to plot
  if(manual_axis)
    set(gca,'Xtick',[0:5:fplot])    % set x-axis tick label positions
    set(gca,'Ytick',[-100:10:10])   % set y-axis tick label positions
    set(gca,'FontSize',fig_scale*12);       % set font size of axis labels (def. 10)
    set(gca,'LineWidth',fig_scale*1.0); % set width of plot box lines (def. 0.5)
    set(get(gca,'Children'),'LineWidth',fig_scale*1.5); % set width of ploted line (def. 0.5)
    set(gcf,'Position', [100 100 fig_scale*560 fig_scale*420])  % scale plot
  end
  grid on
  zoom on
  % print c:\work\sps\filter\filtsoft\junk.eps -deps -tiff
  % print c:\work\sps\filter\filtsoft\junk.tif -tiff
  % print -dbitmap      % Send figure to clipboard in bitmap format

end % for i  1:length(funca)

disp(' ')
disp('Type: ''close all'' to close all figure windows')
disp(' ')
disp('Type: ''print -dbitmap'' to send figure to clipboard in bitmap format')
disp(' ')

