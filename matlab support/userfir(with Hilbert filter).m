% This MATLAB script shows how to design User FIR filters
% for the Versa-Filter System. It designs a differentator
% for channel A and a 

%and generates a text output file of coefficients for downloading into the Versa-Filter.

fs = 48000;     % sampling rate

% Design a differentaiator filter:
n_diff = 20;        % even
diff_coefs = remez(n_diff-1, [0 1], [0 1],'differentiator');

% Plot filter design results:
figure, plot(diff_coefs), zoom on
figure, freqz(diff_coefs,1,1024,48000), zoom on


% Design a Hilbert transform filters (90 degree phase shift between ChA and ChB outputs):
n_hilbert = 128;
minfreq = 200;  % 100Hz minimum Hilbert frequency
hilbert_coefs1 = remez(n_hilbert-1, [2*minfreq/fs 1.0-2*minfreq/fs], [1 1],'Hilbert');
hilbert_coefs2 = zeros(1,n_hilbert);
hilbert_coefs2(n_hilbert/2) = 1;

% Alternate Hilbert design using modulated lowpass filter:
h = remez(n_hilbert-1, [0 .47 .5 1], [1 1 0 0]);

t = 0:(n_hilbert-1);
fc = 12000;
hs = exp(j*(2*pi*fc/fs)*t).*h;      % modulate real lowpass filter
% hilbert_coefs1 = real(hs);
% hilbert_coefs2 = imag(hs);


% Plot filter design results:
figure, plot(h), zoom on
figure, freqz(h,1,1024,48000), zoom on

figure, plot(hilbert_coefs1), zoom on
figure, plot(hilbert_coefs2), zoom on
figure, freqz(hilbert_coefs1,1,1024,48000), zoom on
figure, freqz(hilbert_coefs2,1,1024,48000), zoom on



% Open and write filter coefs to file:
[fid, measage] = fopen('c:\work\sps\filter\filtsoft\userfir.txt','wt');

% Write file that loads Common section of Versa-Filter with differentiator:
fprintf(fid, 'at all Mode:A&B Common\n');
fprintf(fid, 'at all FUNC:UserFIR:');
for i=1:n_diff
   fprintf(fid, ' %d', round(32768*diff_coefs(i)));
end
fprintf(fid, '\n');     % final carrage return


% Write file that loads A and B sections of Versa-Filter with Hibert transformer:
fprintf(fid, 'at all Mode:A&BSeparate\n');
fprintf(fid, 'at all aFUNC:UserFIR:');
for i=1:n_hilbert
   fprintf(fid, ' %d', round(32768*hilbert_coefs1(i)));
end
fprintf(fid, '\n');     % final carrage return

fprintf(fid, 'at all bFUNC:UserFIR:');
for i=1:n_hilbert
   fprintf(fid, ' %d', round(32768*hilbert_coefs2(i)));
end
fprintf(fid, '\n');     % final carrage return


fclose(fid);            % close file
