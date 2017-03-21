fid = serial('COM1','baudrate', 9600, 'terminator','cr');
%fid = 1
fopen(fid)

while(1)
  str = input('Enter Versa-Filter command: ', 's');
  fprintf(fid, str)
end

fprintf(fid,'aat')
fprintf(fid,'at all display:Hi Hi')


if(0)
fprintf(fid,'at all sendsn\r')
rtext = fscanf(fid);
disp(rtext)
end

if(0)

fs = 48000;     % sampling rate

% Design a differentiator filter:
n_diff = 20;        % even
%diff_coefs = firls(n_diff-1,[0 .95], [0 .95],'differentiator');
diff_coefs = remez(n_diff-1, [0 1], [0 1],'differentiator');

% Plot filter design results:
figure, plot(diff_coefs), title('Impulse response of Differentiator'),zoom on
[h,f] = freqz(diff_coefs,1,1024,fs);
figure, plot(f,abs(h)), grid on, zoom on

% Load the Common section of Versa-Filter with differentiator:
fprintf(fid, 'at all Mode:A&B Common\r');

%disp(sprintf('at all FUNC:UserFIR:'))
fprintf(fid, 'at all FUNC:UserFIR:');

%disp(sprintf(' %d', round(32768*diff_coefs)));
fprintf(fid, ' %d', round(32768*diff_coefs));

fprintf(fid, '\r');     % write final carriage return

end

%stopasync(fid);            % stop read or write
fclose(fid);            % close file



