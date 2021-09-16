% Analyze C# program data
%datainLeft=csvread('LeftMastData_1540573280.cvs');
datain=csvread('RightMastData_1540572876.cvs');
dataLength=length(datain(:,1));
delTime=datain(2:dataLength,1)-datain(1:dataLength-1,1);
freqS1=10000;
freqS2=12000;
Fsamp=48000;
filtLen=12;
filtParm = firgr(filtLen,[freqS1/(Fsamp/2) freqS2/(Fsamp/2)], ...
    [1 0],{'n' 'n'});
dataOut=filtfilt(filtParm,1,datain(:,2));
% sampleInc=floor(7462913/24000);
% sample=datain(sampleInc*24000+(1:24000),2);
% fftT=fft(sample);
% plot(abs(fftT))