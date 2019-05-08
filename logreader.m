% logreader.m
% Use this script to read data from your micro SD card

clear;
%clf;

folder = 'danapoint/';
% 12 goes underwater
filenum = '118'; % file number for the data you want to read
infofile = strcat(folder, 'INF', filenum, '.TXT');
datafile = strcat(folder, 'LOG', filenum, '.BIN');

%% map from datatype to length in bytes
dataSizes.('float') = 4;
dataSizes.('ulong') = 4;
dataSizes.('long') = 4;
dataSizes.('int') = 4;
dataSizes.('int32') = 4;
dataSizes.('uint8') = 1;
dataSizes.('uint16') = 2;
dataSizes.('char') = 1;
dataSizes.('bool') = 1;

%% read from info file to get log file structure
%change the txt file
fileID = fopen(infofile);
items = textscan(fileID,'%s','Delimiter',',','EndOfLine','\r\n');
fclose('all');
[ncols,~] = size(items{1});
ncols = ncols/2;
varNames = items{1}(1:ncols)';
varTypes = items{1}(ncols+1:end)';
varLengths = zeros(size(varTypes));
colLength = 256;
for i = 1:numel(varTypes)
    varLengths(i) = dataSizes.(varTypes{i});
end
R = cell(1,numel(varNames));

%% read column-by-column from datafile
%change the bin file
fid = fopen(datafile,'rb');
for i=1:numel(varTypes)
    %# seek to the first field of the first record
    fseek(fid, sum(varLengths(1:i-1)), 'bof');
    
    %# % read column with specified format, skipping required number of bytes
    R{i} = fread(fid, Inf, ['*' varTypes{i}], colLength-varLengths(i));
    eval(strcat(varNames{i},'=','R{',num2str(i),'};'));
end
fclose(fid);

%% Process you data here
N = length(accelX);

%convert sample number to time vector
t = [0:0.1:N*0.1-0.1]';

%converting to m/s^2 from accel units

realaccelX = 0.01 .* accelX;
%realaccelX = realaccelX - mean(realaccelX);
realaccelY = 0.01 .* accelY;
%realaccelY = realaccelY - mean(realaccelY);
realaccelZ = 0.01 .* accelZ;
realveloX = 0;
realveloY = 0;
realveloZ = 0;
realposX = 0;
realposY = 0;
realposZ = 0;
dt = 0.1;

%assuming starting from (0,0,0)
for i = 2:N
    realveloX(i) = realveloX(i-1) + realaccelX(i)*dt;
    realveloY(i) = realveloY(i-1) + realaccelY(i)*dt;
    realveloZ(i) = realveloZ(i-1) + realaccelZ(i)*dt;
    realposX(i) =  realposX(i-1) + realveloX(i)*dt;
    realposY(i) =  realposY(i-1) + realveloY(i)*dt;
    realposZ(i) =  realposZ(i-1) + realveloZ(i)*dt;
end
% clf
%plot(t, headingIMU)
%hold on
%plot(t, realveloX)
%plot(t, realposX)
%plot(t, rollIMU)

%plot(t, yaw)
%plot(t, headingIMU)
floatTempData = cast(A01,'like', 'float');
TempVoltage = (floatTempData./1024).*3.3;
Temp = (1./(((log((10/47).*((3.3./TempVoltage)-1)))/4108)+(1/25)));
Temp2 = (1./(((log((10/47)*((3.3)./TempVoltage - 1)))./4108)+(1/298)))-273;
Temp3 = 8.89.*TempVoltage+4.03;
floatPresData = cast(A00, 'like', 'float');
presVoltage = (floatPresData./1024).*3.3;
Press = (presVoltage+57.02)./0.5629;
Depth = 0.753.*presVoltage+0.576;
%plot(x, y)
%xlim([-110 110]);
%ylim([-110 110]);
%ylim([-50 50])
%plot(t, gyroZ)
%figure(2)
plot(t, Temp3);
title('Run 1 Temperature');
xlabel('Time (s)');
ylabel('Temperature (degC)');
%imshow("background.png");
% xlabel("time");
% ylabel("control effort");
%%





