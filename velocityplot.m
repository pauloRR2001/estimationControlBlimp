clear;clc

raw = readcell('data_20241105_1350.csv');

raw = cell2mat(raw(2:end,:));
time = raw(:,1);
xpos = raw(:,2);
ypos = raw(:,3);

mask = (isnan(xpos)) | (isnan(ypos));
time(mask) =[];
xpos(mask) = [];
ypos(mask) = [];


xvel = diff(xpos)./diff(time);
yvel = diff(ypos)./diff(time);
time = time(1:end-1);

vel = zeros(1,length(yvel));

for i = 1:length(vel)
    vel(i) = sqrt((xvel(i)^2)+ (yvel(i)^2));
end 

cutoff = 0.5;
Fs = 1/0.01;
wn = cutoff/(Fs/2);
[b,a] = butter(2,wn);

velfilter = filtfilt(b,a,vel);

plot(time,velfilter)
plot(time,vel)
%plot(xpos,ypos)