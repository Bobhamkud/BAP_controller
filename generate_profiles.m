clear
close all

%% Time
dt = 200e-3;
t = [0:1:18000];

%% Wind speed
setting_wind = [6.50 7.50 8.50 10];
v1 = [linspace(setting_wind(1), setting_wind(2), 4501) linspace(setting_wind(2), setting_wind(3) , 4500) linspace(setting_wind(3), setting_wind(4), 4500) linspace(setting_wind(4), setting_wind(4), 4500)];

L1=180;
for i=1:(length(t)-1)/L1
    variation1(i) = normrnd(0,0.7);
end
variation1 = interp1([0:L1:18000-1], variation1, t, 'pchip');
v2 = v1+variation1;

% v2(1) = setting_wind(1);
% v2(4501) = setting_wind(2);
% v2(9001) = setting_wind(3);
% v2(13501) = setting_wind(4);

%% Irradiance
setting_solar = [650 600 800 450];
irradiance1 = [linspace(setting_solar(1), setting_solar(2), 4501) linspace(setting_solar(2), setting_solar(3) , 4500) linspace(setting_solar(3), setting_solar(4), 4500) linspace(setting_solar(4), setting_solar(4), 4500)];

L2=360;
for i=1:(length(t)-1)/L2
    variation2(i) = normrnd(0,10);
end
variation2 = interp1([0:L2:18000-1], variation2, t, 'ppval');

L3=18;
for i=1:(length(t)-1)/L3
    variation3(i) = normrnd(0,2);
end
variation3 = interp1([0:L3:18000-1], variation3, t, 'ppval');

irradiance2 = irradiance1+variation2+variation3;
% irradiance2(1) = setting_solar(1);
% irradiance2(4501) = setting_solar(2);
% irradiance2(9001) = setting_solar(3);
% irradiance2(13501) = setting_solar(4);


figure
plot(t,v1,t,v2)
figure
plot(t,irradiance1,t,irradiance2)