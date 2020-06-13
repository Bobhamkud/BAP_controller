clear all
close all
windspeed = [1 2 3 4 5 6 7 8 9];
irradiance = [1 2 3 4 5 6 7 8 9];
Systemdata = loadcase('system_17');
generate_case(windspeed,irradiance);