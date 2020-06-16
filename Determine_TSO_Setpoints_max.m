%Vindt de maximale active power die de PPM kan leveren op PCC met deze code:
clear all
close all
v = 6.5;
irradiance = 650;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
[Ppvg_string,Qpvg_string] = compute_pq_pvg(irradiance,1);
factor = 1;

P_PCC_max = factor*(sum(Pwtg_string)+ 4*Ppvg_string)
v = 7.5;
irradiance = 600;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
[Ppvg_string,Qpvg_string] = compute_pq_pvg(irradiance,1);
factor = 0.8;

P_PCC_max = factor*(sum(Pwtg_string)+ 4*Ppvg_string)

v = 8.5;
irradiance = 800;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
[Ppvg_string,Qpvg_string] = compute_pq_pvg(irradiance,1);
factor = 0.9;

P_PCC_max = factor*(sum(Pwtg_string)+ 4*Ppvg_string)

v = 10;
irradiance = 450;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
[Ppvg_string,Qpvg_string] = compute_pq_pvg(irradiance,1);
factor = 1;

P_PCC_max = factor*(sum(Pwtg_string)+ 4*Ppvg_string)
