%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Task
% Authors: Farley Rimon & Marouane Mastouri
% Last edited: 16/05/2020
%
% This is a typical layout of an on-shore WPP. Parameters of system 
% components were taken from a on-shore WPP in Zeewolde.
%
% See Matpower user's manual for details on the case file format.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function mpc = system_17
%% MATPOWER Case Format : Version 2
mpc.version = '2';

%%-----  Power Flow Data  -----%%
%% system MVA base
mpc.baseMVA = 350;

%% bus data 
% bus_i	   type	    Pd	   Qd	    Gs	    Bs	   area	    Vm	    Va	    baseKV    zone	   Vmax	    Vmin
mpc.bus = [
	1       3       410.355  0       0       0       1       1       0        150      1       1.1424   0.8576;        
    2       1       0        0       0       0       1       1       0        150      1       1.1424   0.8576;
	3       1       0        0       0       0       1       1       0        150      1       1.1424   0.8576;
	4       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
	5       1       0        0       0       0       1       1       0        150      1       1.1424   0.8576;  
	6       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
    7       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
    8       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
    9       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
   10       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
   11       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
   12       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
   13       1       0        0       0    	 0       1       1       0        33       1       1.1424   0.8576;
   14       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   15       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   16       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   17       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   18       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   19       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   20       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   21       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   22       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   23       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   24       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   25       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   26       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   27       1       0        0       0       0       1       1       0        33       1       1.1424   0.8576;
   28       4       0        0       0       -12     1       1       0        33       1       1.1424   0.8576;%shunt reactor
   ]; 
%% generator data
%% The Qmax and Qmin are set to 0 in order to test if the system convergs.
%% The generating strings as well as the strings are modeled in the generator data and bus data.
%  bus	Pg	      Qg	    Qmax	Qmin	Vg	    mBase  status   Pmax    Pmin	       Pc1	   Pc2	  Qc1min  Qc1max  Qc2min  Qc2max  ramp_agc  ramp_10  ramp_30  ramp_q  apf
mpc.gen = [
	1	0         0       0        0        1.0     100     1      410.35   0       0       0       0       0       0       0       0       0       0       0       0;
    11	12        4       4        4        1.0     100     1      12       12      0       0       0       0       0       0       0       500     0       0       0;
    16	12        4       4        4        1.0     100     1      12       12      0       0       0       0       0       0       0       500     0       0       0;
    22	12        4       4        4        1.0     100     1      12       12      0       0       0       0       0       0       0       500     0       0       0;
    27	12        4       4        4        1.0     100     1      12       12      0       0       0       0       0       0       0       500     0       0       0;
    8	33        22.4    22.4     22.4     1.0     100     1      32       32      0       0       0       0       0       0       0       96.6    0       0       0;
    9	29.4      19.15   19.15    19.15	1.0     100     1      28.6     28.6    0       0       0       0       0       0       0       96.6    0       0       0;
    10	28.8      19.6    19.6     19.6     1.0     100     1      28       28      0       0       0       0       0       0       0       96.6    0       0       0;
    13	28.8      19.6    19.6     19.6     1.0     100     1      28       28      0       0       0       0       0       0       0       96.6    0       0       0;
    14	29.15     13.248  13.248   13.248   1.0     100     1      29.15    29.15   0       0       0       0       0       0       0       78.6    0       0       0;
    15	28        19.6    19.6     19.6     1.0     100     1      28       28      0       0       0       0       0       0       0       96.6    0       0       0;
    18	16        12.2    12.2     12.2     1.0     100     1      16       16      0       0       0       0       0       0       0       55.2    0       0       0;
    19	16.8      10.9    10.9     10.9     1.0     100     1      16.4     16.4    0       0       0       0       0       0       0       55.2    0       0       0;
    20	28        19.6    19.6     19.6     1.0     100     1      28       28      0       0       0       0       0       0       0       96.6    0       0       0;
    21	32        22.4    22.4     22.4     1.0     100     1      32       32      0       0       0       0       0       0       0       10.4    0       0       0;
    24	29.4      18.55   18.55    18.55    1.0     100     1      29.4     29.4    0       0       0       0       0       0       0       96.6    0       0       0;
    25	33.6      21.2    21.2     21.2     1.0     100     1      29.4     29.4    0       0       0       0       0       0       0       96.6    0       0       0;
    26	29.4      19.15   19.15    19.15    1.0     100     1      28.6     28.6    0       0       0       0       0       0       0       96.6    0       0       0;
   %Equivalent grid
];

%% branch data

%%the p.u calculations are checked several times; lecture and the data of
%%ABB are used in order to find the right p.u values for the transformer.

% fbus  tbus	   r	              x	                        b	        rateA        rateB     rateC             ratio   angle   status	 ratiomax	ratiomin
mpc.branch = [
    %Main bus to transformer
    1   2       0.0003437777778     0.001960977778      0.1332427984          400       400       400              0       0        1       0        0;
    2	3       0.000005379555556	0.00001526222222	0.0008796145272       400       400       400              0       0        1       0        0;
    2   5       0.000003615377778	0.00001025711111	0.0005911527043       400       400       400              0       0        1       0        0;
    3	4       0                   0.0625              0.000692791866        240       240       240              1.0     0        1       1.153    0.867;
    5   6       0                   0.0625              0.000692791866        240       240       240              1.0     0        1       1.153    0.867;
    %%T01 and T02 to strings                                                                                 
    4   7       0.00003340220386	0.0001905417815     0.0002600107744       185.19        185.19         185.19        0        0       1        0        0;                                                                                                                                  
    7   8       0.003928099174      0.007469237833      0.005733237576        46.2977       46.2977       46.2977        0        0       1        0        0;
    7   9       0.007997823691      0.01520777472       0.01167318372         46.2977       46.2977       46.2977        0        0       1        0        0;
    7   10      0.004769834711      0.009069788797      0.006961788485        46.2977       46.2977       46.2977        0        0       1        0        0;           
    7   11      0.007575619835      0.01440495868       0.01105695818         46.2977       46.2977       46.2977        0        0       1        0        0;
    7   12      0.0000355399449     0.00006757881849	0.0000518721495       46.2977       46.2977       46.2977        0        0       0        0        0;
    6   12      0.00006680440771	0.00009527089073	0.002080086195        185.19        185.19        185.19         0        0       1        0        0;
    12  13      0.002852102847      0.01626972758       0.0124883175          46.2977       46.2977       46.2977        0        0       1        0        0;
    12  14      0.006977052342      0.03980036731       0.03054996594         46.2977       46.2977       46.2977        0        0       1        0        0;
    12  15      0.004181065197      0.02385074992       0.01830735863         46.2977       46.2977       46.2977        0        0       1        0        0;
    12  16      0.00329835629       0.01881536578       0.01444229846         46.2977       46.2977       46.2977        0        0       1        0        0;
    12  17      0.00005317630854	0.0003033425161     0.0002328396485       46.2977       46.2977       46.2977        0        0       1        0        0;
    6   17      0.00003674242424	0.0000523989899     0.001144047407        185.19        185.19        185.19         0        0       1        0        0;
    17  18      0.00589214876       0.03361157025       0.02579956909         46.2977       46.2977       46.2977        0        0       1        0        0;   
    17  19      0.004723516988      0.02694514845       0.02068255705         46.2977       46.2977       46.2977        0        0       1        0        0;
    17  20      0.003067658402      0.01749935721       0.01343215661         46.2977       46.2977       46.2977        0        0       1        0        0;
    17  21      0.003601202938      0.0205429446        0.01576835341         46.2977       46.2977       46.2977        0        0       1        0        0;
    17  22      0.004634444444      0.02643703704       0.02029254089         46.2977       46.2977       46.2977        0        0       1        0        0;
    17  23      0.0000118466483     0.00006757881849    0.0000518721495       46.2977       46.2977       46.2977        0        0       0        0        0;       
    4   23      0.00003340220386	0.00004763544536	0.001040043098        185.19        185.19        185.19         0        0       1        0        0;
    23  24      0.006733884298      0.03841322314       0.02948522182         46.2977       46.2977       46.2977        0        0       1        0        0;
    23  25      0.006201230487      0.03537471687       0.02715292517         46.2977       46.2977       46.2977        0        0       1        0        0;  
    23  26      0.008417355372      0.01600550964       0.01228550909         46.2977       46.2977       46.2977        0        0       1        0        0;   
    23  27      0.005111873278      0.009720171411      0.007461009172        46.2977       46.2977       46.2977        0        0       1        0        0;
    
    %Shunt reactor to bus bars
    12  28      0.0001581126722     0.0003006495256     0.0002307725628       46.2977       46.2977       46.2977          0        0       0        0        0;     
    17  28      0.0001581126722     0.0003006495256     0.0002307725628       46.2977       46.2977       46.2977          0        0       0        0        0; 
 ];