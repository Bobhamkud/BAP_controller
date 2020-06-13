% Als er geen optimisation is!!!:
% initialize P available per string over tijd (APC)
% initialize Q available die bij P available hoort
% count = 0
% Opti_switch = FALSE
clear all
close all
%% Initialize profiles
dt = 200e-3;                              % 10 ms sample time
Tfinal = 300;                             % simulation will last 200 s
Ntb = 13;                                 % number of wind turbine strings
Npv = 4;                                  % number of PV module strings
t = 0:dt:Tfinal;                          % time vector
Q_sp = zeros(length(t), 1);               % initialize input vector with Q set points

% Set point times and values %
sp_1 = 0e6;                               % first setpoint 0 MVar
sp_2 = 50e6;                              % second setpoint 50 MVar
t_sp2 = 100;                              % second setpoint at 100 seconds
sp_3 = 0e6;                               % third setpoint 0 MVar
t_sp3 = 200;                              % third setpoint at 200 seconds
sp_4 = 0e6;                               % fourth setpoint 0 MVar
t_sp4 = 250;                              % fourth setpoint at 250 seconds

windspeed = linspace(0,30,300);           % wind profile      
irradiance = linspace(0,680,300);         % solar profile
Q_sp_PCC = zeros(1,length(t));      % TSO Q set point request

%% Initialize wind capability
stepsize = 0.1;
vmin = 0;
vmax = 30;
v = vmin:stepsize:vmax;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
endsamp_string = 25/stepsize + 1;
% figure(1)
% plot(P(:,1),Q(:,1))

%% Initialize PV capability

[Ppvg_string,Qpvg_string] = compute_pq_pvg(irradiance,Npv);
Structure_Size = Ntb+Npv;  %number of elements
PQ(Structure_Size) = struct();
for j = 1:Ntb+Npv
    if j <= Ntb
        PQ(j).P = Pwtg_string(:,j);
        PQ(j).Q = Qwtg_string(:,j);
    else
        PQ(j).P = Ppvg_string(j-Ntb,:)';
        PQ(j).Q = Qpvg_string(j-Ntb,:)';        
    end   
end

% plot capability curves %

for i = 1:Ntb+Npv
figure(1)
subplot(5,4,i)
if i<=Ntb
    plot(PQ(i).P(1:endsamp_string),PQ(i).Q(1:endsamp_string))
else
    plot(PQ(i).P,PQ(i).Q)
end
xlabel('P [MW]')
ylabel('Q [MVAr]')
end

%% Obtain Q available for each strings %%

% load in P of each string at every time instant
APC = 10*ones(Ntb+Npv, length(t));

% convert outcome from previous steps into Qavailable for each string at every
% time instant
Q_available(Ntb+Npv)  = struct();
for i = 1:Ntb+Npv
    for j = 1:length(t)
        [val,idx]=min(abs(PQ(i).P-APC(i,j)));
        Q_available(i).string(j) = PQ(i).Q(idx);
    end
end

% for t
% 
% (depends on mode)if the Reactive Power Exchange at the PCC or the Voltage Deviation at the PCC exceeds the predefined 
% Disturbance Threshold, the Reactive Power Support of the PPM must be maintained for at least 15 minutes.
%
% error =  Q_sp_PCC - Qppm
% u(t) = VSPI(error)
% u(t) + TSO_setpoint
% 
% Ben ik stabiel?
% if error < waarde
%     ++count
%     if count = 100
%         implement opti setpoints
%         Opti_switch = TRUE
% else
%     count = 0
% 
% If Opti_switch = FALSE
% Dat verdelen over strings adhv DF (Let op Q_a_i):
%     - Can setpoint be reached with generators?
%     - regels bedenken verdelen en bij Capabillty limits
%     - setpoint feasibility optimisation check
%     - Rules voor interrupt naar optimisation
%     - component status
%     - Last resort: OLTC, Reactor control
% Power fow runnen voor nieuwe Q output
% 
% Elseif Opti_switch = TRUE
%     if error > waarde
%         Opti_switch = FALSE
%         Interrupt ON
%     DF = opti setpoint
% 
% ++t