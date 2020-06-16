% Als er geen optimisation is!!!:
% initialize P available per string over tijd (APC)
% initialize Q available die bij P available hoort
% count = 0
% Opti_switch = FALSE
clear
close all
%% Initializations profiles %%

dt = 200e-3;                              % 200 ms sample time
Tfinal = 300;                             % simulation will last 300 s
Ntb = 13;                                 % number of wind turbine strings
Npv = 4;                                  % number of PV module strings
t = 0:dt:Tfinal;                          % time vector
Q_sp = zeros(length(t), 1);               % initialize input vector with Q set points
k_p = 3;                                  % proportional gain VSPI controller
T_i = 0.7;                                % integral time constant VSPI controller
beta = 2;                                 % parameter beta for VSPI
sigma = 2;                                % parameter sigma for VSPI
error = zeros(length(t),1);               % difference between Q set point TSO and Q delivered by PPM
u = zeros(length(t),1);                   % output VSPI controller
Q_sp_pcc = zeros(1,length(t));            % TSO Q set point request
Q_ppm = zeros(1,length(t));               % Q delivered by PPM at PCC
Q_big = zeros(1,length(t));               % Q that will be devided over the strings
threshold_opt = 3;                        % threshold for implementing optimization set point
Opti_switch = FALSE;                      % logic variable that determines optimization usage

%% Set point times and values %%

sp_1 = 0;                               % first setpoint 0 MVar
t_sp1 = 0;                              % first set point at 0 seconds
sp_2 = 50;                              % second setpoint 50 MVar
t_sp2 = 100;                            % second setpoint at 100 seconds
sp_3 = 0;                               % third setpoint 0 MVar
t_sp3 = 200;                            % third setpoint at 200 seconds
sp_4 = 0;                               % fourth setpoint 0 MVar
t_sp4 = 250;                            % fourth setpoint at 250 seconds

setpoint_values = [sp_1 sp_2 sp_3 sp_4];    % vector containing set points
setpoint_times = [t_sp1 t_sp2 t_sp3 t_sp4]; % vector containing set point times

%% setting up input vector P_sp_PCC containing the setpoints %%

idx_begin = 1;
for i = 2:length(setpoint_values)
    [val,idx_end]=min(abs(t-setpoint_times(i)));
    Q_sp_pcc(idx_begin:idx_end-1)= setpoint_values(i-1);
    idx_begin = idx_end;
end
Q_sp_pcc(idx_begin:end) = setpoint_values(end);

%% Initialize wind capability

stepsize = 0.1;
vmin = 0;
vmax = 30;
v = vmin:stepsize:vmax;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
endsamp_string = 25/stepsize + 1;


%% Initialize PV capability

irradiance = linspace(0,680,300);         % solar profile for capability curver
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

%% plot capability curves %%

% for i = 1:Ntb+Npv
% figure(1)
% subplot(5,4,i)
%     if i<=Ntb
%         plot(PQ(i).P(1:endsamp_string),PQ(i).Q(1:endsamp_string))
%     else
%         plot(PQ(i).P,PQ(i).Q)
%     end
% xlabel('P [MW]')
% ylabel('Q [MVAr]')
% end

%% Obtain Q available for each strings %%

% load in P of each string at every time instant
%APC = P_current_string.';
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

%% Iterate over time %%

for j = 1:length(t)
 
% (depends on mode)if the Reactive Power Exchange at the PCC or the Voltage Deviation at the PCC exceeds the predefined 
% Disturbance Threshold, the Reactive Power Support of the PPM must be maintained for at least 15 minutes.

error(j) =  Q_sp_pcc(j) - Qppm(j);
    
u(j) = k_p*( error(j) + trapz( (error/T_i)*exp(-(error^2)/(2*beta^(2)*sigma^2)) ) ); 

Q_big(j) = u(j) + Q_sp_pcc(j);

    % Check if set point is reached and stable %
    count = 0;
    if error(j) < threshold
        count = count+1;
            if count == 100
                Opti_switch = TRUE; % implement optimazation setpoints
            end         
    else
        count = 0;
    end
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
end