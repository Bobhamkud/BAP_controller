clear
close all
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

%% setting up input vector Q_sp_pcc containing the setpoints %%

dt = 200e-3;                              % 200 ms sample time
Tfinal = 10;                              % simulation will last 300 s
Ntb = 13;                                 % number of wind turbine strings
Npv = 4;                                  % number of PV module strings
t = 0:dt:Tfinal;                          % time vector
Q_sp_pcc = zeros(length(t),1);            % TSO Q set point request

idx_begin = 1;
for i = 2:length(setpoint_values)
    [val,idx_end]=min(abs(t-setpoint_times(i)));
    Q_sp_pcc(idx_begin:idx_end-1)= setpoint_values(i-1);
    idx_begin = idx_end;
end
Q_sp_pcc(idx_begin:end) = setpoint_values(end);

%% Initialize wind capability

stepsize = 0.01;
vmin = 0;
vmax = 30;
v = vmin:stepsize:vmax;
[Pwtg_string,Qwtg_string] = compute_pq_wtg(v);
endsamp_string = 25/stepsize + 1;


%% Initialize PV capability

irradiance = linspace(0,990,3000);         % solar profile for capability curver
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

Q_available_m = zeros(length(t), Ntb+Npv);% Q available for each string at a certain time instant

% load in P of each string at every time instant
load('agreed_profiles.mat') % load wind and solar profiles
load('P_current_RPC.mat')
%[ P_current_string ] = active_power_func( windspeed, irradiance );
APC = P_current_string.';

% convert outcome from previous steps into Qavailable for each string at every
% time instant
Q_available(Ntb+Npv)  = struct();
for i = 1:Ntb+Npv
    for j = 1:length(t)
        [val,idx]=min(abs(PQ(i).P-APC(i,j)));
        Q_available(i).string(j) = PQ(i).Q(idx);
    end
end

for i = 1:Ntb+Npv
    Q_available_m(:,i) = Q_available(i).string(1:end);
end

%% Optimization loop %%

jl1_best = 0;
jl2_best = 0;
jl3_best = 0;
jl4_best = 0;


k_p = 0:0.04:0.99;                % proportional gain VSPI controller
T_i = 0.01:0.5:3;                 % integral time constant VSPI controller
beta = 1.5;                       % parameter beta for VSPI
sigma = 1.5;                      % parameter sigma for VSPI

steady_error = 10000*ones(length(k_p)*length(T_i)*length(beta)*length(sigma)+100,1);
srt = 0;
for jl1 = 1:length(k_p)
    k_p(jl1)
    for jl2 = 1:length(T_i)
        T_i(jl2)
        for jl3 = 1:length(beta)
            for jl4 = 1:length(sigma)
                srt = srt+1;

%% Initializations %%


error = zeros(length(t),1);               % difference between Q set point TSO and Q delivered by PPM
u = zeros(length(t),1);                   % output VSPI controller

Q_sp = zeros(length(t), 1);               % initialize input vector with Q set points
Q_pcc = zeros(length(t),1);               % Q at PCC
Q_sp_strings = zeros(length(t), Ntb+Npv); % Q set point for each string at some time instants
Q_ppm = zeros(length(t),1);               % Q delivered by PPM at PCC
Q_big = zeros(length(t),1);               % Q that will be devided over the strings
Q_current_string = zeros(length(t), Ntb+Npv); % Q currently produced by each string

threshold_opt = 0.1;                      % threshold for implementing optimization set point
Opti_switch = false;                      % logic variable that determines optimization usage
Opti_new = true;                          % logic variable to indicate that optimization set point is based on latest TSO request
count = 0;                                % variable to determine stability PPM output
Interrupt = 0;                            % variable indicating an interrupt signal for optimization
distribution = 0:0.01:1;                  % distribution factor determining set points strings
opti_distribution = zeros(1,length(distribution)); % vector used to determine optimal distribution factor
Optimization_setpoint = zeros(length(t),Ntb+Npv); % optimization set points matrix

Run_pf_setting = mpoption('verbose',0,'out.all',0); % hide MATPOWER output
%% Run initial power flow %%

system = loadcase('system_17');
system.gen(6:end,[3,4,5]) = [(Q_available_m(1,1:Ntb)-0.3*Q_available_m(1,1:Ntb)).' (Q_available_m(1,1:Ntb)-0.3*Q_available_m(1,1:Ntb)).' (Q_available_m(1,1:Ntb)-0.3*Q_available_m(1,1:Ntb)).'];
system.gen(2:5,[3,4,5]) = [(Q_available_m(1, Ntb+1:end)-0.3*Q_available_m(1,Ntb+1:end)).' (Q_available_m(1, Ntb+1:end)-0.3*Q_available_m(1,Ntb+1:end)).' (Q_available_m(1, Ntb+1:end)-0.3*Q_available_m(1,Ntb+1:end)).'];
current_values = runpf(system, Run_pf_setting);
Q_current_string(1,1:Ntb) = current_values.gen(6:18,3);
Q_current_string(1,Ntb+1:end) = current_values.gen(2:5,3);
Q_pcc(1) = -1*current_values.gen(1,3);
%% Iterate over time %%

                z = dec2bin(2^Ntb-1:-1:0)-'0'; % create matrix with each row one the possible
                                                       % combinations of 0 and 1 for Ntb
                                                       % amount of number
                % turn each 0 into -1 %                               
                for l = 1:length(z(1,:))
                    for k = 1:length(z(:,1))
                        if z(k,l) == 0
                            z(k,l) = -1;
                        end
                    end
                end

                for j = 1:length(t)

                % (depends on mode)if the Reactive Power Exchange at the PCC or the Voltage Deviation at the PCC exceeds the predefined 
                % Disturbance Threshold, the Reactive Power Support of the PPM must be maintained for at least 15 minutes.

                    % detect set point change %
                    if j > 1
                        if Q_sp_pcc(j) ~= Q_pcc(j-1)
                            Opti_new = false;
                        end
                    end

                    % detect change in optimization set point after set point change %
                    if Opti_new == false & Optimization_setpoint(j,:) ~= Optimization_setpoint(j-1,:)
                       Opti_new = true;
                    end

                error(j) =  Q_sp_pcc(j) - Q_pcc(j);

                u(j) = k_p(jl1)*( error(j) + trapz(t, (error/T_i(jl2)).*exp(-(error.^2)./(2*beta(jl3)^(2)*sigma(jl4)^2) ) ) ); 

                Q_big(j) = u(j) + Q_sp_pcc(j);

                    % Check if set point is reached and stable %
                    if error(j) < threshold_opt
                        count = count+1;
                            if count == 100
                                Opti_switch = TRUE; % implement optimazation setpoints
                            end         
                    else
                        count = 0;
                    end

                    %--- determine set points ---%
                    if Opti_switch == 0

                        % create a matrix with all possible combinations of the signs %
                        % of the reactive power produced %
                        optQs = zeros(length(z(:,1)),Ntb+Npv);
                        for i = 1:length(z(:,1))
                            optQs(i,:) = [(z(i,:).*Q_available_m(j,1:Ntb)) Q_available_m(j,Ntb+1:end)];
                        end

                        % determine best distribution factor and best combination of signs %
                        Q_options = zeros(length(optQs(:,1))*length(distribution),3);
                        s = 0;
                        for k = 1:length(optQs(:,1))
                            for i = 1:length(distribution)
                                s = s + 1;
                                Q_options(s,1) = abs( Q_big(j) - sum(distribution(i)*optQs(k,:)) );
                                Q_options(s,2) = k;
                                Q_options(s,3) = i;
                            end
                        end

                        [valBest, idxBest] = min(Q_options(:,1));
                        kbest = Q_options(idxBest,2);
                        ibest = Q_options(idxBest,3);

                        Q_sp_strings(j,:) = distribution(ibest)*optQs(kbest,:); 

                    % Dat verdelen over strings adhv DF (Let op Q_a_i):
                %     - regels bedenken verdelen en bij Capabillty limits
                %     - component status
                %     - Last resort: OLTC, Reactor control
                % Power fow runnen voor nieuwe Q output
                % 
                    elseif Opti_switch == 1 && Opti_new == 1

                        Q_sp_strings(j,:) = Optimization_setpoint(j,:); % implement optimization set point
                        %---- check feasibility optimization set points ----% 
                        if error > threshold
                            Opti_switch = false;
                        end
                        for i=1:Ntb+Npv % check for set points being within available Q bounds
                            if abs(Q_available(i).string(j)) < abs(Optimization_setpoint(j,i))
                                Opti_switch = false;
                            end
                        end
                    end


                    %--- apply set points ---%
                    system.gen(6:end,[3,4,5]) = [Q_sp_strings(j,1:Ntb).' Q_sp_strings(j,1:Ntb).' Q_available_m(j,1:Ntb).'];
                    system.gen(2:5,[3,4,5]) = [Q_sp_strings(j,Ntb+1:end).' Q_sp_strings(j,Ntb+1:end).' Q_available_m(j,Ntb+1:end).'];
                    current_values = runpf(system, Run_pf_setting);
                    Q_current_string(j+1,1:Ntb) = current_values.gen(6:18,3);
                    Q_current_string(j+1,Ntb+1:end) = current_values.gen(2:5,3);
                    Q_pcc(j+1) = -1*current_values.gen(1,3);
                end
                steady_error(srt) = abs(Q_sp_pcc(50)-Q_pcc(50));
                if srt > 1
                    if steady_error(srt) < steady_error(srt-1)
                        k_p_best = k_p(jl1);
                        T_i_best = T_i(jl2);
                        beta_best = beta(jl3);
                        sigma_best = sigma(jl4);
                    end    
                end
            end
        end
    end
end

%plot(t,Q_pcc(1:end-1))
%hold on
%plot(t,Q_sp_pcc)
    