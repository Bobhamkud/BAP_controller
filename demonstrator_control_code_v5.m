clear
close all
clear title xlabel ylabel
tic
%% General Initializations %%

dt = 200e-3;                                    % 10 ms sample time
Tfinal = 3600;                                  % simulation will last 18000 s
Ntb = 13;                                       % number of wind turbine strings
Npv = 4;                                        % number of PV module strings
t = 0:dt:Tfinal;                                % time vector

Run_pf_setting = mpoption('verbose',0,'out.all',0); % hide MATPOWER output
load('agreed_profiles.mat')                         % load wind and solar profiles

%% Active power control initializations %%

rise_time = 20;                                 % rise time active power
rise_time_Q = 40;                               % rise time reactive power
rise_end = 0;                                   % parameter used to determine end of rising to set point
rise_end_Q = 0;                                 % parameter used to determine end of rising to set point
time_done = 1;                                  % parameter used for implementing rise time
time_done_Q = 1;                                % parameter used for implementing rise time
N_rise = floor(rise_time/dt);                   % determine time_steps for rise_time
N_rise_Q = floor(rise_time_Q/dt);               % determine time_steps for rise_time
rise = zeros(N_rise+1,Ntb+Npv);                 % initialise vector containing set points to implement rise time
rise_Q = zeros(N_rise_Q+1,Ntb+Npv);                 % initialise vector containing set points to implement rise time

case_2 =0;                                      % initialise case
case_log = zeros(1,length(t));                  % initialize case log

a_p = 70;                                       % initialize a_p constant for the distribution factors 
P_sp_pcc = zeros(length(t), 1);                 % initialize input vector with P set points
P_a_string = zeros(length(t), Ntb+Npv);         % initialize matrix with available P per string over time
P_sp_string = zeros(length(t),Ntb+Npv);         % initialize active power set point per string over time
P_current_string  = zeros(length(t), Ntb+Npv);  % initialize current active power produced per string over time
P_pcc = zeros(1,length(t));                     % initialize vector with current PCC value
P_a_tot = zeros(1,length(t));                   % total available power
P_a_wind = zeros(1,length(t));                  % total available wind power
P_a_tot_pv = zeros(1,length(t));                % total availabel power from solar irradiance

%% Reactive power control initializations %%

Q_sp = zeros(length(t), 1);               % initialize input vector with Q set points
k_p = 0.12;                               % proportional gain VSPI controller
T_i = 8;                                  % integral time constant VSPI controller
bs = 72;                                  % beta^2 * sigma ^2, parameter for VSPI
error = zeros(length(t),1);               % difference between Q set point TSO and Q delivered by PPM
u = zeros(length(t),1);                   % output VSPI controller
Q_pcc = zeros(length(t),1);               % Q at PCC
Q_sp_pcc = zeros(length(t),1);            % TSO Q set point request
Q_sp_strings = zeros(length(t), Ntb+Npv); % Q set point for each string at some time instants
Q_ppm = zeros(length(t),1);               % Q delivered by PPM at PCC
Q_big = zeros(length(t),1);               % Q that will be devided over the strings
Q_available_m = zeros(length(t), Ntb+Npv);% Q available for each string at a certain time instant
Q_current_string = zeros(length(t), Ntb+Npv); % Q currently produced by each string
Q_available_wind = zeros(length(t),1);    % total available reactive power from wind
Q_available_solar = zeros(length(t),1);   % total available reactive power from solar irradiance
Q_total_available = zeros(length(t),1);   % total available reactive power
threshold_opt = 0.1;                      % threshold for implementing optimization set point
Opti_switch = false;                      % logic variable that determines optimization usage
Opti_new = true;                          % logic variable to indicate that optimization set point is based on latest TSO request
count = 0;                                % variable to determine stability PPM output
distribution = 0:0.01:1;                  % distribution factor determining set points strings

opti_distribution = zeros(1,length(distribution));  % vector used to determine optimal distribution factor
Optimization_setpoint = zeros(length(t),Ntb+Npv);   % optimization set points matrix

% initialise variables for RPC %
Q_available(Ntb+Npv)  = struct();
z = [ones(1,Ntb);-1*ones(1,Ntb)];
optQs = zeros(length(z(:,1)),Ntb+Npv);
Q_options = zeros(length(optQs(:,1))*length(distribution),3);

%% Active power Set point times and values %%

sp_1_a = 70;                               % first set point 70 MW
t_sp1_a = 0;                               % first set point at 0 seconds
sp_2_a = 70;                               % second set point 70 MW
t_sp2_a = 100;                             % second set point at 4500 seconds
sp_3_a = 135;                              % third set point 110 MW
t_sp3_a = 1800;                            % third set point at 9000 seconds
sp_4_a = 100;                              % fourth set point 55 MW
t_sp4_a = 2700;                            % fourth set point at 13500 seconds

setpoint_values = [sp_1_a sp_2_a sp_3_a sp_4_a];    % vector containing set points
setpoint_times = [t_sp1_a t_sp2_a t_sp3_a t_sp4_a]; % vector containing set point times
%% setting up input vector P_sp_PCC containing the setpoints %%

idx_begin = 1;
for i = 2:length(setpoint_values)
    [val,idx_end]=min(abs(t-setpoint_times(i)));
    P_sp_pcc(idx_begin:idx_end-1)= setpoint_values(i-1);
    idx_begin = idx_end;
end
P_sp_pcc(idx_begin:end) = setpoint_values(end);

%% Reactive power Set point times and values %%

sp_1_r = 0;                                % first setpoint 0 MVar
t_sp1_r = 0;                               % first set point at 0 seconds
sp_2_r = 300;                              % second setpoint 50 MVar
t_sp2_r =  900;                           % second setpoint at 100 seconds
sp_3_r = 100;                              % third setpoint 0 MVar
t_sp3_r = 1700;                            % third setpoint at 200 seconds
sp_4_r = -50;                              % fourth setpoint 0 MVar
t_sp4_r = 2700;                            % fourth setpoint at 250 seconds

setpoint_values = [sp_1_r sp_2_r sp_3_r sp_4_r];    % vector containing set points
setpoint_times = [t_sp1_r t_sp2_r t_sp3_r t_sp4_r]; % vector containing set point times

%% setting up input vector Q_sp_pcc containing the setpoints %%

idx_begin = 1;
for i = 2:length(setpoint_values)
    [val,idx_end]=min(abs(t-setpoint_times(i)));
    Q_sp_pcc(idx_begin:idx_end-1)= setpoint_values(i-1);
    idx_begin = idx_end;
end
Q_sp_pcc(idx_begin:end) = setpoint_values(end);


%% Initialize wind profile %%

v_profile = zeros(1,length(t));
% for i =1:length(v_profile)
% v_profile(i) = normrnd(7.7,0.1);
% end
v_profile(1:end) = windspeed(1:length(t));
[P_a_string(:,1:Ntb),Qwtg_string_profile] = compute_pq_wtg(v_profile);

%% Initialize irradiance profile  %%

solar_profile = zeros(1,length(t));
% for i =1:length(solar_profile)
%     solar_profile(i) = normrnd(680,10);
% end
solar_profile(1:end) = irradiance(1:length(t));
[P_a_pv,Qpvg_string_profile] = compute_pq_pvg(solar_profile,Npv);
P_a_string(:,Ntb+1:end) = P_a_pv.';

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

%% run initial powerflow APC and RPC %% 

system = loadcase('system_17');
system.gen(6:end,[2,9,10]) = [(P_a_string(1,1:Ntb)-2).' (P_a_string(1,1:Ntb)-2).' (P_a_string(1,1:Ntb)-2).'];
system.gen(2:5,[2,9,10]) = [(P_a_string(1,Ntb+1:end)-2).' (P_a_string(1,Ntb+1:end)-2).' (P_a_string(1,Ntb+1:end)-2).'];
system.gen(6:end,[3,4,5]) = [(Q_available_m(1,1:Ntb)-0.4*Q_available_m(1,1:Ntb)).' (Q_available_m(1,1:Ntb)-0.4*Q_available_m(1,1:Ntb)).' (Q_available_m(1,1:Ntb)-0.4*Q_available_m(1,1:Ntb)).'];
system.gen(2:5,[3,4,5]) = [(Q_available_m(1, Ntb+1:end)-0.4*Q_available_m(1,Ntb+1:end)).' (Q_available_m(1, Ntb+1:end)-0.4*Q_available_m(1,Ntb+1:end)).' (Q_available_m(1, Ntb+1:end)-0.4*Q_available_m(1,Ntb+1:end)).'];
current_values = runpf(system, Run_pf_setting);
P_current_string(1,1:Ntb) = current_values.gen(6:18,2);
P_current_string(1,Ntb+1:end) = current_values.gen(2:5,2);
Q_current_string(1,1:Ntb) = current_values.gen(6:18,3);
Q_current_string(1,Ntb+1:end) = current_values.gen(2:5,3);
P_pcc(1) = -1 * current_values.branch(1,14);
Q_pcc(1) = -1 * current_values.gen(1,3);

tap1_old = system.branch(4,9);    % tap position
tap2_old = system.branch(5,9);    % tap position
react_old = system.bus(28,2);     % reactor status
sp_number = 1;                    % set point number

%% Initialize subplots %%

titles = {'Active power response', 'Reactive power response'};
ylabels = {'Power [MW]', 'Power [MVar]'};
plts = cell(2,17);
hh1 = cell(2,1);

%% ----------- Iterate over time ----------- %%

for j = 1:length(t)-1

    % calculate available active power and active power set points %
    [P_a_tot, P_a_tot_pv, P_a_wind, P_sp_string, case_2] = APC_time_step_setpoint_v2(P_a_string, P_a_pv, P_sp_pcc,P_pcc,P_current_string,j, time_done, P_sp_string, a_p, Ntb, Npv,P_a_tot, P_a_tot_pv, P_a_wind, case_2);
    
    % Implement rise time by adjusting active power set points %
    [P_sp_string, time_done, rise_end] = APC_rise_time_v2( P_sp_string,P_sp_pcc,j, Ntb, Npv, N_rise, rise, rise_end, time_done, P_a_string);
    
    % Determine available Q based on current P produced %
    APC = P_current_string.';
    for i = 1:Ntb+Npv
            [val,idx]=min(abs(PQ(i).P-APC(i,j)));
            Q_available(i).string(j) = PQ(i).Q(idx);
    end
    for i = 1:Ntb+Npv
        Q_available_m(j,i) = Q_available(i).string(j);
    end

    % calculate available reactive power and reactive power set points %
    [Q_sp_strings, Q_available_wind, Q_available_solar, Q_total_available, error, u, Q_big, Opti_new, Opti_switch, count, system, yes] = RPC_time_setpoint_v3( j, Ntb, Npv, Opti_new, Opti_switch, threshold_opt, Optimization_setpoint, Q_available_m, Q_sp_strings, Q_sp_pcc, Q_pcc, error, u, Q_big, count, distribution, z, optQs, Q_options, k_p, bs,t, T_i,Q_available_wind, Q_available_solar, Q_total_available, time_done_Q, Run_pf_setting, system);

     % Implement rise time by adjusting reactive power set points %
    [Q_sp_strings, time_done_Q, rise_end_Q, sp_number] = RPC_rise_time_test_v2( Q_sp_strings,Q_sp_pcc,j, Ntb, Npv, N_rise_Q, rise_Q, rise_end_Q, time_done_Q, Q_available_m, sp_number, setpoint_values, Q_pcc);   
    
    % apply set points %
    system.gen(6:end,[2,9,10]) = [P_sp_string(j,1:Ntb).' P_sp_string(j,1:Ntb).' P_sp_string(j,1:Ntb).'];
    system.gen(2:5,[2,9,10]) = [P_sp_string(j,Ntb+1:end).' P_sp_string(j,Ntb+1:end).' P_sp_string(j,Ntb+1:end).'];
    system.gen(6:end,[3,4,5]) = [Q_sp_strings(j,1:Ntb).' Q_sp_strings(j,1:Ntb).' Q_available_m(j,1:Ntb).'];
    system.gen(2:5,[3,4,5]) = [Q_sp_strings(j,Ntb+1:end).' Q_sp_strings(j,Ntb+1:end).' Q_available_m(j,Ntb+1:end).'];
    current_values = runpf(system, Run_pf_setting);
    P_current_string(j+1,1:Ntb) = current_values.gen(6:18,2);     
    P_current_string(j+1,Ntb+1:end) = current_values.gen(2:5,2);
    P_pcc(j+1) = -1 * current_values.branch(1,14);
    Q_current_string(j+1,1:Ntb) = current_values.gen(6:18,3);
    Q_current_string(j+1,Ntb+1:end) = current_values.gen(2:5,3);
    Q_pcc(j+1) = -1 * current_values.gen(1,3);
    
    %% Create Animated Plot %%
    
    if j==2
        % Loop over subplots and initialise plot lines %
        for p = 1:1:2
            hh1{p}=subplot(2,1,p);
            xlabel('Time [s]', 'FontSize', 24)
            ylabel(ylabels{p}, 'FontSize', 24)
            title(titles{p},'FontSize', 24)
            xlim([t(1) t(end)])

            % Hold on to make 2 plots. Create initial points and set line styles.
            % Store the plots in a cell array for later reference.
            if p == 1
                
            hold on
            plts{p,1} = plot(hh1{p},t(1:2),P_pcc(1:2),'b','LineWidth', 1.9);
            plts{p,2} = plot(hh1{p},t(1:2),P_sp_pcc(1:2),'g', 'LineWidth', 1);
            plts{p,3} = plot(hh1{p},t(1:2),P_a_tot(1:2),'y', 'LineWidth', 0.85);
            plts{p,4} = plot(hh1{p},t(1:2),P_a_wind(1:2),'r');
            plts{p,5} = plot(hh1{p},t(1:2),P_a_tot_pv(1:2),'m');
            legend('PPM Response','TSO Set point', 'Total Available power', 'Wind Power', 'Power from solar irradiance');
            legend.FontSize = 24;
            hold off
            
            else
                
            hold on
            plts{p,1} = plot(hh1{p},t(1:2),Q_pcc(1:2),'b','LineWidth', 1.9);
            plts{p,2} = plot(hh1{p},t(1:2),Q_sp_pcc(1:2),'g', 'LineWidth', 1);
            plts{p,3} = plot(hh1{p},t(1:2),Q_total_available(1:2),'y', 'LineWidth', 0.85);
            plts{p,4} = plot(hh1{p},t(1:2),Q_available_wind(1:2),'r');
            plts{p,5} = plot(hh1{p},t(1:2),Q_available_solar(1:2),'m');
            clear legend
            legend('PPM Response','TSO Set point', 'Total Available power', 'Wind Power', 'Power from solar irradiance');
            legend.FontSize = 24;
            hold off
            
            end
                
        end
    end
    if ( j > 2 && ( (rem(j,200)==0) || j==length(t)-1 ) ) || yes == 1
        % March through time. No replotting required, just update XData and YData
        %for k = 2:1:maxT
            for p = 1:1:2
                if p == 1
                    set(plts{p,1}, 'XData', t(1:j), 'YData', P_pcc(1:j));
                    set(plts{p,2}, 'XData', t(1:j), 'YData', P_sp_pcc(1:j));
                    set(plts{p,3}, 'XData', t(1:j), 'YData', P_a_tot(1:j));
                    set(plts{p,4}, 'XData', t(1:j), 'YData', P_a_wind(1:j));
                    set(plts{p,5}, 'XData', t(1:j), 'YData', P_a_tot_pv(1:j));
                                       
                else
                    set(plts{p,1}, 'XData', t(1:j), 'YData', Q_pcc(1:j));
                    set(plts{p,2}, 'XData', t(1:j), 'YData', Q_sp_pcc(1:j));
                    set(plts{p,3}, 'XData', t(1:j), 'YData', Q_total_available(1:j));
                    set(plts{p,4}, 'XData', t(1:j), 'YData', Q_available_wind(1:j));
                    set(plts{p,5}, 'XData', t(1:j), 'YData', Q_available_solar(1:j));
                    
                    if yes == 1
                        if react_old ~= (system.bus(28,2))
                            react_old = system.bus(28,2);
                        end
                        if react_old == 1
                            if ( tap1_old ~= system.branch(4,9) ) ||  ( tap2_old ~= system.branch(5,9) )
                                tap1_old = system.branch(4,9);
                                tap2_old = system.branch(5,9);
                                str = ' reactor on, tap changed';
                                text(t(j),Q_pcc(j),str)
                            else
                                str = ' reactor on, tap not changed';
                                text(t(j),Q_pcc(j),str)
                            end
                        else

                            if ( tap1_old ~= system.branch(4,9) ) ||  ( tap2_old ~= system.branch(5,9) )
                                tap1_old = system.branch(4,9);
                                tap2_old = system.branch(5,9);
                                str = ' reactor off, tap changed';
                                text(t(j),Q_pcc(j),str)
                            else
                                str = ' reactor off, tap not changed';
                                text(t(j),Q_pcc(j),str)
                            end
                        end
                    end
                    
                end
            end
            drawnow;
        %end  
    end
    
end
    
toc

%% Distribution factor %%
function [ beta ] = calc_DF( P, P_a, a_p)
s = (P_a.^2) ./ (P_a.^2 + a_p);
for k =1:length(s)
    if P(k) >= P_a(k)
        s(k) = 0;
    end
end
if sum(s) == 0
    beta = 0;
else
    beta = (s./sum(s));
end
end