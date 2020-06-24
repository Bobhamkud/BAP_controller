clear
close all
clear title xlabel ylabel
tic
%% Initializations %%

dt = 200e-3;                                    % 10 ms sample time
Tfinal = 3600;                                  % simulation will last 18000 s
Ntb = 13;                                       % number of wind turbine strings
Npv = 4;                                        % number of PV module strings
t = 0:dt:Tfinal;                                % time vector
rise_time = 100;                                  % rise time
rise_end = 0;                                   % parameter used to determine end of rising to set point            
time_done = 1;                                  % parameter used for implementing rise time
N_rise = floor(rise_time/dt);                   % determine time_steps for rise_time
rise = zeros(N_rise+1,Ntb+Npv);                         % initialise vector containing set points to implement rise time

P_sp_pcc = zeros(length(t), 1);                 % initialize input vector with P set points
P_a_string = zeros(length(t), Ntb+Npv);         % initialize matrix with available P per string over time
P_sp_string = zeros(length(t),Ntb+Npv);         % initialize active power set point per string over time
P_current_string  = zeros(length(t), Ntb+Npv);  % initialize current active power produced per string over time
a_p = 70;                                       % initialize a_p constant for the distribution factors 
P_pcc = zeros(1,length(t));                     % initialize vector with current PCC value
case_log = zeros(1,length(t));                  % initialize case log
P_a_tot = zeros(1,length(t));                   % total available power
P_a_wind = zeros(1,length(t));                  % total available wind power
P_a_tot_pv = zeros(1,length(t));                % total availabel power from solar irradiance
Run_pf_setting = mpoption('verbose',0,'out.all',0); % hide MATPOWER output
load('agreed_profiles.mat')                         % load wind and solar profiles

%% Set point times and values %%

sp_1 = 70;                               % first set point 70 MW
t_sp1 = 0;                               % first set point at 0 seconds
sp_2 = 70;                               % second set point 70 MW
t_sp2 = 900;                             % second set point at 4500 seconds
sp_3 = 135;                              % third set point 110 MW
t_sp3 = 1800;                            % third set point at 9000 seconds
sp_4 = 100;                              % fourth set point 55 MW
t_sp4 = 2700;                            % fourth set point at 13500 seconds

setpoint_values = [sp_1 sp_2 sp_3 sp_4];    % vector containing set points
setpoint_times = [t_sp1 t_sp2 t_sp3 t_sp4]; % vector containing set point times
%% setting up input vector P_sp_PCC containing the setpoints %%

idx_begin = 1;
for i = 2:length(setpoint_values)
    [val,idx_end]=min(abs(t-setpoint_times(i)));
    P_sp_pcc(idx_begin:idx_end-1)= setpoint_values(i-1);
    idx_begin = idx_end;
end
P_sp_pcc(idx_begin:end) = setpoint_values(end);

%% Initialize wind profile %%

v_profile = zeros(1,length(t));
for i =1:length(v_profile)
v_profile(i) = normrnd(7.7,0.1);
end
v_profile(1:end) = windspeed(1:length(t));
[P_a_string(:,1:Ntb),Qwtg_string_profile] = compute_pq_wtg(v_profile);

%% Initialize irradiance profile  %%

solar_profile = zeros(1,length(t));
for i =1:length(solar_profile)
    solar_profile(i) = normrnd(680,10);
end
solar_profile(1:end) = irradiance(1:length(t));
[P_a_pv,Qpvg_string_profile] = compute_pq_pvg(solar_profile,Npv);
P_a_string(:,Ntb+1:end) = P_a_pv.';

%% -----------Run simulation--------- %%

% run initial powerflow % 
system = loadcase('system_17');
system.gen(6:end,[2,9,10]) = [(P_a_string(1,1:Ntb)-2).' (P_a_string(1,1:Ntb)-2).' (P_a_string(1,1:Ntb)-2).'];
system.gen(2:5,[2,9,10]) = [(P_a_string(1,Ntb+1:end)-2).' (P_a_string(1,Ntb+1:end)-2).' (P_a_string(1,Ntb+1:end)-2).'];
current_values = runpf(system, Run_pf_setting);
P_current_string(1,1:Ntb) = current_values.gen(6:18,2);
P_current_string(1,Ntb+1:end) = current_values.gen(2:5,2);
P_pcc(1) = -1 * current_values.branch(1,14);

% beta opslaan!!!

% Initialize subplots %
titles = {'Active power response', 'Individual set points for strings'};
for pp = 1:1:1
    hh1(pp)=subplot(1,1,pp);
    xlabel('Time [s]', 'FontSize', 24)
    ylabel('Power [MW]', 'FontSize', 24)
    title(titles{pp},'FontSize', 24)
    xlim([t(1) t(end)])
    hold on
end
plts = cell(2,17);
hh1 = cell(2,1);

% initialise case %
case_2 =0;

% Iterate over time %

for j = 1:length(t)-1

    [ P_a_tot, P_a_tot_pv, P_a_wind, P_sp_string, case_2 ] = RPC_time_step_setpoint(P_a_string, P_a_pv, P_sp_pcc,P_pcc,P_current_string,j, time_done, P_sp_string, a_p, Ntb, Npv,P_a_tot, P_a_tot_pv, P_a_wind, case_2);
    [P_sp_string, time_done, rise_end] = RPC_rise_time( P_sp_string,P_sp_pcc,j, Ntb, Npv, N_rise, rise, rise_end, time_done, P_a_string);
    
    % apply set points %
    system.gen(6:end,[2,9,10]) = [P_sp_string(j,1:Ntb).' P_sp_string(j,1:Ntb).' P_sp_string(j,1:Ntb).'];
    system.gen(2:5,[2,9,10]) = [P_sp_string(j,Ntb+1:end).' P_sp_string(j,Ntb+1:end).' P_sp_string(j,Ntb+1:end).'];
    current_values = runpf(system, Run_pf_setting);
    P_current_string(j+1,1:Ntb) = current_values.gen(6:18,2);     
    P_current_string(j+1,Ntb+1:end) = current_values.gen(2:5,2);
    P_pcc(j+1) = -1 * current_values.branch(1,14);
    
    %% Create Animated Plot %%
    if j==2
        % Loop over subplots and initialise plot lines %
        for p = 1:1:1
            hh1{p}=subplot(2,1,p);
            xlabel('Time [s]', 'FontSize', 24)
            ylabel('Power [MW]', 'FontSize', 24)
            title(titles{p},'FontSize', 24)
            xlim([t(1) t(end)])

            % Hold on to make 2 plots. Create initial points and set line styles.
            % Store the plots in a cell array for later reference.
            hold on
            plts{p,1} = plot(hh1{p},t(1:2),P_pcc(1:2),'b','LineWidth', 1.9);
            plts{p,2} = plot(hh1{p},t(1:2),P_sp_pcc(1:2),'g', 'LineWidth', 1);
            plts{p,3} = plot(hh1{p},t(1:2),P_a_tot(1:2),'y', 'LineWidth', 0.85);
            plts{p,4} = plot(hh1{p},t(1:2),P_a_wind(1:2),'r');
            plts{p,5} = plot(hh1{p},t(1:2),P_a_tot_pv(1:2),'m');
            legend('PPM Response','TSO Set point', 'Total Available power', 'Wind Power', 'Power from solar irradiance');
            legend.FontSize = 24;
            hold off
        end
    end
    if j > 2 && ( (rem(j,60)==0) || j==length(t)-1 )
        % March through time. No replotting required, just update XData and YData
        %for k = 2:1:maxT
            for p = 1:1:1
                set(plts{p,1}, 'XData', t(1:j), 'YData', P_pcc(1:j));
                set(plts{p,2}, 'XData', t(1:j), 'YData', P_sp_pcc(1:j));
                set(plts{p,3}, 'XData', t(1:j), 'YData', P_a_tot(1:j));
                set(plts{p,4}, 'XData', t(1:j), 'YData', P_a_wind(1:j));
                set(plts{p,5}, 'XData', t(1:j), 'YData', P_a_tot_pv(1:j));
            end
            drawnow;
        %end  
    end
    
end
    
% % % Plot set point response and available powers % 
% plot(t, P_pcc, 'b','LineWidth', 1.9)
% hold on
% plot(t,P_sp_pcc, 'g', 'LineWidth', 1)
% plot(t,P_a_tot, 'y', 'LineWidth', 0.85)
% plot(t, P_a_wind,'r', t, P_a_tot_pv, 'm')
% xlim([t(1) t(end)])
% title('Active power response', 'FontSize', 24)
% xlabel('Time [s]', 'FontSize', 24)
% ylabel('Power [MW]', 'FontSize', 24)
% legend('PPM Response','TSO Set point', 'Total Available power', 'Wind Power', 'Power from solar irradiance')
% legend.FontSize = 24;
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