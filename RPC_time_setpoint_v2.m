function [Q_sp_strings, Q_available_wind, Q_available_solar, Q_total_available, error, u, Q_big, Opti_new, Opti_switch, count] = RPC_time_setpoint_v2( j, Ntb, Npv, Opti_new, Opti_switch, threshold_opt, Optimization_setpoint, Q_available_m, Q_sp_strings, Q_sp_pcc, Q_pcc, error, u, Q_big, count, distribution, z, optQs, Q_options, k_p, bs, t, T_i,Q_available_wind, Q_available_solar, Q_total_available, time_done_Q)

%   This function calculates the 
%   reactive power set points for each string

    Q_available_wind(j) = sum(Q_available_m(j,(1:Ntb)));
    Q_available_solar(j) = sum(Q_available_m(j,(Ntb+1:end)));
    Q_total_available(j) = sum(Q_available_m(j,:));
 
%(depends on mode)if the Reactive Power Exchange at the PCC or the Voltage Deviation at the PCC exceeds the predefined 
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
%if j<2
%trap(j) = trapz(t, (error/T_i).*exp(-(error.^2)./(2*(bs^2))) );
    u(j) = k_p*( error(j) + trapz(t, (error/T_i).*exp(-(error.^2)./(2*(bs^2))) ));
%else
%    u(j) = k_p*( error(j) + trapz(t, (error/T_i).*exp(-(error.^2)./(2*(bs^2))) )+ (error(j)-error(j-1))/(t(j)-t(j-1))/k_i);
%end


Q_big(j) = u(j) + Q_sp_pcc(j);

    % Check if set point is reached and stable %
    if error(j) < threshold_opt
        count = count+1;
            if count == 5
                %Opti_switch = 1; % implement optimazation setpoints
                Opti_switch = 0;
            end         
    else
        count = 0;
    end
    
    %--- determine set points ---%
    if Opti_switch == 0
        
        if j > 1
            
            if ( time_done_Q==0 && Q_sp_pcc(j)~=Q_sp_pcc(j-1) ) || (time_done_Q == 1)
        
                % create a matrix with all possible combinations of the signs %
                % of the reactive power produced %
                for i = 1:length(z(:,1))
                    optQs(i,:) = [(z(i,:).*Q_available_m(j,1:Ntb)) Q_available_m(j,Ntb+1:end)];
                end

                % determine best distribution factor and best combination of signs %
                s = 0;
                for k = 1:length(optQs(:,1))
                    for i = 1:length(distribution)
                        s = s + 1;
                        Q_options(s,1) = abs( Q_big(j) - sum(distribution(i)*optQs(k,:)) );
                        Q_options(s,2) = k;
                        Q_options(s,3) = i;
                    end
                end

                [~, idxBest] = min(Q_options(:,1));
                kbest = Q_options(idxBest,2);
                ibest = Q_options(idxBest,3);

                Q_sp_strings(j,:) = distribution(ibest)*optQs(kbest,:); 
                %abs(Q_big(j)-sum(Q_sp_strings(j,:)))


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
            
        elseif time_done_Q == 1 
            
                % create a matrix with all possible combinations of the signs %
                % of the reactive power produced %
                for i = 1:length(z(:,1))
                    optQs(i,:) = [(z(i,:).*Q_available_m(j,1:Ntb)) Q_available_m(j,Ntb+1:end)];
                end

                % determine best distribution factor and best combination of signs %
                s = 0;
                for k = 1:length(optQs(:,1))
                    for i = 1:length(distribution)
                        s = s + 1;
                        Q_options(s,1) = abs( Q_big(j) - sum(distribution(i)*optQs(k,:)) );
                        Q_options(s,2) = k;
                        Q_options(s,3) = i;
                    end
                end

                [~, idxBest] = min(Q_options(:,1));
                kbest = Q_options(idxBest,2);
                ibest = Q_options(idxBest,3);

                Q_sp_strings(j,:) = distribution(ibest)*optQs(kbest,:); 
                %abs(Q_big(j)-sum(Q_sp_strings(j,:)))


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
    end

end
