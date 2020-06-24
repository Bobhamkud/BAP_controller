function [ P_a_tot, P_a_tot_pv, P_a_wind, P_sp_string, case_2 ] = RPC_time_step_setpoint(P_a_string, P_a_pv, P_sp_pcc,P_pcc,P_current_string,j, time_done, P_sp_string, a_p, Ntb, Npv,P_a_tot, P_a_tot_pv, P_a_wind, case_2)

%   This function calculates the 
%   active power set points for each string

    % calculate total available power %
    P_a_tot(j) = sum(P_a_string(j,:));
    P_a_tot_pv(j) = sum(P_a_pv(:,j));
    P_a_wind(j) = sum(P_a_string(j,1:13));
    
    % error at PCC %
    Delta_pcc = P_sp_pcc(j)-P_pcc(j);

    % check for which case applies %
    if j>1
        if P_sp_pcc(j) < P_a_tot(j) && P_sp_pcc(j)<P_sp_pcc(j-1)
            if P_sp_pcc(j) < P_a_tot(j) && P_sp_pcc(j)<P_sp_pcc(j-1) && case_2 == 1
                case_1 = 0;
                case_2 = 0;
                case_3 = 1;
                case_log(j) = 3;
                case_3b = 1;
            else
                case_1 = 0;
                case_2 = 0;
                case_3 = 1;
                case_log(j) = 3;
                case_3b = 0;
            end
        elseif P_sp_pcc(j) < P_a_tot(j)
            case_1 = 1;
            case_2 = 0;
            case_3 = 0;
            case_3b  =0;
            case_log(j) = 1;
        else
            case_1 = 0;
            case_2 = 1;
            case_3 = 0;
            case_3b = 0;
            case_log(j) = 2;
        end
    else
        if P_sp_pcc(j) < P_a_tot(j)
                case_1 = 1;
        else
                case_1 = 0;
                case_2 = 1;
        end
    end

%% If rise time over or if a setpoint change occurs, calculate new set points %%

if j > 1
    
    if ( time_done==0 && P_sp_pcc(j)~=P_sp_pcc(j-1) ) || (time_done == 1)
            
        % determine set points %
        if case_1 == 1

            beta = calc_DF(P_current_string(j,:),P_a_string(j,:),a_p);
            %beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
            P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;

        elseif case_2 == 1

            beta = (P_a_string(j,:) - P_current_string(j,:))/Delta_pcc;
            %beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));       
            P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;

        else

            % find out which strings are producing at their max
            Ps_op_max = zeros(1,Ntb+Npv);  % initialize vector with info on which strings are producing at their max
            Ns_max = 0;                    % initialize number of strings producing at their max
            for i = 1:Ntb+Npv
                if P_current_string(j,i) >= P_a_string(j,i)
                    Ns_max = Ns_max + 1;
                    Ps_op_max(i) = 1;
                end
            end

            if case_3b == 1
                Psub = ( sum(P_current_string(j,:))- P_sp_pcc(j) ) / Ns_max;
            else
                Psub = ( P_sp_pcc(j-1)-P_sp_pcc(j) ) / Ns_max;  % amount of active power that will be deducted
                                                                % from turbines producing at their max
            end

            % reduce the output of the wind turbines producing at maximum % 
            for i = 1:Ntb+Npv
                if Ps_op_max(i)== 1
                    P_sp_string(j,i) = P_current_string(j,i) - Psub;
                else
                    P_sp_string(j,i) = P_current_string(j,i);
                end
            end
            if Ns_max == 0
                beta = calc_DF(P_current_string(j,:),P_a_string(j,:),a_p);
                %beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
                P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;
            end

        end

        % in case a set point becomes negative, the set point is set to zero %
        % and the amount the needs to be deducted is equally divided among other turbines %

         n=0;
         redivide = 0;
         for l = 1:Ntb+Npv
             if P_sp_string(j,l) < 0
                n = n+1;
                redivide = redivide + abs(P_sp_string(j,l));
                P_sp_string(j,l) = 0;
             end   
         end
         Psubsub = redivide/(Ntb+Npv-n);
         d=0;
         k=0;
        for l = 1:Ntb+Npv
             if P_sp_string(j,l) > Psubsub
                 k = k + 1;
                 P_sp_string(j,l) = P_sp_string(j,l)-Psubsub;
             else
                 d = d+1;
                 Psubsub = Psubsub + ((Psubsub-P_sp_string(j,l)) / (Ntb+Npv-n-k-d));
                 P_sp_string(j,l) = 0;
             end
        end
    end
    
elseif time_done == 1 
    
    % determine set points %
    
    if case_1 == 1
        
        beta = calc_DF(P_current_string(j,:),P_a_string(j,:),a_p);
        %beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
        P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;
        
    elseif case_2 == 1
        
        beta = (P_a_string(j,:) - P_current_string(j,:))/Delta_pcc;
        %beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));       
        P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;
        
    else
        
        % find out which strings are producing at their max
        Ps_op_max = zeros(1,Ntb+Npv);  % initialize vector with info on which strings are producing at their max
        Ns_max = 0;                    % initialize number of strings producing at their max
        for i = 1:Ntb+Npv
            if P_current_string(j,i) >= P_a_string(j,i)
                Ns_max = Ns_max + 1;
                Ps_op_max(i) = 1;
            end
        end
        
        if case_3b == 1
            Psub = ( sum(P_current_string(j,:))- P_sp_pcc(j) ) / Ns_max;
        else
            Psub = ( P_sp_pcc(j-1)-P_sp_pcc(j) ) / Ns_max;  % amount of active power that will be deducted
                                                            % from turbines producing at their max
        end
        
        % reduce the output of the wind turbines producing at maximum % 
        for i = 1:Ntb+Npv
            if Ps_op_max(i)== 1
                P_sp_string(j,i) = P_current_string(j,i) - Psub;
            else
                P_sp_string(j,i) = P_current_string(j,i);
            end
        end
        if Ns_max == 0
            beta = calc_DF(P_current_string(j,:),P_a_string(j,:),a_p);
            %beta = ones(1,Ntb+Npv)* (1/(Ntb+Npv));
            P_sp_string(j,:) = P_current_string(j,:) + beta*Delta_pcc;
        end
        
    end
    
    % in case a set point becomes negative, the set point is set to zero %
    % and the amount the needs to be deducted is equally divided among other turbines %
       
     n=0;
     redivide = 0;
     for l = 1:Ntb+Npv
         if P_sp_string(j,l) < 0
            n = n+1;
            redivide = redivide + abs(P_sp_string(j,l));
            P_sp_string(j,l) = 0;
         end   
     end
     Psubsub = redivide/(Ntb+Npv-n);
     d=0;
     k=0;
     for l = 1:Ntb+Npv
         if P_sp_string(j,l) > Psubsub
             k = k + 1;
             P_sp_string(j,l) = P_sp_string(j,l)-Psubsub;
         else
             d = d+1;
             Psubsub = Psubsub + ((Psubsub-P_sp_string(j,l)) / (Ntb+Npv-n-k-d));
             P_sp_string(j,l) = 0;
         end
     end   
end

end

