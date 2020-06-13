clear all
close all

stepsize = 1;
Imin = 0;
Imax = 100;
irradiance = Imin:stepsize:Imax;
Nstrings = 4;
[P,Q] = compute_pq_pvg(irradiance, Nstrings);
endsamp_string = 25/stepsize + 1;
% figure(1)
% plot(P(:,1),Q(:,1))
L = length(P(1,:));
for i = 1:L
figure(1)
subplot(4,4,i)
plot(P(1:endsamp_string,i),Q(1:endsamp_string,i))    
xlabel('P [MW]')
ylabel('Q [MVAr]')

figure(2)
subplot(4,4,i)
plot(v,P(:,i))
xlabel('windspeed [m/s]')
ylabel('P [MW]')


figure(3)
subplot(4,4,i)
plot(v,Q(:,i))
xlabel('windspeed [m/s]')
ylabel('Q [MVAr]')
end