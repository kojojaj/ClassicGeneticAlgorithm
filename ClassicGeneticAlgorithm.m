%% THE CLASSIC GENETIC ALGORITHM
% Classig Genetic Algorithm implemented in order to tune PID Regulator and
% then compared with PID Tuner results.

%% TRANSFER FUNCTION
%numerator
n1 = 3; n2 = 7;
n = [n1 n2];
%denominator
d1 = 0.5; d2 = -2; d3 = 11;
d = [d1 d2 d3];
%open loop transfer function
plant_tf = tf(n, d);
%% STABILITY CHECK
% subplot(121)
% rlocus(plant_tf)
% subplot(122)
% step(plant_tf)
%% 
%create symbolic variable 's'
s = tf('s');
%load tuned PID regulator called 'C1'
load PID;
%% PID TUNER
%Limit values for control terms estimated with 'PIDTuner' tool:
% Kp = <0, 16>; Ki = <0, 64>; Kd = <0, 0.0256>
Kp = C1.Kp; Ki = C1.Ki; Kd = C1.Kd; Tf = 0.1;
%Regulators' transfer function
PID_fcn = pid(C1.Kp, C1.Ki, C1.Kd, Tf);
PID_tf = tf(PID_fcn);
%closed loop transfer function
system_tf = (plant_tf * PID_tf) / (1 + plant_tf * PID_tf);
%PIDs' transfer function
u_tf = PID_tf / (1 + PID_tf * plant_tf);
%simulation and step time
t = 0:1e-2:4;
%step response of the whole system
[y0, t] = step(system_tf, t);
step_info = stepinfo(system_tf);
%PIDs' step response
[u0, t] = step(u_tf, t);
%perfect (unit)step function
unitstep = ones(1, length(t));
%% FIRST GENERATION INITIALIZATION
% ch - chromosome, 
% F - fitness function value
% pk - crossover probability
% lk - crossover point
% pm - mutation probability
% vk - roulette wheel sector
individual = struct('ch', 0, 'F', 0, 'ps', 0, 'vk', [0 1], 'pk', 1, 'pm', 0.12, 'e', 0); 
parent = individual;
couple = individual;
ch = zeros(30,1);
%population size
population = 100;
%create random population
for i = 1:population
    for j=1:length(ch)
        ch(j) = rand();
        if ch(j) >= 0.5
            ch(j) = 1;
        else 
            ch(j) = 0;
        end
    end
    ch1 = num2str(ch);
    individual(i).ch = ch1; individual(i).pk = 1; individual(i).pm = 0.12;
end
%parateres for decoding genotype to phenotype
kp_param = 16/1024;
ki_param = 64/1024;
kd_param = 0.0256/1024;
%% MAIN LOOP
e1 = zeros(1, length(t));
F_best = 0;   %fitness function value
F_prev = 0;   %previous fitness function value
it = 0;       %iteration count
it_max = 100; %maximum iterations count without changing "F_best" value
it_stag = 0;  %current iterations count without changing "F_best" value
Fmin = 329;   %minimum fitness function value that we want to reach
while  F_best < Fmin
    it = it+1; %iteration counter
    %% INFORMATION DECODING
    F_sum = 0;
    for i=1:population
        e = 0;
        ch = individual(i).ch;
        ch = [ch(1) ch(2) ch(3) ch(4) ch(5) ch(6) ch(7) ch(8) ch(9) ch(10)...
            ch(11) ch(12) ch(13) ch(14) ch(15) ch(16) ch(17) ch(18) ch(19) ch(20)...
            ch(21) ch(22) ch(23) ch(24) ch(25) ch(26) ch(27) ch(28) ch(29) ch(30)];
        %particural genes extraction from chromosome
        kp_string = extractBetween(ch,1,10);
        kp = bin2dec(kp_string) * kp_param;

        ki_string = extractBetween(ch,11,20);
        ki = bin2dec(ki_string) * ki_param;

        kd_string = extractBetween(ch,21,30);
        kd = bin2dec(kp_string) * kd_param;
        %regulators' transfer function
        PID_fcn = pid(kp,ki,kd,Tf);
        PID_tf = tf(PID_fcn);
        %closed loop system transfer function
        system_tf = (plant_tf*PID_tf)/(1+plant_tf*PID_tf);
        %whole system step response
        [y, t] = step(system_tf, t);
        %PIDs' transfer function
        u_tf = PID_tf/(1+ PID_tf*plant_tf);
        %PIDs' step response
        [u, t] = step(u_tf, t);
        %% FITNESS FUNCTION
        for j = 1 : length(t)
            e1(j) = 1000 * (unitstep(j) - y(j))^2;
            e = e + e1(j);
        end
        individual(i).e = e / length(t);
        %dzielenie "przez 1" w celu szukania maksimum a nie minimum
        %(ułatwienie przypisywania wycinków koła ruletki)
        %division "by 1" in order to find maximum rather than minimum
        %(easing wheel section assign)
        individual(i).F = 1000 / individual(i).e;
        F_sum = F_sum + individual(i).F;
        if individual(i).F > F_best
            best_one = individual(i);
            F_best = best_one.F;
            kp_best = kp;
            ki_best = ki;
            kd_best = kd;
        end
    end
    if (F_prev == F_best)
        it_stag = it_stag+1;
        if it_stag == it_max
            Info = ["Maximum iterations count without improvement reached!"]
            visualize_results(t, y0, y, u0, u, e1, Kp, Ki, Kd, kp_best, ki_best, kd_best);
            F_best
            return
        end
    else 
        it_stag = 0;
    end
    F_prev = F_best;
    if F_best < Fmin
        %% ROULETTE
        %assign wheel section:
        circle = 0;
        for i = 1:population
            individual(i).ps = individual(i).F / F_sum;
            individual(i).vk(1) = circle;
            circle = circle + individual(i).ps;
            individual(i).vk(2) = circle;
        end
    
        for i = 1:population
            %'spin the wheel'
            spin_the_wheel = rand();
            %check in which section was hit
            for j = 1:population
                if (spin_the_wheel > individual(j).vk(1)) && (spin_the_wheel <= individual(j).vk(2))
                    parent(i) = individual(j); %if hit -> pass to the next generation
                end
            end
        end
        %% ONE-POINT CROSSOVER (osobnik.pk = 1)
        for i = 1:(population/2)
            lk = ceil(30 * rand()); %crossover point drawing
            %pick parents (in order)
            couple(1) = parent(2*i-1); 
            couple(2) = parent(2*i);
            %swap alleles
            l = length(couple(1).ch);
            allel_to_flip = couple(1).ch(lk:l);
            couple(1).ch(lk:l) = couple(2).ch(lk:l);
            couple(2).ch(lk:l) = allel_to_flip;
            individual(2*i-1) = couple(1);
            individual(2*i)   = couple(2);
        end
        %% MUTATION
        for i =1:population
        %%
        % 
        % <<FILENAME.PNG>>
        % 
            for j=1:30
                if_mut = rand(); %draw if to mutate
                if if_mut <= individual(i).pm
                    if individual(i).ch(j) == 1
                        individual(i).ch(j) = '0';
                    else 
                        individual(i).ch(j) = '1';
                    end
                end
            end
        end
    end
end
%% SHOW RESULTS (COMPARISON TO PIDTuner)
visualize_results(t, y0, y, u0, u, e1, Kp, Ki, Kd, kp_best, ki_best, kd_best);
Info = ["Assigned fitness function value reached:"]
F_best