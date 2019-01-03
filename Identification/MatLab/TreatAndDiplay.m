Vcc = 5;                % 5 Volts
R1 = 10e3;              % 10 kOmh
Analog_Resolution = 10; % 10 bits
C=1e-6;                 % 1 uF

%%% Filter data
output_filt = medfilt1(output,7);   % 5th order median filter

%%% Compute LDR resistance from measured voltage
v_out = output * Vcc / (2^Analog_Resolution - 1);
v_in = input * Vcc / 255;
R = R1*(Vcc./v_out - 1);

%% Compute best value of m
b = 5.5;
mi = -1.5:1e-2:-0.2;
Reg = [];

for j=1:length(mi)

    L = 10.^((log10(R)-b)/mi(j));

    t0 = 0.1;
    a = 0;
    sum = 0;
    values = [];
    step = [];
    for i=1:length(L)
        if time(i)-t0 > 0 && time(i)-t0 < 0.8
            a = a + 1;
            sum = sum + L(i);
        end
        if time(i)-t0 > 0.8
            values = [values sum/a];
            a = 0;
            t0 = t0+1;
            step = [step v_in(i)];
            sum = 0;
        end
    end
    
    Reg = [Reg regression(step,values)];
    
end

[~,m_ind] = max(Reg);
m = mi(m_ind);

%% Obtain static model
L = 10.^((log10(R)-b)/m);

t0 = 0.1;
a = 0;
sum = 0;
values = [];
step = [];
for i=1:length(L)
    if time(i)-t0 > 0 && time(i)-t0 < 0.8
        a = a + 1;
        sum = sum + L(i);
    end
    if time(i)-t0 > 0.8
        values = [values sum/a];
        a = 0;
        t0 = t0+1;
        step = [step input(i)];
        sum = 0;
    end
end

[~,G0,o] = regression(step,values);

%% Obtain tau
t0 = 0.1;
a = 0;
sum = 0;
values_v = [];
for i=1:length(R)
    if time(i)-t0 > 0 && time(i)-t0 < 0.8
        a = a + 1;
        sum = sum + v_out(i);
    end
    if time(i)-t0 > 0.8
        values_v = [values_v sum/a];
        a = 0;
        t0 = t0+1;
        sum = 0;
    end
end

step_index = 1;
i = 2;
tau_down = [];
tau_climb = [];
v_climb = [];
v_down = [];
flag = 0;
threshold = 1-exp(-1);
while(time(i)<20.5)
    while(abs(v_in(i)-v_in(i-1))<0.01)
        i= i+1;
        if time(i)>20.5
            flag = 1;
            break;
        end
    end
    if flag == 1
        break;
    end
    
    final = i;
    
    while((v_out(i)-values_v(step_index))/(values_v(step_index+1)-values_v(step_index)) < threshold)
        i=i+1;
    end
    
    prev = abs((v_out(i-1)-values_v(step_index))/(values_v(step_index+1)-values_v(step_index)));
    now = abs((v_out(i)-values_v(step_index))/(values_v(step_index+1)-values_v(step_index)));
    Reta = polyfit([prev,now],[time(i-1) time(i)],1);
    
    if(values_v(step_index+1)-values_v(step_index) > 0)
        tau_climb = [tau_climb,(polyval(Reta,threshold) - time(final))];
        v_climb = [v_climb values_v(step_index+1)];
    else
        tau_down = [tau_down,(polyval(Reta,threshold) - time(final))];
        v_down = [v_down values_v(step_index+1)];
    end
    if polyval(Reta,threshold) - time(final) > 1
        break;
    end
    
    i=i+1;
    step_index=step_index+1;
end

%% Adjust tau
[Q_climb,m_climb,b_climb] = regression(v_climb,tau_climb);
[Q_down,m_down,b_down] = regression(v_down,tau_down);
[Q,m_total,b_total] = regression([v_down v_climb],[tau_down tau_climb]);

%% Teoretical tau computation
v_m = 0:0.01:5;

R_2 = R1*(Vcc./v_m - 1);
R_eq=R1*R_2./(R1+R_2);

tau_teo=R_eq*C;
tau_teo(1) = R1*C;

%% Display data
figure
plot(time,L,'LineWidth',1);
grid minor

figure
hold on
plot(step,values,'x','Linewidth',2);
plot([0,255],G0*[0,255]+o)
grid on
grid minor

figure
hold on
plot(v_climb,tau_climb,'x','Color','blue','Linewidth',2);
plot(v_down,tau_down,'x','Color','red','Linewidth',2);
plot([0,4],m_climb*[0,4]+b_climb,'-.','Color','blue');
plot([0,4],m_down*[0,4]+b_down,'-.','Color','red');
plot([0,4],m_total*[0,4]+b_total,'-.','Color','green');
plot(v_m,tau_teo,'Color','black');
grid minor
ylabel('Tau (s)');
xlabel('V_{desired} (V)');

fprintf(1,"\nLDR Characteristic:\tm = %f\tb = %f\n",m,b)
fprintf(1,"Static model:\t\tG0 = %f\to = %f\n",G0,o)
fprintf(1,"Time constant:\t\tTau = %f v + %f\n",m_total,b_total)
