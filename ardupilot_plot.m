data = 'log1.mat';
close all
clc
load(data);
% (1) Attitude angle tracking performance
% (2) Attitude angular rate tracking performance
% (3) Position tracking performance
% (4) Velocity tracking performance
% (5) Servo output pwm 
% (6) Control input
% (7) Pilot input pwm

flag_plot = [0 0 0 0 1 1 0];

% (1) Attitude angle tracking performance
%---------------------------------------------%
t = ATT(:,2)/1e6;
time_ATT= t - t(1);

roll_ref = ATT(:,3);
roll = ATT(:,4);
pitch_ref = ATT(:,5);
pitch = ATT(:,6);
yaw_ref = ATT(:,7);
yaw = ATT(:,8);
yaw_ref = f_yaw_filter(yaw_ref);
yaw = f_yaw_filter(yaw);

disp('----------------------------------------------------------')
disp('(1) Attitude angle tracking performance (deg):')
att_rms = [rms(roll_ref - roll) rms(pitch_ref - pitch) rms(yaw_ref - yaw)]
att_ref_rms = [rms(roll_ref) rms(pitch_ref) rms(yaw_ref)];
att_rms_p = att_rms./(att_rms + att_ref_rms)*100;
% yaw_e_p = rms(yaw_ref - yaw)/(rms(yaw_ref) + rms(yaw))

if flag_plot(1) == 1
    figure(1)
    subplot(3,1,1)
    plot(time_ATT, roll_ref,'r--', time_ATT, roll, 'b-');
    ylabel('\phi/(deg)')
    legend('\phi_{ref}','\phi')
    title('(1) Attitude angle tracking performance')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,2)
    plot(time_ATT, pitch_ref,'r--', time_ATT, pitch, 'b-');
    ylabel('\theta/(deg)')
    legend('\theta_{ref}','\theta')
    ylabel('\theta/(deg)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,3)
    plot(time_ATT, yaw_ref,'r--', time_ATT, yaw, 'b-');
    legend('\psi_{ref}','\psi')
    ylabel('\psi/(deg)')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    Ts_ATT = mean(diff(time_ATT));
end


% (2) Attitude angular rate tracking performance
%---------------------------------------------%
t = RATE(:,2)/1e6;
time_RATE= t - t(1);
p_ref = RATE(:,3);
p = RATE(:,4);
q_ref = RATE(:,6);
q = RATE(:,7);
r_ref = RATE(:,9);
r = RATE(:,10);

disp('----------------------------------------------------------')
disp('(2) Attitude angular rate tracking performance (deg/s):')
rate_rms = [rms(p_ref - p) rms(q_ref - q) rms(r_ref - r)]
rate_ref_rms = [rms(p_ref) rms(q_ref) rms(r_ref)]
rate_rms_p = rate_rms./(rate_rms + rate_ref_rms)*100
% A_ref = RATE(:,12);
% A = RATE(:,13); 

if flag_plot(2) == 1
    figure(2)
    subplot(3,1,1)
    plot(time_RATE, p_ref,'r--', time_RATE, p, 'b-');
    ylabel('\itp/\rm(deg/s)')
    legend('\itp_{\rmref}','\itp')
    title('(2) Attitude angular rate tracking performance')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,2)
    plot(time_RATE, q_ref,'r--', time_RATE, q, 'b-');
    ylabel('\itq/\rm(deg/s)')
    legend('\itq_{\rmref}','\itq')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,3)
    plot(time_RATE, r_ref,'r--', time_RATE, r, 'b-');
    ylabel('\itr/\rm(deg/s)')
    legend('\itr_{\rmref}','\itr')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    Ts_RATE = mean(diff(time_RATE));
end

% (3) Position tracking performance
%---------------------------------------------%
t = PSC(:,2)/1e6;
time_PSC= t - t(1);
t = CTUN(:,2)/1e6;
time_CTUN= t - t(1);
X_ref = PSC(:,3);
X = PSC(:,5);
Y_ref = PSC(:,4);
Y = PSC(:,6);
vX_ref = PSC(:,7);
vX = PSC(:,9);
vY_ref = PSC(:,8);
vY = PSC(:,10);
vZ_ref = CTUN(:,13);
vZ = CTUN(:,14);
Z_ref = CTUN(:,7)*100;%m to cm
Z = CTUN(:,8)*100;%m to cm

disp('----------------------------------------------------------')
disp('(3) Position tracking performance: (cm)')
pos_rms = [rms(X_ref - X) rms(Y_ref - Y) rms(Z_ref - Z)]
pos_ref_rms = [rms(X_ref) rms(Y_ref) rms(Z_ref)];
pos_rms_p = pos_rms./(pos_rms + pos_ref_rms)*100;

if flag_plot(3) == 1
    figure(3)
    subplot(3,1,1)
    plot(time_PSC, X_ref,'r--', time_PSC, X, 'b-');
    ylabel('\itX/(\rmcm)')
    legend('\itX_{\rmref}','\itX')
    title('(3) Position tracking performance')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,2)
    plot(time_PSC, Y_ref,'r--', time_PSC, Y, 'b-');
    ylabel('\itY/(\rmcm)')
    legend('\itY_{\rmref}','\itY')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,3)
    plot(time_CTUN, Z_ref,'r--', time_CTUN, Z, 'b-');
    ylabel('\itZ/(\rmcm)')
    legend('\itZ_{\rmref}','\itZ')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    Ts_PSC = mean(diff(time_PSC));
end

% (4) Velocity tracking performance
%---------------------------------------------%
disp('----------------------------------------------------------')
disp('(4) Velocity tracking performance (cm/s):')
vel_rms = [rms(vX_ref - vX) rms(vY_ref - vY) rms(vZ_ref - vZ)]
vel_ref_rms = [rms(vX_ref) rms(vY_ref) rms(vZ_ref)]
vel_rms_p = vel_rms./(vel_rms + vel_ref_rms)*100

if flag_plot(4) == 1
    figure(4)
    subplot(3,1,1)
    plot(time_PSC, vX_ref,'r--', time_PSC, vX, 'b-');
    ylabel('\itvX/(\rmcm)')
    legend('\itvX_{\rmref}','\itvX')
    title('(4) Velocity tracking performance')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,2)
    plot(time_PSC, vY_ref,'r--', time_PSC, vY, 'b-');
    ylabel('\itvY/(\rmcm)')
    legend('\itvY_{\rmref}','\itvY')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(3,1,3)
    plot(time_CTUN, vZ_ref,'r--', time_CTUN, vZ, 'b-');
    ylabel('\itvZ/(\rmcm)')
    legend('\itvZ_{\rmref}','\itvZ')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    Ts_PSC = mean(diff(time_PSC));
end


% (5) Servo output pwm 
%---------------------------------------------%
t = RCOU(:,2)/1e6;
time_RCOU= t - t(1);
pwm1 = RCOU(:,3);
pwm2 = RCOU(:,4);
pwm3 = RCOU(:,5);
pwm4 = RCOU(:,6);
pwm5 = RCOU(:,7);
pwm6 = RCOU(:,8);


disp('----------------------------------------------------------')
disp('(5) Servo output pwm:')

if flag_plot(5) == 1
    figure(5)
    subplot(6,1,1)
    plot(time_RCOU, pwm1, 'b-');
    ylabel('\itpwm1')
    title('(5) Servo output pwm')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(6,1,2)
    plot(time_RCOU, pwm2, 'b-');
    ylabel('\itpwm2')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(6,1,3)
    plot(time_RCOU, pwm3, 'b-');
    ylabel('\itpwm3')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
   
    subplot(6,1,4)
    plot(time_RCOU, pwm4, 'b-');
    ylabel('\itpwm4')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    
    subplot(6,1,5)
    plot(time_RCOU, pwm5, 'b-');
    ylabel('\itpwm5')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    
    subplot(6,1,6)
    plot(time_RCOU, pwm6, 'b-');
    ylabel('\itpwm6')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
    Ts_RCOU = mean(diff(time_RCOU));
end

% (6) Control input
%---------------------------------------------%


disp('----------------------------------------------------------')
disp('(6) Control input:')
trim_servo1 = -70;
trim_servo2 =  40;

T1 = pwm2thrust(pwm1);
T2 = pwm2thrust(pwm2);
T3 = pwm2thrust(pwm3);
T4 = pwm2thrust(pwm4);
T5 = pwm2thrust(pwm5);
T6 = pwm2thrust(pwm6);

u_col = 1/4*(T1 + T2 + T3 + T4);
u_lat = (T5 - T6 - 1/100*0.25*(trim_servo1 - trim_servo2))/0.44;
u_lon = T1 - T4;
u_ped = T1 - T2;

if flag_plot(6) == 1
    figure(6)
    subplot(4,1,1)
    plot(time_RCOU, u_col, 'b-');
    ylabel('\itu_{\rmcol}')
    title('(6) Control input')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(4,1,2)
    plot(time_RCOU, u_lat, 'b-');
    ylabel('\itu_{\rmlat}')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(4,1,3)
    plot(time_RCOU, u_lon, 'b-');
    ylabel('\itu_{\rmlon}')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
   
    subplot(4,1,4)
    plot(time_RCOU, u_ped, 'b-');
    ylabel('\itu_{\rmped}')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
end


% (7) Pilot input pwm
t_RCIN = RCIN(:,2)/1e6;
time_RCIN= t_RCIN - t_RCIN(1);
p_col = RCIN(:,5); 
p_lat = RCIN(:,3);
p_lon = RCIN(:,4);
p_ped = RCIN(:,6);


%---------------------------------------------%
disp('----------------------------------------------------------')
disp('(7) Pilot input pwm:')

if flag_plot(7) == 1
    figure(7)
    subplot(4,1,1)
    plot(time_RCIN, p_col, 'b-');
    ylabel('\itp_{\rmcol}')
    title('(7) Pilot input pwm')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(4,1,2)
    plot(time_RCIN, p_lat, 'b-');
    ylabel('\itp_{\rmlat}')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');

    subplot(4,1,3)
    plot(time_RCIN, p_lon, 'b-');
    ylabel('\itp_{\rmlon}')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman');
   
    subplot(4,1,4)
    plot(time_RCIN, p_ped, 'b-');
    ylabel('\itp_{\rmped}')
    xlabel('Time/(s)')
    set(gca,'FontSize',15,'Fontname', 'Times New Roman'); 
    Ts_RCIN = mean(diff(time_RCIN));
end




% 

% 
% Ts_id = 0.02;
% rs = [10 110];
% range_p = rs;
% 
% [~, p_id] = f_Data_sel(time_RATE, p, rs(1), rs(2), Ts_id);
% [~, q_id] = f_Data_sel(time_RATE, q, rs(1), rs(2), Ts_id);
% [~, r_id] = f_Data_sel(time_RATE, r, rs(1), rs(2), Ts_id);
% [~, p_ref_id] = f_Data_sel(time_RATE, p_ref, rs(1), rs(2), Ts_id);
% [~, q_ref_id] = f_Data_sel(time_RATE, q_ref, rs(1), rs(2), Ts_id);
% [~, r_ref_id] = f_Data_sel(time_RATE, r_ref, rs(1), rs(2), Ts_id);
% 
% 
% [~, roll_ref_id] = f_Data_sel(time_ATT, roll_ref, rs(1), rs(2), Ts_id);
% [~, pitch_ref_id] = f_Data_sel(time_ATT, pitch_ref, rs(1), rs(2), Ts_id);
% [~, yaw_ref_id] = f_Data_sel(time_ATT, yaw_ref, rs(1), rs(2), Ts_id);
% [~, roll_id] = f_Data_sel(time_ATT, roll, rs(1), rs(2), Ts_id);
% [~, pitch_id] = f_Data_sel(time_ATT, pitch, rs(1), rs(2), Ts_id);
% [~, yaw_id] = f_Data_sel(time_ATT, yaw, rs(1), rs(2), Ts_id);
% 
% 
% range_r = rs;
% M_u2pwm = [1 -0.5 0.5 0.5;1 0.5 -0.5 0.5;1 0.5 0.5 -0.5;1 -0.5 -0.5 -0.5];
% [ucol, uroll, upitch, uyaw] = f_pwm2u(pwm1, pwm2, pwm3, pwm4,M_u2pwm);
% time_u = time_RCOU;
% 
% [time_id, upitch_id] = f_Data_sel(time_u, upitch, rs(1), rs(2), Ts_id);
% 
% figure(5)
% subplot(4,1,1)
% plot(time_u, ucol);
% ylabel('ucol')
% subplot(4,1,2)
% plot(time_u, uroll);
% ylabel('uroll')
% subplot(4,1,3)
% plot(time_u, upitch);
% ylabel('upitch')
% subplot(4,1,4)
% plot(time_u, uyaw);
% ylabel('uyaw')
% xlabel('Time/(s)')
% 
% 
