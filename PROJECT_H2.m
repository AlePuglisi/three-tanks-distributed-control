%% PROJECT, Three tanks system 
clear 
close all
clc

%% MODELLING
% system state space matrix description:
Atot=[-1  0  0
       1 -1  0
       0  1 -1];

Ctot=eye(3);

Btot=[1  0
      0  1
      0  0];

%discrete time realization
h = 0.2; % sampling time, to chose wisely according to system dynamic
[Ftot,Gtot,Htot,Ltot,h]=ssdata(c2d(ss(Atot,Btot,Ctot,[]),h));

% System decomposition:
N = 2; %number of subsystems, (1)first tank, (2)second+third tank

for i=1:N
    % continuous time decomposition
    B{i} = Btot(:,i);
    C{i} = Ctot(i:2*i-1,:);
    % discrete time decomposition 
    G{i} = Gtot(:,i);
    H{i} = Htot(i:2*i-1,:);
end

%% ANALYSIS

% Open Loop analysis:
% CONTROL STRUCTURES
ContStructure_centralized = ones(N,N); % centralized
ContStructure_decentralized = diag(ones(N,1)); % decentralized
% smart choice is to use info from first sub system to affect second
% controller (due to the inherent influence of the first to the second)
ContStructure_distributedString = [1    0;  
                                   1    1]; % BETTER ONE PHISICALLY

ContStructure_distributedString2 = [1    1;  
                                   0    1];

% OPEN LOOP EIGEN VALUES and SPECTRAL ABSCISSA:

% CONTINUOUS
eigenvalues=eig(Atot); % eigen values 
rho=max(real(eig(Atot))); % spectral abscissa

% DISCRETE
eigenvalues_DT=eig(Ftot); % eigen values 
rho_DT=max(abs(eig(Ftot))); % spectral radius

% State-feedback FIXED MODES:

% Centralized
[cfm]=di_fixed_modes(Atot,B,C,N,ContStructure_centralized,3);
[cfm_DT]=di_fixed_modes(Ftot,G,H,N,ContStructure_centralized,3);
% Decentralized
[Dfm]=di_fixed_modes(Atot,B,C,N,ContStructure_decentralized,3);
[Dfm_DT]=di_fixed_modes(Ftot,G,H,N,ContStructure_decentralized,3);

% Distributed1, BETTER
[Distfm]=di_fixed_modes(Atot,B,C,N,ContStructure_distributedString,3);
[Distfm_DT]=di_fixed_modes(Ftot,G,H,N,ContStructure_distributedString,3); 

% Distributed2
[Distfm2]=di_fixed_modes(Atot,B,C,N,ContStructure_distributedString2,3);
[Distfm2_DT]=di_fixed_modes(Ftot,G,H,N,ContStructure_distributedString2,3); 
%-----------------------------------------------------------------------
% SPEED UP, REDUCE OVERSHOOT AND LIMIT CONTROL ACTION CONTROL LAW:

%% CONTINUOUS TIME:
rho_desired = 1.7; % speed up to get a setting time of 2.5 sec
alfa_desired = (10/180)*pi; % lower bound on dumping factor, +/- 40 deg region

% CENTRALIZED
[K_c_opt1,rho_c_opt1,feas_c_opt1]=LMI_CT_opt1(Atot,B,C,N,ContStructure_centralized,rho_desired,alfa_desired);
% DECENTRALIZED
[K_De_opt1,rho_De_opt1,feas_De_opt1]=LMI_CT_opt1(Atot,B,C,N,ContStructure_decentralized,rho_desired,alfa_desired);
% DISTRIBUTED
[K_string_opt1,rho_string_opt1,feas_string_opt1]=LMI_CT_opt1(Atot,B,C,N,ContStructure_distributedString,rho_desired,alfa_desired);
% DISTRIBUTED2
[K_string2_opt1,rho_string2_opt1,feas_string2_opt1]=LMI_CT_opt1(Atot,B,C,N,ContStructure_distributedString2,rho_desired,alfa_desired);

%% DISCRETE TIME:
% first compare with same performance request, than try to optimally design
% the controller for each control structure, according to its limitations

% PERFORMANCE
rho_desired_DT = 0.05; % limit the circle region radius of DT eigenvalues
alfa_desired_DT = -0.75; % place the circle center where eigs are constrained 

% CENTRALIZED
[K_c_opt1_DT,rho_c_opt1_DT,feas_c_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_centralized,rho_desired_DT,alfa_desired_DT);
% DECENTRALIZED
[K_De_opt1_DT,rho_De_opt1_DT,feas_De_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_decentralized,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED
[K_string_opt1_DT,rho_string_opt1_DT,feas_string_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_distributedString,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED2
[K_string2_opt1_DT,rho_string2_opt1_DT,feas_string2_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_distributedString2,rho_desired_DT,alfa_desired_DT);

%% LQ CONTROL (BY H2 PROPER MATRIX CHOICE)
n = length(Atot); % number of states
m = size(Btot,2);

% CONTINUOUS TIME
Bw = eye(n);
%Bw=zeros(n,n);
Q = 100; % State weight
R = 1; % Control action weight
Cz = [eye(n)*sqrt(Q);zeros(m,n)];
Dz = [zeros(n,m);eye(m)*sqrt(R)];

% CENTRALIZED
[K_c_H2,rho_c_H2,feas_c_H2]=LMI_CT_H2(Atot,B,C,Bw,Cz,Dz,N,ContStructure_centralized);
% DECENTRALIZED
[K_De_H2,rho_De_H2,feas_De_H2]=LMI_CT_H2(Atot,B,C,Bw,Cz,Dz,N,ContStructure_decentralized);
% DISTRIBUTED
[K_string_H2,rho_string_H2,feas_string_H2]=LMI_CT_H2(Atot,B,C,Bw,Cz,Dz,N,ContStructure_distributedString);
% DISTRIBUTED2
[K_string2_H2,rho_string2_H2,feas_string2_H2]=LMI_CT_H2(Atot,B,C,Bw,Cz,Dz,N,ContStructure_distributedString2);

% DISCRETE TIME

Gw = eye(n);
Q = 100; % State weight
R = 100; % Control action weight
Hz = [eye(n)*sqrt(Q);zeros(m,n)];
Dz = [zeros(n,m);eye(m)*sqrt(R)];
% CENTRALIZED
[K_c_H2_DT,rho_c_H2_DT,feas_c_H2_DT]=LMI_DT_H2(Ftot,G,H,Hz,Dz,Gw,N,ContStructure_centralized);
% DECENTRALIZED
[K_De_H2_DT,rho_De_H2_DT,feas_De_H2_DT]=LMI_DT_H2(Ftot,G,H,Hz,Dz,Gw,N,ContStructure_decentralized);
% DISTRIBUTED
[K_string_H2_DT,rho_string_H2_DT,feas_string_H2_DT]=LMI_DT_H2(Ftot,G,H,Hz,Dz,Gw,N,ContStructure_distributedString);
% DISTRIBUTED2
[K_string2_H2_DT,rho_string2_H2_DT,feas_string2_H2_DT]=LMI_DT_H2(Ftot,G,H,Hz,Dz,Gw,N,ContStructure_distributedString2);


%% SIMULATIONS
% optimal design
% H2 control

%-------------------------------------------------------------------------
Tf = 10;
T = [0:0.01:Tf];

x0 = 0.1*randn(n,1); %random initial states
%x0 = [-0.1444, -0.0991, -0.0732]';

%---------------------------------------------------------------------
%% REDUCING OVERSHOOT AND SPEED UP CONTROL, WITH LIMITED CONTROL ACTION (OPT1)

i = 0;
% CONTINUOUS
for t=T
    i=i+1;
    x_c_opt1(:,i)=expm((Atot+Btot*K_c_opt1)*t)*x0;
    x_De_opt1(:,i)=expm((Atot+Btot*K_De_opt1)*t)*x0;
    x_string_opt1(:,i)=expm((Atot+Btot*K_string_opt1)*t)*x0;
    x_string2_opt1(:,i)=expm((Atot+Btot*K_string2_opt1)*t)*x0;

    x_c_H2(:,i)=expm((Atot+Btot*K_c_H2)*t)*x0;
    x_De_H2(:,i)=expm((Atot+Btot*K_De_H2)*t)*x0;
    x_string_H2(:,i)=expm((Atot+Btot*K_string_H2)*t)*x0;
    x_string2_H2(:,i)=expm((Atot+Btot*K_string2_H2)*t)*x0;
    
end
u_c_opt1 = K_c_opt1*x_c_opt1;
u_De_opt1 = K_De_opt1*x_De_opt1;
u_string_opt1 = K_string_opt1*x_string_opt1;
u_string2_opt1 = K_string2_opt1*x_string2_opt1;

u_c_H2 = K_c_H2*x_c_H2;
u_De_H2 = K_De_H2*x_De_H2;
u_string_H2 = K_string_H2*x_string_H2;
u_string2_H2 = K_string2_H2*x_string2_H2;


% CENTRALIZED:
figure('Name',' CENTRALIZED SPEED+OVERSHOOT ')
subplot(1,2,1)
plot(T,x_c_opt1(1,:), 'r');
hold on
plot(T,x_c_opt1(2,:), 'b');
plot(T,x_c_opt1(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Centralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_c_opt1(1,:), 'm');
hold on
plot(T,u_c_opt1(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Centralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" CENTRALIZED CONTROL ")

% DECENTRALIZED
figure('Name',' DECENTRALIZED SPEED+OVERSHOOT ')
subplot(1,2,1)
plot(T,x_De_opt1(1,:), 'r');
hold on
plot(T,x_De_opt1(2,:), 'b');
plot(T,x_De_opt1(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Decentralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_De_opt1(1,:), 'm');
hold on
plot(T,u_De_opt1(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Decentralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DECENTRALIZED CONTROL ")

% DISTRIBUTED 

figure('Name',' DISTRIBUTED SPEED+OVERSHOOT ')
subplot(1,2,1)
plot(T,x_string_opt1(1,:), 'r');
hold on
plot(T,x_string_opt1(2,:), 'b');
plot(T,x_string_opt1(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_string_opt1(1,:), 'm');
hold on
plot(T,u_string_opt1(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED CONTROL ")

% DISTRIBUTED 2

figure('Name',' DISTRIBUTED SPEED+OVERSHOOT ')
subplot(1,2,1)
plot(T,x_string2_opt1(1,:), 'r');
hold on
plot(T,x_string2_opt1(2,:), 'b');
plot(T,x_string2_opt1(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed 2 States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_string2_opt1(1,:), 'm');
hold on
plot(T,u_string2_opt1(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed 2 Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED 2 CONTROL ")

%% H2 CONTINUOUS

% CENTRALIZED:
figure('Name',' CENTRALIZED H2 ')
subplot(1,2,1)
plot(T,x_c_H2(1,:), 'r');
hold on
plot(T,x_c_H2(2,:), 'b');
plot(T,x_c_H2(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Centralized States H2')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_c_H2(1,:), 'm');
hold on
plot(T,u_c_H2(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Centralized Control H2')
legend('u_{1}', 'u_{2}')

sgtitle(" CENTRALIZED CONTROL H2")

% DECENTRALIZED
figure('Name',' DECENTRALIZED H2 ')
subplot(1,2,1)
plot(T,x_De_H2(1,:), 'r');
hold on
plot(T,x_De_H2(2,:), 'b');
plot(T,x_De_H2(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Decentralized States H2')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_De_H2(1,:), 'm');
hold on
plot(T,u_De_H2(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Decentralized Control H2')
legend('u_{1}', 'u_{2}')

sgtitle(" DECENTRALIZED CONTROL H2")

% DISTRIBUTED 

figure('Name',' DISTRIBUTED H2 ')
subplot(1,2,1)
plot(T,x_string_H2(1,:), 'r');
hold on
plot(T,x_string_H2(2,:), 'b');
plot(T,x_string_H2(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed States H2')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_string_H2(1,:), 'm');
hold on
plot(T,u_string_H2(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed Control H2')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED CONTROL H2")

% DISTRIBUTED 2

figure('Name',' DISTRIBUTED H2')
subplot(1,2,1)
plot(T,x_string2_H2(1,:), 'r');
hold on
plot(T,x_string2_H2(2,:), 'b');
plot(T,x_string2_H2(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed 2 States H2')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,u_string2_H2(1,:), 'm');
hold on
plot(T,u_string2_H2(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed 2 Control H2')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED 2 CONTROL H2")

figure('Name','PZPLOT CONTINUOUS')
subplot(2,3,1)
pzmap(ss(Atot,Btot,Ctot,0))
title("Open Loop")
subplot(2,3,2)
pzmap(ss(Atot+Btot*K_c_opt1,zeros(3,2),Ctot,0))
title("Centralized")
subplot(2,3,3)
pzmap(ss(Atot+Btot*K_De_opt1,zeros(3,2),Ctot,0))
title("Decentralized")
subplot(2,3,4)
pzmap(ss(Atot+Btot*K_string_opt1,zeros(3,2),Ctot,0))
title("Distributed")
subplot(2,3,5)
pzmap(ss(Atot+Btot*K_string2_opt1,zeros(3,2),Ctot,0))
title("Distributed 2")

figure('Name','PZPLOT CONTINUOUS H2')
subplot(2,3,1)
pzmap(ss(Atot,Btot,Ctot,0))
title("Open Loop")
subplot(2,3,2)
pzmap(ss(Atot+Btot*K_c_H2,zeros(3,2),Ctot,0))
title("Centralized H2")
subplot(2,3,3)
pzmap(ss(Atot+Btot*K_De_H2,zeros(3,2),Ctot,0))
title("Decentralized H2")
subplot(2,3,4)
pzmap(ss(Atot+Btot*K_string_H2,zeros(3,2),Ctot,0))
title("Distributed H2")
subplot(2,3,5)
pzmap(ss(Atot+Btot*K_string2_H2,zeros(3,2),Ctot,0))
title("Distributed 2")

%% PLACE EIGENVALUES IN A CIRCLE REGION WITH LIMITED RADIUS AND CHOSEN CENTER

%DICRETE
% i = 0;
% steps = [0:Tf/h];
% for k=steps
%     i = i+1;
%     x_c_opt1_DT(:,i)=((Ftot+Gtot*K_c_opt1_DT)^k)*x0;
%     x_De_opt1_DT(:,i)=((Ftot+Gtot*K_De_opt1_DT)^k)*x0;
%     x_string_opt1_DT(:,i)=((Ftot+Gtot*K_string_opt1_DT)^k)*x0;
%     x_string2_opt1_DT(:,i)=((Ftot+Gtot*K_string2_opt1_DT)^k)*x0;
% 
%      x_c_H2_DT(:,i)=((Ftot+Gtot*K_c_H2_DT)^k)*x0;
%     x_De_H2_DT(:,i)=((Ftot+Gtot*K_De_H2_DT)^k)*x0;
%     x_string_H2_DT(:,i)=((Ftot+Gtot*K_string_H2_DT)^k)*x0;
%     x_string2_H2_DT(:,i)=((Ftot+Gtot*K_string2_H2_DT)^k)*x0;
% 
% end
% u_c_opt1_DT = K_c_opt1_DT*x_c_opt1_DT;
% u_De_opt1_DT = K_De_opt1_DT*x_De_opt1_DT;
% u_string_opt1_DT = K_string_opt1_DT*x_string_opt1_DT;
% u_string2_opt1_DT = K_string2_opt1_DT*x_string2_opt1_DT;
% 
% u_c_H2_DT = K_c_H2_DT*x_c_H2_DT;
% u_De_H2_DT = K_De_H2_DT*x_De_H2_DT;
% u_string_H2_DT = K_string_H2_DT*x_string_H2_DT;
% u_string2_H2_DT = K_string2_H2_DT*x_string2_H2_DT;
% 
% % CENTRALIZED:
% figure('Name',' CENTRALIZED DISCRETE OPT1 ')
% subplot(1,2,1)
% plot(steps*h,x_c_opt1_DT(1,:), 'r');
% hold on
% plot(steps*h,x_c_opt1_DT(2,:), 'b');
% plot(steps*h,x_c_opt1_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Centralized States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_c_opt1_DT(1,:), 'm');
% hold on
% plot(steps*h,u_c_opt1_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Centralized Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" CENTRALIZED CONTROL ")
% 
% % DECENTRALIZED
% figure('Name',' DECENTRALIZED DISCRETE OPT1 ')
% subplot(1,2,1)
% plot(steps*h,x_De_opt1_DT(1,:), 'r');
% hold on
% plot(steps*h,x_De_opt1_DT(2,:), 'b');
% plot(steps*h,x_De_opt1_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Decentralized States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_De_opt1_DT(1,:), 'm');
% hold on
% plot(steps*h,u_De_opt1_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Decentralized Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DECENTRALIZED CONTROL ")
% 
% % DISTRIBUTED 
% 
% figure('Name',' DISTRIBUTED DISCRETE OPT1 ')
% subplot(1,2,1)
% plot(steps*h,x_string_opt1_DT(1,:), 'r');
% hold on
% plot(steps*h,x_string_opt1_DT(2,:), 'b');
% plot(steps*h,x_string_opt1_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Distributed States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_string_opt1_DT(1,:), 'm');
% hold on
% plot(steps*h,u_string_opt1_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Distributed Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DISTRIBUTED CONTROL DISCRETE ")
% 
% % DISTRIBUTED 2
% 
% figure('Name',' DISTRIBUTED 2 DISCRETE OPT1 ')
% subplot(1,2,1)
% plot(steps*h,x_string2_opt1_DT(1,:), 'r');
% hold on
% plot(steps*h,x_string2_opt1_DT(2,:), 'b');
% plot(steps*h,x_string2_opt1_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Distributed 2 States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_string2_opt1_DT(1,:), 'm');
% hold on
% plot(steps*h,u_string2_opt1_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Distributed 2 Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DISTRIBUTED 2 CONTROL DISCRETE ")
% 
% %% H2 DISCRETE
% 
% % CENTRALIZED:
% figure('Name',' CENTRALIZED DISCRETE H2 ')
% subplot(1,2,1)
% plot(steps*h,x_c_H2_DT(1,:), 'r');
% hold on
% plot(steps*h,x_c_H2_DT(2,:), 'b');
% plot(steps*h,x_c_H2_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Centralized States H2')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_c_H2_DT(1,:), 'm');
% hold on
% plot(steps*h,u_c_H2_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Centralized Control H2')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" CENTRALIZED CONTROL H2")
% 
% % DECENTRALIZED
% figure('Name',' DECENTRALIZED DISCRETE H2 ')
% subplot(1,2,1)
% plot(steps*h,x_De_H2_DT(1,:), 'r');
% hold on
% plot(steps*h,x_De_H2_DT(2,:), 'b');
% plot(steps*h,x_De_H2_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Decentralized States H2')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_De_H2_DT(1,:), 'm');
% hold on
% plot(steps*h,u_De_H2_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Decentralized Control H2')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DECENTRALIZED CONTROL H2")
% 
% % DISTRIBUTED 
% 
% figure('Name',' DISTRIBUTED DISCRETE H2 ')
% subplot(1,2,1)
% plot(steps*h,x_string_H2_DT(1,:), 'r');
% hold on
% plot(steps*h,x_string_H2_DT(2,:), 'b');
% plot(steps*h,x_string_H2_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Distributed States H2')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_string_H2_DT(1,:), 'm');
% hold on
% plot(steps*h,u_string_H2_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Distributed Control H2')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DISTRIBUTED CONTROL DISCRETE H2")
% 
% % DISTRIBUTED 2
% 
% figure('Name',' DISTRIBUTED 2 DISCRETE H" ')
% subplot(1,2,1)
% plot(steps*h,x_string2_H2_DT(1,:), 'r');
% hold on
% plot(steps*h,x_string2_H2_DT(2,:), 'b');
% plot(steps*h,x_string2_H2_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Distributed 2 States H2')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(1,2,2)
% plot(steps*h,u_string2_H2_DT(1,:), 'm');
% hold on
% plot(steps*h,u_string2_H2_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Distributed 2 Control H2')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DISTRIBUTED 2 CONTROL DISCRETE H2")
% 
% 
% 
% figure('Name','PZPLOT DISCRETE')
% subplot(2,3,1)
% pzmap(ss(Ftot,Gtot,Htot,0))
% title("Open Loop")
% subplot(2,3,2)
% pzmap(ss(Ftot+Gtot*K_c_opt1_DT,zeros(3,2),Ctot,0))
% title("Centralized")
% subplot(2,3,3)
% pzmap(ss(Ftot+Gtot*K_De_opt1_DT,zeros(3,2),Ctot,0))
% title("Decentralized")
% subplot(2,3,4)
% pzmap(ss(Ftot+Gtot*K_string_opt1_DT,zeros(3,2),Ctot,0))
% title("Distributed")
% subplot(2,3,5)
% pzmap(ss(Ftot+Gtot*K_string2_opt1_DT,zeros(3,2),Ctot,0))
% title("Distributed 2")
% 
% 
% figure('Name','PZPLOT DISCRETE H2')
% subplot(2,3,1)
% pzmap(ss(Ftot,Gtot,Htot,0))
% title("Open Loop")
% subplot(2,3,2)
% pzmap(ss(Ftot+Gtot*K_c_H2_DT,zeros(3,2),Ctot,0))
% title("Centralized H2")
% subplot(2,3,3)
% pzmap(ss(Ftot+Gtot*K_De_H2_DT,zeros(3,2),Ctot,0))
% title("Decentralized H2")
% subplot(2,3,4)
% pzmap(ss(Ftot+Gtot*K_string_H2_DT,zeros(3,2),Ctot,0))
% title("Distributed H2")
% subplot(2,3,5)
% pzmap(ss(Ftot+Gtot*K_string2_H2_DT,zeros(3,2),Ctot,0))
% title("Distributed 2 H2")



%% DISPLAY RESULT:

disp("---------THREE-TANK SYSTEM-----------");
disp([' Number of sub-systems N = ', num2str(N)]);
disp([' Sampling time h = ', num2str(h)]);


disp('Results (Continuous-time):')

if rho < 0
    disp(['-  Open-loop: Asyptotically stable,  Spectral abscissa = ',num2str(rho)]);
elseif rho == 0
    disp(['-  Open-loop: Simply stable,  Spectral abscissa = ',num2str(rho)]);
else
    disp(['-  Open-loop: Unstable,  Spectral abscissa = ',num2str(rho)]);
end

disp(['-  Centralized OPT1: Feasibility=',num2str(feas_c_opt1),', rho=',num2str(rho_c_opt1),', FM=',num2str(cfm),'.'])
disp(['-  Decentralized OPT1: Feasibility=',num2str(feas_De_opt1),', rho=',num2str(rho_De_opt1),', FM=',num2str(Dfm),'.'])
disp(['-  Distributed (string) OPT1: Feasibility=',num2str(feas_string_opt1),', rho=',num2str(rho_string_opt1),', FM=',num2str(Distfm),'.'])
disp(['-  Distributed (string 2) OPT1: Feasibility=',num2str(feas_string2_opt1),', rho=',num2str(rho_string2_opt1),', FM=',num2str(Distfm2),'.'])

disp(['-  Centralized H2: Feasibility=',num2str(feas_c_H2),', rho=',num2str(rho_c_H2),', FM=',num2str(cfm),'.'])
disp(['-  Decentralized H2: Feasibility=',num2str(feas_De_H2),', rho=',num2str(rho_De_H2),', FM=',num2str(Dfm),'.'])
disp(['-  Distributed (string) H2: Feasibility=',num2str(feas_string_H2),', rho=',num2str(rho_string_H2),', FM=',num2str(Distfm),'.'])
disp(['-  Distributed (string 2) H2: Feasibility=',num2str(feas_string2_H2),', rho=',num2str(rho_string2_H2),', FM=',num2str(Distfm2),'.'])


% disp('Results (Discrete-time):')
% 
% if rho_DT < 1
%     disp(['-  Open-loop: Asyptotically stable,  Spectral radius = ',num2str(rho_DT)]);
% elseif rho_DT == 1
%     disp(['-  Open-loop: Simply stable,  Spectral radius = ',num2str(rho_DT)]);
% else
%     disp(['-  Open-loop: Unstable,  Spectral radius = ',num2str(rho_DT)]);
% end
% 
% disp(['-  Centralized OPT1: Feasibility=',num2str(feas_c_opt1_DT),', rho=',num2str(rho_c_opt1_DT),', FM=',num2str(cfm_DT),'.'])
% disp(['-  Decentralized OPT1: Feasibility=',num2str(feas_De_opt1_DT),', rho=',num2str(rho_De_opt1_DT),', FM=',num2str(Dfm_DT),'.'])
% disp(['-  Distributed (string) OPT1: Feasibility=',num2str(feas_string_opt1_DT),', rho=',num2str(rho_string_opt1_DT),', FM=',num2str(Distfm_DT),'.'])
% disp(['-  Distributed (string 2) OPT1: Feasibility=',num2str(feas_string2_opt1_DT),', rho=',num2str(rho_string2_opt1_DT),', FM=',num2str(Distfm2_DT),'.'])
% 
% disp(['-  Centralized H2: Feasibility=',num2str(feas_c_H2_DT),', rho=',num2str(rho_c_H2_DT),', FM=',num2str(cfm_DT),'.'])
% disp(['-  Decentralized H2: Feasibility=',num2str(feas_De_H2_DT),', rho=',num2str(rho_De_H2_DT),', FM=',num2str(Dfm_DT),'.'])
% disp(['-  Distributed (string) H2: Feasibility=',num2str(feas_string_H2_DT),', rho=',num2str(rho_string_H2_DT),', FM=',num2str(Distfm_DT),'.'])
% disp(['-  Distributed (string 2) H2: Feasibility=',num2str(feas_string2_H2_DT),', rho=',num2str(rho_string2_H2_DT),', FM=',num2str(Distfm2_DT),'.'])
