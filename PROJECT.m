%% PROJECT, Three tanks system 
clear all
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

%% State-feedback control analysis and Design:

%STABILIZING CONTROL LAW:

% CENTRALIZED
[K_c,rho_c,feas_c]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_centralized);
[K_c_DT,rho_c_DT,feas_c_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_centralized);

% DECENTRALIZED
[K_De,rho_De,feas_De]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_decentralized);
[K_De_DT,rho_De_DT,feas_De_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_decentralized);

% DISTRIBUTED
[K_string,rho_string,feas_string]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_distributedString);
[K_string_DT,rho_string_DT,feas_string_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_distributedString);

% DISTRIBUTED2
[K_string2,rho_string2,feas_string2]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_distributedString2);
[K_string2_DT,rho_string2_DT,feas_string2_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_distributedString2);

%-----------------------------------------------------------------------
% SPEED UP, REDUCE OVERSHOOT AND LIMIT CONTROL ACTION CONTROL LAW:

%% CONTINUOUS TIME:
rho_desired = 1.7; % speed up to get a setting time of 2.9 sec
alfa_desired = (15/180)*pi; % lower bound on dumping factor, +/- 15 deg region

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
alfa_desired_DT = -0.1; % place the circle center where eigs are constrained 

% CENTRALIZED
[K_c_opt1_DT,rho_c_opt1_DT,feas_c_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_centralized,rho_desired_DT,alfa_desired_DT);
% DECENTRALIZED
[K_De_opt1_DT,rho_De_opt1_DT,feas_De_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_decentralized,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED
[K_string_opt1_DT,rho_string_opt1_DT,feas_string_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_distributedString,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED2
[K_string2_opt1_DT,rho_string2_opt1_DT,feas_string2_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_distributedString2,rho_desired_DT,alfa_desired_DT);


%% SIMULATIONS
% open loop
% stabilizing
% optimal design 

%-------------------------------------------------------------------------
% OPEN LOOP SIMULATION:
Tf = 10;
T = [0:0.01:Tf];
n = length(Atot); % number of states

x0 = 0.1*randn(n,1); %random initial states
%x0 = [-0.15, 0.092, -0.086]';
%x0 = [0.0467941733051545,-0.0114443682929410,-0.0477911695074762]';
%x0=[0.248;0.103;-0.259];

%x01 = [0.040, 0.083, -0.104]';
%x02= [0.055, 0.0018, 0.0165]';
%x03 = [-0.109, -0.039, -0.012]';

u=[1, 1]'; % unitary step 

% CONTINUOUS
i = 0;
for t=T
    i = i+1;
    x_free(:,i) = expm(Atot*t)*x0;
    x_forced(:,i)=Ctot/Atot*(expm(Atot*t)-eye(n))*Btot*u;
end
x = x_free + x_forced;
x_bar = -Ctot/Atot*Btot*u; % unitary step equilibrium point

% DISCRETE
steps = [0:Tf/h];
i = 0;
for k=steps
    i=i+1;
    x_free_DT(:,i)=Htot*(Ftot^k)*x0;
    x_forced_DT(:,i)=Htot/(eye(n)-Ftot)*(eye(n)-Ftot^k)*Gtot*u;
end
x_DT = x_free_DT + x_forced_DT;
x_bar_DT=Htot/(eye(n)-Ftot)*Gtot*u; % unitary step equilibrium point


figure('Name','OPEN LOOP STATES')
% Free Motion(CONTINUOUS):
subplot(2,2,1)
plot(T,x_free(1,:), 'r');
hold on
plot(T,x_free(2,:), 'b');
plot(T,x_free(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('Continuous-time free response (OPEN-LOOP)')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

% Forced Motion (CONTINUOUS):
subplot(2,2,3)
plot(T,x(1,:), 'r');
hold on
plot(T,x(2,:), 'b');
plot(T,x(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('Continuous-time step response (OPEN-LOOP)')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

% Free Motion(DISCRETE):
subplot(2,2,2)
plot(steps*h,x_free_DT(1,:), 'r');
hold on
plot(steps*h,x_free_DT(2,:), 'b');
plot(steps*h,x_free_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('Discrete-time free response (OPEN-LOOP)')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

% Forced Motion (DISCRETE):
subplot(2,2,4)
plot(steps*h,x_DT(1,:), 'r');
hold on
plot(steps*h,x_DT(2,:), 'b');
plot(steps*h,x_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('Discrete-time step response (OPEN-LOOP)')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

sgtitle(" OPEN LOOP DYNAMIC ")


%-------------------------------------------------------------------------
% CLOSED LOOP SIMULATION 

% STABILIZING CONTROL LAW
i = 0;
%CONTINUOUS
for t=T
    i=i+1;
    x_c(:,i)=expm((Atot+Btot*K_c)*t)*x0;
    x_De(:,i)=expm((Atot+Btot*K_De)*t)*x0;
    x_string(:,i)=expm((Atot+Btot*K_string)*t)*x0;
    x_string2(:,i)=expm((Atot+Btot*K_string2)*t)*x0;
end
u_c = K_c*x_c;
u_De = K_De*x_De;
u_string = K_string*x_string;
u_string2 = K_string2*x_string2;


%DICRETE
i = 0;
for k=steps
    i = i+1;
    x_c_DT(:,i)=((Ftot+Gtot*K_c_DT)^k)*x0;
    x_De_DT(:,i)=((Ftot+Gtot*K_De_DT)^k)*x0;
    x_string_DT(:,i)=((Ftot+Gtot*K_string_DT)^k)*x0;
    x_string2_DT(:,i)=((Ftot+Gtot*K_string2_DT)^k)*x0;
end
u_c_DT = K_c_DT*x_c_DT;
u_De_DT = K_De_DT*x_De_DT;
u_string_DT = K_string_DT*x_string_DT;
u_string2_DT = K_string2_DT*x_string2_DT;

% CENTRALIZED:
figure('Name',' CENTRALIZED STABILIZING')
subplot(2,2,1)
plot(T,x_c(1,:), 'r');
hold on
plot(T,x_c(2,:), 'b');
plot(T,x_c(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Centralized Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,2)
plot(steps*h,x_c_DT(1,:), 'r');
hold on
plot(steps*h,x_c_DT(2,:), 'b');
plot(steps*h,x_c_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Centralized Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,3)
plot(T,u_c(1,:), 'm');
hold on
plot(T,u_c(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Centralized Stabilizing Control')
legend('u_{1}', 'u_{2}')


subplot(2,2,4)
plot(steps*h,u_c_DT(1,:), 'm');
hold on
plot(steps*h,u_c_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Centralized Stabilizing Control')
legend('u_{1}', 'u_{2}')

sgtitle(" CENTRALIZED STABILIZING CONTROL ")

% DECENTRALIZED
figure('Name',' DECENTRALIZED STABILIZING')
subplot(2,2,1)
plot(T,x_De(1,:), 'r');
hold on
plot(T,x_De(2,:), 'b');
plot(T,x_De(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Decentralized Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,2)
plot(steps*h,x_De_DT(1,:), 'r');
hold on
plot(steps*h,x_De_DT(2,:), 'b');
plot(steps*h,x_De_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Decentralized Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,3)
plot(T,u_De(1,:), 'm');
hold on
plot(T,u_De(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Decentralized Stabilizing Control')
legend('u_{1}', 'u_{2}')


subplot(2,2,4)
plot(steps*h,u_De_DT(1,:), 'm');
hold on
plot(steps*h,u_De_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Decentralized Stabilizing Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DECENTRALIZED STABILIZING CONTROL ")

% DISTRIBUTED 

figure('Name',' DISTRIBUTED STABILIZING')
subplot(2,2,1)
plot(T,x_string(1,:), 'r');
hold on
plot(T,x_string(2,:), 'b');
plot(T,x_string(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,2)
plot(steps*h,x_string_DT(1,:), 'r');
hold on
plot(steps*h,x_string_DT(2,:), 'b');
plot(steps*h,x_string_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Distributed Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,3)
plot(T,u_string(1,:), 'm');
hold on
plot(T,u_string(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed Stabilizing Control')
legend('u_{1}', 'u_{2}')


subplot(2,2,4)
plot(steps*h,u_string_DT(1,:), 'm');
hold on
plot(steps*h,u_string_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Distributed Stabilizing Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED STABILIZING CONTROL ")

% DISTRIBUTED 2

figure('Name',' DISTRIBUTED STABILIZING')
subplot(2,2,1)
plot(T,x_string2(1,:), 'r');
hold on
plot(T,x_string2(2,:), 'b');
plot(T,x_string2(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed 2 Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,2)
plot(steps*h,x_string2_DT(1,:), 'r');
hold on
plot(steps*h,x_string2_DT(2,:), 'b');
plot(steps*h,x_string2_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Distributed 2 Stabilizing States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(2,2,3)
plot(T,u_string2(1,:), 'm');
hold on
plot(T,u_string2(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed 2 Stabilizing Control')
legend('u_{1}', 'u_{2}')


subplot(2,2,4)
plot(steps*h,u_string2_DT(1,:), 'm');
hold on
plot(steps*h,u_string2_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Distributed 2 Stabilizing Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED 2 STABILIZING CONTROL ")

figure('Name','PZPLOT STABILIZING CONTINUOUS')
subplot(2,3,1)
pzmap(ss(Atot,Btot,Ctot,0))
title("Open Loop")
subplot(2,3,2)
pzmap(ss(Atot+Btot*K_c,zeros(3,2),Ctot,0))
title("Centralized")
subplot(2,3,3)
pzmap(ss(Atot+Btot*K_De,zeros(3,2),Ctot,0))
title("Decentralized")
subplot(2,3,4)
pzmap(ss(Atot+Btot*K_string,zeros(3,2),Ctot,0))
title("Distributed")
subplot(2,3,5)
pzmap(ss(Atot+Btot*K_string2,zeros(3,2),Ctot,0))
title("Distributed 2")

figure('Name','PZPLOT STABILIZING DISCRETE')
subplot(2,3,1)
pzmap(ss(Ftot,Gtot,Htot,0))
title("Open Loop")
subplot(2,3,2)
pzmap(ss(Ftot+Gtot*K_c_DT,zeros(3,2),Htot,0))
title("Centralized")
subplot(2,3,3)
pzmap(ss(Ftot+Gtot*K_De_DT,zeros(3,2),Htot,0))
title("Decentralized")
subplot(2,3,4)
pzmap(ss(Ftot+Gtot*K_string_DT,zeros(3,2),Htot,0))
title("Distributed")
subplot(2,3,5)
pzmap(ss(Ftot+Gtot*K_string2_DT,zeros(3,2),Htot,0))
title("Distributed 2")
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
end
u_c_opt1 = K_c_opt1*x_c_opt1;
u_De_opt1 = K_De_opt1*x_De_opt1;
u_string_opt1 = K_string_opt1*x_string_opt1;
u_string2_opt1 = K_string2_opt1*x_string2_opt1;


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

%% PLACE EIGENVALUES IN A CIRCLE REGION WITH LIMITED RADIUS AND CHOSEN CENTER

%DICRETE
i = 0;
for k=steps
    i = i+1;
    x_c_opt1_DT(:,i)=((Ftot+Gtot*K_c_opt1_DT)^k)*x0;
    x_De_opt1_DT(:,i)=((Ftot+Gtot*K_De_opt1_DT)^k)*x0;
    x_string_opt1_DT(:,i)=((Ftot+Gtot*K_string_opt1_DT)^k)*x0;
    x_string2_opt1_DT(:,i)=((Ftot+Gtot*K_string2_opt1_DT)^k)*x0;
end
u_c_opt1_DT = K_c_opt1_DT*x_c_opt1_DT;
u_De_opt1_DT = K_De_opt1_DT*x_De_opt1_DT;
u_string_opt1_DT = K_string_opt1_DT*x_string_opt1_DT;
u_string2_opt1_DT = K_string2_opt1_DT*x_string2_opt1_DT;

% CENTRALIZED:
figure('Name',' CENTRALIZED DISCRETE OPT1 ')
subplot(1,2,1)
plot(steps*h,x_c_opt1_DT(1,:), 'r');
hold on
plot(steps*h,x_c_opt1_DT(2,:), 'b');
plot(steps*h,x_c_opt1_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Centralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,u_c_opt1_DT(1,:), 'm');
hold on
plot(steps*h,u_c_opt1_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Centralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" CENTRALIZED CONTROL ")

% DECENTRALIZED
figure('Name',' DECENTRALIZED DISCRETE OPT1 ')
subplot(1,2,1)
plot(steps*h,x_De_opt1_DT(1,:), 'r');
hold on
plot(steps*h,x_De_opt1_DT(2,:), 'b');
plot(steps*h,x_De_opt1_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Decentralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,u_De_opt1_DT(1,:), 'm');
hold on
plot(steps*h,u_De_opt1_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Decentralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DECENTRALIZED CONTROL ")

% DISTRIBUTED 

figure('Name',' DISTRIBUTED DISCRETE OPT1 ')
subplot(1,2,1)
plot(steps*h,x_string_opt1_DT(1,:), 'r');
hold on
plot(steps*h,x_string_opt1_DT(2,:), 'b');
plot(steps*h,x_string_opt1_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Distributed States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,u_string_opt1_DT(1,:), 'm');
hold on
plot(steps*h,u_string_opt1_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Distributed Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED CONTROL DISCRETE ")

% DISTRIBUTED 2

figure('Name',' DISTRIBUTED 2 DISCRETE OPT1 ')
subplot(1,2,1)
plot(steps*h,x_string2_opt1_DT(1,:), 'r');
hold on
plot(steps*h,x_string2_opt1_DT(2,:), 'b');
plot(steps*h,x_string2_opt1_DT(3,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Distributed 2 States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,u_string2_opt1_DT(1,:), 'm');
hold on
plot(steps*h,u_string2_opt1_DT(2,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Distributed 2 Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED 2 CONTROL DISCRETE ")


figure('Name','PZPLOT DISCRETE')
subplot(2,3,1)
pzmap(ss(Ftot,Gtot,Htot,0))
title("Open Loop")
subplot(2,3,2)
pzmap(ss(Ftot+Gtot*K_c_opt1_DT,zeros(3,2),Ctot,0))
title("Centralized")
subplot(2,3,3)
pzmap(ss(Ftot+Gtot*K_De_opt1_DT,zeros(3,2),Ctot,0))
title("Decentralized")
subplot(2,3,4)
pzmap(ss(Ftot+Gtot*K_string_opt1_DT,zeros(3,2),Ctot,0))
title("Distributed")
subplot(2,3,5)
pzmap(ss(Ftot+Gtot*K_string2_opt1_DT,zeros(3,2),Ctot,0))
title("Distributed 2")



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


disp(['-  Centralized: Feasibility=',num2str(feas_c),', rho=',num2str(rho_c),', FM=',num2str(cfm),'.'])
disp(['-  Decentralized: Feasibility=',num2str(feas_De),', rho=',num2str(rho_De),', FM=',num2str(Dfm),'.'])
disp(['-  Distributed (string): Feasibility=',num2str(feas_string),', rho=',num2str(rho_string),', FM=',num2str(Distfm),'.'])
disp(['-  Distributed (string 2): Feasibility=',num2str(feas_string2),', rho=',num2str(rho_string2),', FM=',num2str(Distfm2),'.'])

disp(['-  Centralized OPT1: Feasibility=',num2str(feas_c_opt1),', rho=',num2str(rho_c_opt1),', FM=',num2str(cfm),'.'])
disp(['-  Decentralized OPT1: Feasibility=',num2str(feas_De_opt1),', rho=',num2str(rho_De_opt1),', FM=',num2str(Dfm),'.'])
disp(['-  Distributed (string) OPT1: Feasibility=',num2str(feas_string_opt1),', rho=',num2str(rho_string_opt1),', FM=',num2str(Distfm),'.'])
disp(['-  Distributed (string 2) OPT1: Feasibility=',num2str(feas_string2_opt1),', rho=',num2str(rho_string2_opt1),', FM=',num2str(Distfm2),'.'])

disp('Results (Discrete-time):')

if rho_DT < 1
    disp(['-  Open-loop: Asyptotically stable,  Spectral radius = ',num2str(rho_DT)]);
elseif rho_DT == 1
    disp(['-  Open-loop: Simply stable,  Spectral radius = ',num2str(rho_DT)]);
else
    disp(['-  Open-loop: Unstable,  Spectral radius = ',num2str(rho_DT)]);
end

disp(['-  Centralized: Feasibility=',num2str(feas_c_DT),', rho=',num2str(rho_c_DT),', FM=',num2str(cfm_DT),'.'])
disp(['-  Decentralized: Feasibility=',num2str(feas_De_DT),', rho=',num2str(rho_De_DT),', FM=',num2str(Dfm_DT),'.'])
disp(['-  Distributed (string): Feasibility=',num2str(feas_string_DT),', rho=',num2str(rho_string_DT),', FM=',num2str(Distfm_DT),'.'])
disp(['-  Distributed (string 2): Feasibility=',num2str(feas_string2_DT),', rho=',num2str(rho_string2_DT),', FM=',num2str(Distfm2_DT),'.'])


disp(['-  Centralized OPT1: Feasibility=',num2str(feas_c_opt1_DT),', rho=',num2str(rho_c_opt1_DT),', FM=',num2str(cfm_DT),'.'])
disp(['-  Decentralized OPT1: Feasibility=',num2str(feas_De_opt1_DT),', rho=',num2str(rho_De_opt1_DT),', FM=',num2str(Dfm_DT),'.'])
disp(['-  Distributed (string) OPT1: Feasibility=',num2str(feas_string_opt1_DT),', rho=',num2str(rho_string_opt1_DT),', FM=',num2str(Distfm_DT),'.'])
disp(['-  Distributed (string 2) OPT1: Feasibility=',num2str(feas_string2_opt1_DT),', rho=',num2str(rho_string2_opt1_DT),', FM=',num2str(Distfm2_DT),'.'])
