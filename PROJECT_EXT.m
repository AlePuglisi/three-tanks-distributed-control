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

% try to model an extended system to minimize control action rate
% obtain that control action rate limitation by extending system state
% as x = [h1, u1, h2, h3, u2] and introduce as new virtual control action
% [v1, v2] which represents du1, du2
Aext = [Atot(1,1)   Btot(1,1)   0           0           0;
        0               0       0           0           0;
        Atot(2,1)       0      Atot(2,2)    0       Btot(2,2);
        0               0      Atot(3,2)   Atot(3,3)    0; 
        0               0       0           0           0;];
Bext = [0   0;
        1   0;
        0   0;
        0   0;
        0   1;];
% Cext = [1   0   0   0   0;
%        0   1   0   0   0;
%        0   0   1   0   0;];
Cext = eye(5);


%discrete time realization
h = 0.1; % sampling time, to chose wisely according to system dynamic
[Ftot,Gtot,Htot,Ltot,h]=ssdata(c2d(ss(Atot,Btot,Ctot,[]),h));

[Fext,Gext,Hext,Lext,h]= ssdata(c2d(ss(Aext,Bext,Cext,[]),h));

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

for i=1:N
    % continuous time decomposition
    B_e{i} = Bext(:,i);
    % discrete time decomposition 
    G_e{i} = Gext(:,i);
   
end
C_e{1} = Cext(1:2,:);
C_e{2} = Cext(3:5,:);

H_e{1} = Hext(1:2,:);
H_e{2} = Hext(3:5,:);

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

% CONTINUOUS EXT
eigenvalues_ext=eig(Aext); % eigen values 
rho_ext=max(real(eig(Aext))); % spectral abscissa
%DISCRETE EXT
eigenvalues_DT_ext=eig(Fext); % eigen values 
rho_DT_ext=max(abs(eig(Fext))); % spectral radius

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

%% EXTENDED SYSTEM
% Centralized ext
[cfm_ext]=di_fixed_modes(Aext,B_e,C_e,N,ContStructure_centralized,3);
[cfm_DT_ext]=di_fixed_modes(Fext,G_e,H_e,N,ContStructure_centralized,3);
% Decentralized
[Dfm_ext]=di_fixed_modes(Aext,B_e,C_e,N,ContStructure_decentralized,3);
[Dfm_DT_ext]=di_fixed_modes(Fext,G_e,H_e,N,ContStructure_decentralized,3);

% Distributed1, BETTER
[Distfm_ext]=di_fixed_modes(Aext,B_e,C_e,N,ContStructure_distributedString,3);
[Distfm_DT_ext]=di_fixed_modes(Fext,G_e,H_e,N,ContStructure_distributedString,3); 

% Distributed2
[Distfm2_ext]=di_fixed_modes(Aext,B_e,C_e,N,ContStructure_distributedString2,3);
[Distfm2_DT_ext]=di_fixed_modes(Fext,G_e,H_e,N,ContStructure_distributedString2,3); 


%% State-feedback control analysis and Design:

% % STABILIZING CONTROL LAW:
% 
% % CENTRALIZED
% [K_c,rho_c,feas_c]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_centralized);
% [K_c_DT,rho_c_DT,feas_c_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_centralized);
% 
% % DECENTRALIZED
% [K_De,rho_De,feas_De]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_decentralized);
% [K_De_DT,rho_De_DT,feas_De_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_decentralized);
% 
% % DISTRIBUTED
% [K_string,rho_string,feas_string]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_distributedString);
% [K_string_DT,rho_string_DT,feas_string_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_distributedString);
% 
% % DISTRIBUTED2
% [K_string2,rho_string2,feas_string2]=LMI_CT_DeDicont(Atot,B,C,N,ContStructure_distributedString2);
% [K_string2_DT,rho_string2_DT,feas_string2_DT]=LMI_DT_DeDicont(Ftot,G,H,N,ContStructure_distributedString2);

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

%% EXTENDED
% CENTRALIZED
[K_c_opt1_ext,rho_c_opt1_ext,feas_c_opt1_ext]=LMI_CT_opt1(Aext,B_e,C_e,N,ContStructure_centralized,rho_desired,alfa_desired);
% DECENTRALIZED
[K_De_opt1_ext,rho_De_opt1_ext,feas_De_opt1_ext]=LMI_CT_opt1(Aext,B_e,C_e,N,ContStructure_decentralized,rho_desired,alfa_desired);
% DISTRIBUTED
[K_string_opt1_ext,rho_string_opt1_ext,feas_string_opt1_ext]=LMI_CT_opt1(Aext,B_e,C_e,N,ContStructure_distributedString,rho_desired,alfa_desired);
% DISTRIBUTED2
[K_string2_opt1_ext,rho_string2_opt1_ext,feas_string2_opt1_ext]=LMI_CT_opt1(Aext,B_e,C_e,N,ContStructure_distributedString2,rho_desired,alfa_desired);



%% DISCRETE TIME:
% first compare with same performance request, than try to optimally design
% the controller for each control structure, according to its limitations

% PERFORMANCE
rho_desired_DT = 0.1; % limit the circle region radius of DT eigenvalus
alfa_desired_DT = -0.6; % place the circle center where eigs are constrained 

% CENTRALIZED
[K_c_opt1_DT,rho_c_opt1_DT,feas_c_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_centralized,rho_desired_DT,alfa_desired_DT);
% DECENTRALIZED
[K_De_opt1_DT,rho_De_opt1_DT,feas_De_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_decentralized,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED
[K_string_opt1_DT,rho_string_opt1_DT,feas_string_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_distributedString,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED2
[K_string2_opt1_DT,rho_string2_opt1_DT,feas_string2_opt1_DT]=LMI_DT_opt1(Ftot,G,H,N,ContStructure_distributedString2,rho_desired_DT,alfa_desired_DT);

%% EXTENDED
% CENTRALIZED
[K_c_opt1_DT_ext,rho_c_opt1_DT_ext,feas_c_opt1_DT_ext]=LMI_DT_opt1(Fext,G_e,H_e,N,ContStructure_centralized,rho_desired_DT,alfa_desired_DT);
% DECENTRALIZED
[K_De_opt1_DT_ext,rho_De_opt1_DT_ext,feas_De_opt1_DT_ext]=LMI_DT_opt1(Fext,G_e,H_e,N,ContStructure_decentralized,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED
[K_string_opt1_DT_ext,rho_string_opt1_DT_ext,feas_string_opt1_DT_ext]=LMI_DT_opt1(Fext,G_e,H_e,N,ContStructure_distributedString,rho_desired_DT,alfa_desired_DT);
% DISTRIBUTED2
[K_string2_opt1_DT_ext,rho_string2_opt1_DT_ext,feas_string2_opt1_DT_ext]=LMI_DT_opt1(Fext,G_e,H_e,N,ContStructure_distributedString2,rho_desired_DT,alfa_desired_DT);

%% SIMULATIONS
% open loop
% stabilizing
% optimal design 

%-------------------------------------------------------------------------
% OPEN LOOP SIMULATION:
Tf = 10;
T = [0:0.01:Tf];
n = length(Atot); % number of states
% 
x0 = 0.1*randn(n,1); %random initial states
%x0=[-0.039055106466591;-0.003966891777107;-0.088098092380076];
%x0 = [-0.109, -0.039, -0.012]';
% u=[1, 1]'; % unitary step 
% 
% % CONTINUOUS
% i = 0;
% for t=T
%     i = i+1;
%     x_free(:,i) = expm(Atot*t)*x0;
%     x_forced(:,i)=Ctot/Atot*(expm(Atot*t)-eye(n))*Btot*u;
% end
% x = x_free + x_forced;
% x_bar = -Ctot/Atot*Btot*u; % unitary step equilibrium point
% 
% % DISCRETE
steps = [0:Tf/h];
% i = 0;
% for k=steps
%     i=i+1;
%     x_free_DT(:,i)=Htot*(Ftot^k)*x0;
%     x_forced_DT(:,i)=Htot/(eye(n)-Ftot)*(eye(n)-Ftot^k)*Gtot*u;
% end
% x_DT = x_free_DT + x_forced_DT;
% x_bar_DT=Htot/(eye(n)-Ftot)*Gtot*u; % unitary step equilibrium point
% 
% 
% figure('Name','OPEN LOOP STATES')
% % Free Motion(CONTINUOUS):
% subplot(2,2,1)
% plot(T,x_free(1,:), 'r');
% hold on
% plot(T,x_free(2,:), 'b');
% plot(T,x_free(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('Continuous-time free response (OPEN-LOOP)')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% % Forced Motion (CONTINUOUS):
% subplot(2,2,3)
% plot(T,x(1,:), 'r');
% hold on
% plot(T,x(2,:), 'b');
% plot(T,x(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('Continuous-time step response (OPEN-LOOP)')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% % Free Motion(DISCRETE):
% subplot(2,2,2)
% plot(steps*h,x_free_DT(1,:), 'r');
% hold on
% plot(steps*h,x_free_DT(2,:), 'b');
% plot(steps*h,x_free_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('Discrete-time free response (OPEN-LOOP)')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% % Forced Motion (DISCRETE):
% subplot(2,2,4)
% plot(steps*h,x_DT(1,:), 'r');
% hold on
% plot(steps*h,x_DT(2,:), 'b');
% plot(steps*h,x_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('Discrete-time step response (OPEN-LOOP)')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% sgtitle(" OPEN LOOP DYNAMIC ")
% 
% 
% %-------------------------------------------------------------------------
% % CLOSED LOOP SIMULATION 
% 
% %% STABILIZING CONTROL LAW
% i = 0;
% %CONTINUOUS
% for t=T
%     i=i+1;
%     x_c(:,i)=expm((Atot+Btot*K_c)*t)*x0;
%     x_De(:,i)=expm((Atot+Btot*K_De)*t)*x0;
%     x_string(:,i)=expm((Atot+Btot*K_string)*t)*x0;
%     x_string2(:,i)=expm((Atot+Btot*K_string2)*t)*x0;
% end
% u_c = K_c*x_c;
% u_De = K_De*x_De;
% u_string = K_string*x_string;
% u_string2 = K_string2*x_string2;
% 
% 
% %DICRETE
% i = 0;
% for k=steps
%     i = i+1;
%     x_c_DT(:,i)=((Ftot+Gtot*K_c_DT)^k)*x0;
%     x_De_DT(:,i)=((Ftot+Gtot*K_De_DT)^k)*x0;
%     x_string_DT(:,i)=((Ftot+Gtot*K_string_DT)^k)*x0;
%     x_string2_DT(:,i)=((Ftot+Gtot*K_string2_DT)^k)*x0;
% end
% u_c_DT = K_c_DT*x_c_DT;
% u_De_DT = K_De_DT*x_De_DT;
% u_string_DT = K_string_DT*x_string_DT;
% u_string2_DT = K_string2_DT*x_string2_DT;
% 
% % CENTRALIZED:
% figure('Name',' CENTRALIZED STABILIZING')
% subplot(2,2,1)
% plot(T,x_c(1,:), 'r');
% hold on
% plot(T,x_c(2,:), 'b');
% plot(T,x_c(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('CONTINUOUS: Centralized Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,2)
% plot(steps*h,x_c_DT(1,:), 'r');
% hold on
% plot(steps*h,x_c_DT(2,:), 'b');
% plot(steps*h,x_c_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Centralized Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,3)
% plot(T,u_c(1,:), 'm');
% hold on
% plot(T,u_c(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('CONTINUOUS: Centralized Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% 
% subplot(2,2,4)
% plot(steps*h,u_c_DT(1,:), 'm');
% hold on
% plot(steps*h,u_c_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Centralized Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" CENTRALIZED STABILIZING CONTROL ")
% 
% % DECENTRALIZED
% figure('Name',' DECENTRALIZED STABILIZING')
% subplot(2,2,1)
% plot(T,x_De(1,:), 'r');
% hold on
% plot(T,x_De(2,:), 'b');
% plot(T,x_De(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('CONTINUOUS: Decentralized Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,2)
% plot(steps*h,x_De_DT(1,:), 'r');
% hold on
% plot(steps*h,x_De_DT(2,:), 'b');
% plot(steps*h,x_De_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Decentralized Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,3)
% plot(T,u_De(1,:), 'm');
% hold on
% plot(T,u_De(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('CONTINUOUS: Decentralized Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% 
% subplot(2,2,4)
% plot(steps*h,u_De_DT(1,:), 'm');
% hold on
% plot(steps*h,u_De_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Decentralized Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DECENTRALIZED STABILIZING CONTROL ")
% 
% % DISTRIBUTED 
% 
% figure('Name',' DISTRIBUTED STABILIZING')
% subplot(2,2,1)
% plot(T,x_string(1,:), 'r');
% hold on
% plot(T,x_string(2,:), 'b');
% plot(T,x_string(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('CONTINUOUS: Distributed Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,2)
% plot(steps*h,x_string_DT(1,:), 'r');
% hold on
% plot(steps*h,x_string_DT(2,:), 'b');
% plot(steps*h,x_string_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Distributed Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,3)
% plot(T,u_string(1,:), 'm');
% hold on
% plot(T,u_string(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('CONTINUOUS: Distributed Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% 
% subplot(2,2,4)
% plot(steps*h,u_string_DT(1,:), 'm');
% hold on
% plot(steps*h,u_string_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Distributed Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DISTRIBUTED STABILIZING CONTROL ")
% 
% % DISTRIBUTED 2
% 
% figure('Name',' DISTRIBUTED STABILIZING')
% subplot(2,2,1)
% plot(T,x_string2(1,:), 'r');
% hold on
% plot(T,x_string2(2,:), 'b');
% plot(T,x_string2(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('CONTINUOUS: Distributed 2 Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,2)
% plot(steps*h,x_string2_DT(1,:), 'r');
% hold on
% plot(steps*h,x_string2_DT(2,:), 'b');
% plot(steps*h,x_string2_DT(3,:), 'g');
% grid on
% xlabel('t [s]')
% ylabel('Dh [m]')
% title('DISCRETE: Distributed 2 Stabilizing States')
% legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')
% 
% subplot(2,2,3)
% plot(T,u_string2(1,:), 'm');
% hold on
% plot(T,u_string2(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('CONTINUOUS: Distributed 2 Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% 
% subplot(2,2,4)
% plot(steps*h,u_string2_DT(1,:), 'm');
% hold on
% plot(steps*h,u_string2_DT(2,:), 'b');
% grid on
% xlabel('t [s]')
% ylabel('u [m^{3}/s]')
% title('DISCRETE: Distributed 2 Stabilizing Control')
% legend('u_{1}', 'u_{2}')
% 
% sgtitle(" DISTRIBUTED 2 STABILIZING CONTROL ")
% 
% figure('Name','PZPLOT STABILIZING CONTINUOUS')
% subplot(2,3,1)
% pzmap(ss(Atot,Btot,Ctot,0))
% title("Open Loop")
% subplot(2,3,2)
% pzmap(ss(Atot+Btot*K_c,zeros(3,2),Ctot,0))
% title("Centralized")
% subplot(2,3,3)
% pzmap(ss(Atot+Btot*K_De,zeros(3,2),Ctot,0))
% title("Decentralized")
% subplot(2,3,4)
% pzmap(ss(Atot+Btot*K_string,zeros(3,2),Ctot,0))
% title("Distributed")
% subplot(2,3,5)
% pzmap(ss(Atot+Btot*K_string2,zeros(3,2),Ctot,0))
% title("Distributed 2")
% 
% figure('Name','PZPLOT STABILIZING DISCRETE')
% subplot(2,3,1)
% pzmap(ss(Ftot,Gtot,Htot,0))
% title("Open Loop")
% subplot(2,3,2)
% pzmap(ss(Ftot+Gtot*K_c_DT,zeros(3,2),Htot,0))
% title("Centralized")
% subplot(2,3,3)
% pzmap(ss(Ftot+Gtot*K_De_DT,zeros(3,2),Htot,0))
% title("Decentralized")
% subplot(2,3,4)
% pzmap(ss(Ftot+Gtot*K_string_DT,zeros(3,2),Htot,0))
% title("Distributed")
% subplot(2,3,5)
% pzmap(ss(Ftot+Gtot*K_string2_DT,zeros(3,2),Htot,0))
% title("Distributed 2")
%---------------------------------------------------------------------
%% REDUCING OVERSHOOT AND SPEED UP CONTROL, WITH LIMITED CONTROL ACTION (OPT1)
n_ext = length(Aext);
%x0_ext = 0.1*randn(n,1); %random initial states
x0_ext = [x0(1,1), 0, x0(2,1), x0(3,1), 0]';

i = 0;
% CONTINUOUS
for t=T
    i=i+1;
    x_c_opt1(:,i)=expm((Atot+Btot*K_c_opt1)*t)*x0;
    x_De_opt1(:,i)=expm((Atot+Btot*K_De_opt1)*t)*x0;
    x_string_opt1(:,i)=expm((Atot+Btot*K_string_opt1)*t)*x0;
    x_string2_opt1(:,i)=expm((Atot+Btot*K_string2_opt1)*t)*x0;
   
    % extended states [h1, u1, h2, h3, u2]
    x_c_opt1_ext(:,i)=expm((Aext+Bext*K_c_opt1_ext)*t)*x0_ext;
    x_De_opt1_ext(:,i)=expm((Aext+Bext*K_De_opt1_ext)*t)*x0_ext;
    x_string_opt1_ext(:,i)=expm((Aext+Bext*K_string_opt1_ext)*t)*x0_ext;
    x_string2_opt1_ext(:,i)=expm((Aext+Bext*K_string2_opt1_ext)*t)*x0_ext;
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

%% RATE LIMIT
% CENTRALIZED:
figure('Name',' CENTRALIZED SPEED+OVERSHOOT RATE LIMIT ')
subplot(1,2,1)
plot(T,x_c_opt1_ext(1,:), 'r');
hold on
plot(T,x_c_opt1_ext(3,:), 'b');
plot(T,x_c_opt1_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Centralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,x_c_opt1_ext(2,:), 'm');
hold on
plot(T,x_c_opt1_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Centralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" CENTRALIZED CONTROL EXT")

% DECENTRALIZED
figure('Name',' DECENTRALIZED SPEED+OVERSHOOT RATE LIMIT')
subplot(1,2,1)
plot(T,x_De_opt1_ext(1,:), 'r');
hold on
plot(T,x_De_opt1_ext(3,:), 'b');
plot(T,x_De_opt1_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Decentralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,x_De_opt1_ext(2,:), 'm');
hold on
plot(T,x_De_opt1_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Decentralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DECENTRALIZED CONTROL EXT")

% DISTRIBUTED 

figure('Name',' DISTRIBUTED SPEED+OVERSHOOT RATE LIMIT')
subplot(1,2,1)
plot(T,x_string_opt1_ext(1,:), 'r');
hold on
plot(T,x_string_opt1_ext(3,:), 'b');
plot(T,x_string_opt1_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,x_string_opt1_ext(2,:), 'm');
hold on
plot(T,x_string_opt1_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED CONTROL EXT")

% DISTRIBUTED 2

figure('Name',' DISTRIBUTED SPEED+OVERSHOOT RATE LIMIT')
subplot(1,2,1)
plot(T,x_string2_opt1_ext(1,:), 'r');
hold on
plot(T,x_string2_opt1_ext(3,:), 'b');
plot(T,x_string2_opt1_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('CONTINUOUS: Distributed 2 States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(T,x_string2_opt1_ext(2,:), 'm');
hold on
plot(T,x_string2_opt1_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('CONTINUOUS: Distributed 2 Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED 2 CONTROL EXT")

% figure('Name','PZPLOT CONTINUOUS')
% subplot(2,3,1)
% pzmap(ss(Atot,Btot,Ctot,0))
% title("Open Loop")
% subplot(2,3,2)
% pzmap(ss(Atot+Btot*K_c_opt1,zeros(3,2),Ctot,0))
% title("Centralized")
% subplot(2,3,3)
% pzmap(ss(Atot+Btot*K_De_opt1,zeros(3,2),Ctot,0))
% title("Decentralized")
% subplot(2,3,4)
% pzmap(ss(Atot+Btot*K_string_opt1,zeros(3,2),Ctot,0))
% title("Distributed")
% subplot(2,3,5)
% pzmap(ss(Atot+Btot*K_string2_opt1,zeros(3,2),Ctot,0))
% title("Distributed 2")

%% PLACE EIGENVALUES IN A CIRCLE REGION WITH LIMITED RADIUS AND CHOSEN CENTER

%DICRETE
i = 0;
for k=steps
    i = i+1;
    x_c_opt1_DT(:,i)=((Ftot+Gtot*K_c_opt1_DT)^k)*x0;
    x_De_opt1_DT(:,i)=((Ftot+Gtot*K_De_opt1_DT)^k)*x0;
    x_string_opt1_DT(:,i)=((Ftot+Gtot*K_string_opt1_DT)^k)*x0;
    x_string2_opt1_DT(:,i)=((Ftot+Gtot*K_string_opt1_DT)^k)*x0;

    x_c_opt1_DT_ext(:,i)=((Fext+Gext*K_c_opt1_DT_ext)^k)*x0_ext;
    x_De_opt1_DT_ext(:,i)=((Fext+Gext*K_De_opt1_DT_ext)^k)*x0_ext;
    x_string_opt1_DT_ext(:,i)=((Fext+Gext*K_string_opt1_DT_ext)^k)*x0_ext;
    x_string2_opt1_DT_ext(:,i)=((Fext+Gext*K_string2_opt1_DT_ext)^k)*x0_ext;
    
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

%% EXTENDED
% CENTRALIZED:
figure('Name',' CENTRALIZED DISCRETE OPT1 EXT')
subplot(1,2,1)
plot(steps*h,x_c_opt1_DT_ext(1,:), 'r');
hold on
plot(steps*h,x_c_opt1_DT_ext(3,:), 'b');
plot(steps*h,x_c_opt1_DT_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Centralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,x_c_opt1_DT_ext(2,:), 'm');
hold on
plot(steps*h,x_c_opt1_DT_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Centralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" CENTRALIZED CONTROL EXTENDED")

% DECENTRALIZED
figure('Name',' DECENTRALIZED DISCRETE OPT1 EXT')
subplot(1,2,1)
plot(steps*h,x_De_opt1_DT_ext(1,:), 'r');
hold on
plot(steps*h,x_De_opt1_DT_ext(3,:), 'b');
plot(steps*h,x_De_opt1_DT_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Decentralized States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,x_De_opt1_DT_ext(2,:), 'm');
hold on
plot(steps*h,x_De_opt1_DT_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Decentralized Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DECENTRALIZED CONTROL EXTEDNED")

% DISTRIBUTED 

figure('Name',' DISTRIBUTED DISCRETE OPT1 EXT')
subplot(1,2,1)
plot(steps*h,x_string_opt1_DT_ext(1,:), 'r');
hold on
plot(steps*h,x_string_opt1_DT_ext(3,:), 'b');
plot(steps*h,x_string_opt1_DT_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Distributed States ')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,x_string_opt1_DT_ext(2,:), 'm');
hold on
plot(steps*h,x_string_opt1_DT_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Distributed Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED CONTROL DISCRETE EXTENDED")

% DISTRIBUTED 2

figure('Name',' DISTRIBUTED 2 DISCRETE OPT1 EXT')
subplot(1,2,1)
plot(steps*h,x_string2_opt1_DT_ext(1,:), 'r');
hold on
plot(steps*h,x_string2_opt1_DT_ext(3,:), 'b');
plot(steps*h,x_string2_opt1_DT_ext(4,:), 'g');
grid on
xlabel('t [s]')
ylabel('Dh [m]')
title('DISCRETE: Distributed 2 States')
legend('Dh_{1}', 'Dh_{2}', 'Dh_{3}')

subplot(1,2,2)
plot(steps*h,x_string2_opt1_DT_ext(2,:), 'm');
hold on
plot(steps*h,x_string2_opt1_DT_ext(5,:), 'b');
grid on
xlabel('t [s]')
ylabel('u [m^{3}/s]')
title('DISCRETE: Distributed 2 Control')
legend('u_{1}', 'u_{2}')

sgtitle(" DISTRIBUTED 2 CONTROL DISCRETE EXTENDED")
 
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

if rho_ext < 0
    disp(['-  Open-loop EXTENDED: Asyptotically stable,  Spectral abscissa = ',num2str(rho_ext)]);
elseif rho_ext == 0
    disp(['-  Open-loop EXTENDED: Simply stable,  Spectral abscissa = ',num2str(rho_ext)]);
else
    disp(['-  Open-loop EXTENDED: Unstable,  Spectral abscissa = ',num2str(rho_ext)]);
end


% disp(['-  Centralized: Feasibility=',num2str(feas_c),', rho=',num2str(rho_c),', FM=',num2str(cfm),'.'])
% disp(['-  Decentralized: Feasibility=',num2str(feas_De),', rho=',num2str(rho_De),', FM=',num2str(Dfm),'.'])
% disp(['-  Distributed (string): Feasibility=',num2str(feas_string),', rho=',num2str(rho_string),', FM=',num2str(Distfm),'.'])
% disp(['-  Distributed (string 2): Feasibility=',num2str(feas_string2),', rho=',num2str(rho_string2),', FM=',num2str(Distfm2),'.'])

disp(['-  Centralized OPT1: Feasibility=',num2str(feas_c_opt1),', rho=',num2str(rho_c_opt1),', FM=',num2str(cfm),'.'])
disp(['-  Decentralized OPT1: Feasibility=',num2str(feas_De_opt1),', rho=',num2str(rho_De_opt1),', FM=',num2str(Dfm),'.'])
disp(['-  Distributed (string) OPT1: Feasibility=',num2str(feas_string_opt1),', rho=',num2str(rho_string_opt1),', FM=',num2str(Distfm),'.'])
disp(['-  Distributed (string 2) OPT1: Feasibility=',num2str(feas_string2_opt1),', rho=',num2str(rho_string2_opt1),', FM=',num2str(Distfm2),'.'])
disp('EXTENDED')
disp(['-  Centralized OPT1 EXTENDED: Feasibility=',num2str(feas_c_opt1_ext),', rho=',num2str(rho_c_opt1_ext),', FM=',num2str(cfm_ext),'.'])
disp(['-  Decentralized OPT1 EXTENDED: Feasibility=',num2str(feas_De_opt1_ext),', rho=',num2str(rho_De_opt1_ext),', FM=',num2str(Dfm_ext),'.'])
disp(['-  Distributed (string) OPT1 EXTENDED: Feasibility=',num2str(feas_string_opt1_ext),', rho=',num2str(rho_string_opt1_ext),', FM=',num2str(Distfm_ext),'.'])
disp(['-  Distributed (string 2) OPT1 EXTENDED: Feasibility=',num2str(feas_string2_opt1_ext),', rho=',num2str(rho_string2_opt1_ext),', FM=',num2str(Distfm2_ext),'.'])

disp('Results (Discrete-time):')

if rho_DT < 1
    disp(['-  Open-loop: Asyptotically stable,  Spectral radius = ',num2str(rho_DT)]);
elseif rho_DT == 1
    disp(['-  Open-loop: Simply stable,  Spectral radius = ',num2str(rho_DT)]);
else
    disp(['-  Open-loop: Unstable,  Spectral radius = ',num2str(rho_DT)]);
end

if rho_DT_ext < 1
    disp(['-  Open-loop EXTENDED: Asyptotically stable,  Spectral radius = ',num2str(rho_DT_ext)]);
elseif rho_DT_ext == 1
    disp(['-  Open-loop EXTENDED: Simply stable,  Spectral radius = ',num2str(rho_DT_ext)]);
else
    disp(['-  Open-loop EXTENDED: Unstable,  Spectral radius = ',num2str(rho_DT_ext)]);
end

% disp(['-  Centralized: Feasibility=',num2str(feas_c_DT),', rho=',num2str(rho_c_DT),', FM=',num2str(cfm_DT),'.'])
% disp(['-  Decentralized: Feasibility=',num2str(feas_De_DT),', rho=',num2str(rho_De_DT),', FM=',num2str(Dfm_DT),'.'])
% disp(['-  Distributed (string): Feasibility=',num2str(feas_string_DT),', rho=',num2str(rho_string_DT),', FM=',num2str(Distfm_DT),'.'])
% disp(['-  Distributed (string 2): Feasibility=',num2str(feas_string2_DT),', rho=',num2str(rho_string2_DT),', FM=',num2str(Distfm2_DT),'.'])

disp(['-  Centralized OPT1: Feasibility=',num2str(feas_c_opt1_DT),', rho=',num2str(rho_c_opt1_DT),', FM=',num2str(cfm_DT),'.'])
disp(['-  Decentralized OPT1: Feasibility=',num2str(feas_De_opt1_DT),', rho=',num2str(rho_De_opt1_DT),', FM=',num2str(Dfm_DT),'.'])
disp(['-  Distributed (string) OPT1: Feasibility=',num2str(feas_string_opt1_DT),', rho=',num2str(rho_string_opt1_DT),', FM=',num2str(Distfm_DT),'.'])
disp(['-  Distributed (string 2) OPT1: Feasibility=',num2str(feas_string2_opt1_DT),', rho=',num2str(rho_string2_opt1_DT),', FM=',num2str(Distfm2_DT),'.'])
disp('EXTENDED')
disp(['-  Centralized OPT1 EXTENDED: Feasibility=',num2str(feas_c_opt1_DT_ext),', rho=',num2str(rho_c_opt1_DT_ext),', FM=',num2str(cfm_DT_ext),'.'])
disp(['-  Decentralized OPT1 EXTENDED: Feasibility=',num2str(feas_De_opt1_DT_ext),', rho=',num2str(rho_De_opt1_DT_ext),', FM=',num2str(Dfm_DT_ext),'.'])
disp(['-  Distributed (string) OPT1 EXTENDED: Feasibility=',num2str(feas_string_opt1_DT_ext),', rho=',num2str(rho_string_opt1_DT_ext),', FM=',num2str(Distfm_DT_ext),'.'])
disp(['-  Distributed (string 2) OPT1 EXTENDED: Feasibility=',num2str(feas_string2_opt1_DT_ext),', rho=',num2str(rho_string2_opt1_DT_ext),', FM=',num2str(Distfm2_DT_ext),'.'])