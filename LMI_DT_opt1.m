function [K,rho,feas]=LMI_DT_opt1(F,G,H,N,ContStruc,rho_max, alfa)
% Computes, using LMIs, the distributed "state feedback" control law for the discrete-time system, with reference to the control
% information structure specified by 'ContStruc'.

% Objective to:
% 1) constrain inside a circle of radius rho (to control the responce speed)
% 2) place the circle region in the  real axis cohordinate -alfa (try to
%  place in positive real axis to have positive eig, to avoid oscillations of sign) 
% 3) minimize the control effort

% Inputs:
% - F: system matrix.
% - G: input matrices (i.e., G{1},..., G{N} are the input matrices of the decomposed system, one for each channel).
% - H: output matrices  (i.e., H{1},..., H{N} are the output matrices of the decomposed system, one for each channel, where [Hdec{1}',...,
% Hdec{N}']=I).
% - N: number of subsystems.
% - ContStruc: NxN matrix that specifies the information structure
% constraints (ContStruc(i,j)=1 if communication is allowed between channel
% j to channel i, ContStruc(i,j)=0 otherwise).
% - rho_max: maximum eigenvalue region radius 
% - alfa: desired center of circular region 

% Output:
% - K: structured control gain
% - rho: spectral radius of matrix (F+G*K) - note that [H{1}',...,
% H{N}']=I
% - feas: feasibility of the LMI problem (=0 if yes)

Gtot=[];
for i=1:N
    m(i)=size(G{i},2);
    n(i)=size(H{i},1);
    Gtot=[Gtot,G{i}];
end
ntot=size(F,1);
mtot=sum(m);

yalmip clear

if ContStruc==ones(N,N)
    % Centralized design
    P=sdpvar(ntot);
    L=sdpvar(mtot,ntot);
else
    % Dentralized/distributed design
    P=[];
    L=sdpvar(mtot,ntot);
    minc=0;
    for i=1:N
        P=blkdiag(P,sdpvar(n(i)));
        ninc=0;
        for j=1:N
            if ContStruc(i,j)==0
                L(minc+1:minc+m(i),ninc+1:ninc+n(j))=zeros(m(i),n(j));
            end
            ninc=ninc+n(j);
        end
        minc=minc+m(i);
    end
end

% contraint to stabilize but contraining the poles region into a circle
% centered in alfa with radius rho_max
LMIconstr=[[(rho_max^2-alfa^2)*P-alfa*(F*P+P*F'+Gtot*L+L'*Gtot')  F*P+Gtot*L;
             (F*P+Gtot*L)'                                       P]>=1e-2*eye(ntot*2)];


% Try to reduce control action and its rate:
alfaL = sdpvar(1,1);
alfaY = sdpvar(1,1);


LMIconstr = LMIconstr + [[alfaL*eye(ntot)   L';
                              L             eye(mtot)] >=1e-2*eye(ntot+mtot)];

LMIconstr = LMIconstr + [[alfaY*eye(ntot)   eye(ntot);
                              eye(ntot)          P] >= 1e-2*eye(2*ntot)];


J = 10*alfaY + 0.01*alfaL; % cost function to minimize


options=sdpsettings('solver','sedumi');
result=optimize(LMIconstr,J,options);
feas=result.problem;
L=double(L);
P=double(P);

K=L/P;
rho=max(abs(eig(F+Gtot*K)));