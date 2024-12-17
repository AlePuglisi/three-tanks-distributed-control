function [K,rho,feas]=LMI_CT_H2(A,B,C,Bw,Cz,Dz,N,ContStruc)
% Computes, using LMIs, the distributed "state feedback" control law for the continuous-time system, with reference to the control
% information structure specified by 'ContStruc'.
% 
% Objective to:
% find the best control action and state convergence trade-off

% Inputs:
% - A: system matrix.
% - B: input matrices (i.e., B{1},..., B{N} are the input matrices of the decomposed system, one for each channel).
% - C: output matrices  (i.e., C{1},..., C{N} are the output matrices of the decomposed system, one for each channel, where [Cdec{1}',...,
% Cdec{N}']=I).
% - N: number of subsystems.
% - ContStruc: NxN matrix that specifies the information structure
% constraints (ContStruc(i,j)=1 if communication is allowed between channel
% j to channel i, ContStruc(i,j)=0 otherwise).
% - rhomax: maximum desired spectral abscissa, which characterize the system speed 
% - alfa: parameter to define the dumping factor limits, which characterize
% the response overshoot, [cos(alfa) < dumpingfactor < 1]

% Outputs:
% - K: structured control gain
% - rho: spectral abscissa of matrix (A+B*K) - note that [C{1}',...,
% C{N}']=I
% - feas: feasibility of the LMI problem (=0 if yes)

Btot=[];
for i=1:N
    m(i)=size(B{i},2);
    n(i)=size(C{i},1);
    Btot=[Btot,B{i}];
end

ntot=size(A,1);
mtot=sum(m);

yalmip clear

if ContStruc==ones(N,N)
    % Centralized design
    Y=sdpvar(ntot);
    L=sdpvar(mtot,ntot);
else
    % Decentralized/distributed design
    Y=[];
    L=sdpvar(mtot,ntot);
    minc=0;
    for i=1:N
        Y=blkdiag(Y,sdpvar(n(i)));
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

S = sdpvar(ntot+mtot);

LMIconstr = [[A*Y+Btot*L+Y*A'+L'*Btot'+Bw*Bw']<=-1e-2*eye(ntot)];
LMIconstr = LMIconstr + [[S             Cz*Y+Dz*L;
                          L'*Dz'+Y*Cz'      Y] >= 1e-2*eye(ntot+(ntot+mtot)) ];

J = trace(S);

% Optimization 
options=sdpsettings('solver','sedumi');
result=optimize(LMIconstr,J,options);
feas=result.problem;
L=double(L);
Y=double(Y);

K=L/Y;
rho=max(real(eig(A+Btot*K)));
