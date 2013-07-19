function [Omega, Ct, Cp] = turbineDrivetrainModel(x,u,wt,env,DT)
% Parameters

R=wt.rotor.radius;
I=wt.rotor.inertia;
Rho=env.rho;

% Definitons etc.

Omega= x(1); Ve= x(2);
Beta= u(1); Tg= u(2);

% Algorithm

Lambda= Omega*R/Ve;
Cp=embeddedinterpolCPtab(Beta,Lambda,wt.cp.table,wt.cp.beta,wt.cp.tsr);
Ct=embeddedinterpolCTtab(Beta,Lambda,wt.ct.table,wt.ct.beta,wt.ct.tsr);
Tr= 1/2*Rho*pi*R^2*Ve^3*Cp/Omega;
%Tg= P/wt.mu/Omega;
%Tg= P/Omega;
Omega=Omega+DT*(Tr-Tg)/I; %Integration method: Forward Euler
