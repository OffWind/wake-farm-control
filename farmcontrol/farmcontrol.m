function farmcontrolv3
global wind DT
load NREL5mw.mat
DT=0.0125; % time step
Tstart=0; % time start
Tend=400; % time end

% farm properties
parm.N=4; % number of turbines in farm
farmcontrol=1; % enable wind farm control
P_demand=10e6; % some number

%turbine properties
parm.rho=1.227; %air density
parm.radius=63*ones(1,4); % rotor radius (NREL5MW)
parm.rated=5e6*ones(1,4); %rated power (NREL5MW)
parm.ratedSpeed=1.26;
%parm.Cp=0.45*ones(1,4); %max cp (NREL5MW)
parm.Cp = max(max(wt.cp.table))*ones(1,4);


%Pitch control
ee=0; %blade pitch integrator
Ki=0.008068634*360/2/pi; % integral gain (NREL5MW)
Kp=0.01882681*360/2/pi; % proportional gain (NREL5MW)
PC_MaxPit=90;
PC_MinPit=0;

%region control NREL
VS_Rgn2Sp     =      91.21091;
VS_CtInSp     =      70.16224;
VS_RtGnSp     =     121.6805;
VS_Rgn2K      =       2.332287;  
% load dummy wind
wind=load('wind.mat');

% turbine initial conditions
omega0=1.26;
beta0=0;
power0=5e6;
x=[omega0*ones(parm.N,1) wind.wind(1,2)*ones(parm.N,1)];
Ct=ones(1,4);
u=[beta0*ones(parm.N,1) power0*ones(parm.N,1)];
P_ref=zeros(parm.N,((Tend-Tstart)/DT));

for i=2:((Tend-Tstart)/DT) %At each sample time(DT) from Tstart to Tend
    
    v_nac(:,i)=wake(Ct(i-1,:),i);
    x(:,2) = v_nac(:,i);
    %Farm control
    if farmcontrol==1
        [P_ref(:,i) Pa(:,i)]  = Farm_Controller(v_nac(:,i),P_demand,parm);
    end
    
    %Turbine control
    for j=1:parm.N
        if ( (   x(j,1) >= parm.ratedSpeed ) || (  u(j,1) >= 1 ) )   % We are in region 3 - power is constant
            u(j,2) = P_ref(j,i)./x(j,1);
        elseif ( x(j,1)*97 <= VS_CtInSp )                            %! We are in region 1 - torque is zero
            u(j,2) = 0.0;
        else                                                         %! We are in region 2 - optimal torque is proportional to the square of the generator speed
            u(j,2) = VS_Rgn2K*x(j,1)*x(j,1)*97^2*(60/2/pi)^2;
        end
    end
    
    
    Mg(:,i)=u(:,2);
    
    e=97*(omega0*ones(parm.N,1)-x(:,1));
    ee =ee-DT*e;
    ee=min( max( ee, PC_MinPit/Ki), PC_MaxPit/Ki); 
                                       
    u(:,1)=-Kp*DT*e+Ki*ee;
    for j=1:parm.N
        u(j,1)=min( max( u(j,1), PC_MinPit), PC_MaxPit);
    end
    
    beta(:,i)=u(:,1);
    
    %Turbine dynamics
    for j=1:parm.N
        [x(j,1), Ct(i,j), Cp(i,j)]=Turbine_Drivetrain(x(j,:),u(j,:),wt,env);
    end
    Omega(:,i)=x(:,1);
    
end
out=[DT*[0:((Tend-Tstart)/DT)-1]' v_nac' Omega' beta' P_ref' Ct Cp Pa' Mg'];
save offwind.mat out
figure(1)
plot(out(:,1),out(:,6:9)) % rotor speed
title('Rotor speed')
figure(2)
plot(out(:,1),out(:,10:13)) % Blade pitch angle
title('Blade pitch angle')
figure(3)
plot(out(:,1),out(:,14:17)) % Power reference
title('Power reference')
figure(4)
plot(out(:,1),out(:,30:33)) % generator torque reference
title('Generator torque reference')
plot(out(:,1),out(:,26:29)) % Available power
title('Available power')
figure(5)
plot(out(:,1),out(:,2:5)) % Wind speed
title('Wind speed')
save slet_wrkspc
plot(out(:,1),sum(out(:,14:17),2),out(:,1),sum(out(:,26:29),2),out(:,1),sum(Mg.*Omega)'); %Ttotal power demand
title('Total power')
legend('Demand','Available','Actual')



function v_nac=wake(Ct,i)
% run wake simulation as function of Ct of each turbine
global wind
v_nac=[.8 .7 .6 .5]*wind.wind(i,2);

function [P_ref, P_a]  = Farm_Controller(v_nac,P_demand,parm)

%P_ref is a vector of power refenreces for tehe wind turbine with dimension 1xN
%v_nac is a vector of wind speed at each wind turbine with dimension 1xN
%P_demand is a scale of the wind farm power demand.
%parm is a struct of wind turbine parameters e.g. NREL5MW

rho=parm.rho; %air density for each wind turbine(probably the same for all)
R=parm.radius; %rotor radius for each wind turbine(NREL.r=63m)
rated=parm.rated; %Rated power for each wind turbine(NREL.Prated=5MW)
N=parm.N; %Number of turbines in windfarm
Cp=parm.Cp; % Max cp of the turbines for each wind turbine(NREL.Cp.max=0.45)

P_a=zeros(N,1);
P_ref=zeros(N,1);

% Compute available power at each turbine
for i=1:N
    %P_a=A*pi*r*r*Cp*v*v*v
    P_a(i)=min(rated(i),(pi/2)*rho*R(i)^2*v_nac(i)^3*Cp(i));
end

%Compute total available power
P_avail=sum(P_a);

%Distribute power according to availibility
for i=1:N
    if P_demand<P_avail
        P_ref(i)=max(0,min(rated(i),P_demand*P_a(i)/P_avail));
    else
        P_ref(i)=P_a(i);
    end
end

function [Omega, Ct, Cp] =Turbine_Drivetrain(x,u,wt,env)
% Parameters
global DT
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

function Finalval= embeddedinterpolCTtab(Beta,Lambda,table2,Betavec2,Lambdavec2)
%Finalval is the result of the look up of Lambda and Beta in the CP map.
%The function uses a bilinear interpolation method, and has been developed
%to replace interpn in an embedded matlab environment

%Setting up persistent variables
persistent table
persistent Betavec
persistent Lambdavec
persistent Firstrun

%Function initialization

%The first time the function is run, it stores supplied map as a persistent
%variable.
if isempty(Firstrun)% Is only run once
    table=table2;
    Betavec=Betavec2;
    Lambdavec=Lambdavec2;
    Firstrun=1;
end

%Step 1, finding two adjecent indexes of the BetaVec, which contain the
%supplied beta value

[trash,Bt]=min(abs(Betavec-Beta));      %Finding index 1
B1=Bt(1);                               %Necessary specification in embedded
%matlab

if Beta>Betavec(B1)                     %Finding index 2
    if B1==length(Betavec)              %testing if endpoint-extrapolation
        B2=B1;                          %should be used
        B1=B1-1;
    else
        B2=B1+1;
    end
else
    if B1==1
        B1=2;
        B2=1;
    else
        B2=B1-1;
    end
end

%Step 2, finding two adjecent indexes of the LambdaVec, which contain the
%supplied Lambda value
[trash,Lt]=min(abs(Lambdavec-Lambda));
L1=Lt(1);
if Lambda>Lambdavec(L1)%Need to work out of indexes
    if L1==length(Lambdavec)
        L2=L1;
        L1=L1-1;
    else
        L2=L1+1;
    end
else
    if L1==1
        L1=2;
        L2=1;
    else
        L2=L1-1;
    end
end

%Step 3
%Finding the four indexed values by means of the indexes
Yvals=[ table(B1,L1) table(B1,L2);
    table(B2,L1) table(B2,L2)];

%Step 4
%Making two sets of linear interpolations by using the different lambda values
Yintervals=[(Yvals(1,2)-Yvals(1,1))/(Lambdavec(L2)-Lambdavec(L1))*(Lambda-Lambdavec(L1))+Yvals(1,1);
    (Yvals(2,2)-Yvals(2,1))/(Lambdavec(L2)-Lambdavec(L1))*(Lambda-Lambdavec(L1))+Yvals(2,1)];
%Step 5
%Making the final linear interpolation on the results obtained in
%stepp 4
Finalval=((Yintervals(2)-Yintervals(1))/(Betavec(B2)-Betavec(B1)))*(Beta-Betavec(B1))+Yintervals(1);

function Finalval= embeddedinterpolCPtab(Beta,Lambda,table2,Betavec2,Lambdavec2)
%Finalval is the result of the look up of Lambda and Beta in the CP map.
%The function uses a bilinear interpolation method, and has been developed
%to replace interpn in an embedded matlab environment

%Setting up persistent variables
persistent table
persistent Betavec
persistent Lambdavec
persistent Firstrun

%Function initialization

%The first time the function is run, it stores supplied map as a persistent
%variable.
if isempty(Firstrun)% Is only run once
    table=table2;
    Betavec=Betavec2;
    Lambdavec=Lambdavec2;
    Firstrun=1;
end

%Step 1, finding two adjecent indexes of the BetaVec, which contain the
%supplied beta value

[trash,Bt]=min(abs(Betavec-Beta));      %Finding index 1
B1=Bt(1);                               %Necessary specification in embedded
%matlab

if Beta>Betavec(B1)                     %Finding index 2
    if B1==length(Betavec)              %testing if endpoint-extrapolation
        B2=B1;                          %should be used
        B1=B1-1;
    else
        B2=B1+1;
    end
else
    if B1==1
        B1=2;
        B2=1;
    else
        B2=B1-1;
    end
end

%Step 2, finding two adjecent indexes of the LambdaVec, which contain the
%supplied Lambda value
[trash,Lt]=min(abs(Lambdavec-Lambda));
L1=Lt(1);
if Lambda>Lambdavec(L1)%Need to work out of indexes
    if L1==length(Lambdavec)
        L2=L1;
        L1=L1-1;
    else
        L2=L1+1;
    end
else
    if L1==1
        L1=2;
        L2=1;
    else
        L2=L1-1;
    end
end

%Step 3
%Finding the four indexed values by means of the indexes
Yvals=[ table(B1,L1) table(B1,L2);
    table(B2,L1) table(B2,L2)];

%Step 4
%Making two sets of linear interpolations by using the different lambda values
Yintervals=[(Yvals(1,2)-Yvals(1,1))/(Lambdavec(L2)-Lambdavec(L1))*(Lambda-Lambdavec(L1))+Yvals(1,1);
    (Yvals(2,2)-Yvals(2,1))/(Lambdavec(L2)-Lambdavec(L1))*(Lambda-Lambdavec(L1))+Yvals(2,1)];
%Step 5
%Making the final linear interpolation on the results obtained in
%stepp 4
Finalval=((Yintervals(2)-Yintervals(1))/(Betavec(B2)-Betavec(B1)))*(Beta-Betavec(B1))+Yintervals(1);
