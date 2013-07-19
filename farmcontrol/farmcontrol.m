%% The main file for running the wind farm controll and wake simulation.
% It is not completely done yet. Further updates will come
% Currently there are only 4 turbines, for test purposes. But is should be
% easily updated to a larger number of turbines.
% Similarly there is a lot of room for speed optimizations, even though it
% now runs slowly with only 4 turbines
% 19/07-13 MS

clear
close all


%% Initializations
%General settings to be changed
saveData = false;  %Save all the simulated data to a .mat file?
enablePlotting = true; %Make plots?
enablePowerDistribution=true; % enable wind farm control and not only constant power
enableTurbineDynamics = true; %Enable dynamical turbine model. Disabling this will increase the speed significantly, but also lower the fidelity of the results (setting to false does not work properly yet)

%Add path to wake code
if(~exist('plot_wind_field'))
addpath([pwd '/../wake']);
end

%Time parameters (in seconds)
Tstart=0; % time start
Tend=100; % time end
DT=0.0125; % time step
P_ref_sample_time = 5; %Update inverval for farm controller



% Wind farm properties
parm.N=4; % number of turbines in farm
P_demand=3*5e6; % some number

%turbine properties
load NREL5mw.mat %Load parameters from the NREL 5MW turbine
nTurbines = 4;
parm.rho=env.rho; %air density
parm.radius=wt.rotor.radius*ones(1,nTurbines); % rotor radius (NREL5MW)
parm.rated=5e6*ones(1,nTurbines); %rated power (NREL5MW)
parm.ratedSpeed=wt.rotor.ratedspeed; %rated rotor speed

[dummy idx]=max(wt.cp.table(:)); %Find index for max Cp;
parm.Cp = ones(1,nTurbines)*wt.cp.table(idx); %Set power coefficent to maximum value in the cp table
parm.Ct = ones(1,nTurbines)*wt.ct.table(idx); %Set power coefficent to maximum value in the cp table




% NOTE: controller parameters should be imported from the wt....struct in
% NREL5mw.mat
%Pitch control
ee=0; %blade pitch integrator
Ki=0.008068634*360/2/pi; % integral gain (NREL5MW)
Kp=0.01882681*360/2/pi; % proportional gain (NREL5MW)
PC_MaxPit=90;
PC_MinPit=0;

%region control NREL
VS_CtInSp     =      70.16224;
VS_RtGnSp     =     121.6805;
VS_Rgn2K      =       2.332287;



% load initial wind data
wind=load('wind.mat');

%% Set initial conditions
omega0=1.267; %Rotation speed
beta0=0; %Pitch
power0=parm.rated(1); %Power production
x=[omega0*ones(parm.N,1) wind.wind(1,2)*ones(parm.N,1)];
u0=[beta0*ones(parm.N,1) power0*ones(parm.N,1)];
u = u0;
Mg_old =u(:,2);
P_ref=zeros(parm.N,((Tend-Tstart)/DT)); %Initialize matrix to save the power production history for each turbine
Pa = P_ref; %Initialize available power matrix
Power=P_ref;
Ct = parm.Ct; %Initialize Ct - is this correct?
P_ref_new = power0*ones(nTurbines,1);


%% Simulate wind farm operation
for i=2:((Tend-Tstart)/DT) %At each sample time(DT) from Tstart to Tend
    
    %Calculate the wake using the current Ct values
    v_nac(:,i)=wakeCalculation(Ct(i-1,:),i,wind);
    x(:,2) = v_nac(:,i);

    
    %Farm control 
    %Calculate the power distribution references for each turbine
    if enablePowerDistribution==1
        [P_ref_new Pa(:,i)]  = powerDistributionControl(v_nac(:,i),P_demand,Power(:,i-1),parm);
    end
    
    %Hold  the demand for some seconds
    if(  mod(i,round(P_ref_sample_time/DT))==2)
        P_ref(:,i) = P_ref_new;
    else

        powerRefInterpolation = true;
        if(powerRefInterpolation)
        alpha = 0.01;
        P_ref(:,i) = (1-alpha)*P_ref(:,i-1)+ (alpha)*P_ref_new;;
        else
        P_ref(:,i) = P_ref_new;       
        end
    end
    
    
    
    %Calculate control for each individual turbine - should be moved to the
    %turbine (drivetrain) model.

    
    %Torque controller
    for j=1:parm.N
        if ( (   x(j,1)*97 >= VS_RtGnSp ) || (  u(j,1) >= 1 ) )   % We are in region 3 - power is constant
            u(j,2) = P_ref(j,i)./x(j,1);
        elseif ( x(j,1)*97 <= VS_CtInSp )                            %! We are in region 1 - torque is zero
            u(j,2) = 0.0;
        else                                                         %! We are in region 2 - optimal torque is proportional to the square of the generator speed
            u(j,2) =97*VS_Rgn2K*x(j,1)*x(j,1)*97^2;
        end
    end
    
    
    %Rate limit torque change
  %  u(:,2) - Mg_old;
    Mg_max_rate = 1e6*DT;
    u(:,2) =   sign(u(:,2) - Mg_old) .* min(abs(u(:,2) - Mg_old), Mg_max_rate) + Mg_old;
    
    
    
    %Pitch controller
    e=97*(omega0*ones(parm.N,1)-x(:,1));
    ee =ee-DT*e;
    ee=min( max( ee, PC_MinPit/Ki), PC_MaxPit/Ki);
    
    u(:,1)=-Kp*DT*e+Ki*ee;
    for j=1:parm.N
        u(j,1)=min( max( u(j,1), PC_MinPit), PC_MaxPit);
    end
    
if(~enableTurbineDynamics)
    u = u0;
end

    Mg(:,i)=u(:,2);
    Mg_old = Mg(:,i);
    beta(:,i)=u(:,1); %Set pitch

    
    %Turbine dynamics - can be simplified
if(enableTurbineDynamics)
    
    for j=1:parm.N
        [x(j,1), Ct(i,j), Cp(i,j)]=turbineDrivetrainModel(x(j,:),u(j,:),wt,env,DT);
    end
else
    Ct(i,:) = parm.Ct;
    Cp(i,:) = parm.Cp;
x(:,1) = parm.ratedSpeed;%Rotational speed
end
    
    Omega(:,i)=x(:,1);
    Power(:,i)=Omega(:,i).*Mg(:,i);


end



%% Save output data
out=[DT*[0:((Tend-Tstart)/DT)-1]' v_nac' Omega' beta' P_ref' Ct Cp Pa' Mg' Power'];
if(saveData)
    save offwindFarmData.mat out
end



%% Plotting
%Below a number of different plots are made. Most of them for test purposes

if(enablePlotting)
    figure(1)
    plot(out(:,1),out(:,6:9)) % rotor speed
    title('Rotor speed')
    figure(2)
    plot(out(:,1),out(:,10:13)) % Blade pitch angle
    title('Blade pitch angle')
    figure(3)
    plot(out(:,1),out(:,14:17)/1e6) % Power reference
    title('Individual turbine power reference')
    xlabel('time')
    ylabel('Power reference [MW]')
    figure(4)
    plot(out(:,1),out(:,30:33)) % generator torque reference
    title('Generator torque reference')
    plot(out(:,1),out(:,26:29)) % Available power
    title('Available power')
    figure(5)
    plot(out(:,1),out(:,2:5)) % Wind speed
    title('Wind speed')
    save slet_wrkspc
    plot(out(:,1),sum(out(:,14:17),2)/1e6,out(:,1),sum(out(:,26:29),2)/1e6,out(:,1),sum(Power)'/1e6); %Ttotal power demand
    title('Total farm power')
    legend('Power Demand','Available Power','Actual Production')
    xlabel('time')
    ylabel('Power [MW]')
    figure(6)
    plot(out(:,1),sum(P_ref',2),out(:,1),sum(Pa',2),out(:,1),sum(Mg.*Omega)'); %Ttotal power demand
    title('Total power')
    legend('Demand','Available','Actual')
    figure(7)
    plot(out(:,1),(Mg.*Omega)'); %Total power produced
    title('Total power produced')
end

