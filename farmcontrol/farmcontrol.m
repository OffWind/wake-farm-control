function farmcontrol
DT=0.125 % time step
Tstart=0 % time start
Tend=1000 % time end
farmcontrol=1; % enable wind farm control
P_demand=5000000 % some number
v_nac = 9*ones(4,1); %Wind speed at nacelle in m/s

parm.N=4; % number of turbines in farm
farmcontrol=1; % enable wind farm control
P_demand=10e6; % some number

%turbine properties
parm.rho=1.227; %air density
parm.radius=63*ones(1,4); % rotor radius (NREL5MW)
parm.rated=5e6*ones(1,4); %rated power (NREL5MW)
parm.ratedSpeed=1.26;
parm.Cp=0.45*ones(1,4); %max cp (NREL5MW)


for i=Tstart:DT:Tend %At each sample time(DT) from Tstart to Tend 

	%Do wake calculations.

	if farmcontrol==1
	P_ref  = Farm_Controller(v_nac,P_demand,parm);
	
	%%%%%% start Turbine_Controller
	%%% Placeholder for [Ct]=Turbine_Controller(P_ref)
	%%%%%% end Turbine_Controller
	end

	%Print to file (time, p_ref)

end

function [P_ref]  = Farm_Controller(v_nac,P_demand,parm)

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
		P_ref(i)=max(0,min(rated(i),P_demand*P_a(i)/P_avail));
	end
end
end