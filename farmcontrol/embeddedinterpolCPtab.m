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
