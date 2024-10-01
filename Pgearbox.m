function [z]=Pgearbox(rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs)
global lambda3 rho3;
lambda3=0;
rho3=0.1;
% clear
% close all

disp(' ')
disp('----Gearbox Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m

x0=[8,80,650,1,5000,0.9,16];%[rProp,V,mtow,S,rpm,eta_motor,m_gb]
lb=[0.01,10,100,1,2600,0.01,20];
ub=[10, 100, 9999,30,8800,1,200];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@ mycon;
func=@myfun;

%options = optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',9000,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',900,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(func,x0,A,b,Aeq,beq,lb,ub,constraints,options,rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs);

%[~,sOUT]=myfun1(xopt,xU,xLf,vU,vLf,wU,wL,misc);
%sOUT.xopt=xopt;
%sOUT.funcCount = OUTPUT.funccount;
disp('______Wing ends_________')
z=[xopt,lambda3,rho3];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%Objective
    function [foriginal]=myfun(x,rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs)
       global lambda3 rho3;
        %global rho  lambda;
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);

         vehicle=misc.vehicle;
        range=misc.range;
        payload=misc.payload;
        %get original objective
        %foriginal =mtow ;
        
        [ENominal,flightTime,hoverOutput,cruiseOutput] = simpleMission(vehicle,rProps,Vs,mtows*9.8,range,Ss,rpms,eta_motors,m_gbs);
        
        
        % Mass estimate
        mass = configWeight(vehicle,rProps,mBatterys,MMotors,mtows,hoverOutput,cruiseOutput,payload,m_gbs);
        
        % Compute operating cost
        C = operatingCost(vehicle,rProps,flightTime,ENominal,mass,cruiseOutput);
        f=C.costPerFlight;
        
        %get original objective
        %foriginal =m_gb ;
        
        %Assemble c: target - response
        
        c(1) = diffc(rProps,rProp);
        c(2) = diffc(Vs,V);
        c(3) = diffc(mtows,mtow);
        c(4) = diffc(Ss,S) ;
        c(5) = diffc(rpms,rpm);
        c(6) = diffc(eta_motors,eta_motor);
        c(7) = diffc(m_gbs,m_gb);%Assemble v and w.Make it consistently ordered
         foriginal=abs(f+lambda3*(c(1)+c(2)+c(3)+c(4)+c(5)+c(6)+c(7))+0.5*rho3*(c(1)^2+c(2)^2+c(3)^2+c(4)^2+c(5)^2+c(6)^2+c(7)^2)); % as equality constraint so no weight wf needed
        lambda3=lambda3+rho3*(c(1)+c(2)+c(3)+c(4)+c(5)+c(6)+c(7))
        rho3=rho3*1.1
        
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        
        %Pack all data to a struct
        sOUT.rProp=x(1);
        sOUT.V=x(2);
        sOUT.mtow=x(3);
        sOUT.S=x(4);
        sOUT.rpm=x(5);
        sOUT.eta_motor=x(6);
        sOUT.m_gb=x(7);
        sOUT.x = x;
        sOUT.xU = [rProps,Vs,mtows,Ss,rpms,eta_motors,m_gbs];
        sOUT.xL = [];
        %sOUT.vU = vU;
        %sOUT.vL = vL;
        %sOUT.wU = wU;
        %sOUT.wL = wL;
        sOUT.misc = misc;
        %sOUT.v = v;
        %sOUT.w = w;
        sOUT.c=c;
        %sOUT.phi=phi;
        sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        
        vehicle=misc.vehicle;
        range=misc.range;
        
        
        
        %[~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
        %ceqn(1)= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
        %ceqn(2)= -(x(7)-mass_gb(rpm,rProp,hoverOutput.PMax));
        %ceqn(1)= (x(7)/mass_gb(rpm,rProp,hoverOutput.PMax))-1;
        %ceqn(2)= (x(7)/mass_gb(rpm,rProp,hoverOutput.PMax))+1;
        %cn=[];
 
    end

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
    function [cn,ceqn] = mycon(x,rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs)

        rProp=x(1);
        V=x(2);
        mtow=x(3);
        S=x(4);
        rpm=x(5);
        eta_motor=x(6);
        m_gb=x(7);
        
        
         rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;        vehicle=misc.vehicle;
        range=misc.range;
        
        
        
        [~,~,hoverOutput,~] = simpleMission(vehicle,rProp,V,mtow*9.8,range,S,rpm,eta_motor,m_gb);
       ceqn= x(7)-mass_gb(rpm,rProp,hoverOutput.PMax);
        cn=[];
    end


    function cd= diffc(aT,aR)
        %cd=0.5*(aT-aR).^2; %target - response
           cd=-(aT-aR) ;
        %                c=abs(c);
    end




