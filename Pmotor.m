function [z]=Pmotor(rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs)
global lambda2 rho2;
lambda2=0;
rho2=0.1;
% clear
% close all

disp(' ')
disp('----Motor Begins----')


%         AR = x(1);
%         S = x(2); %m2
%         rootaoa = x(3); %degree
%         vinf = x(4); %cruise velocity- m/s
%         rProp = x(5); %rotor radius, m

x0 =[5000,0.9]; %  motor: [RPM, efficiency]

lb=[2600,0.01];
ub= [8800,1];
A =[];
b = [];
Aeq = [];
beq = [];


constraints=@ mycon;
func=@myfun;

%options = optimoptions('surrogateopt','Display','iter','MaxFunctionEvaluations',900,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',900,'Algorithm','sqp','FiniteDifferenceType','central','ScaleProblem','obj-and-constr');% ,'PlotFcn','optimplotfvalconstr');
%Run optimization for PS
[xopt,fvalopt, FLAG, OUTPUT] = fmincon(func,x0,A,b,Aeq,beq,lb,ub,constraints,options,rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs);

%[~,sOUT]=myfun1(xopt,xU,xLf,vU,vLf,wU,wL,misc);
%sOUT.xopt=xopt;
%sOUT.funcCount = OUTPUT.funccount;
disp('______Wing ends_________')
z=[xopt,lambda2,rho2];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Objective
    function [foriginal]=myfun(x,rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs)
        global lambda2 rho2;
        %do calculations for both objective and constraints here itself.
        %save and load in constraint section
       % global rho  lambda; 
        rangei=50000;
         misc.range=rangei;
         misc.vehicle='tiltwing';
         misc.payload=300;
        rpm = x(1);
        eta_motor = x(2); %m2
        
        
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
        %foriginal =eta_motor ;
        
        
        %Assemble c: target - response
        
        c(1) = diffc(rpms,rpm);
        c(2) = diffc(eta_motors,eta_motor);
        foriginal=abs(f+lambda2*(c(1)+c(2))+0.5*rho2*(c(1)^2+c(2)^2)); % as equality constraint so no weight wf needed
        lambda2=lambda2+rho2*(c(1)+c(2))
        rho2=rho2*1.1
        
        %Assemble v and w.Make it consistently ordered
        %get all phi
        %v = vU; w=wU;
        %phi= v.*c+(w.*c).^2;
        
        %ATC objective
        %fATC=foriginal+ sum(phi);
        
        sOUT.rpm=rpm;
        sOUT.eta_motor=eta_motor;
        sOUT.x = x;
        sOUT.xU = [rpms,eta_motors];
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
        %sOUT.foriginal=foriginal;
        %sOUT.fATC = fATC;
         rpm=x(1);
        %ceqn(1)= x(2)-motor_eta(rpm);
        %ceqn(2)= -(x(2)-motor_eta(rpm));
        %ceqn(1)= (x(2)/motor_eta(rpm))-1;
        %ceqn(2)= (x(2)/motor_eta(rpm))+1;
        %cn=[];
        
        
    end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Constraint
    function [cn,ceqn] = mycon(x,rProps,Vs,mBatterys,MMotors,mtows,Ereserves,Ss, rpms, eta_motors,m_gbs)
        
        rpm=x(1);
        ceqn(1)= x(2)-motor_eta(rpm);
        cn=[];
        
    end


    function cd= diffc(aT,aR)
        %cd=0.5*(aT-aR).^2 ; %target - response
            cd=-(aT-aR) ;
        %                c=abs(c);
    end




