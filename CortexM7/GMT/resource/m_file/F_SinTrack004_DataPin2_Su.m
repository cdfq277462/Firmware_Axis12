function [trajM, LdataM,traj,Ldata,flag1] = F_SinTrack004_DataPin2_Su(ToolWS_MinMax,TMtool,Tstart,velocity,Ts,TimeScaling,LineSpace,StepRange,ScanRange,Id,r,u,Thd0,Thd1)

%--Given ----------------------------------
% Stewart 物理最低點 Length 162.5mm
% Stewart 物理最高點 Length 182.5mm
% Stewart Low(Zero) Point Length 163.5mm
% Stewart High Point Length 181.5mm
% Stewart Home Point Length 172.5mm. In this case, the pose of the Home
% point is Phome =[0;0;155.0851057814131;0;0;0];
% At home pose the lengths of each axis are respectively
% Lh = [172.5;172.5;172.5;172.5;172.5;172.5];
%-------------------------------------------
%----pre-Test Bounds---------------------
% Xm=19;
% Ym=19;
% Zm=9;
% Phi_m=13*pi/180;
% Theta_m=6*pi/180;
% Psi_m=6*pi/180;
%%------Example Inputs--------------------------------

% velocity = 3;
% Ts= 0.01;
% TimeScaling = '3';
% LineSpace = 1.0;
% StepRange = 20;ScanRange = 20;
% Id = 'V';
% r = rand(2,1);
% u = [1.5;2.5];
% Thd0 = 0.75;
% Thd1 = 0.95;
% %---------------------------

Pstart = SE3ToPoseRPY(Tstart);
TtoolStart = Tstart*TMtool;

ToolTranslx = TtoolStart(1:3,1);
ToolTransly = TtoolStart(1:3,2);
ToolTranslz = TtoolStart(1:3,3);

ax = Pstart(1);
ay = Pstart(2);
az = Pstart(3);

Lupp=181;
Llow=164;

% StepRange = StepRange / 2;
% ScanRange = ScanRange / 2;

Tf = (StepRange * 2 * pi) / (2 * LineSpace);
t = 1/velocity;

%bbi fixed platform global coordinates
b1=[97.25217186425962; -100.70757204741115; 0];
b2=[38.58922981437988; -134.57663743136464; 0];
b3=[-135.8414016786395; -33.86906538395348; 0];
b4=[-135.8414016786395; 33.86906538395348; 0];
b5=[38.58922981437988; 134.57663743136464; 0];
b6=[97.25217186425962; 100.70757204741115; 0];
bb=[b1 b2 b3 b4 b5 b6];

p1=[83.40418343188286;  -26.4573654406638; 0];
p2=[-18.78934112711802; -85.4588243542396; 0];
p3=[-64.6148423047648; -59.00145891357579; 0];
p4=[-64.6148423047648;  59.00145891357579; 0];
p5=[-18.78934112711802; 85.4588243542396; 0];
p6=[83.40418343188286; 26.4573654406638; 0];
pp=[p1 p2 p3 p4 p5 p6];

K=ceil(Tf/(Ts*(1/t)));

%------------Simulated Light Power Data------------
% y1=ToolWS_MinMax(2,1);
% y2=ToolWS_MinMax(2,2);
% z1=ToolWS_MinMax(3,1);
% z2=ToolWS_MinMax(3,2);
% c1=r(1)*y1+(1-r(1))*y2;
% c2=r(2)*z1+(1-r(2))*z2;
% C = [ax;ay;az]+c1*ToolTransly + c2*ToolTranslz;
% cy = C(2);
% cz = C(3);
% s1=u(1);
% s2=u(2);
% f=@(y,z) exp(-((y-cy)^2/s1^2+(z-cz)^2/s2^2));
%----------------------------------------------
if (Id=='H')    
    Translv = ToolTransly;
    Translh = ToolTranslx;
    scanLimit = min(abs(ToolWS_MinMax(2,:)));
    stepLimit = min(abs(ToolWS_MinMax(1,:)));
elseif (Id=='V')    
    Translv = ToolTranslx;
    Translh = ToolTransly;
    scanLimit = min(abs(ToolWS_MinMax(1,:)));
    stepLimit = min(abs(ToolWS_MinMax(2,:)));
end

%----
if (ScanRange > scanLimit)    
    ScanRange = scanLimit;
end
if (StepRange > stepLimit)
    StepRange = stepLimit;
end

X1=StepRange * (-1)*Translh;
Y1=ScanRange * Translv;
Z1= 0.0*ToolTranslz;
Trans(:,1) = [ax;ay;az]+X1+Y1+Z1; 
Traj(:,1)=[Trans(:,1);Pstart(4:6)];
Pend = Traj(:,1);
LL = F_Stewart_IK(Pend,pp,bb);    

while min(LL)<Llow||max(LL)>Lupp
    ScanRange=(0.75)*ScanRange;
    StepRange=(0.75)*StepRange;
    X1=StepRange * (-1)*Translh;
    Y1=ScanRange * Translv;
    Z1= 0.0*ToolTranslz;
    Trans(:,1) = [ax;ay;az]+X1+Y1+Z1; 
    Traj(:,1)=[Trans(:,1);Pstart(4:6)];
    Pend = Traj(:,1);
    LL = F_Stewart_IK(Pend,pp,bb);    
end

%----------------------------------------------
y1=-StepRange;
y2= StepRange;
z1=-ScanRange;
z2= ScanRange;
c1=r(1)*y1+(1-r(1))*y2;
c2=r(2)*z1+(1-r(2))*z2;
C = [ax;ay;az]+c1*ToolTransly + c2*ToolTranslz;
cy = C(2);
cz = C(3);
s1=u(1);
s2=u(2);
f=@(y,z) exp(-((y-cy)^2/s1^2+(z-cz)^2/s2^2));
%-----------------------------------------------

%---------------------------
Tend = poseRPY2SE3(Pend);
[traj0, Ldata0] = F_moveL_PP_Data(ToolWS_MinMax,TMtool,Tstart, Tend, TimeScaling,velocity, Ts,r,u);

for i=1:size(traj0,2)
    Tooltraj0(:,i) = TMtool(1:3,4)+traj0(1:3,i);
end

for k=1:(2*K+1)
    X=StepRange * ((k-1)/K-1)*Translh;
    Y=ScanRange * cos((k-1)*Ts*(1/t))*Translv;
    Z= 0.0*ToolTranslz;
    Trans(:,k) = [ax;ay;az]+X+Y+Z;     
    ToolTrans(:,k)=TMtool(1:3,4)+Trans(:,k);
    Traj(:,k)=[Trans(:,k);Pstart(4:6)];  
    end
    %---------------
    yy=Traj(2,k);
    zz=Traj(3,k);
    Ldata1(k)=f(yy,zz);
    %---------------
    LL(:,k) = F_Stewart_IK(Traj(:,k),pp,bb);
    if (min(LL(:,k)))<Llow||(max(LL(:,k)))>Lupp 
        M=k-1;
        break;
    end 
    M=k;
end
traj = [traj0 Traj(:,1:M)];
Ldata = [Ldata0 Ldata1(1:M)];

%-----Return to Pstart or where Ldata is max or the weighting average point-------
flag0 = any(Ldata>=Thd0);
flag1 = any(Ldata>=Thd1);
id0 = find(Ldata>=Thd0);
id1 = find(Ldata>=Thd1);

if flag1
    Ldatm = Ldata(id1);
    idm = find(Ldatm==max(Ldatm));
    idm = min(idm);
    LdataM = Ldatm(idm);
    idM = find(Ldata==LdataM);
    trajM = traj(:,idM);
    Pstart = Traj(:,M);
    Pend = trajM;
elseif flag0 && ~(flag1)
    Ldat = Ldata(id0);
    weit = sum(Ldat);
    idw = Ldat'*id0/weit;
    ids = abs(id0-idw);
    idm = find(ids==min(ids));
    idm = min(idm);
    LdataM = Ldata(idm);
    trajM = traj(:,idm);    
    Pstart = Traj(:,M);
    Pend = trajM;
else
    LdataM = 0;
    trajM = zeros(6,1);
    temp = Pstart;
    Pend = temp;    
    Pstart = Traj(:,M);
end  
Tend = poseRPY2SE3(Pend);
Tstart = poseRPY2SE3(Pstart);

[traj2, Ldata2] = F_moveL_PP_Data(ToolWS_MinMax,TMtool,Tstart, Tend, TimeScaling,velocity, Ts,r,u);

for i=1:size(traj2,2)
    Tooltraj2(:,i) = TMtool(1:3,4)+traj2(1:3,i);
end
%-------------------------

traj = [traj traj2];
Ldata = [Ldata Ldata2];

Xmin = min(traj(1,1:end));Xmax=max(traj(1,1:end));
Ymin = min(traj(2,1:end));Ymax=max(traj(2,1:end));
Zmin = min(traj(3,1:end));Zmax=max(traj(3,1:end));

figure;
plot3(Tooltraj0(1,1:end)',Tooltraj0(2,1:end)',Tooltraj0(3,1:end)','o'); hold on;
plot3(ToolTrans(1,1:M)',ToolTrans(2,1:M)',ToolTrans(3,1:M)','o'); 
plot3(Tooltraj2(1,1:end)',Tooltraj2(2,1:end)',Tooltraj2(3,1:end)','o');
%----------
RR=[ToolTranslx ToolTransly ToolTranslz];
PP=[ax;ay;az]+TMtool(1:3,4);
TT=[RR PP;0 0 0 1];
plotH(TT);
%-----
xlabel('x');ylabel('y');zlabel('z');
grid on; 

%-------Background Lighting----
Cer = [cy;cz];
[X,Y,Z]=GaussianData(Ymin,Ymax,Zmin,Zmax,Cer,u);
figure;
contourf(X,Y,Z,50,'LineColor','non');hold on;
plot(traj(2,1:end)',traj(3,1:end)','c');
figure;
surf(X,Y,Z);hold on;
y0 = traj(2,1:end);
z0 = traj(3,1:end);
w0=(y0-cy).^2/s1^2+(z0-cz).^2/s2^2;
w=exp(-(w0));
plot3(y0',z0',w','c','LineWidth',1.5);

% -------=------------------
% if ~(flag1)
%     start=Pend;
%     Tstart = poseRPY2SE3(start);
% %     figure;    
%     [trajM,LdataM,traj,Ldata,flag1]=F_ToolSpiralTrack004_DataPin2_Su(ToolWS_MinMax,TMtool,Tstart,ScanRange,LineSpace, TimeScaling,velocity,Ts,r,u,Thd0,Thd1)
% end
% if ~(flag1)
%      error("Cannot find the pose at which the light power is over the threshold=0.95.");
% end
%---------------------------
end
