function [trajM,LdataM,traj,Ldata,flag1] = F_ToolSpiralTrack004_DataPin2_Su(ToolWS_MinMax,TMtool,Tstart,ScanRange,LineSpace, TimeScaling,velocity,Ts,r,u,Thd0,Thd1)

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
%------------------------------------------
%%------Example Inputs--------------------------------
% ScanRange = 10;  % ScanRange(max) = Zmax;
% LineSpace = 0.1;
% TimeScaling ='3';
% velocity = 3;
% Ts= 0.01;
% r = rand(2,1);
% u = [2.0;3.5];
% Thd0 = 0.75;
% Thd1 = 0.95;
%----------------------------------------------

Pstart = SE3ToPoseRPY(Tstart);
TtoolStart = Tstart*TMtool;

PMtool = TMtool(1:3,4);

ToolTranslx = TtoolStart(1:3,1);
ToolTransly = TtoolStart(1:3,2);
ToolTranslz = TtoolStart(1:3,3);

%------------------------------------
% v=v/500;
ax = Pstart(1);
ay = Pstart(2);
az = Pstart(3);
% b = b / 6.283;

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

Lupp=181;
Llow=164;

%------------Simulated Light Power Data------------
y1=ToolWS_MinMax(2,1);
y2=ToolWS_MinMax(2,2);
z1=ToolWS_MinMax(3,1);
z2=ToolWS_MinMax(3,2);
c1=r(1)*y1+(1-r(1))*y2;
c2=r(2)*z1+(1-r(2))*z2;
C = [ax;ay;az]+c1*ToolTransly + c2*ToolTranslz;
cy = C(2);
cz = C(3);
s1=u(1);
s2=u(2);
f=@(y,z) exp(-((y-cy)^2/s1^2+(z-cz)^2/s2^2));
%----------------------------------------------

b=LineSpace/(2*pi);
ScanRange = ScanRange/2;
v=1/1000;
Tf=(2*pi)*ScanRange/LineSpace;
K=ceil(Tf/(0.0001))+1;
X=[];
Y=[];
ToolTrans=[];
traj1=[];
traj0=[];
traj=[];
LL=[];
XY=[];
% M=K;

for k=1:K 
    F= @(T) (b/2)*(log((T*k+sqrt(T^2*k^2+1))/(T*(k-1)+sqrt(T^2*(k-1)^2+1)))...
        +T*k*sqrt(T^2*k^2+1)-T*(k-1)*sqrt(T^2*(k-1)^2+1))-v*T;  
    x0 = 0.1;  % TolX = 1e-5; MaxIter = 50; %with initial guess 1.8
    T = newton(F,x0,1e-5,50); %1st order derivative
    %--------------------------------------      
    X=(b*(k-1)*T)*cos((k-1)*T)*ToolTranslx;
    Y=(b*(k-1)*T)*sin((k-1)*T)*ToolTransly;
    Z= 0.0*ToolTranslz;
    Trans(:,k) = [ax;ay;az]+X+Y+Z;  
%     ToolTrans(:,k)=PMtool+RMtool*Trans(:,k);
    ToolTrans(:,k)=PMtool+Trans(:,k);
    traj1(:,k)=[Trans(:,k);Pstart(4:6)];  
%         if k==120
%         traji = traj1(:,k)
%         k
%         end
   %---------------
    yy=traj1(2,k);
    zz=traj1(3,k);
    Ldata1(k)=f(yy,zz);
    %---------------
    LL(:,k) = F_Stewart_IK(traj1(:,k),pp,bb);
    if (min(LL(:,k)))<Llow||(max(LL(:,k)))>Lupp 
        M=k-1;
        break;
    end 
    M=k;    
end

%-----------------
traj = traj1(:,1:M);
Ldata = Ldata1(1:M);

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
    Pstart = traj(:,M);
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
    Pstart = traj(:,M);
    Pend = trajM;
else
    LdataM = 0;
    trajM = zeros(6,1);
    temp = Pstart;
    Pend = temp;    
    Pstart = traj(:,M);
end  
Tend=poseRPY2SE3(Pend);
Tstart=poseRPY2SE3(Pstart);
%----------------

[traj0, Ldata0] = F_moveL_PP_Data(ToolWS_MinMax,TMtool,Tstart, Tend, TimeScaling,velocity, Ts, r,u);

% traj = [traj traj0];
% Ldata = [Ldata Ldata0];
%--------------------------------------
Xmin = min(traj(1,1:M));Xmax=max(traj(1,1:M));
Ymin = min(traj(2,1:M));Ymax=max(traj(2,1:M));
Zmin = min(traj(3,1:M));Zmax=max(traj(3,1:M));

ToolTrans0 =PMtool+traj0(1:3,:);

figure;
plot3(ToolTrans(1,1:M)',ToolTrans(2,1:M)',ToolTrans(3,1:M)','o'); hold on;
plot3(ToolTrans0(1,1:end)',ToolTrans0(2,1:end)',ToolTrans0(3,1:end)','o');
%---
RR=[ToolTranslx ToolTransly ToolTranslz];
PP=[ax;ay;az]+PMtool;
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
%------------------------------

end