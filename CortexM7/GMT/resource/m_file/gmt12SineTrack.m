function [traj traj0 traj1 traj2] = gmt12SineTrack(Tstart,velocity,Ts,TimeScaling,LineSpace,StepRange,ScanRange,Id)

%%%-- 說明
% 在PI的軟體介面中，給定的變數
% 1. PI在Sine波的mid pose，就是我們的當前pose，所以"使用前必須用Move L移動至Start Pose"
% 2. v 為速度
% 3. Ts 為Sample Time
% 4. LineSpace為Sine間距
% 5. ScanRange為Sine震幅
%-----------------------------------
% 
%%------Example Inputs--------------------------------
% clear;clc;close all;
% velocity = 3;
% Ts= 0.01;
% TimeScaling = '3';
% LineSpace = 0.1;
% StepRange = 2;ScanRange = 2;
% Id='V';
%----------------------------------------------

format long

TPhome = eye(4);
v = velocity;

xmin=-25; xmax=25;
ymin=-25; ymax=25;
zmin=-10; zmax=10;
rzmin= (-8.2)*pi/180; rzmax= (8.2)*pi/180;
rymin= (-3.4)*pi/180; rymax= (3.4)*pi/180;
rxmin= (-4.4)*pi/180; rxmax= (4.4)*pi/180;
%------- Tstart----------------------------
Pstart = SE3ToPoseRPY(Tstart);
Rstart = Tstart(1:3,1:3);
pstart = Tstart(1:3,4);
[phi,theta,psi]=SO3ToRPY(Rstart);
ax = pstart(1);
ay = pstart(2);
az = pstart(3);

Pdx = xmax-ax; Mdx= xmin-ax;
Pdy = ymax-ay; Mdy= ymin-ay;
Pdz = zmax-az; Mdz= zmin-az;
Pdrz = rzmax-phi; Mdrz = rzmin-phi;
Pdry = rymax-theta; Mdry = rymin-theta;
Pdrx = rxmax-psi; Mdrx = rxmin-psi;

dx= min(abs(Pdx),abs(Mdx)); 
dy= min(abs(Pdy),abs(Mdy)); 
dz= min(abs(Pdz),abs(Mdz)); 

ToolTranslx=Rstart(:,1);
ToolTransly=Rstart(:,2);
ToolTranslz=Rstart(:,3);

if (Id=='V')
    ScanLimit = abs(dz);
    StepLimit = abs(dy);
    Translv = ToolTranslz;
    Translh = ToolTransly;   
elseif (Id=='H')
    ScanLimit = abs(dy);
    StepLimit = abs(dz);
    Translv = ToolTransly;
    Translh = ToolTranslz;
end

%-----------------------------
if (ScanRange<= ScanLimit)
    Scan = ScanRange;
else
    Scan = ScanLimit;
end
if (StepRange<= StepLimit)
    Step = StepRange;
else
    Step = StepLimit;
end
%-----------Move to the starting Scan point from Tstart----
hs=Step*(-1);
H1=hs*Translh;
vs=Scan;
V1=vs*Translv;
x=0.0;
X1=x*ToolTranslx;
Trans(:,1) = [ax;ay;az]+X1+H1+V1; 
Traj(:,1) = [Trans(:,1);phi;theta;psi];
Pend = Traj(:,1);
traj0 = gmt12_moveL_PP(Pstart, Pend, TimeScaling,velocity, Ts);
%------------------------

Tf = (Step* 2 * pi) / (2 * LineSpace);
t = 1/v;

K=ceil(Tf/(Ts*(1/t)));

for k=1:(2*K+1)
    hs=Step*((k-1)/K-1);
    H=hs*Translh;
    vs=Scan*cos((k-1)*Ts*(1/t));
    V=vs*Translv;
    x=0.0;
    X=x*ToolTranslx;
    Trans(:,k) = [ax;ay;az]+X+H+V;     
%     deltav = Scan < abs(vs);    
    if (Scan < abs(vs))
        M=k-1;
        break;
    end
%     deltah = Step < abs(hs)
    if (Step < abs(hs))
        M=k-1;
        break;
    end
    traj1(:,k)=[Trans(:,k);phi;theta;psi];
    M=k;
end
traj1(:,M)=[Trans(:,M);phi;theta;psi]; 

%------Move back to Tstart--------
temp = Pstart;
Pend = temp;
Pstart = traj1(:,M);
traj2 = gmt12_moveL_PP(Pstart, Pend, TimeScaling,velocity, Ts);
%-----Output traj------

traj = [traj0 traj1 traj2];

%----plot-----
plot3(traj0(1,:)',traj0(2,:)',traj0(3,:)','o');hold on;
plot3(traj1(1,:)',traj1(2,:)',traj1(3,:)','o');
plot3(traj2(1,:)',traj2(2,:)',traj2(3,:)','o');

plotH(TPhome);hold on;
plotH(Tstart);
xlabel('x');ylabel('y');zlabel('z');
grid on;
% count = count+1;
end
   
