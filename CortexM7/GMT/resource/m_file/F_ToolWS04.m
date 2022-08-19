function [ToolWS_MinMax, TMtool,Tstart] = F_ToolWS04(Phome,pivot,alpha,beta,ToolPoseWorking,base)
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
%----Example Inputs----
% clear;clc;
% Phome =[0;0;155.0851057814131;0;0;0];
% pivot=[30.0;0.0;10.0];
% alpha = pi/2;
% beta = 0.0 ;  % depends on real situation; 
% ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0]; % one component at a time
% base = 1;
%---------------------
format long

b1=[97.25217186425962; -100.70757204741115; 0];
b2=[38.58922981437988; -134.57663743136464; 0];
b3=[-135.8414016786395; -33.86906538395348; 0];
b4=[-135.8414016786395; 33.86906538395348; 0];
b5=[38.58922981437988; 134.57663743136464; 0];
b6=[97.25217186425962; 100.70757204741115; 0];
bb0=[b1 b2 b3 b4 b5 b6];

p1=[83.40418343188286;  -26.4573654406638; 0];
p2=[-18.78934112711802; -85.4588243542396; 0];
p3=[-64.6148423047648; -59.00145891357579; 0];
p4=[-64.6148423047648;  59.00145891357579; 0];
p5=[-18.78934112711802; 85.4588243542396; 0];
p6=[83.40418343188286; 26.4573654406638; 0];
pp0=[p1 p2 p3 p4 p5 p6];

%----Setup Reference Zero Plane Pose -------
TPhome = poseRPY2SE3(Phome);

persistent TposeStart
persistent firstRun
if isempty(firstRun)
    TposeStart = TPhome;    
    firstRun = 1;
end
%--------------Tool Reference Zero Pose w.r.t Moving Frame ----
Ry=Roty(alpha);
Rx=Rotx(beta);
PMtool = pivot;
RMtool = Ry*Rx;
TMtool = [RMtool PMtool;0 0 0 1];

Lupp=181;
Llow=164;

bb=bb0;
pp=pp0;

%------Tool Work-----------------------------
TtoolPoseWorking=poseRPY2SE3(ToolPoseWorking);
if (base)
    MposiWork = TtoolPoseWorking(1:3,4);
else
    MposiWork=RMtool*TtoolPoseWorking(1:3,4);
end
ToolOrientWork = TtoolPoseWorking(1:3,1:3);
w = SO3ToAxisAngle(ToolOrientWork);
w = RMtool*w;

theta=norm(w);
if NearZero(theta)
    TRotstart = eye(4);
else
    w =w/theta;
    r=pivot;
    wx = [0 -w(3) w(2);...
          w(3) 0 -w(1);...
          -w(2) w(1) 0];
    S=[wx -wx*r;0 0 0 0];
    TRotstart = MatrixExp6(S*theta);
end
%--------------------------------------------------------
THMwork = [eye(3) MposiWork;0 0 0 1];
Twork = THMwork*MatrixExp6(MatrixLog6(inv(THMwork)*TRotstart));

TposeStart = TposeStart*THMwork*Twork;
Tstart = TposeStart;
poseStart = SE3ToPoseRPY(TposeStart);

XRtool = RMtool(:,1);
YRtool = RMtool(:,2);
ZRtool = RMtool(:,3);

ToolPlus=zeros(3,1);
ToolMinus=zeros(3,1);
%---------Translation----------------------
deltp = 0.001;
for i = 0:deltp:50
    Ptest = poseStart + (i)*[XRtool;0;0;0];
    LL = F_Stewart_IK(Ptest,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        XPlus = (i-deltp)*[XRtool;0;0;0];
        ToolPlus(1) = (i-deltp);
        break;
    end
end
for i = 0:-deltp:-50
    Ptest = poseStart + (i)*[XRtool;0;0;0];
    LL = F_Stewart_IK(Ptest,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        XMinus = (i+deltp)*[XRtool;0;0;0];
        ToolMinus(1) = (i+deltp);
        break;
    end
end
for i = 0:deltp:50
    Ptest = poseStart + (i)*[YRtool;0;0;0];
    LL = F_Stewart_IK(Ptest,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        YPlus = (i-deltp)*[YRtool;0;0;0];
        ToolPlus(2) = (i-deltp);
        break;
    end
end
for i = 0:-deltp:-50
    Ptest = poseStart + (i)*[YRtool;0;0;0];
    LL = F_Stewart_IK(Ptest,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        YMinus = (i+deltp)*[YRtool;0;0;0];
        ToolMinus(2) = (i+deltp);
        break;
    end
end
for i = 0:deltp:50
    Ptest = poseStart + (i)*[ZRtool;0;0;0];
    LL = F_Stewart_IK(Ptest,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ZPlus = (i-deltp)*[ZRtool;0;0;0];
        ToolPlus(3) = (i-deltp);
        break;
    end
end
for i = 0:-deltp:-50
    Ptest = poseStart + (i)*[ZRtool;0;0;0];
    LL = F_Stewart_IK(Ptest,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ZMinus = (i+deltp)*[ZRtool;0;0;0];
        ToolMinus(3) = (i+deltp);
        break;
    end
end
%----------Rotation-------------------------------------------------------
ToolanglePlus=zeros(3,1);
ToolangleMinus=zeros(3,1);
deltR=0.00001;
for j = 0:deltR:0.3
    w = ZRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    LL = F_Stewart_IK(trajEND,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(1)=(j-deltR);
        break;
    end    
end
for j = 0:-deltR:-0.3
    w = ZRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    LL = F_Stewart_IK(trajEND,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(1)=(j+deltR);
        break;
    end    
end
for j = 0:deltR:0.3
    w = YRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    LL = F_Stewart_IK(trajEND,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(2)=(j-deltR);
        break;
    end    
end
for j = 0:-deltR:-0.3
    w = YRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    LL = F_Stewart_IK(trajEND,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(2)=(j+deltR);
        break;
    end    
end
for j = 0:deltR:0.3
    w = XRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    LL = F_Stewart_IK(trajEND,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(3)=(j-deltR);
        break;
    end    
end
for j = 0:-deltR:-0.3
    w = XRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    LL = F_Stewart_IK(trajEND,pp,bb); 
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(3)=(j+deltR);
        break;
    end    
end

%-------------Output------------------------------------------------

ToolWS_MinMax = [[ToolMinus;ToolangleMinus] [ToolPlus;ToolanglePlus]];
end
