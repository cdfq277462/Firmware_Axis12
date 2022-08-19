function  [ToolWS_MinMax, Tstart, TMtool] = N_ToolWS04_Su(Phome,pivot,alpha,beta,ToolPoseWorking,base)
% function  [ToolWS_MinMax, Tstart] = N_ToolWS04_Su(ToolPoseWorking)
%--Given ----------------------------------
% Stewart 物理最低點 下球形關節最低高度 = 49.41mm
% Stewart 物理最高點 = 59.41mm
% 
% Stewart Low Point Length 50.0 mm (Alarm for safety)
% Stewart High Point Length 59.0 mm (Alarm for safety)

% Stewart Low Point Llow = 50.5 mm (For working spacee)
% Stewart High Point Lupper =  58.5 mm (For working space)

% ---poseRPY--------
% Phome = [0;0;66.4236;0;0;0];
% Corresponding to dg=[5.000006622609646; 5.000006622609718;...
% 5.000006622609718; 5.000006622609718; 5.000006622609718;...
% 5.000006622609646];
%-------------------------------------------

%----pre-Test Bounds---------------------
% for pivot = [0;0;0];
% Tool: [Xmin Xmax;Ymin Ymax; Zmin Zmax; Phi_min Phi_max;...
%        Theta_min Theta_max; Psi_min Psi_max] =
%       -3.911000000000000   4.090000000000000
%       -2.218000000000000   2.218000000000000
%       -2.340000000000000   2.714000000000000
%       -8.479775367936183   8.479775367936183
%       -9.224620501606255   9.110028942580090
%       -5.156620156177409   5.156620156177409
%----  with 0.0001 incremental radium resolution, the results are-----
%       -3.911000000000000   4.090000000000000
%       -2.218000000000000   2.218000000000000
%       -2.340000000000000   2.714000000000000
%       -8.468316212033569   8.468316212033569
%       -9.173054300044482   9.092840208726166
%       -5.145161000274793   5.145161000274793
%------------------------------------------
% ---input frome work space----------------------------
% clear;clc;
% Phome = [0;0;66.4236;0;0;0];
% pivot=[4.0;0.0;2.0];
% alpha = pi/2;
% beta = 0.0 ;  % depends on real situation; 
% % ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0];
% base = 0;
%------------------------------------------------------
format long
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

%bbi fixed platform global coordinates
b1=[27.79095418441866; -25.6223118691413; 0];
b2=[8.294095890154637; -36.87882825368669; 0];
b3=[-36.0850500745733; -11.25651638454532; 0];
b4=[-36.0850500745733; 11.25651638454532; 0];
b5=[8.294095890154637; 36.87882825368669; 0];
b6=[27.79095418441866; 25.6223118691413; 0];

bb0=[b1 b2 b3 b4 b5 b6];

%ppi moving platform coordinates
p1=[25.20360654775701; -8.3474976481494; 0];
p2=[-5.372658252550269; -26.00071236142009; 0];
p3=[-19.83094829520674; -17.65321471327068; 0];
p4=[-19.83094829520674; 17.65321471327068; 0];
p5=[-5.372658252550269; 26.00071236142009; 0];
p6=[25.20360654775701; 8.3474976481494; 0];

pp0=[p1 p2 p3 p4 p5 p6];

Lupp=58.5;
Llow=50.5;
% h0
h0 = 21.2;
% g0
g0 = 49.41;
bb=bb0;
pp=pp0;

%-------------------

% TtoolPoseHome = poseRPY2SE3(ToolPoseHome);
% TM=TMtool*TtoolPoseHome;

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
% MOrientWork = RMtool*ToolOrientWork;
% MOrientWork = RMtool*ToolOrientWork;

theta=norm(w);
if NearZero(theta)
    TRotstart = eye(4);
else
    w =w/theta;
    r=pivot;
%     r=TM(1:3,4);
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

% XRtool = MOrientWork(:,1);
% YRtool = MOrientWork(:,2);
% ZRtool = MOrientWork(:,3);

ToolPlus=zeros(3,1);
ToolMinus=zeros(3,1);
%---------Translation----------------------
deltp = 0.001;
for i = 0:deltp:50
    Ptest = poseStart + (i)*[XRtool;0;0;0];
    [Ls,alpha] = N_Stewart_IK(Ptest,pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        XPlus = (i-deltp)*[XRtool;0;0;0];
        ToolPlus(1) = (i-deltp);
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        XPlus = (i-deltp)*[XRtool;0;0;0];
        ToolPlus(1) = (i-deltp);
        break;
    end
end

for i = 0:-deltp:-50
    Ptest = poseStart + (i)*[XRtool;0;0;0];
    [Ls,alpha] = N_Stewart_IK(Ptest,pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        XMinus = (i+deltp)*[XRtool;0;0;0];
        ToolMinus(1)=(i+deltp);
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        XMinus = (i+deltp)*[XRtool;0;0;0];
        ToolMinus(1)=(i+deltp);
        break;
    end
end

for i = 0:deltp:40
    Ptest = poseStart+ (i)*[YRtool;0;0;0];
    [Ls,alpha] = N_Stewart_IK(Ptest,pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        YPlus = (i-deltp)*[YRtool;0;0;0];
        ToolPlus(2)=(i-deltp);
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        YPlus = (i-deltp)*[YRtool;0;0;0];
        ToolPlus(2)=(i-deltp);
        break;
    end
end

for i = 0:-deltp:-40
    Ptest = poseStart + (i)*[YRtool;0;0;0];
    [Ls,alpha] = N_Stewart_IK(Ptest,pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        YMinus = (i+deltp)*[YRtool;0;0;0];
        ToolMinus(2)=(i+deltp);
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        YMinus = (i+deltp)*[YRtool;0;0;0];
        ToolMinus(2)=(i+deltp);
        break;
    end
end

for i = 0:deltp:25
    Ptest = poseStart + (i)*[ZRtool;0;0;0];
    [Ls,alpha] = N_Stewart_IK(Ptest,pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        ZPlus = (i-deltp)*[ZRtool;0;0;0];
        ToolPlus(3)=(i-deltp);
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ZPlus = (i-deltp)*[ZRtool;0;0;0];
        ToolPlus(3)=(i-deltp);
        break;
    end
end

for i = 0:-deltp:-25
    Ptest = poseStart + (i)*[ZRtool;0;0;0];
   [Ls,alpha] = N_Stewart_IK(Ptest,pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        ZMinus = (i+deltp)*[ZRtool;0;0;0];
        ToolMinus(3)=(i+deltp);
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ZMinus = (i+deltp)*[ZRtool;0;0;0];
        ToolMinus(3)=(i+deltp);
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
    [Ls,alpha] = N_Stewart_IK(trajEND,pp,bb); 
    if any((alpha.^2 + h0^2 - Ls.^2)<0)  
        ToolanglePlus(1)=(j-deltR);
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(1)=(j-deltR);
        break;
    end    
end
for j = 0:-deltR:-0.3
    w = ZRtool;   
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,alpha] = N_Stewart_IK(trajEND,pp,bb); 
    if any((alpha.^2 + h0^2 - Ls.^2)<0)  
        ToolangleMinus(1)=(j+deltR);
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(1)=(j+deltR);
        break;
    end    
end

for j = 0:deltR:0.3
    w = YRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,alpha] = N_Stewart_IK(trajEND,pp,bb); 
    if any((alpha.^2 + h0^2 - Ls.^2)<0)  
        ToolanglePlus(2)=(j-deltR);
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(2)=(j-deltR);
        break;
    end    
end

for j = 0:-deltR:-0.3
    w = YRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,alpha] = N_Stewart_IK(trajEND,pp,bb); 
    if any((alpha.^2 + h0^2 - Ls.^2)<0)  
        ToolangleMinus(2)=(j+deltR);
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(2)=(j+deltR);
        break;
    end    
end

for j = 0:deltR:0.3
    w = XRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,alpha] = N_Stewart_IK(trajEND,pp,bb); 
    if any((alpha.^2 + h0^2 - Ls.^2)<0)  
        ToolanglePlus(3)=(j-deltR);
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(3)=(j-deltR);
        break;
    end    
end
for j = 0:-deltR:-0.3
    w = XRtool;    
    theta = j;
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,alpha] = N_Stewart_IK(trajEND,pp,bb); 
    if any((alpha.^2 + h0^2 - Ls.^2)<0)  
        ToolangleMinus(3)=(j+deltR);
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(3)=(j+deltR);
        break;
    end    
end
%-------------Output------------------------------------------------

% ToolWS_max = [ToolPlus;ToolanglePlus]; 
ToolWS_MinMax = [[ToolMinus;ToolangleMinus] [ToolPlus;ToolanglePlus]];
end
