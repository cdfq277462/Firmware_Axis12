function [traj,ToolangleWS] = N_ToolPivotTraj04_Su(pivot,alpha, beta, Tstart,Tf,Ts, TimeScaling)
% %--Given ----------------------------------
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
% Xmin=-2.72; Xmax=2.34
% Ymin=-2.22; Ymax=2.22
% Zmin=-3.92; Zmax=4.09
% Phi_min=-5.145161000274793*pi/180; Phi_max=5.145161000274793*pi/180;
% Theta_min=-9.173054300044482*pi/180; Theta_max=9.092840208726166*pi/180
% Psi_min=-8.468316212033569*pi/180; Psi_max=8.468316212033569*pi/180;
%------------------------------------------

%%------Example Inputs--------------------------------
% clear;clc;close all;
% pivot=[4.0;0.0;2.0];
% alpha = pi/2;
% beta = 0.0 ;  % depends on real situation;
% % Tstart=[eye(3) [0;0;66.4236]; 0 0 0 1];
% Tf = 1.5;
% Ts = 0.01;
% TimeScaling = '3';
%%----------------------------------------


%----Start-----
%--------------Tool Reference Zero Pose w.r.t Moving Frame ----
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

%--------------Tool Reference Zero Pose w.r.t Moving Frame ----
Ry=Roty(alpha);
Rx=Rotx(beta);
PMtool = pivot;
RMtool = Ry*Rx;
TMtool = [RMtool PMtool;0 0 0 1];

% ----------------

TtoolStart = Tstart*TMtool;
MposiStart = TtoolStart(1:3,4);
MOrientStart = TtoolStart(1:3,1:3);

poseStart = SE3ToPoseRPY(Tstart);

Xtool = MOrientStart(:,1);
Ytool = MOrientStart(:,2);
Ztool = MOrientStart(:,3);

traj= [];

%----------Available angles of Rotation-------------------------------------------------------

ToolanglePlus=zeros(3,1);
ToolangleMinus=zeros(3,1);
k=0;
deltR=0.0001;
%----------------------Rot about z-tool axis--------
for j = 0:deltR:0.3
    w = Ztool;    
    theta = j;
%     Ptraj(:,:,k) = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);    
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,gamma] = N_Stewart_IK(trajEND,pp,bb); 
    if any((gamma.^2 + h0^2 - Ls.^2)<0)  
        ToolanglePlus(1)=(j-deltR);        
        break; 
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(1)=(j-deltR);       
        break;
    end 
    k=k+1;
end
traj(:,:,1)=pivotTraj(poseStart,pivot, Ztool, (k-1)*deltR, Tf,Ts, TimeScaling);

k=0;
for j = 0:-deltR:-0.3
    w = Ztool;    
    theta = j;
%     Ptraj(:,:,k) = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,gamma] = N_Stewart_IK(trajEND,pp,bb); 
    if any((gamma.^2 + h0^2 - Ls.^2)<0)  
        ToolangleMinus(1)=(j+deltR);
        break; 
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(1)=(j+deltR);
        break;
    end                
    k=k+1;    
end
traj(:,:,2) = pivotTraj(poseStart,pivot, Ztool, (1-k)*deltR, Tf,Ts, TimeScaling);

%-------------Rot about Y-tool axis-----------------------------
k=0;
for j = 0:deltR:0.3
    w = Ytool;   
    theta = j;
%     Ptraj(:,:,k) = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,gamma] = N_Stewart_IK(trajEND,pp,bb); 
    if any((gamma.^2 + h0^2 - Ls.^2)<0)  
        ToolanglePlus(2)=(j-deltR);
        break; 
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(2)=(j-deltR);
        break;
    end                
    k=k+1;   
end
traj(:,:,3) = pivotTraj(poseStart,pivot, Ytool, (k-1)*deltR, Tf,Ts, TimeScaling);

k=0;
for j = 0:-deltR:-0.3
    w = Ytool;    
    theta = j;
%     Ptraj(:,:,k) = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,gamma] = N_Stewart_IK(trajEND,pp,bb); 
    if any((gamma.^2 + h0^2 - Ls.^2)<0)  
        ToolangleMinus(2)=(j+deltR);
        break; 
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(2)=(j+deltR);
        break;
    end                
    k=k+1;    
end
traj(:,:,4) = pivotTraj(poseStart,pivot, Ytool, (1-k)*deltR, Tf,Ts, TimeScaling);

%--------------Rot about x-tool axis-------------------

k=0;
for j = 0:deltR:0.3
    w = Xtool;    
    theta = j;
%     Ptraj(:,:,k) = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,gamma] = N_Stewart_IK(trajEND,pp,bb); 
    if any((gamma.^2 + h0^2 - Ls.^2)<0)  
        ToolanglePlus(3)=(j-deltR);
        break; 
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolanglePlus(3)=(j-deltR);
        break;
    end                
    k=k+1;   
end
traj(:,:,5) = pivotTraj(poseStart,pivot, Xtool, (k-1)*deltR, Tf,Ts, TimeScaling);

k=0;
for j = 0:-deltR:-0.3
    w = Xtool;
    theta = j;
%     Ptraj(:,:,k) = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);
    trajEND= pivotTrajCheck(poseStart,pivot, w, theta);
    [Ls,gamma] = N_Stewart_IK(trajEND,pp,bb); 
    if any((gamma.^2 + h0^2 - Ls.^2)<0)  
        ToolangleMinus(3)=(j+deltR);
        break; 
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp
        ToolangleMinus(3)=(j+deltR);
        break;
    end                
    k=k+1;    
end
traj(:,:,6) = pivotTraj(poseStart,pivot, Xtool, (1-k)*deltR, Tf,Ts, TimeScaling);



%---------------------------------------

ToolangleWS_max = ToolanglePlus*180/pi; 
ToolangleWS_min = ToolangleMinus*180/pi;
% disp('The Tool-angle working space for the tool plane when observed from the tool frame is:')
ToolangleWS=[ToolangleWS_min ToolangleWS_max];

for m=1:size(traj,3)
    for j=1:size(traj,2)
        Ttraj(:,:,m)=poseRPY2SE3(traj(:,j,m));
        plotH(Ttraj(:,:,m),0.1);
        hold on;
    end
end
xlabel('x');ylabel('y');zlabel('z');
    
end

