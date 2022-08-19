function Ptraj = N_ToolPivotTrajY_Su(Phome,pivot,ToolPoseStart, alpha, beta,theta, Tf,Ts, TimeScaling)
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
% Xm = Zmax

%%------Example Inputs--------------------------------
% clear;clc;close all;
% %----Inputs--------
% Phome = [0;0;66.4236;0;0;0];  
% pivot=[10;0;0];
% ToolPoseStart=[0;0;0;0;0;0];
% alpha = pi/2;
% beta = 0.0 ;  % depends on real situation;
% theta = 6.0*pi/180;    %% -8.479775367936183<=theta<=7.219268218648373;
% Tf = 1.5;
% Ts = 0.01;
% TimeScaling = '3';

%----Start-----
%--------------Tool Reference Zero Pose w.r.t Moving Frame ----
Ry=Roty(alpha);
Rx=Rotx(beta);
PMtool = pivot;
RMtool = Ry*Rx;
TMtool = [RMtool PMtool;0 0 0 1];

% Given ToolPoseStart---> (MpositionStart,MOrientationStart)
ToolPstart=ToolPoseStart(1:3);
MposiStart=RMtool*ToolPstart;
thetaZ = ToolPoseStart(4);
thetaY = ToolPoseStart(5);
thetaX = ToolPoseStart(6);
MRz = AxisAngleToSO3(RMtool(:,3)*thetaZ);
MRy = AxisAngleToSO3(RMtool(:,2)*thetaY);
MRx = AxisAngleToSO3(RMtool(:,1)*thetaX);
MOrientStart = MRz*MRy*MRx;
Mpose456Start = SO3ToRPY(MOrientStart);
DposeStart= [MposiStart;Mpose456Start];
poseStart = Phome + DposeStart;

Ytool = RMtool(:,2);

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

traj= [];
Ptraj=[];

%----------Pivot motion w.r.t. pivot about Xtool-axis--------------------

w = Ytool; 
Ptraj = pivotTraj(poseStart,pivot, w, theta, Tf,Ts, TimeScaling);
%--- Check for the available rotation angles for the present tool frame---
I = size(Ptraj);
for i = 1:I(2)
    [Ls,alpha] = N_Stewart_IK(Ptraj(:,i),pp,bb);
    if any((alpha.^2 + h0^2 - Ls.^2)<0)   
        error('The angle of rotation is too big to move out of working space');
        break; 
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);
    LL = g;
    if min(LL)<Llow||max(LL)>Lupp    
        error('The angle of rotation is too big to move out of working space');
        break;
    end
end

for j=1:size(Ptraj,2)
    Ttraj(:,:,j)=poseRPY2SE3(Ptraj(:,j));
    plotH(Ttraj(:,:,j),0.1);
    hold on;
end    
xlabel('x');ylabel('y');zlabel('z');
end

