function traj = N_SinTrack04_Su(TMtool,Tstart,velocity,Ts,TimeScaling,LineSpace,StepRange,ScanRange,Id)

%%%-- 說明
% 在PI的軟體介面中，給定的變數
% 1. PI在Sine波的mid pose，就是我們的當前pose，所以"使用前必須用Move L移動至Start Pose"
% 2. v 為速度
% 3. Ts 為Sample Time
% 4. LineSpace為Sine間距
% 5. ScanRange為Sine震幅
%-----------------------------------
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
% clear;clc;
% % pstart = [0;0;155.0851057814131;0;0;0];
% % pivot=[4.0;0.0;2.0];
% % alpha = pi/2;
% % beta = 0.0 ;  % depends on real situation; 
% velocity = 3;
% Ts= 0.01;
% TimeScaling = '3';
% LineSpace = 0.1;
% StepRange = 3;ScanRange = 3;
% Id = 'V';
%---------------------------

% Ry=Roty(alpha);
% Rx=Rotx(beta);
% PMtool = pivot;
% RMtool = Ry*Rx;
% TMtool = [RMtool PMtool;0 0 0 1];

Pstart = SE3ToPoseRPY(Tstart);
TtoolStart = Tstart*TMtool;

ToolTranslx = TtoolStart(1:3,1);
ToolTransly = TtoolStart(1:3,2);
ToolTranslz = TtoolStart(1:3,3);

ax = Pstart(1);
ay = Pstart(2);
az = Pstart(3);

Lupp=58.5;
Llow=50.5;

h0 = 21.2;
g0 = 49.41;

StepRange = StepRange / 2;
ScanRange = ScanRange / 2;
Tf = (StepRange * 2 * pi) / (2 * LineSpace);
t = 1/velocity;

%bbi fixed platform global coordinates
b1=[27.79095418441866; -25.6223118691413; 0];
b2=[8.294095890154637; -36.87882825368669; 0];
b3=[-36.0850500745733; -11.25651638454532; 0];
b4=[-36.0850500745733; 11.25651638454532; 0];
b5=[8.294095890154637; 36.87882825368669; 0];
b6=[27.79095418441866; 25.6223118691413; 0];

bb=[b1 b2 b3 b4 b5 b6];

%ppi moving platform coordinates
p1=[25.20360654775701; -8.3474976481494; 0];
p2=[-5.372658252550269; -26.00071236142009; 0];
p3=[-19.83094829520674; -17.65321471327068; 0];
p4=[-19.83094829520674; 17.65321471327068; 0];
p5=[-5.372658252550269; 26.00071236142009; 0];
p6=[25.20360654775701; 8.3474976481494; 0];

pp=[p1 p2 p3 p4 p5 p6];
K=ceil(Tf/(Ts*(1/t)));
X=[];
Y=[];


if (Id=='H')    
    Translv = ToolTransly;
    Translh = ToolTranslx;
elseif (Id=='V')    
    Translv = ToolTranslx;
    Translh = ToolTransly;
end

X1=StepRange * (-1)*Translh;
Y1=ScanRange * Translv;
Z1= 0.0*ToolTranslz;
Trans(:,1) = [ax;ay;az]+X1+Y1+Z1; 
Traj(:,1)=[Trans(:,1);Pstart(4:6)];
Pend = Traj(:,1);
traj0 = N_moveL_PP(Pstart, Pend, TimeScaling,velocity, Ts);
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
    [Ls,gamma] = N_Stewart_IK(Traj(:,k),pp,bb);
    % length of the g-actuators
    if any((gamma.^2 + h0^2 - Ls.^2)<0) 
        M=k-1;
        break;  
    end
    g = gamma-sqrt(gamma.^2 + h0^2 - Ls.^2);    
    LL(:,k) = g;
    if (min(min(LL(:,k)))<Llow)||(max(max(LL(:,k)))>Lupp) 
        M=k-1;
        break;
    end 
    M=k;
end

%-----Return to Pstart----
temp = Pstart;
Pend = temp;
Pstart = Traj(:,end);
traj2 = N_moveL_PP(Pstart, Pend, TimeScaling,velocity, Ts);

for i=1:size(traj2,2)
    Tooltraj2(:,i) = TMtool(1:3,4)+traj2(1:3,i);
end
%-------------------------

traj = [traj0 Traj(:,1:M) traj2];

Xmin = min(Traj(1,1:M));Xmax=max(Traj(1,1:M));
Ymin = min(Traj(2,1:M));Ymax=max(Traj(2,1:M));
Zmin = min(Traj(3,1:M));Zmax=max(Traj(3,1:M));
XYZ=[Xmin Xmax; Ymin Ymax; Zmin Zmax]; 

plot3(Tooltraj0(1,1:end)',Tooltraj0(2,1:end)',Tooltraj0(3,1:end)','o'); hold on;
plot3(ToolTrans(1,1:M)',ToolTrans(2,1:M)',ToolTrans(3,1:M)','o'); 
plot3(Tooltraj2(1,1:end)',Tooltraj2(2,1:end)',Tooltraj2(3,1:end)','o');
%----------
RR=[ToolTranslx ToolTransly ToolTranslz];
% TP=TPstart*TMtool;
% PP=TP(1:3,4);
PP=[ax;ay;az]+TMtool(1:3,4);
TT=[RR PP;0 0 0 1];
plotH(TT);
%-----
xlabel('x');ylabel('y');zlabel('z');
grid on; 
