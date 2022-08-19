function [traj,XYZ] = N_ToolSpiralTrack04_Su(pivot,alpha,beta,Tstart,ScanRange,LineSpace,velocity)

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
% pivot=[4.0;0.0;2.0];
% alpha = pi/2;
% beta = 0.0 ;  % depends on real situation;
%% Tstart=[eye(3) [0;0;66.4236]; 0 0 0 1];
% ScanRange = 4.09;  % ScanRange(max) = Zmax;
% LineSpace = 0.2;    % ScanRange*2/#(circles)
% velocity=1;   % 1<=v<=10;
%----------------------------------------------

%-----------------------------------------------------------------------
Ry=Roty(alpha);
Rx=Rotx(beta);
PMtool = pivot;
RMtool = Ry*Rx;
TMtool = [RMtool PMtool;0 0 0 1];

Pstart = SE3ToPoseRPY(Tstart);
TtoolStart = Tstart*TMtool;

ToolTranslx = TtoolStart(1:3,1);
ToolTransly = TtoolStart(1:3,2);
ToolTranslz = TtoolStart(1:3,3);

%------------------------------------
% v=v/500;
ax = Pstart(1);
ay = Pstart(2);
az = Pstart(3);
% b = b / 6.283;
Lupp=58.5;
Llow=50.5;

% h0
h0 = 21.2;
% g0
g0 = 49.41;


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

b=LineSpace/(2*pi);
ScanRange = ScanRange/2;
v=velocity/100;
Tf=(2*pi)*ScanRange/LineSpace;
K=ceil(Tf/(0.0001))+1;
X=[];
Y=[];
ToolTrans=[];
traj=[];
LL=[];
XY=[];
M=K;

for k=1:K 
    F= @(T) (b/2)*(log((T*k+sqrt(T^2*k^2+1))/(T*(k-1)+sqrt(T^2*(k-1)^2+1)))...
        +T*k*sqrt(T^2*k^2+1)-T*(k-1)*sqrt(T^2*(k-1)^2+1))-v*T;  
    x0 = 0.1;  % TolX = 1e-5; MaxIter = 50; %with initial guess 1.8
    T = newton(F,x0,1e-5,50) %1st order derivative
    %--------------------------------------      
    X=(b*(k-1)*T)*cos((k-1)*T)*ToolTranslx;
    Y=(b*(k-1)*T)*sin((k-1)*T)*ToolTransly;
    Z= 0.0*ToolTranslz;
    Trans(:,k) = [ax;ay;az]+X+Y+Z;  
%     ToolTrans(:,k)=PMtool+RMtool*Trans(:,k);
    ToolTrans(:,k)=PMtool+Trans(:,k);
    traj(:,k)=[Trans(:,k);Pstart(4:6)];  
    [Ls,alpha] = N_Stewart_IK(traj(:,k),pp,bb);    
    if any((alpha.^2 + h0^2 - Ls.^2)<0) 
        M=k-1;
        break;  
    end
    g = alpha-sqrt(alpha.^2 + h0^2 - Ls.^2);    
    LL(:,k) = g;
    if (min(min(LL(:,k)))<Llow)||(max(max(LL(:,k)))>Lupp) 
        M=k-1;
        break;
    end        
end
% avn=mean(D);  

Xmin = min(traj(1,1:M));Xmax=max(traj(1,1:M));
Ymin = min(traj(2,1:M));Ymax=max(traj(2,1:M));
Zmin = min(traj(3,1:M));Zmax=max(traj(3,1:M));
XYZ=[Xmin Xmax; Ymin Ymax; Zmin Zmax];   
% plotH(eye(4)); hold on;
plot3(ToolTrans(1,1:M)',ToolTrans(2,1:M)',ToolTrans(3,1:M)','o'); hold on;
%---
RR=[ToolTranslx ToolTransly ToolTranslz];
% TP=TPstart*TMtool;
% PP=TP(1:3,4);
PP=[ax;ay;az]+PMtool;
TT=[RR PP;0 0 0 1];
plotH(TT);
%-----
xlabel('x');ylabel('y');zlabel('z');
grid on; 

end