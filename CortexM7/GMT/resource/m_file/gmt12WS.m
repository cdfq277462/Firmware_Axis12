function  [ToolWS, Tstart,TPhome] = gmt12WS(ToolPoseWorking)
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
% clear;clc;close all;
% ToolPoseWorking=[0.0;0.0;0.0;0.0;0.0;0.0]; % one component at a time
%------------------------------------------------------
format long

TPhome = eye(4);

persistent T0
persistent firstRun

if isempty(firstRun)
    T0 = TPhome;    
    firstRun = 1;
end
%-------------Parameters------------------
xmin=-25; xmax=25;
ymin=-25; ymax=25;
zmin=-10; zmax=10;
rzmin= (-8.2)*pi/180; rzmax= (8.2)*pi/180;
rymin= (-3.4)*pi/180; rymax= (3.4)*pi/180;
rxmin= (-4.4)*pi/180; rxmax= (4.4)*pi/180;

%----------Tstart------------------------
TtoolPoseWorking=poseRPY2SE3(ToolPoseWorking);
T0 =T0*TtoolPoseWorking;
Tstart=T0;
Rstart = Tstart(1:3,1:3);
Pstart = Tstart(1:3,4);

[phi, theta, psi]=SO3ToRPY(Rstart);
ax=Pstart(1);
ay=Pstart(2);
az=Pstart(3);

Pdx = xmax-ax; Mdx= xmin-ax;
Pdy = ymax-ay; Mdy= ymin-ay;
Pdz = zmax-az; Mdz= zmin-az;
Pdrz = rzmax-phi; Mdrz = rzmin-phi;
Pdry = rymax-theta; Mdry = rymin-theta;
Pdrx = rxmax-psi; Mdrx = rxmin-psi;

Pd = [Pdx;Pdy;Pdz;Pdrz;Pdry;Pdrx];
Md = [Mdx;Mdy;Mdz;Mdrz;Mdry;Mdrx];

P = Pd<0;
M = Md>0;
if any(P)||any(M)
    Pd
    Md
    error('Tool Frame is out of range')
end

ToolWS = [Mdx Pdx;Mdy Pdy;Mdz Pdz;Mdrz Pdrz;Mdry Pdry;Mdrx Pdrx];

% plotH(TPhome);hold on;
% plotH(Tstart);

end



