function [x,fx,xx] = newton(f,df,x0,Tolx,MaxIter)
% [x,fx,xx] = newton(f,x0,TolX,MaxIter)  % df is numeric
% newton.m to solve f(x) = 0 by using Newton method.
% input: f = ftn to be given as a string 
% df = df(x)/dx (If not given, numerical derivative is used.)
% x0 = the initial guess of the solution
% TolX = the upper limit of |x(k) - x(k-1)|
% MaxIter = the maximum # of iteration
% output: x = the point which the algorithm has reached
% fx = f(x(last)), xx = the history of x

%-----Example Inputs-------
% clear;clc;
% Xm=19.0;
% c = 10
% b=Xm/(2*pi*c);
% Rx=Xm/2;
% N=5000;
% f = @(T) (b/2)*(log(T+sqrt(T^2+1))+T*sqrt(T^2+1))-2*pi*Rx*N;
% syms T
% df=diff(f,T);
% df=matlabFunction(df);
% x0 = 1.8; Tolx = 1e-5; MaxIter = 50;  %with initial guess 1.8
% ----------------------------


h = 1e-4; h2 = 2*h; TolFun=eps; % eps =  2.220446049250313e-16;
if nargin == 4 && isnumeric(df)
    MaxIter = Tolx; 
    TolX = x0; x0 = df; 
end
xx(1) = x0; 
fx = feval(f,x0);  
% fx = f(x0);
for k = 1: MaxIter
    if ~isnumeric(df), dfdx = feval(df,xx(k)) %derivative function
    else dfdx = (feval(f,xx(k) + h)-feval(f,xx(k) - h))/h2; %numerical drv
    end
%         dfdx = feval(df,xx(k));
%         dfdx = (feval(f,xx(k) + h)-feval(f,xx(k) - h))/h2;   
    dx = -fx/dfdx;
    xx(k+1) = xx(k)+dx; %Eq.(4.4.2)
    fx = feval(f,xx(k + 1));
%     abs(fx)
%     TolFun
%     abs(dx)
%     Tolx
    if abs(fx)<TolFun | abs(dx) < Tolx
        break; 
    end
end
x = xx(k + 1);
if k == MaxIter 
    fprintf('The best in %d iterations\n',MaxIter);
end