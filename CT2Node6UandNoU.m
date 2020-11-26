clear
clc

% 程序主函数代码如下：
t0 = 0; tt = 20;
y0 = 0; yt = 6;

X0 = [-7.50, 0.25, -6.25, 4.75, -2.25, 6.50];


[t, Xt] = ode45(@SunFun, [t0 tt], X0);
[tU, XtU] = ode45(@SunFunU, [t0 tt], X0);

% 绘制结果图
% subplot(2,3,1); plot(t,Xt(:,1),'linewidth',1.5); grid; axis([t0,6,y0,yt]); 
% subplot(2,3,2); plot(t,Xt(:,2),'linewidth',1.5); grid;
% subplot(2,3,3); plot(t,Xt(:,3),'linewidth',1.5); grid;
% subplot(2,3,4); plot(t,Xt(:,3),'linewidth',1.5); grid;
% subplot(2,3,5); plot(t,Xt(:,5),'linewidth',1.5); grid;
% subplot(2,3,6); plot(t,Xt(:,6),'linewidth',1.5); grid;

subplot(2,1,1); plot(t,Xt(:,1), t,Xt(:,2), t,Xt(:,3), t,Xt(:,4), t,Xt(:,5), t,Xt(:,6), 'linewidth',1.5); 
xlabel('Time(sec)'); ylabel('Nodes States');
legend('x_1','x_2','x_3','x_4','x_5','x_6')
title('No U Control');
grid

subplot(2,1,2); plot(tU,XtU(:,1), tU,XtU(:,2), tU,XtU(:,3), tU,XtU(:,4), tU,XtU(:,5), tU,XtU(:,6), 'linewidth',1.5); 
xlabel('Time(sec)'); ylabel('Nodes States');
legend('x_1','x_2','x_3','x_4','x_5','x_6')
title('Contains U Control');
grid

% 微分方程函数，状态导数
function xdot = SunFun(t,x)
% 参数
T1 = 1.0; T2 = 2.0; T3 = 3.0; T4 = 4.0; T5 = 5.0; T6 = 6.0;
K1 = 6.0; K2 = 2.2; K3 = 2.8; K4 = 4.4; K5 = 1.5; K6 = 2.6;
A =[-1/T1 0 0 0 0 0;
    0 -1/T2 0 0 0 0;
    0 0 -1/T3 0 0 0;
    0 0 0 -1/T4 0 0;
    0 0 0 0 -1/T5 0;
    0 0 0 0 0 -1/T6;];
B =[K1/T1 0 0 0 0 0;
    0 K2/T2 0 0 0 0;
    0 0 K3/T3 0 0 0;
    0 0 0 K4/T4 0 0;
    0 0 0 0 K5/T5 0;
    0 0 0 0 0 K6/T6;];

% 导数关系式
xdot = (A) * x;
end

function xdot = SunFunU(t,x)
% 参数
T1 = 1.0; T2 = 2.0; T3 = 3.0; T4 = 4.0; T5 = 5.0; T6 = 6.0;
K1 = 6.0; K2 = 2.2; K3 = 2.8; K4 = 4.4; K5 = 1.5; K6 = 2.6;
A =[-1/T1 0 0 0 0 0;
    0 -1/T2 0 0 0 0;
    0 0 -1/T3 0 0 0;
    0 0 0 -1/T4 0 0;
    0 0 0 0 -1/T5 0;
    0 0 0 0 0 -1/T6;];
B =[K1/T1 0 0 0 0 0;
    0 K2/T2 0 0 0 0;
    0 0 K3/T3 0 0 0;
    0 0 0 K4/T4 0 0;
    0 0 0 0 K5/T5 0;
    0 0 0 0 0 K6/T6;];
DegreeMatrix = [
    1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;];
AdjaceMatrix = [
    0 1 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 1 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1;
    1 0 0 0 0 0;];
L = DegreeMatrix - AdjaceMatrix;

% 导数关系式
xdot = (A) * x + (B) * (-L) * x;
end

