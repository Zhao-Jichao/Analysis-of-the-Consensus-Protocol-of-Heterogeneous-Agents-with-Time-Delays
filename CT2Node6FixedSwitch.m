clear
clc

% 程序主函数代码如下：
t0 = 0; tt = 5;

X0 = [-7.50, 0.25, -6.25, 4.75, -2.25, 7.00];

global A B L_a L_b L_c

T1 = 0.81; T2 = 0.27; T3 = 0.95; T4 = 0.79; T5 = 0.67; T6 = 0.70;
K1 = 0.91; K2 = 0.55; K3 = 0.49; K4 = 0.96; K5 = 0.76; K6 = 0.03;

A =[-1/T1 0 0 0 0 0;0 -1/T2 0 0 0 0;0 0 -1/T3 0 0 0;0 0 0 -1/T4 0 0;0 0 0 0 -1/T5 0;0 0 0 0 0 -1/T6;];
B =[K1/T1 0 0 0 0 0;0 K2/T2 0 0 0 0;0 0 K3/T3 0 0 0;0 0 0 K4/T4 0 0;0 0 0 0 K5/T5 0;0 0 0 0 0 K6/T6;];

D_a = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_a = [0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_a = D_a - A_a;

D_b = [2 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_b = [0 1 0 1 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_b = D_b - A_b;

D_c = [2 0 0 0 0 0;0 2 0 0 0 0;0 0 2 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_c = [0 1 0 1 0 0;0 0 1 0 1 0;0 0 0 1 0 1;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_c = D_c - A_c;


[tUa, XtUa] = ode45(@SunFunUa, [t0 tt], X0);
[tUb, XtUb] = ode45(@SunFunUb, [t0 tt], X0);
[tUc, XtUc] = ode45(@SunFunUc, [t0 tt], X0);
[t, Xt] = ode45(@SunFun, [t0 tt], X0);

% 绘制结果图
subplot(2,2,1); 
plot(tUa,XtUa(:,1), tUa,XtUa(:,2), tUa,XtUa(:,3), tUa,XtUa(:,4), tUa,XtUa(:,5), tUa,XtUa(:,6), 'linewidth',1.5); 
xlabel('Time(sec)'); ylabel('Nodes States');
legend('x_1','x_2','x_3','x_4','x_5','x_6');
title('Fixed Topology G_a');
grid

subplot(2,2,2); 
plot(tUb,XtUb(:,1), tUb,XtUb(:,2), tUb,XtUb(:,3), tUb,XtUb(:,4), tUb,XtUb(:,5), tUb,XtUb(:,6), 'linewidth',1.5); 
xlabel('Time(sec)'); ylabel('Nodes States');
legend('x_1','x_2','x_3','x_4','x_5','x_6');
title('Fixed Topology G_b');
grid

subplot(2,2,3); 
plot(tUc,XtUc(:,1), tUc,XtUc(:,2), tUc,XtUc(:,3), tUc,XtUc(:,4), tUc,XtUc(:,5), tUc,XtUc(:,6), 'linewidth',1.5); 
xlabel('Time(sec)'); ylabel('Nodes States');
legend('x_1','x_2','x_3','x_4','x_5','x_6');
title('Fixed Topology G_c');
grid

subplot(2,2,4)
plot(t,Xt(:,1), t,Xt(:,2), t,Xt(:,3), t,Xt(:,4), t,Xt(:,5), t,Xt(:,6), 'linewidth', 1.5)
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
xlabel('Time(sec)'); ylabel('Nodes States'); 
title('Switching Topology G_a -> G_b -> G_c'); 
grid


% 微分方程函数，状态导数
function xdot = SunFunUa(t,x)
% 参数
global A B L_a

% 导数关系式
xdot = (A) * x + (B) * (-L_a) * x;
end

function xdot = SunFunUb(t,x)
% 参数
global A B L_b

% 导数关系式
xdot = (A) * x + (B) * (-L_b) * x;
end

function xdot = SunFunUc(t,x)
% 参数
global A B L_c

% 导数关系式
xdot = (A) * x + (B) * (-L_c) * x;
end

function xdot = SunFun(t,x)
% 参数
global A B L_a L_b L_c
L = L_a;

% 切换信号
% 切换是判断时间对 3 取余数
% 如果余数为 0 那就使用 L_a 拓扑结构，以此类推

st = mod(t,3);

if st>0 && st<=1
    L = L_a;
elseif st>1 && st<=2
    L = L_b;
elseif st>2 && st<=3
    L = L_c;
end

% 导数关系式
xdot = (A) * x + (B) * (-L) * x;

end