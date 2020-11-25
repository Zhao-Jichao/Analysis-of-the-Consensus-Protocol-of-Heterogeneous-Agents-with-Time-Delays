clear
clc

% 连续时间一致性协议验证



% 程序主函数代码如下：
t0 = 0;
tfinal = 10;

X0 = [-7.50; 0.25; -7.50; 4.75; 3.00; 7.00];

[t, Xt] = ode45(@SunFun, [t0 tfinal], X0);

% 绘制结果图
plot(t,Xt(:,1), t,Xt(:,2), t,Xt(:,3), t,Xt(:,4), t,Xt(:,5), t,Xt(:,6), 'linewidth', 1.5)
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6')
xlabel('time(sec)')
ylabel('node values')
title('fixed topology convergence')
grid

% 微分方程函数，状态导数
function xdot = SunFun(t,x)
D = [
1 0 0 0 0 0;
0 1 0 0 0 0;
0 0 1 0 0 0;
0 0 0 1 0 0;
0 0 0 0 1 0;
0 0 0 0 0 1;];

A = [
0 1 0 0 0 0;
0 0 1 0 0 0;
0 0 0 1 0 0;
0 0 0 0 1 0;
0 0 0 0 0 1;
1 0 0 0 0 0;];
L = D - A;

% 导数关系式
xdot = -L * x;

end
