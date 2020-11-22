% 连续时间一致性协议验证
% 包含固定拓扑，和放置在一起的切换拓扑

clear
clc

% 程序主函数代码如下：
t0 = 0;
tfinal = 10;

X0 = [-7.50; 0.25; -7.50; 4.75; 3.00; 7.00];

[t_a, Xt_a] = ode45(@SunFun_a, [t0 tfinal], X0);
[t_b, Xt_b] = ode45(@SunFun_b, [t0 tfinal], X0);
[t_c, Xt_c] = ode45(@SunFun_c, [t0 tfinal], X0);
[t, Xt] = ode45(@SunFun, [t0 tfinal], X0);

% 绘制结果图
figure(1)
subplot(2,2,1)
plot(t_a,Xt_a(:,1), t_a,Xt_a(:,2), t_a,Xt_a(:,3), t_a,Xt_a(:,4), t_a,Xt_a(:,5), t_a,Xt_a(:,6), 'linewidth', 1.5)
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
xlabel('time(sec)'); ylabel('node values'); title('Fixed topology a'); grid

subplot(2,2,2)
plot(t_b,Xt_b(:,1), t_b,Xt_b(:,2), t_b,Xt_b(:,3), t_b,Xt_b(:,4), t_b,Xt_b(:,5), t_b,Xt_b(:,6), 'linewidth', 1.5)
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
xlabel('time(sec)'); ylabel('node values'); title('Fixed topology b'); grid

subplot(2,2,3)
plot(t_c,Xt_c(:,1), t_c,Xt_c(:,2), t_c,Xt_c(:,3), t_c,Xt_c(:,4), t_c,Xt_c(:,5), t_c,Xt_c(:,6), 'linewidth', 1.5)
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
xlabel('time(sec)'); ylabel('node values'); title('Fixed topology c'); grid

subplot(2,2,4)
plot(t,Xt(:,1), t,Xt(:,2), t,Xt(:,3), t,Xt(:,4), t,Xt(:,5), t,Xt(:,6), 'linewidth', 1.5)
legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6');
xlabel('time(sec)'); ylabel('node values'); title('Switching topology a-->b-->c'); grid


% 微分方程函数，状态导数
function xdot = SunFun_a(t,x)
D_a = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_a = [0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_a = D_a - A_a;

L = L_a;
% 导数关系式
xdot = -L * x;

end
function xdot = SunFun_b(t,x)
D_b = [2 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_b = [0 1 0 1 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_b = D_b - A_b;

L = L_b;
% 导数关系式
xdot = -L * x;

end
function xdot = SunFun_c(t,x)
D_c = [2 0 0 0 0 0;0 2 0 0 0 0;0 0 2 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_c = [0 1 0 1 0 0;0 0 1 0 1 0;0 0 0 1 0 1;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_c = D_c - A_c;

L = L_c;
% 导数关系式
xdot = -L * x;

end
function xdot = SunFun(t,x)
D_a = [1 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_a = [0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_a = D_a - A_a;

D_b = [2 0 0 0 0 0;0 1 0 0 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_b = [0 1 0 1 0 0;0 0 1 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_b = D_b - A_b;

D_c = [2 0 0 0 0 0;0 2 0 0 0 0;0 0 2 0 0 0;0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;];
A_c = [0 1 0 1 0 0;0 0 1 0 1 0;0 0 0 1 0 1;0 0 0 0 1 0;0 0 0 0 0 1;1 0 0 0 0 0;];
L_c = D_c - A_c;

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
xdot = -L * x;

end