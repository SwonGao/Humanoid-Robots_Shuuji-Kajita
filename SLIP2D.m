%{
g = 9.8;
z = 0.8;
Tc = sqrt(z/g);

x0 = -0.2;
x_d0 = 0.8;
t=(0: 0.01: 0.8);
x = x0 .* cosh( t/Tc ) + Tc .* x_d0 .*sinh(t/Tc);
x_d = x0./Tc .* sinh(t/Tc) + x_d0 .* cosh(t/Tc);
E = x_d.*x_d./2 - g./(2.*z).*x.*x;  %已验证：轨道能量相同
figure;
plot(t , x , 'g', t , x_d , 'r', t,E ,'b');

pause;
%}
%% 双足步态规划
% E0, E1, E2, s1, s2，Xs, Xe
clc;clear;
g = 9.8;
z = 0.8;
Tc = sqrt(z/g);

s = 0.4; %步长
v  = 0.1; %初始速度

Xs = 0; %初始位置
E0 = - g./(2.*z).*Xs.^2;
E1 = v.^2/2;

xf0 = z/g/s*(E1 - E0)+s/2;
vf0 = sqrt(2.*E0 + g/z * xf0^2);
t0 = Tc * log( (xf0+Tc * vf0) / (Xs + Tc * v) ); %求得交换点的时间
t = (0: 0.001 : t0);
x = Xs .* cosh( t/Tc ) + Tc .* v .*sinh(t/Tc); %交换之前的轨迹
x_d = Xs./Tc .* sinh(t/Tc) + v .* cosh(t/Tc); %交换之前的速度
subplot(211); plot(t,x,'r');    hold on;
subplot(212); plot(t,x_d,'b');  hold on;

Xs = -0.2; %初始位置相对于最低速度的点是-0.2
E1 = v.^2/2;
E2 = E1;
xf1 = z/g/s*(E2 - E1)+s/2;
vf1 = sqrt(2.*E1 + g/z * xf1^2);
t1 = Tc * log( (xf1+Tc * vf1) / (Xs + Tc * vf0) ); %求得交换点的时间
t = (0: 0.001 : t1);
x = Xs .* cosh( t/Tc ) + Tc .* vf0 .*sinh( t/Tc); %交换之前的轨迹
x_d = Xs./Tc .* sinh(t/Tc) + vf0 .* cosh(t/Tc);
t = t + t0;
subplot(211); plot(t,x,'g');    hold on;
subplot(212); plot(t,x_d,'b');  hold on;

Xs = -0.2; Xe = 0;
E2 = v.^2/2;
E3 = - g./(2.*z).*(Xe).^2;
xf2 = z/g/s*(E3 - E2)+s/2;
vf2 = sqrt(2.*E2 + g/z * xf1^2);
t2 = Tc * log( (xf2+Tc * vf2) / (Xs + Tc * vf2) ); %求得交换点的时间
t = (0: 0.001 : t2/2);
x = Xs .* cosh( t/Tc ) + Tc .* vf1 .*sinh( t/Tc); %交换之前的轨迹
x_d = Xs./Tc .* sinh(t/Tc) + vf1 .* cosh(t/Tc);
t = t + t1 + t0;
subplot(211); plot(t,x,'r');    title('X'); hold on;
subplot(212); plot(t,x_d,'b');  title('V'); hold on;
