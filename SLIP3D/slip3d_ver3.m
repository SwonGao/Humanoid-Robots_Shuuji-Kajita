clc,clear;
%% step 1
global Tsup; Tsup = 0.8; %���е�Ԫ
global z_c; z_c = 0.8; %CoM�ĸ߶�
g = 9.8;
global Tc; Tc = sqrt(z_c/g); %IPMģ�͵Ĳ���
global C; C = cosh(Tsup/Tc);    global S; S = sinh(Tsup/Tc);
global a; a = 10;   global b; b = 1;
global D; D = a*(C-1).^2 + b*(S/Tc).^2;
global dt; dt = 0.001;
x_step = 0.3; y_step = 0.2;
%{
s�� ���в�������������
P�� ���λ��
xbar��vbar: ���в�����ȷ�����й켣��
xi ,vi����n��ʱ�ĳ�ʼ״̬
xf ,vf����n��ʱ������״̬
xd ,xd_d��������ֹ״̬ desired
Px��������ŵ�
%}
step_table = [0.0 0.0 0.3 0.3 0.3 0.0 0.0 ; 0.0 0.2 0.2 0.2 0.2 0.2 0.0];
temp = size(step_table);
n = temp(2);
P = [0;0];
x = [0;0];
v = [0;0];
p_star = [0;0];
t = 0; 
global i;
for i = 1 : n-1
    [p_star,p_sup] = calc_p_star(p_star,x,v,step_table,i);
    for j = 1: 800
        [x,v] = cal_ipm(x,v,p_star,dt);
        if mod(j,50) == 1
            plot3(x(1,1),x(2,1),z_c,'.','markersize',10);hold on;
            plot3([p_star(1,1), x(1,1)], [p_star(2,1),x(2,1)], [0,z_c]);
        end
    end
    plot3(p_sup(1,1),p_sup(2,1), 0,'o','markersize',5); hold on;
    plot3(p_star(1,1),p_star(2,1),0 ,'x','markersize',5);
end

function [p_star,p_sup_n] = calc_p_star(p_sup,x,v,step_table,i)
    global C;global S; global D; global Tc; 
    global a; global b; global dt; global z_c;
    p_sup_n = p_sup + step_table(:,i) .* [1; (-1)^i];
    xbar = step_table(1:2, i+1) .* [1; (-1)^(i+1)]./2;
    vbar = [(C+1)/Tc/S .* (xbar(1,1)) ; (C-1)/Tc/S .* (xbar(2,1))];
    xd =  xbar + p_sup_n;
    xd_d = vbar;
    p_star = -a*(C-1)/D *( xd - C*x - Tc*S*v ) - b*S/Tc/D * (xd_d - S/Tc*x- C*v); % P*
end

function [x_n,v_n] = cal_ipm(x, v, p_star,dt)
    global z_c;
    g = 9.8;
    x_dd = g/z_c*(x-p_star);
    v_n = v + x_dd * dt;
    x_n = x + v * dt;
end