clc,clear;
%% step 1
Tsup = 0.8; %���е�Ԫ
z_c = 0.8; %CoM�ĸ߶�
g = 9.8;
Tc = sqrt(z_c/g); %IPMģ�͵Ĳ���
C = cosh(Tsup/Tc); S = sinh(Tsup/Tc);
a = 10; b = 1;
D = a*(C-1).^2 + b*(S/Tc).^2;
x_step = 0.3; y_step = 0.2
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
P = [0 0];
x = [0 0];
v = [0 0];
p_star = [0 0];



for i = 2 : n
    P(1:2,i) = P(1:2,i-1) + step_table(1:2,i) .* [1;-(-1).^i];
end % ��ŵ� Px & Py
for i = 1:n-1
    xbar(1:2,i) = step_table(1:2, i+1) .* [1; (-1)^(i+1)]./2;
end
vbar = [(C+1)/Tc/S.* (xbar(1,:)) ; (C-1)/Tc/S.* (xbar(2,:))];
%
xi = xbar .*[-1;1];
vi = vbar .*[1;-1];

for i= 2 : n-1
        t = (0:0.01:Tsup);
        X = xi(1:2,i-1) .* cosh( t./Tc ) + Tc .* vi(1:2,i-1) .*sinh(t./Tc);
        X = X + P(1:2,i);
        plot(X(1,:),X(2,:)); hold on;
        t = t + Tsup;
end
plot(P(1,1:n), P(2,1:n),'*');
figure;
%% ���в����������ַ���ȷ���Ĳ��е�Ԫ�����ڲ��п�ʼ�ͽ���ʱ�ǲ������ġ���
% ������Ϊɶ

xd =  xbar + P(1:2,1:n-1);
xd_d = vbar;
Px = -a*(C-1)/D *( xd - C*xi - Tc*S*vi ) - b*S/Tc/D * (xd_d - S/Tc*xi- C*vi); % Px* Py*
%
xf_mod = C .* xi  + Tc*S .* vi + (1-C) * Px; 
xf_d_mod = S/Tc .*xi  + C.* vi  + (-S/Tc) * Px;
%{
for i= 3 : n    %n?
        t = (0:0.01:Tsup);
        X = xi(1:2,i-1) .* cosh( t./Tc ) + Tc .* vi(1:2,i-1) .*sinh(t./Tc) + (1 - cosh(t./Tc)) .* Px(1:2,i-2);
        X = X + P(1:2,i);
        plot(X(1,:),X(2,:)); hold on;
        t = t + Tsup;
end
%}
Px = [0 , Px(1,:) ; 0 , Px(2,:)];
for i = 2 : n-1
    t = (0:0.01:Tsup);
    X = xi(1:2,i-1) .* cosh( t./Tc ) + Tc .* vi(1:2,i-1) .*sinh(t./Tc) + (1 - cosh(t./Tc)) .* Px(1:2,i-1);    
    X = X + P(1:2,i);
    plot(X(1,:),X(2,:)); hold on;
    P(1:2,i) = P(1:2,i-1) + step_table(1:2,i) .* [1;-(-1).^i];
    %P(1:2,i) = P(1:2,i-1) + -+ s(1:2,i);
    xbar(1:2,i) = step_table(1:2, i+1) .* [1; (-1)^(i+1)]./2;
    % xbar ybar
    xd(1:2,i) = P(1:2,i) + xbar(1:2,i);
    xd_d(1:2,i) = vbar(1:2,i);
    % xd vd
    Px(1:2,i) = -a*(C-1)/D *( xd(1:2,i) - C*xi(1:2,i) - Tc*S*vi(1:2,i) ) - b*S/Tc/D * (xd_d(1:2,i) - S/Tc*xi(1:2,i)- C*vi(1:2,i)); % Px* Py*
    % ����õ�Px
    
end

plot(P(1,1:n), P(2,1:n),'*');
    