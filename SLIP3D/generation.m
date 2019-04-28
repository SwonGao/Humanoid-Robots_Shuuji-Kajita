function generation()
    clc;clear;
    z_c = 0.8; %CoM的高度
    g = 9.8;
    Tc = sqrt(z_c/g); %IPM模型的参数
    Tsup = 0.8; %步行单元
    p0=[0 0]; %xbar(n-1)
    p1=[0 0.2];
    p2=[0.3 0];
    xbar = (p1+p2)/2;
    
    theta = atan(xbar(1,2)- p0(1,2))/(xbar(1,1)-p0(1,1));
    trans = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    xbar(1,1) = norm(xbar - p0)/2;
    xbar(1,2) = norm(p1 - ( xbar+p0)/2 );
    C = cosh(Tsup/Tc); S = sinh(Tsup/Tc);
    vbar = [(C+1)/Tc/S.*xbar(1,1) ; (C-1)/Tc/S.*xbar(1,2)];
    xi = xbar .*[-1 1];
    vi = vbar .*[1 -1];
    t = 0:0.01:Tsup ;
    X = [xi(1,1).*cosh(t./Tc) + Tc.* vi(1,1).*sinh(t./Tc);xi(1,2).*cosh(t./Tc) + Tc.* vi(1,2).*sinh(t./Tc)]';
    
    theta = atan(-0.1/0.15);
    trans = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    X = X;
    plot(X(:,1),X(:,2)); hold on;

    theta = atan(-0.1/0.15);
    trans = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    X = trans'*X;
    plot(X(1,:),X(2,:)); hold on;
    
    
    
%{
        xbar = [0.15; -0.1];
    
    Tsup = 0.8; %步行单元
    C = cosh(Tsup/Tc); S = sinh(Tsup/Tc);
    vbar = [(C+1)/Tc/S.*xbar(1,1) ; (C-1)/Tc/S.*xbar(2,1)];
    xi = xbar .*[-1;1];
    vi = vbar .*[1;-1];
    t = 0:0.01:Tsup ;
    X = xi.*cosh(t./Tc) + Tc.* vi.*sinh(t./Tc);
    plot(X(1,:),X(2,:)); hold on;
%}
end