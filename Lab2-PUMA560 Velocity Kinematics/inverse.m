
function q = inverse(H, myrobot)

    Oh = H(1:3,4);
    Rd = H(1:3,1:3);
    Oc = Oh - Rd*[0;0;myrobot.d(6)];

    xc = Oc(1);
    yc = Oc(2);
    zc = Oc(3);

    %setting up some varibles.
    a2 = myrobot.a(2);
    d1 = myrobot.d(1);
    d2 = myrobot.d(2);
    d4 = myrobot.d(4);
    
    % CAlculate theta 1
    b = atan2(yc,xc);
    c = atan2(-d2,sqrt(xc^2+yc^2 - d2^2)); 
    theta1 = b - c;
    
    %cacluate theta 3
    PRsqed = (zc-d1)^2+xc^2+yc^2-d2^2;
    cosPhi= (PRsqed-(a2)^2-(d4)^2)/(2*a2*d4);
    theta3 = atan2(cosPhi,real(sqrt(1-cosPhi^2)));

    %calculate theta 2
    Psi1 = atan2(-d4*cos(theta3),a2+d4*sin(theta3));
    Psi2 = atan2(zc-d1, sqrt(xc^2+yc^2-(d2)^2));
    theta2 = Psi2 - Psi1;
    
    thetas = [theta1, theta2, theta3];

    % Find R30 and R63
    H30 = 1.00000;
    for i = 1:3
        A = DHFunction(thetas(i), myrobot.alpha(i), myrobot.a(i), myrobot.d(i));
        H30 = H30*A;
    end
    R30 = H30(1:3,1:3);
    R63 = R30'*Rd;
    
    % Find theta4, theta5, theta6
    theta5 = atan2(real(sqrt(1-R63(3,3)^2)),R63(3,3));
    theta4 = atan2(R63(2,3),R63(1,3));
    theta6 = atan2(R63(3,2),-R63(3,1));
    
    q = [theta1,theta2,theta3,theta4,theta5,theta6];

end

function A = DHFunction(theta, alpha, a, d)
    A = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1
        ];
end