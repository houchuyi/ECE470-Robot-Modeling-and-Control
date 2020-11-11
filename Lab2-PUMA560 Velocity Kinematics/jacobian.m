function J = jacobian(q,myrobot)
    oi = [0;0;0];
    zi = [0;0;1];
    H = forward(q, myrobot);
    o06 = H(1:3,4);
    %obtain zi and oi
    for i = (1:6)
        if (i == 1)
            Hi = eye(4);
        else
            Hi = forward(q(1:i-1),myrobot);
        oi = [oi Hi(1:3,4)]; %o0 to o5
        zi = [zi Hi(1:3,3)]; %z0 to z5
        end
    end 
    %Jvi = zi-1 x (o06-oi) for i = 1,...,6
    Jv = cross(zi,o06-oi);
    %Jwi = zi-1 for i = 1,...,6
    Jw = zi;
    J = [Jv;Jw];
end
function A = DHFunction(theta, alpha, a, d)
    A = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1
        ];
end
function H = forward(joint, myrobot)
    H = 1;
    for i = 1:length(joint)
        A = DHFunction(joint(i), myrobot.alpha(i), myrobot.a(i), myrobot.d(i));
        H = H*A;
    end
end