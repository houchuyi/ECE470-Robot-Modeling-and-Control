
function H = forward(joint, myrobot)
    H = 1;
    for i = 1:6
        A = DHFunction(joint(i), myrobot.alpha(i), myrobot.a(i), myrobot.d(i));
        H = H*A;
    end
end

function A = DHFunction(theta, alpha, a, d)
    A = [
        cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
        sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
        0 sin(alpha) cos(alpha) d;
        0 0 0 1
        ];
end
