function Ja = ajacobian(q,myrobot)

    H = forward(q,myrobot);
    R = H(1:3,1:3);
    
    phi = atan2(R(2,3),R(1,3));
    theta = atan2(real(sqrt(1-R(3,3)^2)),R(3,3));
    
    B = [0 -sin(phi) cos(phi)*sin(theta);
         0 cos(phi)  sin(phi)*sin(theta);
         1    0                cos(theta)];
    
    Binv = inv(B);
    I = eye(3);
    o = I-I;
    J = jacobian(q,myrobot);
    Ja = [I o;
          o Binv] * J;

end