function dx = motors(t,x,myrobot,splineqref, ...
                          splineqdref,splineqddref)

    
    % Computation of reference signal and its first two time derivatives
    
    qr=ppval(splineqref,t);
    qdr=ppval(splineqdref,t);
    qddr=ppval(splineqddref,t);

    % state vectors
    q = x(1:6);
    qd = x(7:12);
  
    for i = 1:6    
        link = myrobot.links(i);
        J(i,1) = link.Jm;
        B(i,1) = link.B;
    end

    % Controller parameters
    % Enter your gain matrices Kp and Kd here. Kp and Kd should be
    % diagonal matrices 6x6.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Kp = [
        0.0008, 0, 0, 0, 0, 0;
        0, 0.0008, 0, 0, 0, 0;
        0, 0, 0.0008, 0, 0, 0;
        0, 0, 0, 0.000132, 0, 0;
        0, 0, 0, 0, 0.000132, 0;
        0, 0, 0, 0, 0, 0.000132
        ];
    
    Kd = [
        -0.00068, 0, 0, 0, 0, 0;
        0, -0.000017, 0, 0, 0, 0;
        0, 0, -0.00058, 0, 0, 0;
        0, 0, 0, 0.0000608, 0, 0;
        0, 0, 0, 0, 0.00000494, 0;
        0, 0, 0, 0, 0, 0.0000953
        ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ft = J.*qddr + B.*qdr; % feedforward term
    Vt = ft + Kd*(qdr-qd) + Kp*(qr-q);   % feedback controller
    qdd = (Vt-B.*qd)./J;   % acceleration
    
    dx = [qd; qdd;];