close
clc
clear

load("Href.mat")

% Set up myrobot
DH = [
    % theta  d       a       alpha
       0,    0.76,     0,      pi/2; 
       0,    -0.2365, 0.4323, 0;
       0,    0,      0,      pi/2; 
       0,    0.4318,  0,      -pi/2; 
       0,    0,      0,      pi/2;
       0,    0.20,     0,      0 
    ];
myrobot = mypuma560(DH);


% Section 4.1 - Computation of joint reference signals
qref = zeros(6,length(Href));
qdref = zeros(size(qref));
qdref1 = zeros(size(qref));
for i = 1:length(Href)
    % Question 1: Fill in qref
    q = inverse(Href(:,:,i), myrobot);  
    qref(:,i) = q';
 
    % Question 2a: Find odot and omega
    H_notdot = Href(:,:,i);
    H_odot = Hrefdot(:,:,i);
    odot = H_odot(1:3,4);
    Rdot = H_odot(1:3,1:3);
    R_notdot = H_notdot(1:3,1:3);
    s_omega = Rdot*transpose(R_notdot);
    omega = [s_omega(3,2); s_omega(1,3); s_omega(2,1)];
    
    % Question 2b
    qdref(:,i) = inv(jacobian(q, myrobot))*[odot;omega];
    
    % Question 2c
    ptp = tr2eul(Href(:,:,i));
    phi = ptp(1);
    theta = ptp(2);
    psi = ptp(3);
    
    if(sin(theta) < 0.00001)
        continue
    else
        % Question 2d
        B = [
            0 -sin(phi) cos(phi)*sin(theta);
            0 cos(phi)  sin(phi)*sin(theta);
            1 0         cos(theta)        ];
        alphadot = inv(B)*omega;
        
        % Question 2e
        qdref1(:,i) = inv(ajacobian(q, myrobot))*[odot;alphadot];
        if (norm(qdref1(:,i)-qdref(:,i)) >= 0.00001)
            str = sprintf("There's a mistake in your code.")
        end
    end
end

% Question 3
splineqref = spline(t,qref);
splineqdref = spline(t,qdref);

% Question 4
d = length(splineqdref.coefs);
splineqddref = splineqdref;
splineqddref.coefs = splineqdref.coefs(:,1:3).*(ones(d,1)*[3 2 1]);
splineqddref.order = 3;

% Section 4.2 - Test Independent Joint Controller
% Question #2
sys = @(t,x)motors(t,x,myrobot,splineqref,splineqdref,splineqddref);
Ts = 0.02;
q0 = [3*pi/2; zeros(11,1)];
[t,q] = ode45(sys,[0:Ts:6*pi]', q0);

% Question #3 
qref = ppval(splineqref, t)';
for i = 1:6
    figure
    plot(t, q(:,i)', t, qref(:,i)')
    title('q and qref')
    xlabel('time (t)')
    ylabel('q and qref')
    legend('q','qref')
end

% Question #4
figure
hold on;
oref = squeeze(Href(1:3,4,:));
plot3(oref(1,:), oref(2,:), oref(3,:), 'r')
view(-125, 40);
plot(myrobot,q(:,1:6))
hold off;














