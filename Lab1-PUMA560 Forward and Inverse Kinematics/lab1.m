clear;
close all;

% Section 4.1 
% DH parameters in rest position
DH = [
    % theta  d       a       alpha
       0,    0.76,     0,      pi/2; 
       0,    -0.2365, 0.4323, 0;
       0,    0,      0,      pi/2; 
       0,    0.4318,  0,      -pi/2; 
       0,    0,      0,      pi/2;
       0,    0.20,     0,      0 
    ];
% DH= [
%     % theta  d       a       alpha
%        0,    76,     0,      -pi/2; 
%        0,    -23.65, 43.18,  0;
%        0,    -15,    0,      -pi/2; 
%        0,    43.18,  0,      pi/2; 
%        0,    0,      0,      -pi/2;
%        0,    20,     0,      0 
%     ];

% Create robot structure myrobot
myrobot = mypuma560(DH);

% Section 4.2
% 200 joint angles
n = 200;
% Joint angle tnrasitions
theta1 = linspace(0,pi,n)';
theta2 = linspace(0,pi/2,n)';
theta3 = linspace(0,pi,n)';
theta4 = linspace(pi/4,3*pi/4,n)';
theta5 = linspace(-pi/3,pi/3,n)';
theta6 = linspace(0,2*pi,n)';

% Plot robot evolution corresponding to joint angles
% q = [theta1 theta2 theta3 theta4 theta5 theta6];
% figure
% plot(myrobot,q);

% Section 4.3
o = [];
% For each row of q, computer the coordinates of the end effector
% for row = 1:length(q)
%     H = forward(q(row,:),myrobot);
%     D = H(1:3,4)';
%     o = [o;D];
% end

% Plot the trajectory using red colour
% figure
% plot3(o(:,1),o(:,2),o(:,3),'r')
% hold on
% plot(myrobot,q)

% Section 4.4 
% Use the following H to test that inverse function is working
H = [
    cos(pi/4), -sin(pi/4), 0, 0.2;
    sin(pi/4), cos(pi/4),  0, 0.23;
    0,         0,          1, 0.15;
    0,         0,          0, 1
    ];
% This should equal 
% [-0.0331, -1.0667, 1.0283, 3.1416, 3.1032, 0.8185]
q = inverse(H, myrobot)

% Pick up object and move it with end effector orientation constant
x = linspace(0.10,0.30,100);
y = linspace(0.23,0.30,100);
z = linspace(0.15,1.00,100);
d = [x',y',z'];
R = [
    cos(pi/4), -sin(pi/4), 0;
    sin(pi/4), cos(pi/4),  0;
    0,         0,          1
    ];

q = [];
for i=1:length(d)
    bigH = [R d(i,:)'; 0 0 0 1];
    insideq = inverse(bigH, myrobot);
    q = [q;insideq];
end
figure
plot3(d(:,1),d(:,2),d(:,3),'r')
hold on
plot(myrobot,q);
