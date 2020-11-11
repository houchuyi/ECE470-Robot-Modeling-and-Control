% Chuyi (Sky) Hou - 1004197834
% YuQing (Kelly) Yuan - 1004039827

%A function that initialize the robotic arm PUMA560 using robotics toolbox.
function myrobot =  mypuma560(DH)
    Jm = [2*10^-4;2*10^-4;2*10^-4;3.3*10^-5;3.3*10^-5;3.3*10^-5];
    B = [1.48*10^-3; 8.17*10^-3; 1.38*10^-3; 7.12*10^-5; 8.26*10^-5; 3.67*10^-5];
    for i = 1:6
        L(i) = Link(DH(i,:));
        L(i).Jm = Jm(i);
        L(i).B = B(i);
    end
    myrobot = SerialLink(L,'name','PUMA560');
end