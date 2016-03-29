function RoboticsProject_Team5
%Declare joint variables symbolic 
syms t1 t2 t3 t4 t5 t6 pi
paren = @(x, varargin) x(varargin{:});
syms q1 q2 q3 L1 L2 L3 q1_d q2_d q3_d M1 M2 M3 M4 g real;
%plug in DH values
a = [3.14/2, 0, 3.14, -3.14/2, 0];
r = [33 155 135 0 0];
d = [147 0 0 0 171];
t = [t1,t2+3.14/2,t3,t4-3.14/2,t5];
%Transformation matricies
T01 = DH(a(1), r(1), d(1), t(1));
T12 = DH(a(2), r(2), d(2), t(2));
T23 = DH(a(3), r(3), d(3), t(3));
T34 = DH(a(4), r(4), d(4), t(4));
T45 = DH(a(5), r(5), d(5), t(5)); 
%Obtain T05
T02 = T01*T12; 
T03 = T02*T23; 
T04 = T03*T34; 
T05 = T04*T45;  
T05= simplify(T05);
fprintf('T05 =\n');
disp(T05);
% from Lecture 4,slide 20: Jacobian differential with respect to each angle (linear velocity)
A = [diff(T05(1:3,4), t1), diff(T05(1:3,4), t2), diff(T05(1:3,4), t3),...
    diff(T05(1:3,4), t4), diff(T05(1:3,4), t5)];
% all z are into the page therefore B can be found as (angular velocities)
B=[0 0 0 0 0;0 1 1 1 0; 1 0 0 0 1];
% combine to obtain full Jacobian
J=[A;B];
fprintf('Jacobian =\n');
disp(J);
J_spec = double(subs(J, [t1 t2 t3 t4 t5],[0 0 0 0 0]));
% tip velocity = Jspec*linear velocities
tipvel = J_spec * [pi/4 pi/4 pi/4 pi/4 pi/4]';
fprintf('J specific = \n');
disp(J_spec);
fprintf('Tip Velocity= \n');
disp(tipvel);
%find joint torques
F_gravity = [0 -1*9.81 0 0 0 0]';
Joint_Torques = J_spec'*F_gravity;
fprintf('Joint Torques =\n');
disp(Joint_Torques);

% new variables, phiWl- phi for left wheel
syms a pi r alpha phiWl phiWr
% assume(alpha, 'real');
% assign values, left wheel
alpha2 = -pi/2; 
beta = 0; 
len = 500/2; %mm
g = pi/4;
disp(alpha2);
% Equations for omni wheel, left (Lecture 5 slide 23)
rollingL = [sin(alpha2+beta+g), -cos(alpha2+beta+g), -len/(2*cos(beta+g))];
slidingL = [cos(alpha2+beta+g), sin(alpha2+beta+g),-len*sin(beta+g)];
fprintf('RollingLeft= ');
disp(rollingL);
fprintf('SlidingLeft= ');
disp(slidingL);
% assign values, right wheel
alpha2 = pi/2; 
beta = 0; 
len = 500/2; %mm
g = pi/4;
% Equations for fixed wheel, right
rollingR = [sin(alpha2+beta+g), -cos(alpha2 + beta+g), len/(2*cos(beta+g))];
slidingR = [cos(alpha2+ beta+g), sin(alpha2+beta+g), len/(2*sin(beta+g))];
fprintf('RollingRight= ');
disp(rollingR);
fprintf('SlidingRight= ');
disp(slidingR);
% simplify these two matrix equations into a single matrix equation
SimpConstraint = [rollingL;rollingR;slidingL];
%velocity kinematics of the mobile robot base 
%(defined by the robot-fixed frame, FR, on the robot between the wheels)
r=50; 
RRob0 = [1,0,0;0,1,0;0,0,1];
xi = RRob0'*inv(SimpConstraint)*[r*phiWl*cos(g);r*phiWr*cos(g);0];
fprintf('RRob0 = ');
disp(RRob0);
fprintf('xi = ');
disp(xi);
%lengths of mass positions, each mass at the center of each link
Lengths= [147, 155,135,81,137];
MLengths = Lengths/2;
syms q1 q2 q3 q4 q1dot q2dot q3dot q4dot q1dd q2dd q3dd q4dd g;
assume(q1dot, 'real');
assume(q2dot, 'real');
assume(q3dot, 'real');
assume(q4dot, 'real');
assume(q1dd, 'real');
assume(q2dd, 'real');
assume(q3dd, 'real');
assume(q4dd, 'real');
gv=[0;-g;0]; %m/s^2
r1c1=[MLengths(1);0;0];
r2c1=[Lengths(1)-MLengths(1);0;0];
r12=[Lengths(1);0;0];
r2c2=[MLengths(2);0;0];
r3c2=[Lengths(2)-MLengths(2);0;0];
r23=[Lengths(2);0;0];
r3c3=[MLengths(3);0;0];
r4c3=[Lengths(3)-MLengths(3);0;0];
r34=[Lengths(3);0;0];
r4c4=[MLengths(4);0;0];
r5c4=[Lengths(4)-MLengths(4);0;0];
r45=[Lengths(4);0;0];
r5c5=[MLengths(5);0;0];
r6c5=[Lengths(5)-MLengths(5);0;0];
r56=[Lengths(5);0;0];
w1=[0;0;q1dot];
w2=[0;0;q1dot+q2dot];
w3=[0;0;q1dot+q2dot+q3dot];
w4=[0;0;q1dot+q2dot+q3dot+q4dot];
alpha1=[0;0;q1dd];
alpha2=[0;0;q1dd+q2dd];
alpha3=[0;0;q1dd+q2dd+q3dd];
alpha4=[0;0;q1dd+q2dd+q3dd+q4dd];
% Individual Rotation matrices
R01 = [cos(q1) -sin(q1) 0; sin(q1) cos(q1) 0; 0 0 1];
R12 = [cos(q2) -sin(q2) 0; sin(q2) cos(q2) 0; 0 0 1];
R23 = [cos(q3) -sin(q3) 0; sin(q3) cos(q3) 0; 0 0 1];
R34 = [1 0 0; 0 cos(q4) -sin(q4); 0 sin(q4) cos(q4)];
R45=eye(3,3);
% Forward Recursion
%Link 1
ac1=cross(alpha1,r1c1)+cross(w1,cross(w1,r1c1));
g1=R01*gv;
ae1=cross(alpha1,r12)+cross(w1,cross(w1,r12));
%Link 2
ac2=R12'*ae1+cross(alpha2,r2c2)+cross(w2,cross(w2,r2c2));
g2=simplify(R01*R12*gv);
ae2=R12'*ae1+cross(alpha2,r23)+cross(w2,cross(w2,r23));
%Link 3
ac3=R23'*ae2+cross(alpha3,r3c3)+cross(w3,cross(w3,r3c3));
g3=simplify(R01*R12*R23*gv);
ae3=R23'*ae2+cross(alpha3,r34)+cross(w3,cross(w3,r34));
%Link 4
ac4=R34'*ae3+cross(alpha4,r4c4)+cross(w4,cross(w4,r4c4));
g4=simplify(R01*R12*R23*R34*gv);
ae3=R34'*ae3+cross(alpha4,r45)+cross(w4,cross(w4,r45));
% Backwards Recursion
f5=5*g4;%5 kg is weight of rock
T5=[0;0;0];
%Torque and force for link 4, weight of link 3 kg
f4=R45'*f5+3*ac4+3*g4;
T4=R45'*T5-cross(f4,r4c4)+cross(R45'*f5,r5c4);
%Torque and force for link 3, weight of link 2 kg
f3=R34'*f4+2*ac3+2*g3;
T3=R34'*T4-cross(f3,r3c3)+cross(R34'*f4,r4c3);
%I3 terms cancel out because link is point mass
%Torque and force for link 2, weight 3 kg
f2=R23'*f3+3*ac2+3*g2;
T2=R23'*T3-cross(f2,r2c2)+cross(R23'*f3,r3c2);
%Torque and force for link 1, weight 2 kg
f1=R12'*f2+2*ac1+2*g1;
T1=R12'*T2-cross(f1,r1c1)+cross(R12'*f2,r2c1);
Torques = [T1(3) T2(3) T3(3)]';
%Partial derivatives to find M
M = simplify([diff(Torques,q1dd) diff(Torques,q2dd) diff(Torques,q2dd)]);
%Sub in 0 for velocities and accelrations to find G
v = [q1dot q2dot q3dot q4dot q1dd q2dd q3dd q4dd g];
n = [0 0 0 0 0 0 0 0 9.81];
G = simplify([coeffs(subs(Torques(1),v,n),g) ...
coeffs(subs(Torques(2),v,n),g) coeffs(subs(Torques(3),v,n),g)]');
% C is everything left C = simplify(Torques - M*[q1dd q2dd q3dd]' - G)
C = subs(Torques,[q1dd q2dd q3dd q4dd g],[0 0 0 0 0]);
fprintf('M =\n');
disp(M);
fprintf('G =\n');
disp(G);
fprintf('C =\n');
disp(C);
%show initial position of robot in a plot
initialpos = plotArm(0,0,0,0,0);
fprintf('Initial position T05 =\n');
disp(initialpos);

%function that finds T06 and plots according to inputted joints
function T = plotArm(j1,j2,j3,j4,j5)
syms pi
% Input joints  
t1 = j1*pi()/180; 
t2 = j2*pi()/180; 
t3 = j3*pi()/180; 
t4 = j4*pi()/180; 
t5 = j5*pi()/180;       
% Redeclare HD parameters, and calculate transformation matrices 
a = [3.14/2, 0, 3.14, -3.14/2, 0];
r = [33 155 135 0 0];
d = [147 0 0 0 171];
t = [t1,t2+3.14/2,t3,t4-3.14/2,t5]; 
T01 = DH(a(1), r(1), d(1), t(1));
T12 = DH(a(2), r(2), d(2), t(2));
T23 = DH(a(3), r(3), d(3), t(3));
T34 = DH(a(4), r(4), d(4), t(4));
T45 = DH(a(5), r(5), d(5), t(5)); 
T02 = T01*T12; 
T03 = T02*T23; 
T04 = T03*T34; 
T05 = T04*T45; 
T= T05; 
% Plot robot position in Matlab 
figure('Units', 'inches', 'Position', [3,3,5.5,4]) 
points = horzcat([0;0;0], T01(1:3,4),T02(1:3,4),T03(1:3,4),T04(1:3,4),...
    T05(1:3,4)); 
plot3(points(1,:),  points(2,:),points(3,:), '-*');
axis([-1000 1000 -1000 1000 0 1500 0 1]);
xlabel('x [mm]'); 
ylabel('y [mm]'); 
zlabel('z [mm]');  
str = strcat('[',num2str(T05(1,end), '%0 .2f'), ', ',num2str(T05(2,end),...  
 '%0 .2f'), ',  ', num2str(T05(3,end),'%0 .2f'),']'); 
text(T05(1,end), T05(2,end), T05(3,end)+150, str, 'FontSize',12); 
snapnow

