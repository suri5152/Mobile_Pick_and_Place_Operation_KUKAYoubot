clc; clear; close all;
%The input arguments of the function is given in order(Theta, Alpha, a, d)
%The input arguments for joint angles should be given as sym('theta')in
%the input arguments of a function for it to output symbolically
syms q1 q2 q3 q4 q5 a b c l1 l2 l3 l4
T01=Transformation_Matrix(0,0,0.855,0.140); %This will output T01 matrix symbolically
T12=Transformation_Matrix1(sym('q1'),-90,0.033,0.147);%This will output T12 matrix symbolically
T23=Transformation_Matrix1(sym('q2')-90,0,0.155,0);%This will output T23 matrix symbolically
T34=Transformation_Matrix1(sym('q3'),0,0.135,0);%This will output T34 matrix symbolically
T45=Transformation_Matrix1(sym('q4'),0,0.2175,0);%This will output T45 matrix symbolically
T56=Transformation_Matrix1(sym('q5')-90,-90,0,0);%This will output T56 matrix symbolically
T06=T01*T12*T23*T34*T45*T56;
T01=Transformation_Matrix(0,0,0.855,0.140); %This will output T01 matrix symbolically
T12=Transformation_Matrix(0,-90,0.033,0.147);%This will output T12 matrix symbolically
T23=Transformation_Matrix(-90,0,0.155,0);%This will output T23 matrix symbolically
T34=Transformation_Matrix(0,0,0.135,0);%This will output T34 matrix symbolically
T45=Transformation_Matrix(0,0,0.2175,0);%This will output T45 matrix symbolically
T56=Transformation_Matrix(-90,-90,0,0);%This will output T56 matrix symbolically
T06=T01*T12*T23*T34*T45*T56
T01=Transformation_Matrix(0,0,sym('a'),sym('b')); %This will output T01 matrix symbolically
T12=Transformation_Matrix1(sym('q1'),-90,sym('c'),sym('l1'));%This will output T12 matrix symbolically
T23=Transformation_Matrix1(sym('q2')-90,0,sym('l2'),0);%This will output T23 matrix symbolically
T34=Transformation_Matrix1(sym('q3'),0,sym('l3'),0);%This will output T34 matrix symbolically
T45=Transformation_Matrix1(sym('q4'),0,sym('l4'),0);%This will output T45 matrix symbolically
T56=Transformation_Matrix1(sym('q5')-90,-90,0,0);%This will output T56 matrix symbolically
T06=simplify(T01*T12*T23*T34*T45*T56);
A = [diff(T06(1:3,4), q1), diff(T06(1:3,4), q2), diff(T06(1:3,4), q3), diff(T06(1:3,4), q4) diff(T06(1:3,4), q5)];
B = [0 0 0 0 0; 0 0 0 0 0; 1 1 1 1 1];
J = [A;B]
J_specific =subs(J, [a b c l1 l2 l3 l4 q1 q2 q3 q4 q5], [.855, .140, .033, .147, .155, .135, .2175, pi/4, pi/2, pi/3, pi/4, pi/3])
tipVelocity = J_specific * [90 90 90 90 90]'
torques = J_specific'*[50 0 0 0 0 0]'
% Many values have been assumed to obtain the output.