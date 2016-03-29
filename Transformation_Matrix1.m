function [T] = Transformation_Matrix1(t,al,a,d)
%This script is written to give us the transformation matrix symbolically.
T=[ cos(t)  -sin(t)*cosd(al)    sin(t)*sind(al)  a*cos(t);
    sin(t)   cos(t)*cosd(al)   -cos(t)*sind(al)  a*sin(t);
      0          sind(al)           cosd(al)        d;
      0            0                 0              1];
    

end

