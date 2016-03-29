function [T] = Transformation_Matrix(t,al,a,d)
%This script is written to give us the transformation matrix symbolically.
T=[ cosd(t)  -sind(t)*cosd(al)    sind(t)*sind(al)  a*cosd(t);
    sind(t)   cosd(t)*cosd(al)   -cosd(t)*sind(al)  a*sind(t);
      0          sind(al)           cosd(al)        d;
      0            0                 0              1];
    

end

