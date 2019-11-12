syms a alpha d theta;
syms t1 t2 t3 t4 t5 d1 a2 a3 d4 d5 d6;

A = [cos(theta)     -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta) ;...
    sin(theta)      cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta);...
    0               sin(alpha)              cos(alpha)              d;...
    0               0                       0                       1]

A1 = subs(A,[a alpha d theta],[0   (3*pi)/2  d1  t1]);
A2 = subs(A,[a alpha d theta],[a2   0        0   t2+(3*pi/2)]);
A3 = subs(A,[a alpha d theta],[a3   0        0   t3+(pi/2)]);
A4 = subs(A,[a alpha d theta],[0   (3*pi)/2  0   t4+(3*pi/2)]);
A5 = subs(A,[a alpha d theta],[0   0         d5  t5]);
A6 = subs(A,[a alpha d theta],[0   0         d6  t6]);

T1 = A1;
T2 = T1*A2;
T3 = T2*A3;
T4 = T3*A4;
T5 = T4*A5;

T1 = simplify(T1);
T2 = simplify(T2);
T3 = simplify(T3);
T4 = simplify(T4);
T5 = simplify(T5);

column4 = T1(:,4);
column4(4,:) = [];
column4 = transpose(column4)
derivative_Jv1 = jacobian(column4,[t1,t2,t3])

%f =  matlabFunction(derivative_Jv1,'File','Jv1');


