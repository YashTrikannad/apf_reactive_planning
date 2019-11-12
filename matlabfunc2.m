syms t1 t2 t3 t4 t5 t6;

load 'robot.mat' robot

A1 = frame_transformation(0,(3*pi)/2,robot.d1,t1)