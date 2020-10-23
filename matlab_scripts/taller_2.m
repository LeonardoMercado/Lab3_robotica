%% Taller #2 Robótica 2020-2.
%
% Leonardo Fabio Mercado Benítez.
% 
% código: 25481090.
%
% C.C: 1.016.050.737.
%% Punto 2:
clc;
clear;
close all;

syms Q1 Q2 Q3
l1 = 1;
l2 = 1;
  
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('revolute','alpha', pi/2,    'a',l1,   'd',0,   'offset', pi/4,   'modified', 'qlim',[-pi pi]);
L(3) = Link('revolute','alpha', 0,    'a',l2,   'd',0,   'offset', pi/2,   'modified', 'qlim',[-pi pi]);

robot = SerialLink(L,'name','robot_P2');
robot.plot([0 0 0]);
robot.teach;

%% Punto 3: 
%
clc;
clear;
close all;


syms Q1 l2 d2 Q3

T_0_1 = trotx(0)*transl(0,0,0)*trotz(Q1+pi/4)*transl(0,0,0);
T_0_1_aux = matriz_T(0,0,Q1+pi/4,0);
T_1_2 = trotx(sym(-pi/2))*transl(0,0,0)*trotz(0)*transl(0,0,d2+l2);
T_1_2_aux = matriz_T(sym(-pi/2),0,0,d2+l2);
T_2_3 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q3+pi/2)*transl(0,0,0);
T_2_3_aux = matriz_T(sym(pi/2),0,Q3+pi/2,0);
T_0_3 = T_0_1*T_1_2*T_2_3;

disp('La matriz 0_T_1 es: ');
disp(T_0_1);
disp('La matriz 1_T_2 es: ');
disp(T_1_2);
disp('La matriz 2_T_3 es: ');
disp(T_2_3);
disp('La matriz 0_T_3 es: ');
disp(T_0_3);


L(1) = Link('revolute','alpha', 0,    'a',0,   'd',0,   'offset', pi/4,   'modified', 'qlim',[-pi pi]);
L(2) = Link('prismatic','alpha', -pi/2,    'a',0,   'theta',0,   'offset', 1,   'modified', 'qlim',[0 1]);
L(3) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset', pi/2,   'modified', 'qlim',[-pi pi]);

robot = SerialLink(L,'name','robot_P3');
robot.plot([0 0 0],'workspace',[-4 4 -4 6 -1 3],'joints', 'view',[30 30]);
robot.teach;




%% Punto 4:
%
clc;
clear;
close all;

syms Q1 Q2 Q3 Q4 Q5


T_0_1 = trotx(0)*transl(0,0,0)*trotz(Q1)*transl(0,0,0);
T_1_2 = trotx(sym(-pi/2))*transl(0,0,0)*trotz(0)*transl(0,0,Q2+1);
T_2_3 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q3)*transl(0,0,0);
T_3_4 = trotx(sym(-pi/2))*transl(2,0,0)*trotz(Q4)*transl(0,0,0);
T_4_5 = trotx(0)*transl(0.5,0,0)*trotz(sym(-pi/2))*transl(0,0,Q5-0.5);
T_5_NOA = [1 0 0 0;
           0 1 0 0;
           0 0 1 1;
           0 0 0 1];

disp('El modelo de cinemática directa del robot es: ');
T_0_Tool = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_NOA;

L(1) = Link('revolute','alpha', 0,    'a',0,   'd',0,   'offset',0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('prismatic','alpha', -pi/2,    'a',0,   'theta',0,   'offset', 1,   'modified', 'qlim',[0 5]);
L(3) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset',0,   'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', -pi/2,    'a',2,   'd',0,   'offset',0,   'modified', 'qlim',[-pi pi]);
L(5) = Link('prismatic','alpha', 0,    'a',0.5,   'theta',-pi/2,   'offset',0.5,   'modified', 'qlim',[0 5]);

robot = SerialLink(L,'name','robot_P4');
robot.tool = T_5_NOA;
robot.plot([pi/3 1.5 pi/2 pi/4 2],'workspace',[-9 6 -6 6 -3 4],'joints', 'noa','view',[30 30]);
robot.teach;
disp('Dada la configuración en el espacio articular se tiene la siguiente posición y orientación del Tool')
disp(robot.fkine([pi/3 1.5 pi/2 pi/4 2]))

%% Punto 5: 
%
clc;
clear;
close all;

syms Q1 Q2 Q3 Q4 Q5

T_0_1 = trotx(0)*transl(0,0,0)*trotz(Q1)*transl(0,0,0);
T_1_2 = trotx(sym(-pi/2))*transl(0,0,0)*trotz(0)*transl(0,0,Q2+1);
T_2_3 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q3)*transl(0,0,1);
T_3_4 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q4)*transl(0,0,0);
T_4_5 = trotx(sym(-pi/2))*transl(1,0,0)*trotz(Q5)*transl(0,0,-1);
T_5_tool = [0 1 0 0;
            0 0 1 1;
            1 0 0 0;
            0 0 0 1];

T_0_tool = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_tool;
       
disp('La matriz 0_T_1 es: ');
disp(T_0_1);
disp('La matriz 1_T_2 es: ');
disp(T_1_2);
disp('La matriz 2_T_3 es: ');
disp(T_2_3);
disp('La matriz 3_T_4 es: ');
disp(T_3_4);
disp('La matriz 4_T_5 es: ');
disp(T_4_5);
disp('La matriz 5_T_tool es: ');
disp(T_5_tool);
disp('La matriz 0_T_tool es: ');
disp(T_0_tool);

L(1) = Link('revolute','alpha', 0,    'a',0,   'd',0,   'offset',0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('prismatic','alpha', -pi/2,    'a',0,   'theta',0,   'offset',1,   'modified', 'qlim',[0 5]);
L(3) = Link('revolute','alpha', pi/2,    'a',0,   'd',1,   'offset',pi/2,   'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset',0,   'modified', 'qlim',[-pi pi]);
L(5) = Link('revolute','alpha', -pi/2,    'a',1,   'd',-1,   'offset',-pi/2,   'modified', 'qlim',[-pi pi]);


robot = SerialLink(L,'name','robot_P5');
robot.tool = T_5_tool;
robot.plot([0 0 0 0 0],'workspace',[-9 6 -6 6 -2 4],'joints', 'noa','view',[30 30]);
robot.teach;
%% Punto 6:
clc; 
clear all;
close all;

syms Q1 Q2 Q3 Q4 Q5 Q6

l1 = 30;
l2 = 25;
l3 = 25;
l4 = 10;

T_0_1 = trotx(0)*transl(0,0,0)*trotz(Q1)*transl(0,0,l1);
T_1_2 = trotx(sym(-pi/2))*transl(0,0,0)*trotz(Q2)*transl(0,0,0);
T_2_3 = trotx(0)*transl(l2,0,0)*trotz(Q3-pi/2)*transl(0,0,0);
T_3_4 = trotx(sym(-pi/2))*transl(0,0,0)*trotz(Q4+pi/2)*transl(0,0,l3);
T_4_5 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q5)*transl(0,0,0);
T_5_6 = trotx(sym(-pi/2))*transl(0,0,0)*trotz(Q6-pi/2)*transl(0,0,l4/2);
T_6_tool = [1 0 0 0;
            0 1 0 0;
            0 0 1 l4/2;
            0 0 0 1];

T_0_tool = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_tool;
       
disp('La matriz 0_T_1 es: ');
disp(T_0_1);
disp('La matriz 1_T_2 es: ');
disp(T_1_2);
disp('La matriz 2_T_3 es: ');
disp(T_2_3);
disp('La matriz 3_T_4 es: ');
disp(T_3_4);
disp('La matriz 4_T_5 es: ');
disp(T_4_5);
disp('La matriz 5_T_6 es: ');
disp(T_5_6);
disp('La matriz 6_T_tool es: ');
disp(T_6_tool);
disp('La matriz 0_T_tool es: ');
disp(T_0_tool);

L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l1,   'offset',0,   'modified', 'qlim',[-2*pi 2*pi]);
L(2) = Link('revolute','alpha', -pi/2,    'a',0,   'd',0,   'offset',0,   'modified', 'qlim',[-2*pi 2*pi]);
L(3) = Link('revolute','alpha', 0,    'a',l2,   'd',0,   'offset',-pi/2,   'modified', 'qlim',[-2*pi 2*pi]);
L(4) = Link('revolute','alpha', -pi/2,    'a',0,   'd',l3,   'offset',pi/2,   'modified', 'qlim',[-2*pi 2*pi]);
L(5) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset',0,   'modified', 'qlim',[-2*pi 2*pi]);
L(6) = Link('revolute','alpha', -pi/2,    'a',0,   'd',l4/2,   'offset',-pi/2,   'modified', 'qlim',[-2*pi 2*pi]);


robot = SerialLink(L,'name','robot_P6');
robot.tool = [1 0 0 0;
              0 1 0 0;
              0 0 1 l4/2;
              0 0 0 1];
robot.plot([pi/3 pi/6 pi/18 pi/2 pi 3*pi/2],'workspace',[-80 80 -80 80 -80 80],'noa','view',[30 30]);
robot.teach;
posicion_final = robot.fkine([pi/3 pi/6 pi/18 pi/2 pi 3*pi/2]);

%% Punto 7:
%
clc;
clear;
close all;

syms Q1 Q2 Q3 Q4 Q5 Q6 Q7

l1= 310;
l2 = 400;
l3 = 390;
l4 = 84.1;

T_0_1 = trotx(0)*transl(0,0,0)*trotz(Q1)*transl(0,0,l1);
T_1_2 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q2+(3*pi/4))*transl(0,0,0);
T_2_3 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q3+pi)*transl(0,0,l2);
T_3_4 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q4+pi/2)*transl(0,0,0);
T_4_5 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q5+pi)*transl(0,0,l3);
T_5_6 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q6-pi/2)*transl(0,0,0);
T_6_7 = trotx(sym(pi/2))*transl(0,0,0)*trotz(Q7)*transl(0,0,l4);
T_7_tool = [1 0 0 0;
            0 1 0 0;
            0 0 1 0;
            0 0 0 1];

T_0_tool = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6*T_6_7*T_7_tool;
       
disp('La matriz 0_T_1 es: ');
disp(T_0_1);
disp('La matriz 1_T_2 es: ');
disp(T_1_2);
disp('La matriz 2_T_3 es: ');

disp(T_2_3);
disp('La matriz 3_T_4 es: ');
disp(T_3_4);
disp('La matriz 4_T_5 es: ');
disp(T_4_5);
disp('La matriz 5_T_6 es: ');
disp(T_5_6);
disp('La matriz 6_T_7 es: ');
disp(T_6_7);
disp('La matriz 7_T_tool es: ');
disp(T_7_tool);
disp('La matriz 0_T_tool es: ');
disp(T_0_tool);

%% Robot punto 7
L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l1,   'offset',0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset',3*pi/4,   'modified', 'qlim',[-pi pi]);
L(3) = Link('revolute','alpha', pi/2,    'a',0,   'd',l2,   'offset',pi,   'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset',pi/2,   'modified', 'qlim',[-pi 40]);
L(5) = Link('revolute','alpha', pi/2,    'a',0,   'd',l3,   'offset',pi,   'modified', 'qlim',[-pi pi]);
L(6) = Link('revolute','alpha', pi/2,    'a',0,   'd',0,   'offset',-pi/2,   'modified', 'qlim',[-pi pi]);
L(7) = Link('revolute','alpha', pi/2,    'a',0,   'd',l4,   'offset',0,   'modified', 'qlim',[-pi pi]);


robot = SerialLink(L,'name','robot_P7');
robot.tool = [1 0 0 0;
              0 1 0 0;
              0 0 1 0;
              0 0 0 1];
robot.plot([0 0 0 0 0 0 0],'noa','view',[30 30]);
robot.teach
q1 = robot.fkine([0 pi/6 pi/12 20 0 pi pi/2]);
q2 = robot.fkine([pi/3 pi/6 -pi/12 35 0 0 pi/2]);



%%
function [output] = matriz_T(alfa_i_1,a_i_1,theta_i,d_i)

output = [cos(theta_i) -sin(theta_i) 0 a_i_1;
          sin(theta_i)*cos(alfa_i_1) cos(theta_i)*cos(alfa_i_1) -sin(alfa_i_1) -sin(alfa_i_1)*d_i
          sin(theta_i)*sin(alfa_i_1) cos(theta_i)*sin(alfa_i_1) cos(alfa_i_1) cos(alfa_i_1)*d_i
          0 0 0 1];
end
