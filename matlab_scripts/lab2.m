%% Laboratorio #2 Robótica 2020-2
%
% Leonardo Fabio Mercado Benítez.
%
% Código: 25481090.
% 
% C.C:1.016.050.737
%
%% Módelo  Matemático del robot:
clc;
clear;
close all;


syms Q1 Q2 Q3 Q4

assignin('base','Q1',0);
assignin('base','Q2',0);
assignin('base','Q3',0);
assignin('base','Q4',0);


l1 = 137;
l2 = 105;
l3 = 105;
l4 = 95;

T_0_1 = trotx(0)*transl(0,0,0)*trotz(Q1)*transl(0,0,l1);
T_1_2 = T_0_1*(trotx(pi/2)*transl(0,0,0)*trotz(Q2+pi/4)*transl(0,0,0));
T_2_3 = T_1_2*(trotx(0)*transl(l2,0,0)*trotz(Q3-pi/4)*transl(0,0,0));
T_3_4 = T_2_3*(trotx(0)*transl(l3,0,0)*trotz(Q4)*transl(0,0,0));
T_4_tool = T_3_4*[0 0 1 l4;
                  1 0 0 0;
                  0 1 0 0;
                  0 0 0 1];


        
figure(1)
trplot(eye(4,4),'rgb','frame','B','length',50,'thick',2);
hold on;
grid on;
trplot(T_0_1,'rgb','frame','1','length',50,'thick',2);
trplot(T_1_2,'rgb','frame','2','length',50,'thick',2);
trplot(T_2_3,'rgb','frame','3','length',50,'thick',2);
trplot(T_3_4,'rgb','frame','4','length',50,'thick',2);
trplot(T_4_tool,'rgb','frame','T','length',50,'thick',2);
axis([-500 500 -500 500 0 500]);



%% Modelado del robot por el toolbox
clear;
close all;


syms Q1 Q2 Q3 Q4

l1 = 137;
l2 = 105;
l3 = 105;
l4 = 95;

L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l1,  'offset', 0,   'modified', 'qlim',[-pi pi]);
L(2) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', pi/4, 'modified', 'qlim',[-pi pi]);
L(3) = Link('revolute','alpha', 0,    'a',l2,  'd',0,   'offset', -pi/4, 'modified', 'qlim',[-pi pi]);
L(4) = Link('revolute','alpha', 0,    'a',l3,  'd',0,   'offset', 0,   'modified', 'qlim',[-pi pi]);

robot = SerialLink(L,'name','Phantom_x');
robot.tool = [0 0 1 l4;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];
maximo = 500;


robot.plot([0 0 0 0],'workspace',[-maximo maximo -maximo maximo 0 maximo],'noa');
robot.teach;

%%
close all;
robot.plot([0 pi/4 pi/4 0],'workspace',[-maximo maximo -maximo maximo 0 maximo],'noa');
robot.teach;

%%
close all;
robot.plot([0 pi/2 -pi/4 -pi/4],'workspace',[-maximo maximo -maximo maximo 0 maximo],'noa');
robot.teach;

%%
close all;
robot.plot([pi pi/2 -pi/4 -pi/4],'workspace',[-maximo maximo -maximo maximo 0 maximo],'noa');
robot.teach;

%%
close all;
robot.plot([pi -pi/4 pi/4 0],'workspace',[-maximo maximo -maximo maximo 0 maximo],'noa');
robot.teach;

%%

assignin('base','Q1',0);
assignin('base','Q2',0);
assignin('base','Q3',0);
assignin('base','Q4',0);


T_0_1 = transl(0,0,0)*trotz(Q1)*transl(0,0,l1);
T_1_2 = (trotx(pi/2)*transl(0,0,0)*trotz(Q2+pi/4)*transl(0,0,0));
T_2_3 = (trotx(0)*transl(l2,0,0)*trotz(Q3-pi/4)*transl(0,0,0));
T_3_4 = (trotx(0)*transl(l3,0,0)*trotz(Q4)*transl(0,0,0));
T_4_tool = [0 0 1 l4;
                  1 0 0 0;
                  0 1 0 0;
                  0 0 0 1];

T_0_tool = T_0_1*T_1_2*T_2_3*T_3_4*T_4_tool;


