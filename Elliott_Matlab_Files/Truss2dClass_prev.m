% AEM 4501 Homework 4 Problem 4
a = 1;
P = 1000;
MatsSets(1).E = 70e9;
MatsSets(1).A = 7.065e-4;
MatsSets(1).rho = 8.1;
% rho is mass density/length and is used by dynamics code
PD.N = 17;
PD.NodePos = [0, 0, 0; 
              a, 0, 0;
              0, a, 0;
              a, a, 0];
PD.NE = 5; % Change this depending on the number of members
% change PD.ElmConnect and PD.ElmMats as well
PD.ElmConnect = [1, 2;
                 1, 3;
                 3, 4;
                 2, 4;
                 1, 4;
                 1, 5;
                 2, 5;
                 3, 7;
                 3, 8;
                 9, 10;
                 9, 13;
                 10,14];
                
PD.NM = 1;
PD.MatsSets = MatsSets;
PD.ElmMats = [1;
              1;
              1;
              1;
              1];
              
              
PD.BCType = [1, 1, 1;
             0, 1, 1;
             0, 0, 1;
             0, 0, 1];
% Note z displacement fixed for all since we're in 2D
PD.BCVal = [0, 0, 0;
            0, 0, 0;
            0, 0, 0;
            P, 0, 0];
PDans = PD_truss_static(PD);
PlotTruss(PDans,1000,'y','y');

disp(PDans.ElmForce)
disp(PDans.ElmStress)


