% AEM 4501 Homework 4 problem 4
a = 3; b  = 2; c = 2.5; % Truss size 
F = 100000; G = 50000; % Load in Newton

MatsSets(1).E = 70e9; % Pa
MatsSets(1).A = 0.0015; % m^2
MatsSets(1).rho = 8.1; % density

PD.N = 8; % Number of nodes
PD.NodePos = [0, 0, b;
              a, 0, b;
              0, c, b;
              a, c, b;
              a, c ,0;
              a, 0, 0;
              0, 0, 0;
              0, c, 0];
PD.NE = 18; % Number of Elements
PD.ElmConnect = [1, 2;
                 1, 3;
                 3, 4;
                 2, 4;
                 2, 3;
                 1, 7;
                 3, 7;
                 3, 8;
                 8, 7;
                 8, 5;
                 5, 7;
                 7, 6;
                 6, 5;
                 6, 2;
                 2, 5;
                 5, 4;
                 5, 3;
                 2, 7];
PD.NM = 1;
PD.MatsSets = MatsSets;
PD.ElmMats = ones(18,1);
PD.BCType = [1, 0, 1;
             0, 1, 1;
             0, 0, 0;
             0, 0, 0;
             0, 0, 0;
             0, 0, 0;
             0, 0, 0;
             1, 1, 0];
PD.BCVal = [0, 0, 0;
            0, 0, 0;
            G,-F, 0;
            0, 0, 0;
            0, 0, 0;
            0, 0, 0;
            0, 0, 0;
            0, 0, 0];
PDans = PD_truss_static(PD);
PlotTruss(PDans,1000,'y','y');
view(3);

%disp(PDans.ElmForce)
%disp(PDans.ElmStress)
% loop for the forces and stresses in all the elements 
for i = 1:PD.NE
    force = PDans.ElmForce(i); %(unit Newton for force)

    disp(['Force in element ', num2str(i), ':', num2str(force)]);

    stress = PDans.ElmStress(i); %(unit Pascal for Stress)
    disp(['Stress in element ', num2str(i), ':', num2str(stress)]);

    disp(' ');
end

% Deflections at nodes A (3) and G (6) (unit meter)
disp('Deflection at node A: ');
disp(PDans.U(3,:));
disp('Deflection at node G: ');
disp(PDans.U(6,:));