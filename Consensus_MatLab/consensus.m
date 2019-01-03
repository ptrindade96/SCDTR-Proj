close all
%clear all;
%The system
k11 = 2; k12 = 1; k21 = 1; k22 = 2;
K = [k11, k12 ; k21 , k22];

% case 1
%L1 = 150; o1 = 30; L2 = 80; o2 = 0;

% case 2
%L1 = 80; o1 = 50; L2 = 150; o2 = 50;

% case 3
L1 = 20; o1 = 0; L2 = 100; o2 = 0;

%symmetric costs
c1 = 1; c2 = 1; 

%asymmetric costs
%c1 = 1; c2 = 3;


c = [c1 c2]; 
L = [L1;L2]; 
o = [o1;o2];

% SOLVE WITH CONSENSUS
rho = 0.07;
%node 1 initialization
node1.index = 1;
%node1.d = [16;42];
%node1.d_av = [16;42];
%node1.y = [0;0];
node1.k = [k11;k12]; 
node1.n = norm(node1.k)^2;
node1.m = node1.n-k11^2;
node1.c = c1;
node1.o = o1;
node1.L = L1;
node1.Q = 0.1;

%node 2 initialization
node2.index = 2;
%node2.d = [16;42];
%node2.d_av = [16;42];
%node2.y = [0;0];
node2.k = [k22;k21]; 
node2.n = norm(node2.k)^2;
node2.m = node2.n-k22^2;
node2.c = c2;
node2.o = o2;
node2.L = L2;
node2.Q = 0.1;

%Initial contition (iteration = 1)
d11(1) = node1.d(1);
d12(1) = node1.d(2);
d21(1) = node2.d(1);
d22(1) = node2.d(2);
av1(1) = (d11(1)+d21(1))/2;
av2(1) = (d12(1)+d22(1))/2;
%iterations
iter = 1:100;
options = optimoptions('quadprog','Display','off');
for i=2:100
   % node 1
%    if i == 35
%        node1.L = 80;
%        node2.L = 100;
%    end
   
   
   [d1, cost1] = primal_solve(node1, rho);
   node1.d = d1;
  
   %node2
   [d2, cost2] = primal_solve(node2, rho);
   node2.d = d2;
   
   
   % Compute average with available data
   %node 1
   node1.d_av = (node1.d+node2.d)/2;
   
   %node 2
   node2.d_av = (node1.d+node2.d)/2;
   
   % Update local lagrangians
   %node 1
   node1.y = node1.y + rho*(node1.d-node1.d_av);
   %node 2
   node2.y = node2.y + rho*(node2.d-node2.d_av);
   
   
   %save data for plots
   d11(i) = node1.d(1);
   d12(i) = node1.d(2);
   d21(i) = node2.d(1);
   d22(i) = node2.d(2);
   av1(i) = (d11(i)+d21(i))/2;
   av2(i) = (d12(i)+d22(i))/2;
%    
%    if i == 20
%        node1.L = 100;
%    end
%    if i == 40
%        node2.L = 100;
%    end
%    if i == 60
%        node1.L = 20;
%    end
end;

% SOLVE WITH MATLAB QUADPROG
Q_ = [node1.Q, 0; 0 node2.Q];
A_ = -K; 
c_ = c;
b_ = [o1-L1; o2-L2];
lb = [0;0]; ub = [100;100];
disp('Matlab solutions')
d_ = quadprog(Q_,c_,A_,b_,[],[],lb,ub,[],options)
l_ = K*d_+o

disp('Consensus Solutions')
d = node1.d_av
l = K*d+o

%Plots
figure(10);
plot(iter, av1, iter, av2, iter, d11, iter, d12, iter, d21, iter, d22);
legend('av1','av2', 'd11', 'd12', 'd21', 'd22');
set(legend,'FontSize',14)
title('time convergence');
xlabel('iter');

%%
figure(15);
set(gcf,'defaultTextInterpreter','latex');
axis square
t = 0:100;
constr1 = (L1-o1)/k12-(k11/k12)*t;
constr2 = (L2-o2)/k22-(k21/k22)*t;
[x,y]=meshgrid(t,t);
hold on;
z = c1*x + node1.Q*x.^2 + c2*y + node2.Q*y.^2;
contour(x,y,z);
plot(t,constr1,t,constr2,'LineWidth',2);
plot(t,zeros(size(t)),'k','LineWidth',2);
plot(zeros(size(t)),t,'k','LineWidth',2);
plot(t,100*ones(size(t)),'k','LineWidth',2);
plot(100*ones(size(t)),t,'k','LineWidth',2);
plot(av1,av2,'--','LineWidth',2);
plot(av1,av2,'bo');
% plot(d11,d12,'m-.','LineWidth',1);
% plot(d11,d12,'mx');
% plot(d21,d22,'k-.','LineWidth',1);
% plot(d21,d22,'kx');
set(gca,'FontSize',16,'TickLabelInterpreter','latex')
yticks([0 50 100])
title('Solution Space Convergence','FontSize',20);
xlabel('Dimming node 1, $d_1$ (\%)','FontSize',16);
ylabel('Dimming node 2, $d_2$ (\%)','FontSize',16);
plot(d_(1),d_(2),'r*','LineWidth',3)
axis([-5,105,-5,105]);
hold off;






