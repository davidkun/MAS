clear all; clc;
s1 = load('data/e_simple1.csv');
a1 = load('data/e_adv1.csv');
s2 = load('data/e_simple2.csv');
a2 = load('data/e_adv2.csv');
s3 = load('data/e_simple3.csv');
a3 = load('data/e_adv3.csv');
s4 = load('data/e_simple4.csv');
a4 = load('data/e_adv4.csv');
s5 = load('data/e_simple5.csv');
a5 = load('data/e_adv5.csv');
s6 = load('data/e_simple6.csv');
a6 = load('data/e_adv6.csv');

s_avg1 = mean(s1);
a_avg1 = mean(a1);
s_avg2 = mean(s2);
a_avg2 = mean(a2);
s_avg3 = mean(s3);
a_avg3 = mean(a3);
s_avg4 = mean(s4);
a_avg4 = mean(a4);
s_avg5 = mean(s5);
a_avg5 = mean(a5);
s_avg6 = mean(s6);
a_avg6 = mean(a6);

clear s1 s2 s3 s4 s5 s6 a1 a2 a3 a4 a5 a6

%%
close all;

figure; hold on;
plot(s_avg1, 'k-')
plot(a_avg1, 'g-')
title({'Leader Error = Follower Error', 'State Error = Sensor Error'}, ...
  'Interpreter','Latex','FontSize',14)
ax(1) = xlabel('Time Step, [k]'); 
ax(2) = ylabel('Position Error, $e^Te$', 'Rotation', 90);
axis([0, 360, 0, 0.15])
l=legend(strcat('Simple KF, $\bar{e}=$',num2str(mean(s_avg1))), ...
  strcat('New KF, $\bar{e}=$',num2str(mean(a_avg1))), ...
  'Location','Best');
legend('boxoff')
set(l,'Interpreter','Latex')
set(ax,'Interpreter','Latex')
%%
figure; hold on;
plot(s_avg2, 'k-')
plot(a_avg2, 'g-')
title({'Leader Error = Follower Error', 'State Error $>$ Sensor Error'}, ...
  'Interpreter','Latex','FontSize',14)
ax(1) = xlabel('Time Step, [k]'); 
ax(2) = ylabel('Position Error, $e^Te$', 'Rotation', 90);
axis([0, 360, 0, 0.15])
l=legend(strcat('Simple KF, $\bar{e}=$',num2str(mean(s_avg2))), ...
  strcat('New KF, $\bar{e}=$',num2str(mean(a_avg2))), ...
  'Location','Best');
legend('boxoff')
set(l,'Interpreter','Latex')
set(ax,'Interpreter','Latex')
%
figure; hold on;
plot(s_avg3, 'k-')
plot(a_avg3, 'g-')
title({'Leader Error = Follower Error','State Error $<$ Sensor Error'}', ...
  'Interpreter','Latex','FontSize',14)
ax(1) = xlabel('Time Step, [k]'); 
ax(2) = ylabel('Position Error, $e^Te$', 'Rotation', 90);
axis([0, 360, 0, 0.15])
l=legend(strcat('Simple KF, $\bar{e}=$',num2str(mean(s_avg3))), ...
  strcat('New KF, $\bar{e}=$',num2str(mean(a_avg3))), ...
  'Location','Best');
legend('boxoff')
set(l,'Interpreter','Latex')
set(ax,'Interpreter','Latex')
%
figure; hold on;
plot(s_avg4, 'k-')
plot(a_avg4, 'g-')
title('Leader State Error $>$ Follower State Error', ...
  'Interpreter','Latex','FontSize',14)
ax(1) = xlabel('Time Step, [k]'); 
ax(2) = ylabel('Position Error, $e^Te$', 'Rotation', 90);
axis([0, 360, 0, 0.15])
l=legend(strcat('Simple KF, $\bar{e}=$',num2str(mean(s_avg4))), ...
  strcat('New KF, $\bar{e}=$',num2str(mean(a_avg4))), ...
  'Location','Best');
legend('boxoff')
set(l,'Interpreter','Latex')
set(ax,'Interpreter','Latex')
%
figure; hold on;
plot(s_avg5, 'k-')
plot(a_avg5, 'g-')
title('Leader Sensor Error $>$ Follower Sensor Error', ...
  'Interpreter','Latex','FontSize',14)
ax(1) = xlabel('Time Step, [k]'); 
ax(2) = ylabel('Position Error, $e^Te$', 'Rotation', 90);
axis([0, 360, 0, 0.25])
l=legend(strcat('Simple KF, $\bar{e}=$',num2str(mean(s_avg5))), ...
  strcat('New KF, $\bar{e}=$',num2str(mean(a_avg5))), ...
  'Location','Best');
legend('boxoff')
set(l,'Interpreter','Latex')
set(ax,'Interpreter','Latex')
%%
figure; hold on;
plot(s_avg6, 'k-')
plot(a_avg6, 'g-')
title('Leader State \& Sensor Error $>$ Follower State \& Sensor Error', ...
  'Interpreter','Latex','FontSize',12)
ax(1) = xlabel('Time Step, [k]'); 
ax(2) = ylabel('Position Error, $e^Te$', 'Rotation', 90);
axis([0, 360, 0, 0.4])
l=legend(strcat('Simple KF, $\bar{e}=$',num2str(mean(s_avg6))), ...
  strcat('New KF, $\bar{e}=$',num2str(mean(a_avg6))), ...
  'Location','Best');
legend('boxoff')
set(l,'Interpreter','Latex')
set(ax,'Interpreter','Latex')

%%
for i=1:gcf
  print(i,'-depsc','-r600',strcat('data/fig',num2str(i)))
end
