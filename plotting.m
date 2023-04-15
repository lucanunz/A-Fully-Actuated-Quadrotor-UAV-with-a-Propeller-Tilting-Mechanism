% script to plot the data of the simulation
close all
clear variables
clc
%%% set this flag to true if you want to save the shown figures
save=false;
load('q.mat')
load('state.mat')
load('inputs.mat')
figure,plot(state(1,:)',state(8:10,:)'.*180/pi,'LineWidth',1.5),grid on,xlabel('time [s]'),ylabel('[deg]')
hold on,plot(q(1,:)',q(5:7,:)'.*180/pi,'k--'),legend('\phi','\theta','\psi','desired','Location','southeast'),ylim([-10.5 10.5]),grid on
if save
    saveas(gcf,'rpy.jpg')
    saveas(gcf,'rpy.fig')
end
figure,plot(inputs(1,:)',sqrt(abs(inputs(2:5,:))')),legend('\omega_1','\omega_2','\omega_3','\omega_4'),grid on
xlabel('time [s]'),ylabel('[rad/s]')
if save
    saveas(gcf,'w.jpg')
    saveas(gcf,'w.fig')
end
figure,plot(state(1,:)',state(end-1:end,:)'.*180/pi),legend('\phi_m','\theta_m','Location','southwest'),grid on, xlabel('time [s]'),ylabel('[deg]')
if save
    saveas(gcf,'rp_m.jpg')
    saveas(gcf,'rp_m.fig')
end
figure,plot(state(1,:)',state(2:4,:)','LineWidth',1.5),grid on, xlabel('time [s]'),ylabel('[m]')
hold on,plot(q(1,:)',q(2:4,:)','k--'),legend('px','py','pz','desired'),grid on
if save
    saveas(gcf,'p.jpg')
    saveas(gcf,'p.fig')
end
% x=zeros(size(ddq));
% for i=2:size(ddq,2)-1
%     dt=ddq(1,i+1)-ddq(1,i-1);
%     x(:,i)=(ddq(:,i+1)-ddq(:,i-1))/dt;
% end
% x(1,:)=ddq(1,:);
% figure,plot(x(1,:)',x(2:4,:)'),title('jerk'),legend('px','py','pz'),grid on
% figure,plot(x(1,:)',x(5:7,:)'),title('jerk'),legend('\phi','\theta','\psi'),grid on