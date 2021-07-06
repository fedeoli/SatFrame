MyFontSize = 18;
nF = 1.15;
satn  = 2; %which satellite
ttime = [time(1):time_step:time(i)];



% dx = mypseudo_derivative( ttime, satellites_iner_ECI_alltimes(6*(satn-1)+1,1:i),2,6);   
% dy = mypseudo_derivative( ttime, satellites_iner_ECI_alltimes(6*(satn-1)+2,1:i),2,6);   
% dz = mypseudo_derivative( ttime, satellites_iner_ECI_alltimes(6*(satn-1)+3,1:i),2,6);   
% dxhat = mypseudo_derivative( ttime, xHatUKF(6*(satn-1)+1,1:i),2,6);   
% dyhat = mypseudo_derivative( ttime, xHatUKF(6*(satn-1)+2,1:i),2,6);   
% dzhat = mypseudo_derivative( ttime, xHatUKF(6*(satn-1)+3,1:i),2,6);   

figure(1)
ax(1) = subplot(2,1,1);
plot(ttime,xHatUKF(6*(satn-1)+1,1:i)','r-',...
       ttime, xHatUKF(6*(satn-1)+2,1:i)','b-',...
       ttime, xHatUKF(6*(satn-1)+3,1:i)','c-','LineWidth',2);
legend('x','y','z','FontSize',MyFontSize,'Interpreter','Latex')
title(['State of the ' num2str(satn) ' satellite (true dashed)'])
hold on
%plot(ttime, myEulerCopy(6*(satn-1)+1:6*(satn-1)+3,1:i)','--','LineWidth',2);
plot(ttime, satellites_iner_ECI_alltimes(6*(satn-1)+1,1:i)','r--',...
    ttime, satellites_iner_ECI_alltimes(6*(satn-1)+2,1:i)','b--',...
    ttime, satellites_iner_ECI_alltimes(6*(satn-1)+3,1:i)','c--','LineWidth',2);
hold off
grid on
ylabel('[m]')
ax(2) = subplot(2,1,2)
plot(ttime,xHatUKF(6*(satn-1)+4,1:i)','r-',...
       ttime, xHatUKF(6*(satn-1)+5,1:i)','b-',...
       ttime, xHatUKF(6*(satn-1)+6,1:i)','c-','LineWidth',2);
legend('$\dot x$','$\dot y$','$\dot z$','FontSize',MyFontSize,'Interpreter','Latex')
hold on
set(gca,'FontSize',MyFontSize);
%plot(ttime, myEulerCopy(6*(satn-1)+4:6*(satn-1)+6,1:i)','--','LineWidth',2);
plot(ttime, satellites_iner_ECI_alltimes(6*(satn-1)+4,1:i)','r--',...
    ttime, satellites_iner_ECI_alltimes(6*(satn-1)+5,1:i)','b--',...
    ttime, satellites_iner_ECI_alltimes(6*(satn-1)+6,1:i)','c--','LineWidth',2);
hold off
grid on
xlim([time(1) time(i)])
xlabel('Time (s)','FontSize',MyFontSize,'Interpreter','Latex')
ylabel('[m/s]')

% decrease distance between subplots
for h = 1:length(ax)
    vCurrPos = get(ax(h), 'position'); % current position
    %set(ha(h), 'position', (vCurrPos.*[1 1 nF nF])-[vCurrPos(3)*(nF-1)/2 vCurrPos(4)*(nF-1)/2 0 0]);
    set(ax(h), 'position', (vCurrPos.*[1 1 1 nF])-[0 vCurrPos(4)*(nF-1)/2 0 0]);
    if h < length(ax)
        set(ax(h), 'XTickLabel', ' ')
    end
end
linkaxes(ax,'x');
set(gca,'FontSize',MyFontSize);

figure(2)
ax(1) = subplot(2,1,1)
plot(ttime,satellites_iner_ECI_EstimationError_alltimes(6*(satn-1)+1,1:i)','r-',...
       ttime,satellites_iner_ECI_EstimationError_alltimes(6*(satn-1)+2,1:i)','b-',...
       ttime,satellites_iner_ECI_EstimationError_alltimes(6*(satn-1)+3,1:i)','c-','LineWidth',2);
grid on
ylabel('[m]')
legend('$E_x$','$E_y$','$E_z$','FontSize',MyFontSize,'Interpreter','Latex')
set(gca,'FontSize',MyFontSize);
ax(2) = subplot(2,1,2)
plot(ttime,satellites_iner_ECI_EstimationError_alltimes(6*(satn-1)+4,1:i)','r-',...
       ttime,satellites_iner_ECI_EstimationError_alltimes(6*(satn-1)+5,1:i)','b-',...
       ttime,satellites_iner_ECI_EstimationError_alltimes(6*(satn-1)+6,1:i)','c-','LineWidth',2);
grid on
xlim([time(1) time(i)])
legend('$E_{\dot x}$','$E_{\dot y}$','$E_{\dot z}$','FontSize',MyFontSize,'Interpreter','Latex')
ylabel('[m/s]')
set(gca,'FontSize',MyFontSize);

% decrease distance between subplots
for h = 1:length(ax)
    
    vCurrPos = get(ax(h), 'position'); % current position
    %set(ha(h), 'position', (vCurrPos.*[1 1 nF nF])-[vCurrPos(3)*(nF-1)/2 vCurrPos(4)*(nF-1)/2 0 0]);
    set(ax(h), 'position', (vCurrPos.*[1 1 1 nF])-[0 vCurrPos(4)*(nF-1)/2 0 0]);
    
    if h < length(ax)
        
        set(ax(h), 'XTickLabel', ' ')
        
    end
    
end

linkaxes(ax,'x');

figure(3)
semilogy(ttime, NormEstimationErrorEachXYZ(:,1:i),'LineWidth',2);
hold on
semilogy(ttime, sqrt(3)*ErrorAmplitudeGPS*ones(1,i),'k--','LineWidth',2);
hold off
grid on
ylabel('||E_i(x,y,z)||','FontSize',MyFontSize)
xlabel('Time (s)','FontSize',MyFontSize,'Interpreter','Latex')
set(gca,'FontSize',MyFontSize);
