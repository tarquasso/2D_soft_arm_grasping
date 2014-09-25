close all

% figure;
% hold on;
% plot(mtr6_1(:,1),-(mtr6_1(:,2)-1000),'r');
% 
% plot(mtr6_2(:,1),mtr6_2(:,2)-1000,'b');
% 
% plot(ce6(:,1),ce6(:,2),'k');
% 
% figure;
% hold on;
% plot(e6(:,1),e6(:,2),'r');
% plot(k6(:,1),k6(:,2),'k');

% figure;
% offset = 1000;
% 
% hold on;
% plot(mtr2_1(:,1),-(mtr2_1(:,2)-offset),'r');
% 
% plot(mtr2_2(:,1),mtr2_2(:,2)-offset,'b');
% 
% %plot(ce2(:,1),ce2(:,2),'k');
% legend('mtr1','mtr2','ce');
% figure;
% hold on;
% plot(ce2(:,1),ce2(:,2)/83,'k');
% plot(err2(:,1),err2(:,2),'r');
% plot(cur2(:,1),cur2(:,2),'m');
% legend('ce2','err2','cur2');

figure;
offset = 1300;

hold on;
plot(mtr3_1(:,1),-(mtr3_1(:,2)-offset),'r');

plot(mtr3_2(:,1),mtr3_2(:,2)-offset,'b');
plot(ce2(:,1),ce2(:,2),'k');

legend('mtr1','mtr2','ce');

figure;
hold on;
plot(ce3(:,1),ce3(:,2)/80,'k');
plot(err3(:,1),err3(:,2),'r');
legend('ce2','err2');
