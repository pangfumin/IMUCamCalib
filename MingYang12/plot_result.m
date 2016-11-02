%

t = 1:size(rotHat,2);
init_p_I_Cs = repmat(init_p_I_C,[1,size(rotHat,2)]);
p_error = init_p_I_Cs - p_I_CHat;   
figure(1);
subplot(2,1,1);

h = plot(t,p_error(1,:),'r',t,p_error(1,:)+3*sigma(:,4)','r--',t,p_error(1,:)-3*sigma(:,4)','r--');
 h(1).LineWidth = 2;
% h(2).LineWidth = 2;
% h(3).LineWidth = 2;
hold on;
h = plot(t,p_error(2,:),'g',t,p_error(2,:)+3*sigma(:,5)','g--',t,p_error(2,:)-3*sigma(:,5)','g--');
  h(1).LineWidth = 2;
hold on;
h = plot(t,p_error(3,:),'b',t,p_error(3,:)+3*sigma(:,6)','b--',t,p_error(3,:)-3*sigma(:,6)','b--');
 h(1).LineWidth = 2;
 grid on;
axis([0 2600 -0.2 0.3]);
ylabel('Tran.(m)');
xlabel('Image Timestep');

attitudes = repmat(attitude',[1,size(rotHat,2)]);
euler = attitudes - rotHat;
r = 190/3.14;

% subplot(2,1,2);
% h = plot(t,euler(1,:),'r',t,euler(1,:)+3*sigma(:,4)','r--',t,euler(1,:)-3*sigma(:,4)','r--');
%  h(1).LineWidth = 2;
% hold on;
% h = plot(t,euler(2,:),'g',t,euler(2,:)+3*sigma(:,5)','g--',t,euler(2,:)-3*sigma(:,5)','g--');
%  h(1).LineWidth = 2;
% hold on;
% h = plot(t,euler(3,:),'b',t,euler(3,:)+3*sigma(:,6)','b--',t,euler(3,:)-3*sigma(:,6)','b--');
%  h(1).LineWidth = 2;
%  grid on;
% axis([0 2600 -0.2 0.3]);
% ylabel('Rot. (deg)');
% xlabel('Image Timestep');

subplot(2,1,2);
h = plot(t,euler(1,:)*r,'r',t,(euler(1,:)+3*sigma(:,1)')*r,'r--',t,(euler(1,:)-3*sigma(:,1)')*r,'r--');
 h(1).LineWidth = 2;
hold on;
h = plot(t,euler(2,:)*r,'g',t,(euler(2,:)+3*sigma(:,2)')*r,'g--',t,(euler(2,:)-3*sigma(:,2)')*r,'g--');
 h(1).LineWidth = 2;
hold on;
h = plot(t,euler(3,:)*r,'b',t,(euler(3,:)+3*sigma(:,3)')*r,'b--',t,(euler(3,:)-3*sigma(:,3)')*r,'b--');
 h(1).LineWidth = 2;
 grid on;
axis([0 2600 -0.2*r 0.3*r]);
ylabel('Rot.(deg)');
xlabel('Image Timestep');

