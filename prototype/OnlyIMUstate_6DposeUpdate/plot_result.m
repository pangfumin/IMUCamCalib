% plot result 

% for v
t = 1 : size(vEst,2);
err = vEst - vGt;
figure(5);
subplot(3,1,1);
plot(t, vEst(1,:), 'r', t, vGt(1,:), 'g');
xlabel('x')
subplot(3,1,2);
plot(t, vEst(2,:), 'r', t, vGt(2,:), 'g');
xlabel('y')
subplot(3,1,3);
plot(t, vEst(3,:), 'r', t, vGt(3,:), 'g');

t = 1 : size(bgEst,2);
err = vEst - vGt;
figure(6);
subplot(3,1,1);
plot(t, bgEst(1,:), 'r', t, bgGt(1,:), 'g');
xlabel('x')
subplot(3,1,2);
plot(t, bgEst(2,:), 'r', t, bgGt(2,:), 'g');
xlabel('y')
subplot(3,1,3);
plot(t, bgEst(3,:), 'r', t, bgGt(3,:), 'g');

t = 1 : size(bgEst,2);
err = vEst - vGt;
figure(7);
subplot(3,1,1);
plot(t, baEst(1,:), 'r', t, baGt(1,:), 'g');
xlabel('x')
subplot(3,1,2);
plot(t, baEst(2,:), 'r', t, baGt(2,:), 'g');
xlabel('y')
subplot(3,1,3);
plot(t, baEst(3,:), 'r', t, baGt(3,:), 'g');