%plot_result
p0 = ground_truth(1,2:4);
figure(2);
plot3(p0(1),p0(2),p0(3),'*');
hold on;
plot3(ground_truth(:,2),ground_truth(:,3),ground_truth(:,4),'r');
hold on;
plot3(pHat(1,:),pHat(2,:),pHat(3,:),'g');
hold on;
plot3(onlyImuPro(1,:),onlyImuPro(2,:),onlyImuPro(3,:),'b');

legend('Start','Truth','Est.','Pure IMU');

figure(3);
sampleNum = size(p_I_CHat,2);
plot(1:sampleNum,p_I_CHat(1,:),'r',1:sampleNum,p_I_CHat(2,:),'g',1:sampleNum,p_I_CHat(3,:),'b');
