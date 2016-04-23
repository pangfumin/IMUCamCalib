function [pointCloud] = generate3DPointCloud(range,center,pointNum)
%pointCloud = zeros(3,pointNum);
xRange = range(1);
yRange = range(2);
zRange = range(3);
x = xRange*rand(pointNum,1) - xRange/2.0 + center(1);
y = yRange*rand(pointNum,1) - yRange/2.0 + center(2);
z = zRange*rand(pointNum,1) - zRange/2.0 + center(3);
pointCloud=[x,y,z];
end