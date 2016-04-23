function checherBoard = createChecherBoard(width,height,distance,startPosition,Z)
widthEnd = startPosition(1) + (width-1)*distance;
heightEnd = startPosition(2) + (height-1)*distance;
[X,Y] = meshgrid(startPosition(1):distance:widthEnd, startPosition(2):distance:heightEnd);
X = reshape(X,[width*height 1]);
Y = reshape(Y,[width*height 1]);
Z = repmat(Z,[width*height 1]);
checherBoard = [X Y Z];



end