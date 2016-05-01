function Jcam = calJcam(X,Y,Z)
Jcam = [1/Z 0 -X/(Z*Z);
    0 1/Z -Y/(Z*Z)];
end