function Jcam = calJcam(X,Y,Z)
Jcam = (1.0/Z)*[1.0 0 -X/Z;
    0 1.0 -Y/Z];
end