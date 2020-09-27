function H = calH()

H = zeros(6, 15);
H(1:3, 4:6) = eye(3);
H(4:6, 1:3) = eye(3);

end