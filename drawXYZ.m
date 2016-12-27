for i = 1 : 1 : 798
    X(i) = globalPoses{i}(1,4);
    Y(i) = globalPoses{i}(2,4);
    Z(i) = globalPoses{i}(3,4);
end

for i = 1 : 1 : 2649
    Xth(i) = groundTruthPoses{i}(1,4);
    Yth(i) = groundTruthPoses{i}(2,4);
    Zth(i) = groundTruthPoses{i}(3,4);
end

X = X';
Y = Y';
Z = Z';
Xth = Xth';
Yth = Yth';
Zth = Zth';


