function drawPoses(pose)

% plot every pose
figure; hold on; axis equal;
l = 0.01; % coordinate axis length
A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
for i=1:length(pose)
  B = pose{i}*A;
  plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',100); % x: red
  plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',100); % y: green
  plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',100); % z: blue
end
xlabel('x');
ylabel('y');
zlabel('z');