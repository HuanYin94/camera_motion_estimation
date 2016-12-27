%% From ICRA 2016
% by YH
% in ZJU

clc;
clear
close all
%%

imagePath =  '.\rgbd_dataset_freiburg1_xyz\rgb\';
imgPathList = dir(strcat(imagePath, '*.png'));
imgNum = length(imgPathList);

depthPath = '.\rgbd_dataset_freiburg1_xyz\depth\';
depthPathList = dir(strcat(depthPath, '*.png'));
depthNum = length(depthPathList);

%%
result = zeros(798, 8);

initialPose = [ 0.0754    0.9971    0.0102    1.3405;...
                      0.6139   -0.0384   -0.7884    0.6266;...
                    -0.7857    0.0657   -0.6150    1.6575;...   
                            0         0         0    1.0000];

% initialPose = eye(4);

globalPoses{1} = initialPose;

translate = globalPoses{1}(1:3, 4)';
W = rotm2quat(globalPoses{1}(1:3, 1:3));
result(1, :) = [str2num(imgPathList(1).name(1:17)), round(translate, 6),round(W(1, 2:4), 6), round(W(1, 1), 6)];

%%
tic

if imgNum > 0
    
    % Keyframe Selected
        for i = 1 : 3 : imgNum   
            refImageName = imgPathList(i).name;
            refImage =  imread(strcat(imagePath, refImageName));
            
            refDepthName = depthPathList(i).name;
            refDepth = imread(strcat(depthPath, refDepthName));
            
            % Comparison
            for j = i + 1 : 1 : i + 3
               if j > imgNum
%                 if j == 3
                   break;
               end
                    
               curImageName = imgPathList(j).name;
               curImage = imread(strcat(imagePath, curImageName));
               
               cur2RefRT = estimate(refImage, curImage, refDepth);
               
               globalPoses{j} = globalPoses{i} * cur2RefRT;
               
               translate = globalPoses{j}(1:3, 4)';
               W = rotm2quat(globalPoses{j}(1:3, 1:3));
               fileStamp = str2num(imgPathList(j).name(1:17));
               result(j, :) = [1, round(translate, 6),round(W(1, 2:4), 6), round(W(1, 1), 6)];
               result(j,1) = fileStamp;   %???
               
               % Output
               j
%                result(j, :)
                   
            end
        end
end

toc
