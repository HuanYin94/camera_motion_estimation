clc
close all

srcImage = imread('../0000000230.jpg');
imshow(srcImage);title('Source Image')

edgeDetected = edge(srcImage, 'canny');
imshow(edgeDetected);title('After Canny')
