close all;
clear all;

%Read a video
v = VideoReader('video1.avi');

%Extract Frames from video
video = read(v);

%Frame No1 
fr1 = video(:,:,:,1);
im = im2double(rgb2gray(fr1));

%im1 = imresize(im1t, 0.5); % downsize to half

%eigenvalues threshold
thresh = 20;
windowSize = 8;

[corner_i,corner_j] = cornerDetection(im,windowSize, thresh);

Image_corners = fr1;
[m,n] = size(corner_i);

for i = 1:20:n   
      Image_corners = insertMarker(Image_corners,[corner_j(i) corner_i(i)]);
end

figure();
imshow(Image_corners);
hold on;
title('Corners (window size = 8, threshold = 20)', 'FontSize', 10);

function [corner_i, corner_j] = cornerDetection(im,windowSize, thres)

    w = windowSize;
    
    %offset
    off = floor(w/2)+1;
    
    dx = [-1 0 1; -1 0 1; -1 0 1]; % The Mask 
    dy = dx';
    
    % Lucas Kanade
    % for each point, calculate I_x, I_y, I_t
    Ix_m = conv2(im, dx, 'valid'); % partial on x
    Iy_m = conv2(im, dy, 'valid'); % partial on y
  
    corner_i = [];
    corner_j = [];

    % within window w * w
    for i = off+1:size(Ix_m,1)-off
       for j = off+1:size(Ix_m,2)-off
          Ix = Ix_m(i-off:i+off, j-off:j+off);
          Iy = Iy_m(i-off:i+off, j-off:j+off);

          Ix = Ix(:);
          Iy = Iy(:);
  
          A = [Ix Iy]; % get A here

          %second moment matrix
          M = A'*A;
          %eigenvalues of matrix M
          e = eig(M);
          
          %R(i,j) = det(M)-.05*trace(M)^2;
          
          if (e(1)>thres) && (e(2)>thres)
            corner_i = [corner_i i];
            corner_j = [corner_j j];
          end 

       end
    end
end

