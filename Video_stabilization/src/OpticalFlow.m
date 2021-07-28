close all;
clear all;

%Read a video
v = VideoReader('video1.avi');

%Extract Frames from video
video = read(v);

%Frame No1 
fr1 = video(:,:,:,1);
subplot 121
imshow(fr1);
title('Frame No1', 'FontSize', 10);

%Frame No2 
fr2 = video(:,:,:,2);
subplot 122
imshow(fr2);
title('Frame No2', 'FontSize', 10);

% downsize to half
im1t = im2double(rgb2gray(fr1));
im1 = imresize(im1t, 0.5); 

im2t = im2double(rgb2gray(fr2));
im2 = imresize(im2t, 0.5); 

windowSize = 9;

[u,v] = opticalFlow(im1,im2,windowSize);

% downsize u and v
u_deci = u(1:5:end, 1:5:end);
v_deci = v(1:5:end, 1:5:end);

% get coordinate for u and v in the original frame
[m, n] = size(im1t);
[X,Y] = meshgrid(1:n, 1:m);
X_deci = X(1:10:end, 1:10:end);
Y_deci = Y(1:10:end, 1:10:end);

figure();
imshow(fr2);
hold on;
% draw the velocity vectors
quiver(X_deci, Y_deci, u_deci,v_deci, 'y')
title('Computed Optical Flow with theshold 0.4', 'FontSize', 10)

function [u,v] = opticalFlow(im1,im2,windowSize)

    w = windowSize;
    %offset
    off = floor(w/2)+1;  
    % The Mask 
    dx = [-1 0 1; -1 0 1; -1 0 1]; 
    dy = dx';

    % Lucas Kanade 
    % for each point, calculate I_x, I_y, I_t
    Ix_m = conv2(im1, dx, 'valid'); % partial on x
    Iy_m = conv2(im1, dy, 'valid'); % partial on y
    It_m = conv2(im1, ones(3), 'valid') + conv2(im2, -ones(3), 'valid'); % partial on t
    u = zeros(size(im1));
    v = zeros(size(im2));

    % within window w * w
    for i = off+1:size(Ix_m,1)-off
       for j = off+1:size(Ix_m,2)-off
          Ix = Ix_m(i-off:i+off, j-off:j+off);
          Iy = Iy_m(i-off:i+off, j-off:j+off);
          It = It_m(i-off:i+off, j-off:j+off);

          Ix = Ix(:);
          Iy = Iy(:);
          b = -It(:); % get b here

          A = [Ix Iy]; % get A here

          %second moment matrix
          M = A'*A;

          %eigenvalues of matrix M
          e = eig(M);
          %eigenvalues threshold
          thresh = 0.4;
          
          if (e(1) < thresh) || (e(2)<thresh)
            u(i,j)= 0;
            v(i,j)= 0;
          else
            nu = pinv(M)*A'*b; % get velocity here
            u(i,j)=nu(1);
            v(i,j)=nu(2);
          end  
       end
    end
end
