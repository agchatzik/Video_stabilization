close all;
clear all;

%% Find Optical flow
%Read a video
v = VideoReader('video1.avi');

%Extract Frames from video
video = read(v);

%Frame No1 
fr1 = video(:,:,:,1);

%Frame No1 
fr2 = video(:,:,:,2);

subplot 211
imshow(fr1);
im1 = im2double(rgb2gray(fr1));
%im1 = imresize(im1t, 0.5); % downsize to half

subplot 212
imshow(fr2);
im2 = im2double(rgb2gray(fr2));
%im2 = imresize(im2t, 0.5); % downsize to half

windowSize = 8;

[u,v] = opticalFlow(im1,im2,windowSize);


%% Compute Corners

%eigenvalues threshold
thresh = 20;
windowSize = 8;

[corner_i,corner_j] = cornerDetection(im1,windowSize, thresh);

%% Compute optical flow for corners

[m,n] = size(corner_i);

u_deci = [];
v_deci = [];
X_deci = [];
Y_deci = [];

% get coordinate for u and v in the original frame
for i = 1:n     
      X_deci = [X_deci corner_j(i)];
      Y_deci = [Y_deci corner_i(i)];
      u_deci = [u_deci u(corner_i(i),corner_j(i))];
      v_deci = [v_deci v(corner_i(i),corner_j(i))];
end

figure();
imshow(fr1);
hold on;
% draw the velocity vectors
quiver(X_deci, Y_deci, u_deci,v_deci, 'y')
title('Optical flow on corners', 'FontSize', 10);


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
    It_m = conv2(im1, ones(2), 'valid') + conv2(im2, -ones(2), 'valid'); % partial on t
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
          M = A.'*A;

          %eigenvalues of matrix M
          e = eig(M);
          %eigenvalues threshold
          thresh = 0.3;
          
          if (e(1) < thresh) || (e(2)<thresh)
            u(i,j)= 0;
            v(i,j)= 0;
          else
            nu = pinv(M)*A.'*b; % get velocity here
            u(i,j)=nu(1);
            v(i,j)=nu(2);
          end;   
       end;
    end;
end

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
          M = A.'*A;
          %eigenvalues of matrix M
          e = eig(M);
          
          %R(i,j) = det(M)-.05*trace(M)^2;
          
          if (e(1)>thres) && (e(2)>thres)
            corner_i = [corner_i i];
            corner_j = [corner_j j];
          end 

       end;
    end;
end

