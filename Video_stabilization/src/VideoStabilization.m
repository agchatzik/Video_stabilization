close all;
clear all;

%Read a video
v = VideoReader('video1.avi');
video = read(v);
[k,l,n,numFrames]= size(video);

%create the video object
video2 = VideoWriter('stabilized_optical.avi'); 
open(video2); %open the file for writing

stabilized = video(:,:,:,1);

for i = 1:numFrames-1
    
    fr1 = stabilized;
    fr2 = video(:,:,:,i+1);
    
    % downsize to half
    im1t = im2double(rgb2gray(fr1));
    im1 = imresize(im1t, 0.5); % downsize to half

    im2t = im2double(rgb2gray(fr2));
    im2 = imresize(im2t, 0.5); % downsize to half
    
    windowSize = 8;
    [u,v] = opticalFlow(im1,im2,windowSize);
    
    stabilazed = WarpImage(im1,u,v);
  
    im3 = imresize(stabilazed, 2); % upsize to double
    writeVideo(video2,mat2gray(im3)); %write the image to file

end
close(video2); %close the file
implay('stabilized_optical.avi');

%%%%%%%%%%%%%%%%%%%%

function I_warp = WarpImage(I,u,v)
    % I - image to warp
    % u,v - the optical flow parameters gotten from the LK algorithm        

    [x, y] = meshgrid(1:size(I,2),1:size(I,1));
    % interpolate
    I_warp = interp2(I, x+u, y+v, 'cubic');
    
    % in case of NaN values - put the original values of I instead
    I_warp(isnan(I_warp)) = I(isnan(I_warp));

end

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