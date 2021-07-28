close all;
clear all;

%Read a video
v = VideoReader('video1.avi');

video = read(v);
[k,l,n,numFrames]= size(video);

%create the video object
video2 = VideoWriter('stabilized_affine.avi');
open(video2); %open the file for writing

stabilized = video(:,:,:,1);

for i = 1:numFrames-1
    
    fr1 = stabilized;
    fr2 = video(:,:,:,i+1);
    
    % downsize to half
    im1t = im2double(rgb2gray(fr1));
    im1 = imresize(im1t, 0.5);
    
    im2t = im2double(rgb2gray(fr2));
    im2 = imresize(im2t, 0.5); % downsize to half
    
    windowSize = 8;
    [u,v] = opticalFlow(im1,im2,windowSize);
    
    %eigenvalues threshold
    thresh = 20;
    windowSize = 8;

    [corner_i,corner_j] = cornerDetection(im1,windowSize, thresh);
    
    [m,n] = size(corner_i);
    
    %get matrices!!!
    A = [];
    b = [];

    % get coordinate for u and v in the original frame
    for i = 1:n   
          A2 = [corner_j(i) corner_i(i) 0 0 1 0 ; 0 0 corner_j(i) corner_i(i) 0 1];
          b2 = [corner_j(i) +  u(corner_i(i),corner_j(i)); corner_i(i) +  v(corner_i(i),corner_j(i))];
          A = [A ; A2];
          b = [b ; b2];
    end
    
    t = pinv(A.'*A)*A'*b;
    
    T = [t(1) t(2) 0; t(3) t(4) 0; t(5) t(6) 1];
  
    tform = affine2d(T);
    stabilazed = imwarp(im1,tform);
  
    im3 = imresize(stabilazed,[240,320]); % upsize to double
    writeVideo(video2,mat2gray(im3)); %write the image to file

end
close(video2); %close the file
%implay('stabilized_affine.avi');

function [corner_i, corner_j] = cornerDetection(im,windowSize, thres)

    w = windowSize;
    
    %offset
    off = round(w/2);
    
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


function [u,v] = opticalFlow(im1,im2,windowSize)

    w = windowSize;
    %offset
    off = round(w/2);  
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