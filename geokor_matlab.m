%  Example call of the GEOKOR function
%  -------------------------------------------
% function test
%          ====
% f = imread('image1.ppm');                          % Read example images
% g = imread('image2.ppm');
% H = [1 0.1 10; 0 1 20; 0 0 1];   % Example for projective Transformation
% i = geokor_matlab(H, f, g);     % Transform image f using H and combine with g
% figure; imshow(i);                                   % Show image mosaic

function i = geokor_matlab (H, f, g)
%        ===========================
tform = projtform2d(H);
[fy, fx, ~] = size(f);                                    % f original size
[gy, gx, ~] = size(g);                                    % g original size
[lx, ly] = outputLimits(tform, [1 fx], [1 fy]);    % f transformed position

bx(1) = round(min(lx(1), 1));                     % common boundaries for g
bx(2) = round(max(lx(2), gx));                          % and f transformed
by(1) = round(min(ly(1), 1));
by(2) = round(max(ly(2), gy));
RA = imref2d([by(2)-by(1)+1, bx(2)-bx(1)+1], bx, by);

i = imwarp(f, tform, outputView=RA);     % transform image f in common area

ry = 2-by(1):gy-by(1)+1;                     % position of g in common area
rx = 2-bx(1):gx-bx(1)+1;
i(ry, rx, :) = max(g, i(ry, rx, :));             % combine brightest pixels
