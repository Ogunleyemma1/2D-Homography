
function ProjectiveTransformImage

% Importing the images into mathlab
f = imread("a12.jpg");
g = imread("a3.jpg");

%Selecting points on the images
figure, imshow(f);
[Image1X,Image1Y] = ginput(4);
figure, imshow(g);
[Image2X,Image2Y] = ginput(4);

%Computing the midpoint for all 3 Images

t1 = [mean(Image1X), mean(Image1Y)]; % Midpoint for Image 1
disp("The midpoint for Image 1 is:");
disp(t1);

t2 = [mean(Image2X), mean(Image2Y)]; % Midpoint for Image 2
disp("The midpoint for Image 2 is:");
disp(t2);


% Making the midpoint as origin for the 3 Images and computing Translated
% corrdinate

X1_T = (Image1X - t1(1,1));
Y1_T = (Image1Y - t1(1,2));
Image1Translated = [X1_T, Y1_T]; % Translated points for Image 1


X2_T = (Image2X - t2(1,1));
Y2_T = (Image2Y - t2(1,2));
Image2Translated = [X2_T, Y2_T]; % Translated points for Image 2


% Computing the absolute average of the translated points defined as
% SCALING

s1 = [mean(abs(X1_T)), mean(abs(Y1_T))]; % Scaling for Image 1
disp("The scaling for Image 1 is: ");
disp(s1);


s2 = [mean(abs(X2_T)), mean(abs(Y2_T))]; % Scaling for Image 2
disp("The scaling for Image 2 is: ");
disp(s2);


% Computing the Coordinate Transformation for the 3 Images
% T = S TransMatrix X Midpoint TransMatrix

s1_TransMatrix = [1/(s1(1,1)), 0, 0; 0, 1/(s1(1,2)), 0; 0, 0, 1];
t1_TransMatrix = [1, 0, -1*(t1(1,1)); 0, 1, -1*(t1(1,2)); 0, 0, 1];

Image1Transformation = s1_TransMatrix * t1_TransMatrix; % Coordinate transformation for Image 1
disp("The coordinate transformation for Image 1 is: ");
disp(Image1Transformation);


s2_TransMatrix = [1/(s2(1,1)), 0, 0; 0, 1/(s2(1,2)), 0; 0, 0, 1];
t2_TransMatrix = [1, 0, -1*(t2(1,1)); 0, 1, -1*(t2(1,2)); 0, 0, 1];

Image2Transformation = s2_TransMatrix * t2_TransMatrix; % Coordinate transformation for Image 2
disp("The coordinate transformation for Image 2 is: ");
disp(Image2Transformation);



%Computing the conditioned coordinates of the image

Image1Conditioned = [X1_T * Image1Transformation(1,1), Y1_T * Image1Transformation(2,2)]; % Conditioned Coordinaate points for Image 1
disp("The conditioned coordinate of Image 1 is: ");
disp(Image1Conditioned);

Image2Conditioned = [X2_T * Image2Transformation(1,1), Y2_T * Image2Transformation(2,2)]; % Conditioned Coordinaate points for Image 2
disp("The conditioned coordinate of Image 2 is: ");
disp(Image2Conditioned);


conditioned1 = transpose(Image1Conditioned);
conditioned2 = transpose(Image2Conditioned);

% Construction of the design maxtrix A for Image 1 and 2

A12 = zeros(2 * size(conditioned1, 2), 9);

for i = 1:size(conditioned1,2)
    
    A12(2 * i-1, :) = [-conditioned1(1, i), -conditioned1(2, i), -1, 0, 0, 0, conditioned1(1, i) * conditioned2(1, i), conditioned1(2, i) * conditioned2(1, i), conditioned2(1, i)];
    A12(2 * i, :) = [0, 0, 0, -conditioned1(1, i), -conditioned1(2, i), -1, conditioned1(1, i) * conditioned2(2, i), conditioned1(2, i) * conditioned2(2, i), conditioned2(2, i)];
    disp(" The Design Matrix A12 for Image 1&2 is expressed as:");
    disp(A12);
   
end


[U1, D1, V1] = svd(A12); %Single value decomposition

disp("The value of D is");
disp(D1);

H12_p = [V1(1,9), V1(2,9), V1(3,9); V1(4, 9), V1(5,9),V1(6,9); V1(7,9), V1(8,9), V1(9,9)]; %Computing the homography matrix for projective point for image 1&2

H12_r = inv(Image2Transformation) * H12_p * Image1Transformation; %Computing the homography matrix for Euclidean point for image 1&2

H12 = H12_r(:,:)/H12_r(end,end);  %Normalizing


Image12 = geokor_matlab(H12, f, g); %joining image 1&2 with the geokor function

figure, imshow(Image12); %displacing the stiched image 1&2

end



function Image12 = geokor_matlab (H12, f, g)
%        ===========================
tform = projtform2d(H12);
[fy, fx, ~] = size(f);                                    % f original size
[gy, gx, ~] = size(g);                                    % g original size
[lx, ly] = outputLimits(tform, [1 fx], [1 fy]);    % f transformed position

bx(1) = round(min(lx(1), 1));                     % common boundaries for g
bx(2) = round(max(lx(2), gx));                          % and f transformed
by(1) = round(min(ly(1), 1));
by(2) = round(max(ly(2), gy));
RA = imref2d([by(2)-by(1)+1, bx(2)-bx(1)+1], bx, by);

Image12 = imwarp(f, tform, outputView=RA);     % transform image f in common area

ry = 2-by(1):gy-by(1)+1;                     % position of g in common area
rx = 2-bx(1):gx-bx(1)+1;
Image12(ry, rx, :) = max(g, Image12(ry, rx, :));             % combine brightest pixels

end
