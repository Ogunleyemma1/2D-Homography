# 2D-Homography
Computer Vision involving 2D Homography and Image Stitching
 Projective Transformation (Homography)
In this exercise, a fundamental projective transformation should be realized. The task consists of rectifying images geometrically and stitching them together, so that a panoramic image mosaic is generated.
Image acquisition: Take pictures with a digital camera, which overlap at least 30 percent in horizontal direction. Produce thereby three images without disparities (i.e. turn around the projection center or choose a planar object).
Correspondence analysis: Transfer the three images into the computer (imread) and measure interactively (figure, imshow, ginput) at least four corresponding image points ′↔xx between two neighboring images.
Homography computation: Implement an own function in MATLAB to estimate a 2D homography using the singular value decomposition (mean, abs, svd, reshape). Compute the homography matrix H12 from first to second image.
Projective rectification: Use the provided auxiliary program geokor.m, which is located in the “geokor source code” folder, and the estimated homography to adapt the first image to the second one (geokor combines the respective frames to a common mosaic image). Then, compute the homography H32 from third image to the intermediate mosaic image and stitch them together. Note: Newer versions to the geokor function, specific to MATLAB and Octave, can also be found in the same folder. The functionality of all three implementations is equivalent.
Visualization: Show the produced panoramic image on the screen.

