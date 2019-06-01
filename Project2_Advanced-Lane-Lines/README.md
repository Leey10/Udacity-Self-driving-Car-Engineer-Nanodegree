## Advanced Lane Line Detection


### 1. Description of the project
The project is about developing a pipeline for detecting the lanes on the road using openCV, including the following tasks:
* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

### 2. Methods and tools
  The algorithm is implemented using openCV library in Jupyter notebook.
### 3. Algorithm Description
#### Calibrate the Camera
  This step is used to correct the distortion from front camera of the car. Use about 20 - 25 chess board images for the undistortion, the object points on the chess board are preset coordinates of the corners of chess board, image points are the identified corners by openCV. These object - image point pairs are then used to fit correction transformation coefficients, which is then used to pre-process each frame from the video.
#### Filter-Out the Lanes from Image
  Now, each image should first be undistored with the undistortion coefficients. The main goal of this step is then to extract the pixels that belongs to lanes while get rid of the other pixels. To do this, two types of filters are built:  
  * gradient based: including sobel_x and sobel_y filters which detect pixels that belong to vertical and horizontal lines. The sobel_x and sobel_y filter outputs can be further made use of to decide combined magnitude of the gradient and direction of the gradient to obtain more information for lane pixel detection.  
  * color based : to use color information of lanes for detection, including 'HLS', 'HSV' and 'RGB' color space.
  * from the experiment results, sobel_x filter + 'S' channel from 'HLS' space can do a pretty good job. Adding extra 'R' channel to the filter can improve the performance. However, adding gradient magnitude and direction do not improve the performance significantly if a good sobel_x filter has already been used.  
#### Perspective Transformation
  After the lanes pixels are filtered out, the whole image is warped to a bird-eye view. The reason to do this are twofold:
  * for curved lanes, it's much easier to fit a polynomial based on the lane pixels detected from a bird-eye view, this is an important step to extrapolate the whole lane based on broken lane pixels detected
  * doing this can get rid of the perspective effect and give more accurate result for curvature calculation  
  The way to select good 'src' and 'dst' points are as follows:  
  * find an image with long straight lanes only, define a trapezoidal on the lanes with two sides of the trapezoidal exactly align with the lanes  
  
  Theoretically, this trapezoidal will be a rectangle from bird-eye, therefore, the 'dst' should be a rectangle. Usually, the height of the rectangle equals to the height of image, meaning the bird-eye only 'sees' the trapezoidal area. The width of the rectangle has an 'zoom in/out' effect,i.e., larger the width, 'zoom in' to lanes, less is shown for other pixels, smaller the width, 'zoom out' to lanes, more will be shown like shade of trees. Here, get both the transformation matrix M and inverse of M, so that the warped image can be transformed back to normal view for final display.
#### Polyfit the Lane Pixels and Calculate Number
  For a new image, the pixels belongs to a line will show a peak on the histogram, which can then be used to identify start position of lanes. With a starting position of lanes, more pixels can be found using moving window until all the pixels of a lane are found. The detected lane pixels can be used for a polynomial fit which will then be used for calculating curvature. 
  
  If, however, the polynomial for lanes have been previously found, there's no need to do histogram again assuming lanes will not have sudden change. In these cases, starting position of lanes can be calculated using the polynomial from last image. This has been proven a very good guess.  
  
  The numbers to be calculated include curvature of lanes and offest of the car center based on equations given in the course.
  
#### Transform the Image Back with Lane Detected and Information Printed
  Once everything is set, the warped image is transormed back with lane area filled and information printed.
  
### 3. Source of Bugs
  * performance of the computer vision stage is the dominant determinator of the algorithm, low contrast between lane and road, cracks that make a long dark line on the road all contribute to detection failures
