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
    
