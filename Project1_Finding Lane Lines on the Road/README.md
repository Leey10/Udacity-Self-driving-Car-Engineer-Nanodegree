# **Finding Lane Lines on the Road** 


### 1. Description of the project
  There are two taskes in this project:  
  1. identify the lanes on a road image captured by camera,correctly mark the lanes
  and highlight the lanes on the original image; 
  2. apply the algorithm on a video clip to test how the algorithm works 

### 2. Methods and tools
  The algorithm is implemented using openCV library in Jupyter notebook.
### 3. Algorithm Description
  - Step 1 : Convert Image to Grayscale  
  The task is mostly to detect edges, dealing with grayscale image can simplify the processing.  
  - Step 2 : Bluring the Image  
  To reduce the noise and increase the robustness of the edge detection, Gaussian bluring is used to pre-processing image.  
  - Step 3 : Canny Edge Detection  
  Now, the edges in the image can be detected. [Canny edge detection](https://en.wikipedia.org/wiki/Canny_edge_detector) is a 
  popular and powerful tool to do the job. The openCV function [cv2.Canny](https://docs.opencv.org/3.1.0/da/d22/tutorial_py_canny.html)
  is invoked.  
  - Step 4 : Masking Region of Interest  
  To increase the robustness of lane detection algorithm, a mask process is applied to the output from cv2.Canny(), which keeps the
  region containing the lanes while set the rest region to black. The region containing the lanes can be defined as a quadrilateral
  if the region of insterest does not include view too far ahead, or it can be defined as triangle if the far away area is also of 
  interest.  
  - Step 5 : Hough Line Detection  
    (1). Using [cv2.HoughLinesP](https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html) from openCV library, which implements
    [Hough Transformation](https://en.wikipedia.org/wiki/Hough_transform) to detect lines on the masked image of step 4. Basically,
    hough transformation reads in the pixels from input image and output (x1,y1),(x2,y2) points that are on the same line. The
    detected lines can be plotted using cv2.line.  
    (2). The direct output from cv2.HoughLinesP are usually line segments, the detection performance depends heavily on the parameters
    used for the hough transformation as well as the image results from the previous steps. Therefore, the output from hough
    line detection needs to be post processed to find the complete line of the lanes. The method is as follows:  
    * go through all the point-pairs from hough transformation, calculate the slope of the lines, based on the sign of slope, 
    left lane points and right lane points can be seperated;  
    * for left lane, there are two lines associated with it, one is the outter and the other is the inner line. In the original hough line
    detection result, the two lines actually form a triangle, since the two lines merge together in the far perspective. Note that,
    the far perspective has the smallest y value among all the points,while one of the point on the outter line of left lane will have
    the smallest x value. Therefore, if the following two points are found: (x1, smallest_y), (smallest_x, y2), they can be used to approximate
    the average slope of left lane;  
    * for right lane, the same method is used by looking for: (x1, smallest_y), (largest_x, y2);  
    * after the average slope of the lanes are found, we then constrain all the lines to fall in the area with y values between 0.6*height and 
    height;  
    * there is one more protection scheme for the case when one of the lanes can not be detected. In this case, the failed lane 
    will be mirrored by the successful one, assuming the camera is symmetrical to the lanes.
  - Step 6 : Highlight the Lanes on the Original Image  
  Overlap the original image with detected line image with cv2.addWeighte.  
  
### 4. Remaining Bugs  
   (1). sometimes one of the lane can not be detected;  
   (2). sometimes lane detection result shows two (nearly) horizontal lines, this occurs in the Challenge video;  
   (3). for the lanes that are passing (nearly) white road, the lane detection easily fails, this occurs in the Challenge
   video.


