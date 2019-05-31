# build the filters that might be used
def sobel_abs_thresh(img, orient='x',sobel_kernel = 5,thresh=(20, 100)):
    # gradient filters take gray or binary image
    # Apply x or y gradient with the OpenCV Sobel() function
    # and take the absolute value
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
    # Rescale back to 8 bit integer
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    # Create a copy and apply the threshold
    binary_output = np.zeros_like(scaled_sobel)
    # Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
    binary_output[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

    # Return the result
    return binary_output

def sobel_mag_thresh(img, sobel_kernel=9, mag_thresh=(30, 100)):
    # gradient filters take gray or binary image
    # Take both Sobel x and y gradients
    sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the gradient magnitude
    gradmag = np.sqrt(sobelx**2 + sobely**2)
    # Rescale to 8 bit
    scale_factor = np.max(gradmag)/255 
    gradmag = (gradmag/scale_factor).astype(np.uint8) 
    # Create a binary image of ones where threshold is met, zeros otherwise
    binary_output = np.zeros_like(gradmag)
    binary_output[(gradmag >= mag_thresh[0]) & (gradmag <= mag_thresh[1])] = 1

    # Return the binary image
    return binary_output

def sobel_dir_threshold(img, sobel_kernel= 5, thresh=(0.7, 1.3)):
    # gradient filters take gray or binary image
    # Calculate the x and y gradients
    sobelx = cv2.Sobel(img, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(img, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the gradient direction, 
    # apply a threshold, and create a binary image result
    absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
    binary_output =  np.zeros_like(absgraddir)
    binary_output[(absgraddir >= thresh[0]) & (absgraddir <= thresh[1])] = 1

    # Return the binary image
    return binary_output

def color_threshold(img, colorMode = 'HLS', channel = 'S', thresh = (170, 255)):
    # experiment with different color thresholding methods
    if colorMode == 'HLS':
        image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        ChInx = {'H':0,'L':1,'S':2}
        Ch_thresh = image[:,:,ChInx[channel]]
        
    elif colorMode == 'HSV':
        image = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        ChInx = {'H':0,'S':1,'V':2}
        Ch_thresh = image[:,:,ChInx[channel]]
        
    elif colorMode == 'RGB':
        ChInx = {'R':0,'G':1,'B':2}
        Ch_thresh = img[:,:,ChInx[channel]]
        
    else:
        print ("colorMode incorrrect")
        return 0
    
    # convert to 0-255
    Ch_thresh = 255 * (Ch_thresh / np.max(Ch_thresh))
    # apply threshold
    binary_output = np.zeros_like(Ch_thresh)
    binary_output[(Ch_thresh > thresh[0]) & (Ch_thresh <= thresh[1])] = 1
    
    return binary_output

def combined_thresh (img, sobel_kernel = (5,5,5), absx_thresh = (20,100), absy_thresh=(50,100),mag_thresh = (30,100),\
                     dir_thresh = (0.7,1.3), colorMode = ['HLS','RGB'], channel = ['S','R'], col_thresh =[(90,255),(200,255)] ):
    
    
    col1_binary = color_threshold(img, colorMode = colorMode[0], channel = channel[0], thresh = col_thresh[0])
    col2_binary = color_threshold(img, colorMode = colorMode[1], channel = channel[1], thresh = col_thresh[1])
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    gradx = sobel_abs_thresh(gray, orient='x', sobel_kernel=sobel_kernel[0], thresh=absx_thresh)
    grady = sobel_abs_thresh(gray, orient='y', sobel_kernel=sobel_kernel[0], thresh=absy_thresh)
    mag_binary = sobel_mag_thresh(gray, sobel_kernel=sobel_kernel[1], mag_thresh=mag_thresh)
    dir_binary = sobel_dir_threshold(gray, sobel_kernel=sobel_kernel[2], thresh=dir_thresh)
    
    combined_binary = np.zeros_like(dir_binary)
    combined_binary[((gradx == 1)) | \
                    ((mag_thresh == 1)&(dir_thresh == 1)) |\
                    ((col1_binary == 1)&(col2_binary == 1))  ] = 1
    
    return combined_binary

def warp_imag(img,dst,src):
    img_size = (img.shape[1],img.shape[0])
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst,src)
    warped = cv2.warpPerspective(img, M, img_size)
    
    return warped,M,Minv

def find_lane_pixels(binary_warped):
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0]//2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    # HYPERPARAMETERS
    # Choose the number of sliding windows
    nwindows = 9
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50

    # Set height of windows - based on nwindows above and image shape
    window_height = np.int(binary_warped.shape[0]//nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated later for each window in nwindows
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        
        # Draw the windows on the visualization image
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),
        (win_xleft_high,win_y_high),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xright_low,win_y_low),
        (win_xright_high,win_y_high),(0,255,0), 2) 
        
        # Identify the nonzero pixels in x and y within the window #
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices (previously was a list of lists of pixels)
    try:
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
    except ValueError:
        # Avoids an error if the above is not implemented fully
        pass

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty

# define curve fitting function
def fit_polynomial(img_shape,leftx, lefty, rightx, righty):
    
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    return left_fit, right_fit
    
def polynomial_pts(img_shape, left_fit, right_fit):
    
    # Generate x and y values for plotting
    ploty = np.linspace(0, img_shape[0]-1, img_shape[0])
    ### TO-DO: Calc both polynomials using ploty, left_fit and right_fit ###
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    return left_fitx, right_fitx, ploty