# Window_detection
Minimalist approach to detect colored windows on a quadrotor

#### Note:- store the video in the following directory: '../../drone_course_data/window_detection/pink_window.mp4'

## Pipeline
* Read the Video frames and perform the following for each frame:
  * Threshold the frame according to color of window.
  * Convert image to grayscale
  * Apply median filter to remove "salt and pepper" noise 
  * Performed morphological operations such as closing to fill unwanted small gaps
  * Use gaussian blurr 
  * Extract corners using goodFeaturesToTrack
  * Get 4 outermost corner points from the above extracted points
  * Perform PnP(Pespective-n-points)
  * Reproject points to verify PnP
