# Unscented Kalman Filter - Project 1(b)
UKF for orientation estimation from IMU
- [Instructions to run the code](Instructions-to-run-the-code)
- [Graph Plots](Graph-Plots)
- [Summary](Summary)

## Graph Plots
- Case 1
  ![](https://github.com/Pratiquea/drone-course/blob/master/UKF/images/Train_1.png)
- Case 2
  ![](https://github.com/Pratiquea/drone-course/blob/master/UKF/images/Train_2.png)
- Case 3
  ![](https://github.com/Pratiquea/drone-course/blob/master/UKF/images/Train_3.png)
- Case 4
  ![](https://github.com/Pratiquea/drone-course/blob/master/UKF/images/Train_4.png)
- Case 5
  ![](https://github.com/Pratiquea/drone-course/blob/master/UKF/images/Train_5.png)
- Case 6
  ![](https://github.com/Pratiquea/drone-course/blob/master/UKF/images/Train_6.png)
 
  

## Instructions to run the code
The folder contains the code file `ukf.py` which contains the implementaion and a sub-directory `images` which contains the results of estimated orientations on all Training and Test datasets provided.

- To run the code, copy the [`Data`](https://github.com/Pratiquea/drone-course/tree/master/drone_course_data/Data) folder inside the UKF directory and type:
```
python ukf.py
```
This will generate the results for the test case `imuRaw9.mat` by defaut. Feel free to change the file and plot other test cases.

## Summary
