# Instruction to run the code

* Install dependencies:
```
sudo -H pip install pyquaternion
sudo -H pip install pandas
```

* Navigate to the code directory
```
cd madgwick_filter
```


* Execute the following command to run the code: 

```
python madgwick.py --imu_data=./Data/Train/IMU/imuRaw3.mat --vicon_data=./Data/Train/Vicon/viconRot3.mat
```

* There are two command line arguments:
1) --imu_data: Path to imu data
2) --vicon_data: Path to vicon data

# Results
### Dataset 1
![Dataset_1 results](https://github.com/Pratiquea/drone-course/blob/master/madgwick_filter/images/Train_1.png)
### Dataset 2
![Dataset_2 results](https://github.com/Pratiquea/drone-course/blob/master/madgwick_filter/images/Train_2.png)
### Dataset 3
![Dataset_3 results](https://github.com/Pratiquea/drone-course/blob/master/madgwick_filter/images/Train_3.png)
### Dataset 4
![Dataset_4 results](https://github.com/Pratiquea/drone-course/blob/master/madgwick_filter/images/Train_4.png)
### Dataset 5
![Dataset_5 results](https://github.com/Pratiquea/drone-course/blob/master/madgwick_filter/images/Train_5.png)
