# Directory structure -
-> data folder (contains the training and testing data cropped from original video)
-> Python scripts for execution


# Instructions to run the program
[The files 1D_Gauss.py and Train-test.py are wrapper scripts to run the execution of 1d-Gaussian and Multivariate training and testing on original video respectively.]

1. To crop the buoys using event handler clicking, run "python3 Crop_out_buoys.py"
2. To run the 1-D Gaussian for all three buoys sequentially, run "python3 1D_Gauss.py". This will run the green, yellow and orange buoy detection sequentially.
3. To train the mutlivariate gaussian model on the dataset and test the orinigal video for buoy detection, run the below command"
"python3 Train-test.py"


The google link for video outputs is given below:
https://drive.google.com/drive/u/1/folders/1C9TbQrw8_zTn31RksvAmI_Eud8wLiQic

