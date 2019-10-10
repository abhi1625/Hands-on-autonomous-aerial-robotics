import os

print("Training the dataset using multi variate gaussian model")
os.system("python gmm_train_data.py")

print("Testing the trained model on original video")
os.system("python test_data.py")