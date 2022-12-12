# Location-Tracker-using-Particle-Filtering


# Setting the environment
Create the conda environment conda env create -n assign3--file environment.yml 
## For Linux conda env create -nassign3 --file environment_win.yml 
## For Windows conda envcreate -n assign3 --file environment_mac.yml 
## For Mac conda activate assign3 # Run the environment

# Running the code
## Invoke the environment (without estimation) in the ‘small’ layout with 2 StdCars as follows:
python3 drive.py-d -k 2-l small -a -i none
## The estimator implementation can be tested with the default driver ‘Auto Driver’ on the above environment as follows :    
python3 drive.py-d -k 2-l small -a -i estimator
## The intelligent driver implementation can be tested on layouts with multiple goals as follows: 
python3 drive.py-d -k 2-m -l small -a -i estimator -j

Note that the simulation automatically stops when the AutoCar crashes, i.e., 
collides with a StdCar, hits the obstacles, or hits the boundary of the layout. 
After a crash, you can close the simulation window using the GUI or by pressing the ‘q’ key.
