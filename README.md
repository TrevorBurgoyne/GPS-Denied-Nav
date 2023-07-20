## GPS Denied Navigation
AEM 4331, Fall 2022

## Downloading this Repository
If you have git installed on your machine and have github ssh credentials linked to your github account, you can use git clone 
from the command line. Navigate to where you want the repository to be put, and then do the following:

    >> git clone git@github.com:TrevorBurgoyne/GPS-Denied-Nav.git

This automatically begin tracking the repository using git, allowing you to push commits, etc. from the command line.

Otherwise, you can also just download the code by clicking on the green Code button on the github page, and then download the repository as a ZIP file.

## Running a Sim
The two main entrypoints are `Simulation/imu_test.m` and `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_EKF_loose_integration.m`. 

To start, run `Simulation/imu_test.m`, which will create a `.mat` file with the trajectory/sensor details. Within `Simulation/imu_test.m`, to select a trajectory just uncomment the desired trajectory and comment out any other trajectories.

After `imu_test.m` has run, change the load path in `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_data_loader.m` to match the desired trajectory. Then you can run `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_EKF_loose_integration.m` and the integration will begin. The results will automatically be plotted using the script `GNSS INS Filters/Case_Study_1/MATLAB/plot_EKF_output.m`

If the flight time is something other than 20min, the start and end times will need to be changed in `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_filter_config.m`. Other parameters, such as those relating to the GPS/IMU drift, can also be changed in the config.

## Sources
The book referenced in `GNSS INS Filters/` is [GNSS Applications and Methods](https://us.artechhouse.com/GNSS-Applications-and-Methods-P1297.aspx) by Demoz Gebre-Egziabher & Scott Gleason. Much of the code was provided for educational purposes by Professor Gebre-Egziabher.
