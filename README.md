## GPS Denied Navigation
AEM 4331, Fall 2022

## Running a Sim
The two main entrypoints are `Simulation/imu_test.m` and `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_EKF_loose_integration.m`. 

To start, run `Simulation/imu_test.m`, which will create a `.mat` file with the trajectory/sensor details. Within `Simulation/imu_test.m`, to select a trajectory just uncomment the desired trajectory and comment out the rest.

After `imu_test.m` has run, change the load path in `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_data_loader.m` to match the desired trajectory. Then you can run `GNSS INS Filters/Case_Study_1/MATLAB/gnss_ins_EKF_loose_integration.m` and the integration will begin. The results will automatically be plotted using the script `GNSS INS Filters/Case_Study_1/MATLAB/plot_EKF_output.m`