# Rome_Py
This repository is the stress test system used across four different raspberry pi's across a local network.
## Structure
The structure of this repository is that everything needing to be ran is in Post_scripts. The contains all of the launching files, installation files, and DDS config files.
### How to run
SSHPI.sh runs a script that automatically installs all of the specified repositories files at the location specified in the gui, then it will launch a station on each of the pi's that listen for the parcels.
launch_parcels.sh runs a script that lets you specify parameters for the parcels that you will be sending to the stations, these parameters include the amount of parcels to send, what the instruction set is, whether you want the parcels to be sent forever, and whether to log the parcels.

