Clone this repo to src/rome of the workspace directory

Copy .cyclonedds.xml to ~/.cyclonedds.xml

Then navigate to the workspace directory and run
    ```source src/rome/rome_scripts/cyclone_source.sh
       sudo rm -rf build install log
       colcon build --symlink-install
       source install/setup.bash
    ```
Then
    On rospi-0-desktop:
        `./src/rome/rome_scripts/launch_batons.sh`
    On rospi-<1-4>-desktop
        `./src/rome/rome_scripts/launch_relay.sh`
    
