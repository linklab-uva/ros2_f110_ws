# ros2_f110_ws

## Upgrade instructions for Jetson TX2 to ubuntu 22.04 for ros2 humble

Start by flashing the latest BSP and L4T image onto the tx2. At the time of writing that was Ubuntu 18.04 based.
Will need a monitor for initial setup - networking will also be necessary

Once installed
- Purge chromium browser and all components
- update and upgrade
- autoremove any packages
- remove nvidia jetson and connectech orbitty BSP from apt sources list to prevent further updates
- reboot
- modify /etc/update-manager/release-upgrades and change when updates are offered from never to normal
- update and upgrade again
- sudo apt-get dist-upgrade
- sudo do-release-update