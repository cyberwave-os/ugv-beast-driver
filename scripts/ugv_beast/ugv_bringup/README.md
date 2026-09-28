UGV Beast Cyberwave Compatibility Files
========================================

Purpose
-------
This folder contains compatibility files for the Cyberwave UGV Beast setup.
These files were created so we can adapt the Cyberwave configuration without
modifying the original UGV Beast source files in the main workspace.

Important
---------
These files are NOT used directly from this folder. The correct workflow is to
copy/paste them into the real UGV Beast package locations, keeping the original
folder structure.

Files and Destinations
----------------------
1) Launch file
   Source:
     /launch/master_beast.launch.py
   Copy to:
     /home/ws/ugv_ws/src/ugv_main/ugv_bringup/launch/master_beast.launch.py

2) Integrated driver
   Source:
     /ugv_bringup/ugv_integrated_driver.py
   Copy to:
     /home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/ugv_integrated_driver.py

Scope
-----
These files exist only to make Cyberwave compatible with the UGV Beast
configuration while keeping the original code untouched. If you need to apply
the changes, copy them into the actual UGV Beast folders listed above.






