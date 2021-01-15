# unicorn_description

Package contains meshes and URDF files for the UNICORN robot.

`unicorn_target.urdf.xacro` is the main URDF file used on the robot, while an additional file `unicorn_sim.urdf.xacro` is provided for the simulator. 

Both files are identical except for some dummy frames that are inactive in the target file. The target file also has a rotation between the base link and the chassis link of 180 degrees which is necessary since the robots driving direct is flipped.  

![Unicorn](docs/frames_horisontal.png)

## Tools for exporting Models

Models can be exported from Solidworks with the URDF exporter tool. When exporting STL files that will be used for sensors or moving joints then it is very important to set the coordinate systems before exporting. 

* [URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) - URDF export for Solidworks

## Remarks

Note: The in the latest version the odom_chassis link is connected directly to the chassis link and not to the base_link. 
