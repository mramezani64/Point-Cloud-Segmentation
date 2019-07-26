How to make the project executable

Inside the Seg-Example folder open a terminal and follow the steps below:

- mkdir build
- cd build
- cmake ..
- make


Also, I included some of the FSC point clouds in the cloud folder, you can view them by:

In the terminal that clouds exist -> rosrun pcl_tools_drs pcl_viewr NAME_OF_THE_CLOUD

Note: If you would like to segment a point cloud, the point cloud should be coppied in the build folder. Also, for the point clouds other than "table_scene_lms400.pcd", change the name in the code; line 42.
