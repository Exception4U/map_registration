# map_registration
  Register pointcloud on specific frame and visualise dense MAP of joint trajectories

## CAIR milestone review final

Steps
- put tushar.launch in ![rtabmap_ros](https://github.com/introlab/rtabmap_ros)
- generate g2o file of joint trajectory of multiple sessions from our multi-robot framework
- give path of g2o file in point_cloud_publish.ipynb
- Run rtabmap ros with tushar.launch
- run bagfile in same order as joint trajectories in g2o file

