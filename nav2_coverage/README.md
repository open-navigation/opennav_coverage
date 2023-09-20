# Nav2 Complete Coverage Server

This package contains the Complete Coverage Task server utilizing the [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) complete coverage planning system which includes a great deal of options in headland, swath, route, and final path planning. You can find 

TODO bonsai, more details on library

## Interfaces


## Configuration

## Citation

If you use this work, please make sure to cite both Nav2 and Fields2Cover:

[Nav2 Paper](https://arxiv.org/abs/2003.00368)

```
@InProceedings{macenski2020marathon2,
  title = {The Marathon 2: A Navigation System},
  author = {Macenski, Steve and Martín, Francisco and White, Ruffin and Ginés Clavero, Jonatan},
  year = {2020},
  booktitle = {2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  url = {https://github.com/ros-planning/navigation2},
  pdf = {https://arxiv.org/abs/2003.00368}
}
```

[Fields2Cover Paper](https://arxiv.org/pdf/2210.07838.pdf)

```
@article{Mier_Fields2Cover_An_open-source_2023,
  author={Mier, Gonzalo and Valente, João and de Bruin, Sytze},
  journal={IEEE Robotics and Automation Letters},
  title={Fields2Cover: An Open-Source Coverage Path Planning Library for Unmanned Agricultural Vehicles},
  year={2023},
  volume={8},
  number={4},
  pages={2166-2172},
  doi={10.1109/LRA.2023.3248439}
}
```

## Notes of Wisdom

Document: if using non-default params, must fully specify in action message. Uses all or none.
Document: F2C still has some things to add - please follow up there and ping me if implemented to add here. Want to hire us to do it?


Questions
	- coordinate system / frame stuff. consistency, transformation, store, datem. 
	- setting up for convertion to map coords
  - humble defaults in messages. remove params since fully setup?


https://github.com/Fields2Cover/fields2cover_ros/blob/master/src/Fields2CoverVisualizerNode.cpp
https://github.com/Fields2Cover/Fields2Cover/blob/main/include/fields2cover/utils/parser.h#L21














...


# NExt steps
  - convert GPS LLA to euclidean in request like WPF? Accept both.
  - BT node for interpolating line segment. Other BT nodes or util functions?
  - method to take swath/turn and iterate to get next and identify path of which type it is
  - TODO Update https://navigation.ros.org/tutorials/docs/adding_a_nav2_task_server.html wit hcoverage 400s
