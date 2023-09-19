# Nav2 Complete Coverage Server

This package contains the Complete Coverage Task server utilizing the [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) complete coverage planning system which includes a great deal of options in headland, swath, route, and final path planning. You can find 

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


Questions
	- Make sure to validate that all of the objective functions, options are exposed for use
	- coordinate system / frame stuff. consistency, transformation, store, datem. 
	- setting up for convertion to map coords

https://github.com/Fields2Cover/Fields2Cover/issues/73
https://github.com/Fields2Cover/fields2cover_ros/blob/master/src/Fields2CoverVisualizerNode.cpp
https://github.com/open-navigation/bonsai_nav2_coverage_server/issues/1


// TODO(sm) headlands dist/spiral_n/order/swath angle part of action if provided
// TODO(sm) missing robot param get
// TODO(sm) missing robot opt parms get
// TODO(sm) overlap what?
// TODO(sm) convert to F2C types (and/or replace to store that way?). method to get as API woul expect
    // --> or this wraps it entirely! does the API calls eithin the Mode.
    // could also then have the disambiguation of action vs params happen here too

