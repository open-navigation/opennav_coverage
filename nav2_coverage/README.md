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

Document: if using non-default params, must fully specify in action message. Uses all or none.


Questions
	- coordinate system / frame stuff. consistency, transformation, store, datem. 
	- setting up for convertion to map coords

https://github.com/Fields2Cover/fields2cover_ros/blob/master/src/Fields2CoverVisualizerNode.cpp
https://github.com/Fields2Cover/Fields2Cover/blob/main/include/fields2cover/utils/parser.h#L21


// TODO visualization (outer polygon; inner polygon; path; swath vs turn coloring)




TODO Action Definition:
// TODO return of path, segments, etc how it would be desired for use
// TODO each step optional
// TODO action exception error codes
// TODO use request fields
// TODO file parsing / itnerface definition for types / cartesian. To F2C types


Request
  - If to remove headlands
  - If to make swath into a route
  - If to make route into a path

  - File to use with GPS field or GPS list
  
  - Headlands mode to use (+ width to use)
  - swath mode to use (objective, type, best angle if set, whether to allow overlap)
  - route mode to use (+spiral_n / custom order)
  - path mode to use (cont / mode)


Response:
  - nav_msgs/path (total blind just path without distinguishing swath vs turns)
  - A separate structure containing ordered swaths + array of paths connecting to them via turns in path (if enabled)
  - error code (if any)


EXCEPTIONS
  - Invalid request modes
  - Invalid request steps
  - Internal F2C
  - Invalid request field



