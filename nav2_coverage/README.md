# Nav2 Complete Coverage Server

This package contains the Complete Coverage Task server utilizing the [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) complete coverage planning system which includes a great deal of options in headland, swath, route, and final path planning. You can find 

TODO bonsai logo, big ack, etc. more details on library

## Interfaces


## Configuration

Document: if using non-default params, must fully specify in action message. Uses all or none.
Document: F2C still has some things to add - please follow up there and ping me if implemented to add here.

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

Walk through
  - Lifecycle-Component-Action Task Server like you expect in Nav2
  - Fully parameterized with dynamic parameters to easily test / tune
  - Expose all relevent options in the Action for per-request modifications from param defaults
  - Visualize major stages for debugging
  - Modular stages retained; optional to which you'd like when
  - Each stage has factories and enums for options; can be expanded past F2C as well
  - Use GPS, Cartesian; files or direct coordinates
  - Error codes for contextual failures to know when failures what to do about it
  - Return: PathComponents, NavPath, error code, compute time for metrics
  - BT nodes + XML using the nav path from coverage + Navigator type for semantic information of request + demo sim use

  - Tester to demo
    - Basic call
    - Adjust to cartesian
    - Different options
    - RQT

Future
  - Use setup with BT nodes / XML / Navigator. Simulator demo altogether. Demo video.
  - README


  - Python3 API from tester.py
  - Rename packages / repo?
  - A couple of utilities for the BT nodes to iterate through the swath-turn combos (optional)



Navigators require Iron+
Options: (A) upgrade, (B) use Navigate To Pose and hardcode the field file path and ignore the pose action request, (C) call the server manually from your application

