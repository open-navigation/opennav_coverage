# Nav2 Complete Coverage

This package contains the Complete Coverage Task server utilizing the [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) complete coverage planning system which includes a great deal of options in headland, swath, route, and final path planning. You can find more information about Fields2Cover (F2C) in its [ReadTheDocs Documentation](https://fields2cover.github.io/index.html). It can accept both GPS and Cartesian coordinates.

This capability was created by [Open Navigation LLC](https://www.opennav.org/) in partnership with [Bonsai Robotics](https://www.bonsairobotics.ai/). Bonsai Robotics funded the development of this work for their own product and has graciously allowed Open Navigation to open-source it for the community to leverage in their own systems. Please thank Bonsai Robotics for their commendable donation to the ROS community!

TODO Bonsai x Open Navigation logo

This server exposes all of the features of Fields2Cover as a Lifecycle-Component Nav2 Task Server like all others within the Nav2 Framework, so it should feel very familiar to those using Nav2 already. This capability is split into 4 packages:

- `nav2_coverage`: Contains the main Nav2 Task Server.

- `nav2_coverage_msgs`: Contains the action definition for the Coverage Navigator, Coverage Planner. Also contains several useful message types for F2C.

- `nav2_coverage_bt`: Contains the Behavior Tree Nodes and an example XML file using the Task Server to complete a simple coverage navigation task.

- `nav2_coverage_navigator`: Contains the BT Navigator plugin exposing `NavigateCompleteCoverage` action server analog to `NavigateToPose` and `NavigateThroughPoses`.

Note: `NavigateCompleteCoverage` message and its `CoverageNavigator` are subject to API changes to continue to be most useful to users! The current API is relatively basic and will be adapted as more users adopt its use and need additional fields exposed to the application layer. If you need any adjustments, feel free to ask!

Fields2Cover is a living library with new features planned to be added (for example those discussed in a [Nav2 integration ticket](https://github.com/Fields2Cover/Fields2Cover/issues/73)). As such, as new F2C capabilities are created, they will be welcome for integration here. If you see anything missing, please let us know or submit a PR to expose it!

## Interfaces

The two main interfaces are `NavigateCompleteCoverage` and `ComputeCoveragePath`. The first is the action definition to request the BT Navigator's `CoverageNavigator` plugin to navigate usign a Complete Coverage task input. The latter is an analog to the `PlannerServer`'s action definition for computing Complete Coverage paths using the `nav2_coverage` action server. See `nav2_coverage_msgs` for complete details.

### ComputeCoveragePath

This contains `generate_headland`, `generate_route`, and `generate_path` about whether the coverage task should remove a set headland, generate route to order the coverage swaths, or compute a path connecting the ordered route swaths; respectively. You can learn more about these stages in F2C's documentation - particularly from the graphic on its [index page](https://fields2cover.github.io/index.html).

Each of the stages (includeing `generate_swaths`, which is always on) has its own `_mode` message in the action containing its potential parameters to specify a mode. If not modified, it uses the parameters set in the server at launch time or after dynamic reconfiguration. See the parameter information below or the message files for complete details.

Finally, it contains the polygon information. This can be represented either as GML files, with [an example in `nav2_coverage/test`](./nav2_coverage/test/test_field.xml), or as a polygon in the message itself. If using GML files, set `goal.use_gml_file = true`. If your GML file contains multiple fields, set the ID of which to use with `goal.gml_field_id = id`, whereas the number is its ordered postion in the file.

When setting the polygon (`goal.polygons`), this is a vector of polygons. If only considering a bounding field, only populate the first field shape. If there are internal voids, use subsequent polygons to indicate them. The coordinate type has `axis1` and `axis2` instead of X and Y as the server can process both GPS and cartesian coordinates. If specifying the polygon outside of GML files, you must specify the frame of reference of the polygon using the `goal.frame_id` field. This is not used for GML files as those should contain the frame within it.

The result returns a `result.nav_path` -- which is a `nav_msgs/Path` containing the coverage path requested **only if** all `generate_` fields are `true`. This can be followed by a local trajectory planner or controller directly. This is what is used in the `nav2_coverage_bt` examples for basic coverage navigation. It also returns `result.coverage_path` which contains an ordered set of swaths and paths to connect them (if applicable settings enabled) which can be used for more task-specific navigation. For example, navigating with a tool down or enabled on swaths and raised in turns to connect to other swaths. A utility is provided in `nav2_coverage/utils.hpp` for iterating through this custom `coverage_path` for convenience.

It also returns an error code, if any error occurred and the total planning time for metrics analysis.

### NavigateCompleteCoverage

The Coverage Navigator calls the `ComputeCoveragePath` action within its BT XML. This navigator plugin exists to expose to the application layer the fields required to do Coverage-type navigation tasks rather than go-to-pose type tasks. Thus, this Action does not contain a "goal" or "start" pose, but the field filepath or polygon of interest for coverage navigation. See the section above for discussion on those types. It also contains a `goal.behavior_tree` field to specify which behavior tree to navigate using -- if not the default.

It returns the error code from the BT's error code IDs if any error occurs. Otherwise, it returns live regular feedback on the robot's current position, navigation time elapsed, number of recoveries enacted, distance remaining in the path (if `nav_path` valid), and a rough ETA.

Note that Navigator Plugins require **ROS 2 Iron or newer**. Otherwise, you may still use the Coverage Server in **Foxy or newer**, just don't compile the `nav2_coverage_navigator` package.

## Configuration

The complete set of options are exposed as both dynamic parameters and through the Action definition to be used as you prefer. Parameters are useful for consistent requests with the same configurations and actions are useful when changing parameters on a per-request basis. When any of the fields in `XYZMode.msg` messages are filled in, all must be filled in to be a complete request of a stage's mode parameters. `HeadlandMode`, `SwathMode`, `RouteMode`, and `PathMode` contain the complete parameter options for each of those stages in F2C coverage planning. The parameters given (or defaults) will be used for each stage, but any single stage can be overwritten via the Action message.

TODO config guide full

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

TODO
  - Visualize major stages for debugging
  - Modular stages retained; optional to which you'd like when
  - Each stage has factories and enums for options; can be expanded past F2C as well


Future
  - README
  - Rename packages / repo?
  - A couple of utilities for the BT nodes to iterate through the swath-turn combos (optional)



  - Use setup with BT nodes / XML / Navigator. Simulator demo altogether. Demo video (in readme).

  - Python3 API from tester.py
  - Nav2 docs to include / config guide. BT ports. Groot index
