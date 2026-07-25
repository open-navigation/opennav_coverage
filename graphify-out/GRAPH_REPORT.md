# Graph Report - .  (2026-06-29)

## Corpus Check
- Corpus is ~38,809 words - fits in a single context window. You may not need a graph.

## Summary
- 743 nodes · 1045 edges · 52 communities (51 shown, 1 thin omitted)
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 27 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Row Coverage Server|Row Coverage Server]]
- [[_COMMUNITY_Path Generator|Path Generator]]
- [[_COMMUNITY_Row Swath Generator|Row Swath Generator]]
- [[_COMMUNITY_Coverage Navigator Plugin|Coverage Navigator Plugin]]
- [[_COMMUNITY_CI Workflows & Build|CI Workflows & Build]]
- [[_COMMUNITY_Compute Coverage BT Node|Compute Coverage BT Node]]
- [[_COMMUNITY_Cancel Coverage BT Node|Cancel Coverage BT Node]]
- [[_COMMUNITY_Headland Generator|Headland Generator]]
- [[_COMMUNITY_Route Generator|Route Generator]]
- [[_COMMUNITY_Swath Generator|Swath Generator]]
- [[_COMMUNITY_Coverage Server Core|Coverage Server Core]]
- [[_COMMUNITY_Row Parsing Utils|Row Parsing Utils]]
- [[_COMMUNITY_Path Components Iterator|Path Components Iterator]]
- [[_COMMUNITY_Swath Generation Modes|Swath Generation Modes]]
- [[_COMMUNITY_Row Server Bootstrap|Row Server Bootstrap]]
- [[_COMMUNITY_Coverage Server Bootstrap|Coverage Server Bootstrap]]
- [[_COMMUNITY_Server Lifecycle Tests|Server Lifecycle Tests]]
- [[_COMMUNITY_Coverage Navigator Tester|Coverage Navigator Tester]]
- [[_COMMUNITY_Component Headers|Component Headers]]
- [[_COMMUNITY_Path Generation|Path Generation]]
- [[_COMMUNITY_Swath Test Shims|Swath Test Shims]]
- [[_COMMUNITY_Row Navigator Tester|Row Navigator Tester]]
- [[_COMMUNITY_Coverage Demo Tester|Coverage Demo Tester]]
- [[_COMMUNITY_Coverage Tester Node|Coverage Tester Node]]
- [[_COMMUNITY_Test Fixtures|Test Fixtures]]
- [[_COMMUNITY_Route Generation|Route Generation]]
- [[_COMMUNITY_Compute Path Action Node|Compute Path Action Node]]
- [[_COMMUNITY_Navigator Tests|Navigator Tests]]
- [[_COMMUNITY_Headland Generation|Headland Generation]]
- [[_COMMUNITY_Demo Scripts|Demo Scripts]]
- [[_COMMUNITY_Visualizer Publishers|Visualizer Publishers]]
- [[_COMMUNITY_Geometry Conversion|Geometry Conversion]]
- [[_COMMUNITY_Coverage Demo Image|Coverage Demo Image]]
- [[_COMMUNITY_Row Coverage Demo Image|Row Coverage Demo Image]]
- [[_COMMUNITY_Path Message Conversion|Path Message Conversion]]
- [[_COMMUNITY_Row Parser Tests|Row Parser Tests]]
- [[_COMMUNITY_Robot Param Tests|Robot Param Tests]]
- [[_COMMUNITY_Cancel Path Action Node|Cancel Path Action Node]]
- [[_COMMUNITY_Robot Params|Robot Params]]
- [[_COMMUNITY_Coverage Exception|Coverage Exception]]
- [[_COMMUNITY_Visualizer Tests|Visualizer Tests]]
- [[_COMMUNITY_String Conversion Utils|String Conversion Utils]]

## God Nodes (most connected - your core abstractions)
1. `CoverageServer` - 32 edges
2. `RowCoverageServer` - 31 edges
3. `SwathGenerator` - 30 edges
4. `PathGenerator` - 28 edges
5. `RouteGenerator` - 21 edges
6. `CoverageNavigator` - 21 edges
7. `CancelCoverageActionTestFixture` - 19 edges
8. `HeadlandGenerator` - 17 edges
9. `ComputeCoveragePathActionTestFixture` - 17 edges
10. `RowSwathGenerator` - 17 edges

## Surprising Connections (you probably didn't know these)
- `RowCoverageServer Component` --semantically_similar_to--> `CoverageServer Component`  [INFERRED] [semantically similar]
  opennav_row_coverage/CMakeLists.txt → opennav_coverage/CMakeLists.txt
- `RowCoverageServer` --references--> `PathGenerator`  [EXTRACTED]
  opennav_row_coverage/include/opennav_row_coverage/row_coverage_server.hpp → opennav_coverage/include/opennav_coverage/path_generator.hpp
- `RowCoverageServer` --references--> `RouteGenerator`  [EXTRACTED]
  opennav_row_coverage/include/opennav_row_coverage/row_coverage_server.hpp → opennav_coverage/include/opennav_coverage/route_generator.hpp
- `generateHeadlands()` --references--> `field_`  [EXTRACTED]
  opennav_coverage/src/headland_generator.cpp → opennav_row_coverage/test/test_swath_generator.cpp
- `Lint Workflow (ament_lint)` --references--> `OpenNav Coverage (Nav2 Complete Coverage)`  [EXTRACTED]
  .github/workflows/lint.yml → README.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Coverage Navigation Pipeline (Navigator -> BT -> Server -> Msgs)** — readme_coverage_navigator_plugin, opennav_coverage_bt_cmakelists_compute_node, opennav_coverage_cmakelists_coverage_server, opennav_coverage_msgs_cmakelists_opennav_coverage_msgs [INFERRED 0.80]
- **CI Source-Build Workarounds for Lyrical** — github_workflows_test_test, github_workflows_test_colcon_cache, github_workflows_test_nav2_werror_patch, github_workflows_test_drop_shadowing_packages [EXTRACTED 0.85]
- **F2C Coverage Planning Stages** — readme_coverage_planning_stages, readme_compute_coverage_path, readme_swath_generation_modes [INFERRED 0.75]

## Communities (52 total, 1 thin omitted)

### Community 0 - "Row Coverage Server"
Cohesion: 0.05
Nodes (35): LifecycleNode, mutex, RobotParams, SharedPtr, unique_ptr, Visualizer, RowCoverageServer, action_server_ (+27 more)

### Community 1 - "Path Generator"
Cohesion: 0.06
Nodes (34): Logger, NodeT, PathContinuityType, PathType, RobotParams, TurningBasePtr, unique_ptr, PathGenerator (+26 more)

### Community 2 - "Row Swath Generator"
Cohesion: 0.06
Nodes (32): Logger, NodeT, RowSwathType, RowSwathGenerator, adjustRowOrientations, default_offset_, default_type_, generateSwaths (+24 more)

### Community 3 - "Coverage Navigator Plugin"
Cohesion: 0.08
Nodes (29): BehaviorTreeNavigator<
    opennav_coverage_msgs::action::NavigateCompleteCoverage>, BtStatus, ConstSharedPtr, CoverageNavigator, cleanup, configure, field_blackboard_id_, getDefaultBTFilepath (+21 more)

### Community 4 - "CI Workflows & Build"
Cohesion: 0.09
Nodes (35): Lint Workflow (ament_lint), Colcon Cache Strategy, Drop Shadowing apt Packages, nav2_common -Werror Patch, Test Workflow (build_and_test), opennav_cancel_complete_coverage_action_bt_node, opennav_compute_complete_coverage_action_bt_node, opennav_coverage_bt CMake (BT Nodes build) (+27 more)

### Community 5 - "Compute Coverage BT Node"
Cohesion: 0.07
Nodes (23): NodeStatus, BT_REGISTER_NODES(), ComputeCoveragePathAction(), factory, NodeConfiguration, string, on_aborted(), on_cancelled() (+15 more)

### Community 6 - "Cancel Coverage BT Node"
Cohesion: 0.08
Nodes (22): ComputeCoveragePath, BT_REGISTER_NODES(), CoverageCancel(), factory, NodeConfiguration, string, CancelCoverageActionTestFixture, action_server_ (+14 more)

### Community 7 - "Headland Generator"
Cohesion: 0.09
Nodes (24): HeadlandTests, HeadlandGenerator, createGenerator, default_generator_, default_headland_width_, default_type_, generateHeadlands, logger_ (+16 more)

### Community 8 - "Route Generator"
Cohesion: 0.09
Nodes (25): Logger, NodeT, RouteGeneratorPtr, RouteType, vector, RouteGenerator, createGenerator, default_custom_order_ (+17 more)

### Community 9 - "Swath Generator"
Cohesion: 0.07
Nodes (25): BruteForce, Logger, NodeT, RobotParams, SwathAngleType, SwathObjectivePtr, SwathType, unique_ptr (+17 more)

### Community 10 - "Coverage Server Core"
Cohesion: 0.08
Nodes (25): CoverageServer, action_server_, cartesian_frame_, computeCoveragePath, dyn_params_handler_, dynamic_params_lock_, dynamicParametersCallback, getPreemptedGoalIfRequested (+17 more)

### Community 11 - "Row Parsing Utils"
Cohesion: 0.19
Nodes (22): LineString, F2CField, string, parseRows(), removeRowsRefPoint(), transformRowsWithRef(), adjustRowOrientations(), calculateWidth() (+14 more)

### Community 12 - "Path Components Iterator"
Cohesion: 0.09
Nodes (18): PathComponentsIterator, idx_, max_idx_, UtilsTests, RosLockGuard, TEST(), pair, PathComponents (+10 more)

### Community 13 - "Swath Generation Modes"
Cohesion: 0.16
Nodes (19): string, SwathAngleType, SwathObjectivePtr, Swaths, SwathType, createObjective(), generateSwaths(), setSwathAngleMode() (+11 more)

### Community 14 - "Row Server Bootstrap"
Cohesion: 0.17
Nodes (16): computeCoveragePath(), CallbackReturn, NodeOptions, Parameter, SetParametersResult, State, vector, dynamicParametersCallback() (+8 more)

### Community 15 - "Coverage Server Bootstrap"
Cohesion: 0.19
Nodes (16): computeCoveragePath(), CoverageServer(), CallbackReturn, NodeOptions, Parameter, SetParametersResult, State, vector (+8 more)

### Community 16 - "Server Lifecycle Tests"
Cohesion: 0.18
Nodes (9): LifecycleTest, ServerTest, State, testDynamicParams, testServerTransactions, testUtils, RosLockGuard, ServerShim (+1 more)

### Community 17 - "Coverage Navigator Tester"
Cohesion: 0.19
Nodes (6): CoverageNavigatorTester, main(), Get the pending action feedback message., Get the pending action result message., Send a `NavToPose` action request., Check if the task request of any type is complete yet.

### Community 18 - "Component Headers"
Cohesion: 0.20
Nodes (5): string, toUpper(), activate(), NodeT, RosLockGuard

### Community 19 - "Path Generation"
Cohesion: 0.30
Nodes (14): Path, PathContinuityType, PathType, string, Swaths, TurningBasePtr, createCurve(), generatePath() (+6 more)

### Community 20 - "Swath Test Shims"
Cohesion: 0.22
Nodes (11): NodeT, RobotParams, string, SwathAngleType, SwathObjectivePtr, SwathType, SwathShim, TEST() (+3 more)

### Community 21 - "Row Navigator Tester"
Cohesion: 0.20
Nodes (6): main(), Get the pending action feedback message., Get the pending action result message., Send a `NavToPose` action request., Check if the task request of any type is complete yet., RowCoverageNavigatorTester

### Community 22 - "Coverage Demo Tester"
Cohesion: 0.21
Nodes (6): CoverageTester, main(), Send a `ComputeCoveragePath` action request., Get the pending action result message., Activate coverage server., TaskResult

### Community 23 - "Coverage Tester Node"
Cohesion: 0.24
Nodes (6): Node, CoverageTester, main(), Send a `ComputeCoveragePath` action request., Get the pending action result message., Activate coverage server.

### Community 24 - "Test Fixtures"
Cohesion: 0.24
Nodes (3): RosLockGuard, RosLockGuard, RosLockGuard

### Community 25 - "Route Generation"
Cohesion: 0.38
Nodes (10): RouteGeneratorPtr, RouteType, string, Swaths, createGenerator(), generateRoute(), setMode(), toString() (+2 more)

### Community 26 - "Compute Path Action Node"
Cohesion: 0.20
Nodes (8): BtActionNode<
    opennav_coverage_msgs::action::ComputeCoveragePath>, ComputeCoveragePathAction, halt, on_aborted, on_cancelled, on_success, on_tick, PortsList

### Community 27 - "Navigator Tests"
Cohesion: 0.24
Nodes (8): CoverageNavigatorTests, string, vector, getLibs(), RosLockGuard, TEST(), TestBasicFunctionality, TestBasicServer

### Community 28 - "Headland Generation"
Cohesion: 0.42
Nodes (9): HeadlandMode, HeadlandGeneratorPtr, HeadlandType, string, createGenerator(), generateHeadlands(), setMode(), toString() (+1 more)

### Community 29 - "Demo Scripts"
Cohesion: 0.33
Nodes (5): Enum, TaskResult, TaskResult, TaskResult, Time

### Community 30 - "Visualizer Publishers"
Cohesion: 0.22
Nodes (8): SharedPtr, Visualizer, deactivate, headlands_pub_, nav_plan_pub_, planning_field_pub_, swaths_pub_, visualize

### Community 31 - "Geometry Conversion"
Cohesion: 0.29
Nodes (7): getFieldFromGoal(), Point, pointToPoint32(), toMsg(), PathState, Point32, PoseStamped

### Community 32 - "Coverage Demo Image"
Cohesion: 0.43
Nodes (7): Coverage Planning Demo Visualization, Boustrophedon Coverage Pattern, Parallel Coverage Swaths, Field Boundary Polygon, Headland U-Turn Connections, Robot Pose Marker, RViz 3D Visualization Scene

### Community 33 - "Row Coverage Demo Image"
Cohesion: 0.48
Nodes (7): Row Coverage Demo Visualization, Coverage Path, Field Boundary Polygon, Robot Pose / TF Frame, RViz Visualization, Coverage Swath (Row), U-Turn Headland Maneuver

### Community 34 - "Path Message Conversion"
Cohesion: 0.52
Nodes (7): F2CField, Header, Path, Swaths, toCartesianNavPathMsg(), toCoveragePathMsg(), toNavPathMsg()

### Community 35 - "Row Parser Tests"
Cohesion: 0.29
Nodes (5): UtilsTests, RosLockGuard, TEST(), TestRowParserComplex, TestRowParserSimple

### Community 36 - "Robot Param Tests"
Cohesion: 0.33
Nodes (4): RosLockGuard, TEST(), RobotTests, Testrobot

### Community 37 - "Cancel Path Action Node"
Cohesion: 0.40
Nodes (3): BtCancelActionNode<
    opennav_coverage_msgs::action::ComputeCoveragePath>, CoverageCancel, PortsList

### Community 39 - "Robot Params"
Cohesion: 0.40
Nodes (3): NodeT, RobotParams, robot_

### Community 40 - "Coverage Exception"
Cohesion: 0.50
Nodes (3): CoverageException, string, runtime_error

### Community 41 - "Visualizer Tests"
Cohesion: 0.50
Nodes (4): TEST(), Testlifecycle, TestVizPubs, VizTests

## Knowledge Gaps
- **154 isolated node(s):** `computeCoveragePath`, `validateGoal`, `getPreemptedGoalIfRequested`, `on_configure`, `on_activate` (+149 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `SwathTestFixture` connect `Row Swath Generator` to `Row Parsing Utils`, `Swath Generation Modes`?**
  _High betweenness centrality (0.165) - this node is a cross-community bridge._
- **Why does `CoverageServer` connect `Coverage Server Core` to `Row Coverage Server`, `Path Generator`, `Coverage Server Main`, `Headland Generator`, `Route Generator`, `Swath Generator`, `Server Lifecycle Tests`?**
  _High betweenness centrality (0.107) - this node is a cross-community bridge._
- **Why does `RowCoverageServer` connect `Row Coverage Server` to `Route Generator`, `Path Generator`, `Row Swath Generator`?**
  _High betweenness centrality (0.106) - this node is a cross-community bridge._
- **What connects `computeCoveragePath`, `validateGoal`, `getPreemptedGoalIfRequested` to the rest of the system?**
  _173 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Row Coverage Server` be split into smaller, more focused modules?**
  _Cohesion score 0.05454545454545454 - nodes in this community are weakly interconnected._
- **Should `Path Generator` be split into smaller, more focused modules?**
  _Cohesion score 0.06282051282051282 - nodes in this community are weakly interconnected._
- **Should `Row Swath Generator` be split into smaller, more focused modules?**
  _Cohesion score 0.05714285714285714 - nodes in this community are weakly interconnected._