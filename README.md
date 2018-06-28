2017 06 07

RViz Cloud Annotation Tool
--------------------------

This is an annotation tool for 3D point clouds, based on the ROS (Robot Operating System) visualizer RViz.

The annotation tool lets the user partition a point cloud into parts. A text description may be assigned to each part.

In addition to rectangular and polyline selection, the tool provides assisted annotation based on Control Points. This requires normals and colors.

The standalone package `rviz_3d_view_controller` contains the Orbit3D View Controller plugin. The interface is similar to the default Orbit plugin, but it allows true 6DOF movement in 3D space.

### Related publication

- R. Monica, J. Aleotti, M. Zillich, M. Vincze, Multi-Label Point Cloud Annotation by Selection of Sparse Control Points, International Conference on 3D Vision (3DV), Qingdao, China, October 10th-12th 2017

### Installation

System requirements:

- ROS (Kinetic, Lunar, Melodic)
- Point Cloud Library (PCL) 1.7.2 or above
- GPU graphics acceleration is recommended

This repository contains ROS packages for the catkin build system. Place them into your workspace and recompile.

**Notice:**  
Catkin compiles the workspace in debug mode by default. Try enabling optimization. Example commands:

`catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo`

`catkin build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`

**Note:**  
The use of `catkin_make` may interfere with other packages. Use `catkin build` (of the _catkin tools_ suite) whenever possible.

### Usage

A sample launch file is provided:

`roslaunch rviz_cloud_annotation example.launch`

The launch file starts RViz with the required plugin. A background node is also started. It uses the Interactive Marker interface to display the point cloud and intercept the click events.

By clicking on the point cloud, the user places control points on it.
A control point expands its label onto the neighboring points, until a maximum distance is reached or until another label stops it, coming from another control point.
The algorithm operates on a neighborhood graph.

The most effective way to use the tool is by adding control points on the background and on the objects. The control points compete for control of the nearby points. Add control points until the boundary between the labels is satisfying.

The weight system allows for the creation of "weaker" control points, which can control less points around them. A control point with weight 0 can control only the point where it's on.

#### Panel plugin

- Menu
    - File
        - New: clears current annotation.
        - Save: saves current annotation (file name is set by parameter, see below).
        - Restore: reload annotation from file.
    - Edit
        - Undo: cancels last operation.
        - Redo: repeats last cancelled operation.
    - View
        - Increase point size
        - Decrease point size
        - Reset point size: to default value.
    - Label
        - Clear current: clears selected label.
        - Next: selects next label.
        - Previous: selects previous label.
        - Next page: shows next page for label selection.
        - Prev page: shows previous page for label selection.
        - Go to:
            - First: label number 1.
            - First empty: first unused label.
            - Next empty: next unused label.
            - Last empty: last unused label.
        - Weight
            - Increase: increase weight slider.
            - Decrease: decrease weight slider.
            - Minimum: sets weight slider to minimum (0).
            - Maximum: sets weight slider to maximum.
- Action selection
    - None: allows navigation.
    - Set: set label to selected points.
    - Del: remove label from selected points.
    - Pick: set current label to the clicked point label.
    **Note** You can press the right mouse button or the _Ins_ key to quickly switch between the current action and None.
- Tool selection
    - Point: set single (control) point.
    - Shallow rect: rectangle selection, visible points only.
    - Deep rect: rectangle selection, also hidden points.
    - Polyline: selection inside arbitrary polygon, visible points only.
        Left click to add a point, right click to add a point and terminate.
- Show: toggle visualization of
    - Cloud: the point cloud, unlabeled points become black.
    - Labels: labels.
    - Seeds: the control points.
- Label selection: select the current label by clicking. More labels may be requested by switching page.
- Label description: write a description for the current label in the text box, then click _Set name_ or press _Enter_.

#### Background node parameters

##### Save files  
- `cloud_filename` (string): the name of the point cloud to be annotated, in PCD format.
- `annotation_read_filename` (string): the annotation will be read from this file at startup, if existing.
- `annotation_write_filename` (string): the annotation will be saved to this file.
- `annotation_cloud` (string): an annotated cloud in PCD format, with `color` and `label` fields will also be saved into this file. Useful to be read by other software.
- `label_names_filename` (string): label names are saved into this file, in the format: `label_id: label_name`.
- `autosave_time` (double): auto-save period, 0 to disable.
- `autosave_append_timestamp` (bool): appends timestamp to autosave file name.

##### Control point algorithm configuration

**Warning**: When loading an annotation file, this configuration must match the original configuration.

- `neigh_search_type` (int): method to produce the neighborhood graph.
    - 0: radius search, `neigh_search_params` (string) is the distance
    - 1: max K nearest neighbors, `neigh_search_params` (string) is K
    - 2: min K nearest neighbors, `neigh_search_params` (string) is K
- `position_importance` (float): importance of euclidean distance in computations.
- `max_label_distance`: divided by `position_importance`, gives the maximum distance that may be affected by a single Control Point.
- `color_importance` (float): importance of color in computations, set to 0 if no color.
- `normal_importance` (float): importance of normals in computations.
- `weight_steps` (int): number of steps of the weight slider. **Warning:** increases memory usage!

##### Visualization  
- `control_point_weight_scale_fraction` (double): fraction of the control point size which is affected by weight.
- `label_size` (double): label square size in RViz.
- `point_size` (double): initial point size in RViz.
- `rviz_cloud_annotation/point_change_size_multiplier` (double): amount of point size changed when requested by menu option.
- `show_labels_back` (bool): if true, labels are shown on the back of cloud points too.
- `control_point_visual` (string):
    - `sphere`: control points are spheres.
    - `line`: control points are thick lines.
    - `three_spheres`: control points are three sphere on a line.
- `show_zero_weight_control_points`: if false, control points with zero weight (and those generated by rectangular selection) will not be displayed.

#### Panel plugin parameters

- `rviz_cloud_annotation/color_columns_per_page` (int): labels per row in the panel plugin.
- `rviz_cloud_annotation/color_rows_per_page` (int): labels per column in the panel plugin.

2017 10 11
