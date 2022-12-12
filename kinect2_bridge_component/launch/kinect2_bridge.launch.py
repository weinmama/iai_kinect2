from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

# def generate_launch_description():
#     composable_nodes = [
#         package='image_proc',
#         plugin=''
#     ]

#
# <node pkg="kinect2_bridge" type="kinect2_bridge" name="$(arg base_name)_bridge" machine="$(arg machine)"
# respawn="$(arg respawn)" output="$(arg output)" unless="$(arg use_nodelet)">
# <param name="base_name"         type="str"    value="$(arg base_name)"/>
# <param name="sensor"            type="str"    value="$(arg sensor)"/>
# <param name="publish_tf"        type="bool"   value="$(arg publish_tf)"/>
# <param name="base_name_tf"      type="str"    value="$(arg base_name_tf)"/>
# <param name="fps_limit"         type="double" value="$(arg fps_limit)"/>
# <param name="calib_path"        type="str"    value="$(arg calib_path)"/>
# <param name="use_png"           type="bool"   value="$(arg use_png)"/>
# <param name="jpeg_quality"      type="int"    value="$(arg jpeg_quality)"/>
# <param name="png_level"         type="int"    value="$(arg png_level)"/>
# <param name="depth_method"      type="str"    value="$(arg depth_method)"/>
# <param name="depth_device"      type="int"    value="$(arg depth_device)"/>
# <param name="reg_method"        type="str"    value="$(arg reg_method)"/>
# <param name="reg_device"        type="int"    value="$(arg reg_device)"/>
# <param name="max_depth"         type="double" value="$(arg max_depth)"/>
# <param name="min_depth"         type="double" value="$(arg min_depth)"/>
# <param name="queue_size"        type="int"    value="$(arg queue_size)"/>
# <param name="bilateral_filter"  type="bool"   value="$(arg bilateral_filter)"/>
# <param name="edge_aware_filter" type="bool"   value="$(arg edge_aware_filter)"/>
# <param name="worker_threads"    type="int"    value="$(arg worker_threads)"/>
# </node>

#     <!--   qhd point cloud (960 x 540) -->
#     <node pkg="kinect2_bridge_pointcloud" type="kinect2_bridge" name="$(arg base_name)_points_xyzrgb_qhd" machine="$(arg machine)"
#     args="load depth_image_proc/point_cloud_xyzrgb $(arg nodelet_manager)" respawn="$(arg respawn)">
# <remap from="rgb/camera_info"             to="$(arg base_name)/qhd/camera_info"/>
# <remap from="rgb/image_rect_color"        to="$(arg base_name)/qhd/image_color_rect"/>
# <remap from="depth_registered/image_rect" to="$(arg base_name)/qhd/image_depth_rect"/>
# <remap from="depth_registered/points"     to="$(arg base_name)/qhd/points"/>
# <param name="queue_size" type="int" value="$(arg queue_size)"/>
# </node>
#
# <!--  &lt;!&ndash; h