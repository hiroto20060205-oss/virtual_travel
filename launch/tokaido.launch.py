import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    talker = launch_ros.actions.Node(
            package='virtual_travel',
            executable='gnss_simulator',
            )
    listener = launch_ros.actions.Node(
             package='virtual_travel',
             executable='tour_guide',
             output='screen'
             )

    return launch.LaunchDescription([talker, listener])
