from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # 라이프사이클 노드들
    control_node = LifecycleNode(
        package='night_patrol',
        executable='control_node',
        name='control_node_lifecycle',
        namespace='',
        output='screen',
        prefix='python3 '
    )
    patrol_nav2 = LifecycleNode(
        package='night_patrol',
        executable='patrol_nav2',
        name='patrol_nav2_lifecycle',
        namespace='',
        output='screen',
        prefix='python3 '
    )
    patrol_nav3 = LifecycleNode(
        package='night_patrol',
        executable='patrol_nav3',
        name='patrol_nav3_lifecycle',
        namespace='',
        output='screen',
        prefix='python3 '
    )

    # 런치 후 configure 트랜지션 emit (inactive 대기)
    configure_control = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda entity: entity == control_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    configure_patrol2 = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda entity: entity == patrol_nav2,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    configure_patrol3 = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda entity: entity == patrol_nav3,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        control_node,
        patrol_nav2,
        patrol_nav3,
        configure_control,
        configure_patrol2,
        configure_patrol3,
    ])
