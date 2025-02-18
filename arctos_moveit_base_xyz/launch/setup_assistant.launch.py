from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("arctos", package_name="arctos_moveit_base_xyz").to_moveit_configs()
    return generate_setup_assistant_launch(moveit_config)
