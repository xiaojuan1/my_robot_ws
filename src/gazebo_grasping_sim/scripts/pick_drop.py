import time
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

def plan_and_execute(robot, planning_component, logger, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=3.0):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        trajectory = plan_result.trajectory
        robot.execute(trajectory, controllers=[])
    else:
        logger.error("Planning failed")
    time.sleep(sleep_time)

def main():
    rclpy.init()
    logger = get_logger("moveit_py_pose_goal")

    # instantiate MoveItPy instance and get planning component
    robot = MoveItPy(node_name="moveit_py")
    arm_group = robot.get_planning_component("arm")
    hand_group = robot.get_planning_component("hand")
    logger.info("MoveItPy instance created")
    
    time.sleep(10)

    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state("stand")
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=5.0)

    robot_model = robot.get_robot_model()
    robot_state = RobotState(robot_model)
    hand_group.set_start_state_to_current_state()
    robot_state.set_joint_group_positions('hand', [0.045, 0.045])
    hand_group.set_goal_state(robot_state=robot_state)

    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)
    # set plan start state using predefined state
    arm_group.set_start_state_to_current_state()
    
    
    # set plan start state using predefined state
    hand_group.set_start_state_to_current_state()
    # set pose goal using predefined state
    hand_group.set_goal_state("open")
    # plan to goal
    plan_and_execute(robot, hand_group, logger, sleep_time=3.0)

    arm_group.set_start_state_to_current_state()
    arm_group.set_goal_state("stand")
    # plan to goal
    plan_and_execute(robot, arm_group, logger, sleep_time=3.0)

    logger.info("运动完成")
    rclpy.shutdown()

if __name__ == "__main__":
    main()