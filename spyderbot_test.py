from isaacsim import SimulationApp

# Initialize the simulation application. Set 'headless' to False to see the GUI.
simulation_app = SimulationApp({"headless": False})

import sys
import carb
import numpy as np
import time
import os
import ast # Used for safely evaluating Python literal structures from the config file
import re  # Used for robust parsing of the command string

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
import omni.usd
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from pxr import Sdf, UsdLux

# --- Global Configuration ---
CONFIG_FILE_PATH = "C:\\isaacsim\\standalone_examples\\tutorials\\config.txt"
last_modified_time = 0 # Stores the last modification timestamp of the config file
STEPS_PER_SEC = 100  # Defines the number of simulation steps per second for smooth movement

# GroundPlane(prim_path="/World/GroundPlane", z_position=0)

# Add Light Source
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)


# --- Robot Loading ---
spiderbot_usd_path = "D:/spyderbot/spdr.usd" # Path to your spiderbot USD model
add_reference_to_stage(usd_path=spiderbot_usd_path, prim_path="/World/SpiderBot")
spiderbot = Articulation(prim_paths_expr="/World/SpiderBot", name="my_spiderbot")
# Set the initial position of the spiderbot in the world
spiderbot.set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / get_stage_units())

my_world = World(stage_units_in_meters=1.0)
spiderbot.initialize()

my_world.scene.add_default_ground_plane(
            z_position=0,
            name="default_ground_plane",
            prim_path="/World/defaultGroundPlane",
            static_friction=0.2,
            dynamic_friction=0.2,
            restitution=0.01,
        )

# --- Joint Configuration ---
# Defines the mapping of leg names to their corresponding joint indices
legs = {
    "leg1": [0, 4, 8],   # front-left leg's joint indices
    "leg2": [1, 5, 9],   # front-right leg's joint indices
    "leg3": [2, 6, 10],  # rear-left leg's joint indices
    "leg4": [3, 7, 11],  # rear-right leg's joint indices
}
leg_order = ["leg1", "leg3", "leg2", "leg4"] # Defines an example order for leg movements

neutral_pose = [0.0] * 12 # A list of 12 zeros, representing the 'relative' neutral position

# --- Helper Functions ---

def offset_joint_positions(joint_positions):
    """
    Applies a fixed offset to a list of relative joint positions to reach a desired
    'neutral' absolute pose. This is crucial because the spiderbot's physical
    neutral might not be at 0.0 radians for all joints.
    
    Args:
        joint_positions (list): A list of 12 joint positions (in radians) relative
                                to the desired neutral offset.
    Returns:
        list: A list of 12 absolute joint positions (in radians) after applying the offset.
    """
    # These are the absolute radian values that define the spiderbot's physical neutral pose.
    neutral_offset = [np.radians(45.0), np.radians(-45.0), np.radians(45.0), np.radians(-45.0),
                      np.radians(-30.0), np.radians(-30.0), np.radians(-30.0), np.radians(-30.0),
                      np.radians(30.0), np.radians(30.0), np.radians(30.0), np.radians(30.0)]
    return (np.array(neutral_offset) + np.array(joint_positions)).tolist()

# Initialize joints to the defined neutral pose
spiderbot.set_joint_positions(offset_joint_positions(neutral_pose))

def get_current_joint_positions_with_offsets():
    """
    Retrieves the current absolute joint positions directly from the simulator.
    These positions are already in absolute radians and reflect the robot's
    actual state, including any applied offsets.
    
    Returns:
        list: A list of 12 current absolute joint positions in radians.
    """
    return spiderbot.get_joint_positions()[0].tolist()

def drive_single_joint(joint_idx, target_angle_degrees, speed):
    """
    Drives a single joint to a target angle (relative to its neutral offset)
    with a specified speed. The movement is smoothly interpolated.
    
    Args:
        joint_idx (int): The index of the joint to control (0-11).
        target_angle_degrees (float): The target angle in degrees, relative to
                                      the joint's neutral offset.
        speed (float): A factor determining the duration of the movement. Higher
                       values mean faster movement (shorter duration).
    """
    current_pose = get_current_joint_positions_with_offsets() # Get current actual positions in radians

    # Calculate the desired absolute target in radians for the specific joint.
    # It's the relative target angle (from user input) added to the joint's base neutral offset.
    neutral_offset_for_joint = offset_joint_positions([0.0] * 12)[joint_idx]
    target_rad_for_joint = np.radians(target_angle_degrees) + neutral_offset_for_joint

    start_rad = current_pose[joint_idx] # Starting position for interpolation
    
    steps = int(speed * STEPS_PER_SEC) # Calculate total steps for the movement
    if steps <= 0: # Ensure at least one step
        steps = 1

    for i in range(steps):
        alpha = i / steps # Interpolation factor (0 to 1)
        # Linearly interpolate between the start and target radians
        interp_rad = (1 - alpha) * start_rad + alpha * target_rad_for_joint
        
        pose = current_pose.copy() # Create a copy of the current full pose
        pose[joint_idx] = interp_rad # Update only the target joint's position
        
        spiderbot.set_joint_positions([pose]) # Apply the updated pose to the robot
        my_world.step(render=True) # Advance the simulation and render
        time.sleep(1 / STEPS_PER_SEC) # Pause for a short duration to control simulation speed

def drive_multiple_joints(joint_indices, target_angles_degrees, speed):
    """
    Drives multiple joints simultaneously to their respective target angles
    (relative to their neutral offsets) with a specified speed.
    
    Args:
        joint_indices (list): A list of integer indices for the joints to control.
        target_angles_degrees (list): A list of target angles in degrees, corresponding
                                      to `joint_indices`, relative to each joint's
                                      neutral offset.
        speed (float): A factor determining the duration of the movement. Higher
                       values mean faster movement (shorter duration).
    """
    current_pose = get_current_joint_positions_with_offsets() # Get current actual positions in radians

    # Initialize the target pose with the current absolute positions of all joints.
    # This ensures that joints not explicitly moved will stay in their current place.
    target_rad_with_offset = current_pose.copy() 

    # Get the base neutral offsets for all joints.
    neutral_offsets_base = offset_joint_positions([0.0] * 12) 
    
    # Calculate the absolute target positions for the specified joints.
    for idx, angle in zip(joint_indices, target_angles_degrees):
        # The target for this specific joint is its neutral offset PLUS the relative target_angle_degrees
        target_rad_with_offset[idx] = np.radians(angle) + neutral_offsets_base[idx]

    steps = int(speed * STEPS_PER_SEC) # Calculate total steps for the movement
    if steps <= 0: # Ensure at least one step
        steps = 1

    for i in range(steps):
        alpha = i / steps # Interpolation factor (0 to 1)
        # Interpolate each joint. Joints not in `joint_indices` will interpolate
        # from their current position to their current position (i.e., stay still).
        interp_pose = [(1 - alpha) * current_pose[j] + alpha * target_rad_with_offset[j] for j in range(12)]
        
        spiderbot.set_joint_positions([interp_pose]) # Apply the interpolated pose to the robot
        my_world.step(render=True) # Advance the simulation and render
        time.sleep(1 / STEPS_PER_SEC) # Pause for a short duration

def load_and_execute_commands(file_path):
    """
    Loads commands from a specified text file, parses them, and executes them.
    The function also monitors the file for changes and reloads if detected.
    
    Args:
        file_path (str): The path to the configuration file.
    """
    global last_modified_time
    try:
        # If the config file doesn't exist, create a template with instructions.
        if not os.path.exists(file_path):
            print(f"Config file not found: {file_path}. Creating a template...")
            with open(file_path, 'w') as f:
                f.write("# Add your spiderbot movement commands here, one per line.\n")
                f.write("# Example: drive_multiple_joints([0, 1, 2, 3], [-45, 45, -45, 45], 1.0)\n")
                f.write("# Each command will be executed sequentially when the file is saved.\n")
                f.write("# After all commands are executed, the robot will hold its final pose.\n")
                f.write("# To re-run, save the file again (even with no changes).\n")
            last_modified_time = os.path.getmtime(file_path) # Update timestamp after creation
            return # Exit after creating, wait for user to add commands

        # Check if the file has been modified since the last check.
        current_modified_time = os.path.getmtime(file_path)
        if current_modified_time <= last_modified_time:
            return # No changes, do nothing

        print(f"Detected changes in {file_path}. Reloading and executing commands...")
        last_modified_time = current_modified_time # Update the last modified time

        with open(file_path, 'r') as f:
            commands = f.readlines()

        # Reset to neutral pose before executing a new sequence of commands.
        # This provides a consistent starting point for each file reload.
        print("Resetting spiderbot to neutral pose before executing new commands.")
        spiderbot.set_joint_positions(offset_joint_positions([0.0] * 12))
        my_world.step(render=True)
        time.sleep(0.5) # Give the robot a moment to settle into the neutral pose

        # Process each command line from the file
        for line_num, command_line in enumerate(commands):
            command_line = command_line.strip()
            # Skip empty lines and lines starting with '#' (comments)
            if not command_line or command_line.startswith('#'):
                continue

            try:
                # Check if the command starts with "drive_multiple_joints("
                if command_line.startswith("drive_multiple_joints("):
                    # Use regex to extract the arguments string inside the parentheses
                    match = re.match(r"drive_multiple_joints\((.*)\)", command_line)
                    if not match:
                        raise ValueError("Invalid format for drive_multiple_joints command.")
                    
                    args_str = match.group(1) # e.g., "[0, 1, 2, 3], [-45, 45, -45, 45], 1.0"

                    # Parse the first list (joint_indices)
                    first_list_end = args_str.find(']')
                    if first_list_end == -1:
                        raise ValueError("Invalid format: missing first closing bracket for joint_indices.")
                    joint_indices_str = args_str[:first_list_end + 1]
                    
                    # Get the remaining string after the first list and its separating comma
                    remaining_str = args_str[first_list_end + 1:].strip()
                    if not remaining_str.startswith(','):
                        raise ValueError("Invalid format: missing comma after joint_indices.")
                    remaining_str = remaining_str[1:].strip() # Remove the comma

                    # Parse the second list (target_angles_degrees)
                    second_list_end = remaining_str.find(']')
                    if second_list_end == -1:
                        raise ValueError("Invalid format: missing second closing bracket for target_angles_degrees.")
                    target_angles_str = remaining_str[:second_list_end + 1]

                    # The rest of the string should be the speed, preceded by a comma
                    speed_str = remaining_str[second_list_end + 1:].strip()
                    if not speed_str.startswith(','):
                        raise ValueError("Invalid format: missing comma after target_angles_degrees.")
                    speed_str = speed_str[1:].strip() # Remove the comma

                    # Safely evaluate the extracted string parts into Python lists and float
                    joint_indices = ast.literal_eval(joint_indices_str)
                    target_angles_degrees = ast.literal_eval(target_angles_str)
                    speed = float(speed_str)

                    print(f"Executing command: {command_line}")
                    drive_multiple_joints(joint_indices, target_angles_degrees, speed)
                else:
                    print(f"Warning: Unknown command format on line {line_num + 1}: '{command_line}'. Skipping.")

            except (ValueError, SyntaxError) as e:
                print(f"Error parsing command on line {line_num + 1}: '{command_line}' - {e}")
            except Exception as e:
                print(f"An unexpected error occurred during command execution on line {line_num + 1}: {e}")

    except Exception as e: # Catch any other file reading or processing errors
        print(f"Error reading or processing config file: {e}")

# --- Main Simulation Loop ---
while simulation_app.is_running():
    my_world.step(render=True) # Advance the simulation world and render the scene
    load_and_execute_commands(CONFIG_FILE_PATH) # Check for and execute commands from the config file
    time.sleep(0.1) # Small delay to prevent excessive file checking and allow for user editing