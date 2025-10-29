import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.utils import plot as plot_utils
import sys, time, math, array
import numpy as np
sys.path.append("./")
from backend.KoalbyHumanoid.RobotKoalby2 import Robot2
from backend.KoalbyHumanoid.trajPlannerTime import TrajPlannerTime
# from backend.Testing import finlyViaPoints as via


# --- Configuration ---
IS_REAL_ROBOT = False 
TEST_ANGLE_DEG = 180  # The angle (in degrees) to move the motor
TEST_ANGLE_RAD = math.radians(TEST_ANGLE_DEG)

# --- Connect to Robot ---
# This initializes your entire Robot2 class in simulation mode
print("Initializing Robot2 in simulation mode...")
robot = Robot2(IS_REAL_ROBOT)
print("Robot initialized. Ready for testing.")
time.sleep(1)

# --- Main Test Loop ---
try:
    while True:
        print("\n" + "="*50)
        user_input = input("Enter Motor ID to test (or 'exit' to quit): ")

        if user_input.lower() in ['exit', 'quit', 'q']:
            break

        try:
            motor_id = int(user_input)

            # Use the getMotor() method to find the motor object in the list
            motor_to_test = robot.getMotor(motor_id)
            
            # Check if the getMotor() method returned a valid motor
            if motor_to_test is None:
                print(f"--- Error: Motor ID {motor_id} could not be found in the robot.motors list. ---")
                continue

            print(f"Testing Motor ID {motor_id}...")

            # Move the specific motor object to the test angle
            print(f"  -> Moving to {TEST_ANGLE_DEG} degrees.")
            motor_to_test.target = (TEST_ANGLE_RAD, 'P')
            robot.moveAllToTarget() # This command moves all motors to their set targets
            time.sleep(3) 

            # Move the specific motor object to the -test angle
            print(f"  -> Moving to -{TEST_ANGLE_DEG} degrees.")
            motor_to_test.target = (-2*TEST_ANGLE_RAD, 'P')
            robot.moveAllToTarget() # This command moves all motors to their set targets
            time.sleep(5) 

            # Return that motor to zero
            print("  -> Returning to 0 degrees.")
            motor_to_test.target = (0, 'P')
            robot.moveAllToTarget()
            time.sleep(2)

            print(f"Test for Motor ID {motor_id} complete.")

        except ValueError:
            print("--- Invalid input. Please enter a number. ---")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

except KeyboardInterrupt:
    print("\nExiting test script.")

finally:
    print("Script finished.")