import pybullet as p
import pybullet_data
import random
from typing import List

random.seed(1)

### VARIABLES ###

# You can modify these variables for testing, but the final solution needs to be implemented with this setup #

GOAL = [4.5, 4.5]
GOAL_RADIUS = 0.3
START_POS = [1.5, 1.5, 0]
BLOCKS = [[1.5, 2.5], [4, 2], [3,5]]


### PYBULLET SETUP ###

def setup_pybullet(goal):
    """
    This function setups the pybullet environment. You do not need to modify anything in this function.
    """

    # Setup physics client and turtlebot
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    planeId = p.loadURDF("plane.urdf") 
    startPos = START_POS
    turtleId = p.loadURDF("turtlebot.urdf", startPos, globalScaling=2)
    sphereId = p.loadURDF("sphere.urdf", [goal[0], goal[1], 0.01], globalScaling=0.25)
    wallId = p.loadURDF("arena_walls6.urdf") 
    obstacles = [wallId]
    blocks = BLOCKS

    for block in blocks:
        blockId = p.loadURDF("block.urdf", [block[0], block[1], 0]) 
        obstacles.append(blockId)

    # set camera to bird eye view
    # note: can't set camera pitch to 90 exact -> gimble lock
    p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=0, cameraPitch=-89.99, cameraTargetPosition=[4, 4, 0])
    return turtleId, obstacles

### TURTLEBOT SETUP ##

class Turtlebot():
    """
    This class sets up your robot (a turtlebot!). 
    Helper functions to help you interact with the pybullet environment have been provided.
    You are welcome to modify any functions if you'd like as part of your solution but it is not necessary.
    """

    def __init__(self, turtleId, obstacles):
        self.position = None
        self.turtleId = turtleId
        self.obstacles = obstacles
        self.old_goals = []

    def get_position(self):
        """
        Inputs: none
        Outputs: turtlebot position as [x,y,z]
        """
        pos, orn = p.getBasePositionAndOrientation(self.turtleId)
        return pos
    
    def get_orientation(self):
        """
        Inputs: none
        Outputs: turtlebot orientation as a quaternion
        """
        pos, orn = p.getBasePositionAndOrientation(self.turtleId)
        return orn
    
    def collision_check(self):
        """
        Inputs: none
        Outputs: True/False, whether or not the turtlebot is in collision with a wall or block
        """
        collision_check = False
        for obstacle in self.obstacles: # check walls and blocks
            contacts = p.getContactPoints(bodyA=turtlebot.turtleId, bodyB=obstacle)
            if contacts:
                collision_check = True
                print(f"Collision detected at pose: {turtlebot.get_position()}")
                break
        return collision_check
        
    def teleport(self, pos, orn):
        """
        Teleports turtlebot to desired position/orientation

        Inputs:
        -pos: desired turtlebot positon
        -orn: desired turtlebot orientation

        Outputs: None
        """
        p.resetBasePositionAndOrientation(self.turtleId, pos, orn)

    def set_velocities(self, leftWheelVelocity, rightWheelVelocity):
        """
        Set turtlebot wheel velocities

        Inputs:
        -leftWheelVelocity
        -rightWheelVelocity

        Outputs: None
        """
        # set velocities
        p.setJointMotorControl2(self.turtleId,0,p.VELOCITY_CONTROL,targetVelocity=leftWheelVelocity,force=1000)
        p.setJointMotorControl2(self.turtleId,1,p.VELOCITY_CONTROL,targetVelocity=rightWheelVelocity,force=1000)

    def plot_path(self, color_string: str, path_points: List[List[float]]) -> None:
        """
        Debug function for plotting paths in pybullet

        Inputs:
        -color_string: color of path line
        -path_points: path line points to plot List[[x,y,z]]

        Outputs: None
        """

        # Supported colors
        if color_string == "red":
            color = [1, 0, 0]
            line_width = 3
        elif color_string == "green":
            color = [0, 1, 0]
            line_width = 10
        else:
            print("That color does not exist to be viewed. Not plotting Path.")
            return

        # Loop through path points and add debug line
        for i in range(1, len(path_points)):
            p.addUserDebugLine(
                [path_points[i-1][0], path_points[i-1][1], path_points[i-1][2] + 0.05],
                [path_points[i][0],   path_points[i][1],   path_points[i][2] + 0.05],
                color,
                lineWidth=line_width
            )
    
    def show_intermediate_goal(self, intermediate_goal: List[float]) -> None:
        """
        Show randomly generated goals in pybullet

        Inputs:
        -intermediate_goal: path line points to plot [x,y,z]

        Outputs: None
        """

        # Loop through old random goals and change them to black to show they are no longer the active goal
        size = 0.15
        for goal, goal_cross_1, goal_cross_2 in self.old_goals:
            p.removeUserDebugItem(goal_cross_1)
            p.removeUserDebugItem(goal_cross_2)
            goal_cross_1 = p.addUserDebugLine([goal[0] - size, goal[1], 0.1], [goal[0] + size, goal[1], 0.1], lineColorRGB=[0, 0, 0], lineWidth=3)
            goal_cross_2 = p.addUserDebugLine([goal[0], goal[1] - size, 0.1], [goal[0], goal[1] + size, 0.1], lineColorRGB=[0, 0, 0], lineWidth=3)

        # If new random goal provided, plot blue cross marking it.
        if intermediate_goal != None:
            goal_cross_1 = p.addUserDebugLine([intermediate_goal[0] - size, intermediate_goal[1], 0.1], [intermediate_goal[0] + size, intermediate_goal[1], 0.1], lineColorRGB=[0, 0, 1], lineWidth=3)
            goal_cross_2 = p.addUserDebugLine([intermediate_goal[0], intermediate_goal[1] - size, 0.1], [intermediate_goal[0], intermediate_goal[1] + size, 0.1], lineColorRGB=[0, 0, 1], lineWidth=3)
            self.old_goals.append((intermediate_goal, goal_cross_1, goal_cross_2))


### MAIN - implement your solution here! ###

if __name__ == "__main__":
    goal = GOAL
    # Setup pybullet
    turtleId, obstacles = setup_pybullet(goal)
    # Setup turtlebot
    turtlebot = Turtlebot(turtleId, obstacles)

    print('starting at {}'.format(turtlebot.get_position()))
    # Turn off real-time simulation for manual step control
    p.setRealTimeSimulation(0)

    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./search.mp4")
    
    while p.isConnected():

        # Implement your solution in this loop
        
        # Some example code below to get familiar with the simulation loop

        # set turtlebot to move forward
        turtlebot.set_velocities(leftWheelVelocity=10, rightWheelVelocity=10)

        # IMPORTANT - You need to run this command for every step in simulation
        # Example: if you comment this out your turtlebot will not move despite setting the wheel velocities
        p.stepSimulation() 


        # Command to stop recording
        p.stopStateLogging(log_id)

        

        

        



    p.disconnect()