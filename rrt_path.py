from rrt_tree import *
from CP2_startercode import *
import pybullet as p
import math
import random

MAX_ITERS = 20 #number of tries for each 
GOAL_BIAS = 0.5 #50%

#action duration bounds - tune as needed. 240 samples/sec
STEP_MIN = 24 
STEP_MAX = 120 

#TODO: Should we allow negatives?
WHEEL_MIN = 0.0
WHEEL_MAX =  12.0


#TODO: Figure out where these values are stored so we can retrieve them
ARENA_XMIN, ARENA_XMAX = 0.0, 6.0
ARENA_YMIN, ARENA_YMAX = 0.0, 6.0

"""
Assumptions I am making about the Tree API:

#initialize tree (empty constructor)
Tree()

#add root node (pos=[x,y,z], orn=[x,y,z,w], parent=None, action=None, steps=None) -> returns node_id (int)
tree.add_node(bot.get_position(), bot.get_orientation(), parent=None, action=None, steps=None)

#return node_id (int) of node closest to sample=(x,y)
tree.nearest(sample)

#get node object from node_id (int) -> node has fields .pos ([x,y,z]) and .orn ([x,y,z,w])
tree.get_node(node_id)

#given node_id (int), return path from root->node_id as list of (pos, orn) tuples
tree.backtrack_path(node_id)

#add child node (end_pos=[x,y,z], end_orn=[x,y,z,w], parent=closest_id (int), action=(lw,rw), steps=int) -> returns node_id (int)
tree.add_node(end_pos, end_orn, parent=closest_id, action=action, steps=steps)
"""

#helper function, takes in the bot, returns a list of poses to go from start->goal
def get_rrt_path(bot: Turtlebot):
    tree = Tree()
    root_id = tree.add_node(bot.get_position(), bot.get_orientation(),
                            parent=None, action=None, steps=None) #initialize tree root as bot's position at start - need to update action after first sim run

    for _ in range(3000):  # overall RRT iterations cap 
        # sample node  generation
        if random.random() < GOAL_BIAS:
            sample = (GOAL[0], GOAL[1])
        else:
            sample = (random.uniform(ARENA_XMIN, ARENA_XMAX),
                      random.uniform(ARENA_YMIN, ARENA_YMAX))

        new_id = monte_iter(bot, tree, sample)
        if new_id is None:
            continue

        new_pos = tree.get_node(new_id).pos   
        if math.dist((new_pos[0], new_pos[1]), (GOAL[0], GOAL[1])) <= GOAL_RADIUS:
            return tree.backtrack_path(new_id) 

    return None
        

#gets ONE path
def do_rollout(bot: Turtlebot, start_pos, start_orn, action, steps):
    bot.teleport(start_pos, start_orn)

    lw, rw = action
    bot.set_velocities(lw, rw)

    for _ in range(steps):
        p.stepSimulation()
        if bot.collision_check():
            bot.set_velocities(0, 0)
            return None

    bot.set_velocities(0, 0)
    return (bot.get_position(), bot.get_orientation())

    
#calls do_rollout until we have 5 valid paths. If we make it to MAX_ITERS without 5 paths, assume the sample is bad (i.e. we are driving into a wall) and return None
def monte_iter(bot:Turtlebot, tree:Tree, sample):
    closest_id = tree.nearest(sample)          # should return node id/index
    closest = tree.get_node(closest_id)        # adjust to your Tree API

    candidates = []
    attempts = 0

    while attempts < MAX_ITERS and len(candidates) < 5:
        attempts += 1

        steps = random.randint(STEP_MIN, STEP_MAX)
        action = (random.uniform(WHEEL_MIN, WHEEL_MAX),
                  random.uniform(WHEEL_MIN, WHEEL_MAX))

        end = do_rollout(bot, closest.pos, closest.orn, action, steps)
        if end is None:
            continue

        end_pos, end_orn = end
        candidates.append((end_pos, end_orn, action, steps))

    if len(candidates) < 5:
        return None

    # choose endpoint closest to sample S
    best = min(candidates, key=lambda c: math.dist((c[0][0], c[0][1]), sample))
    end_pos, end_orn, action, steps = best

    # add to tree
    new_id = tree.add_node(end_pos, end_orn, parent=closest_id, action=action, steps=steps)
    return new_id


"""
In order to test this code, I replaced the main function in CP2_startercode.py with this:
if __name__ == "__main__":
    from rrt_path import get_rrt_path
    goal = GOAL
    # Setup pybullet
    turtleId, obstacles = setup_pybullet(goal)
    # Setup turtlebot
    turtlebot = Turtlebot(turtleId, obstacles)
    # Let's start by getting the position of the turtlebot
    start_postion = turtlebot.get_position()
    srart_orientation = turtlebot.get_orientation()
    robot_state=RobotPosition(start_postion, srart_orientation)
    print("initial position: ", robot_state.get_position())
    print("initial orientation: ", robot_state.get_orientation())
    print('starting at {}'.format(turtlebot.get_position()))
    # Turn off real-time simulation for manual step control
    p.setRealTimeSimulation(0)

    # Start recording video 
    log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "./search.mp4")
    
    ran = False
    path = None
    while p.isConnected():

        # Implement your solution in this loop
        if not ran:
            path = get_rrt_path(turtlebot)
            ran = True        
            
            if path is None:
                print("RRT failed (path is None)")
            else:
                print(f"RRT returned path with {len(path)} nodes")
                # path is [(pos, orn), ...] from tree.backtrack_path()
                pts = [list(pos) for (pos, orn) in path]
                turtlebot.plot_path("green", pts)
            p.stopStateLogging(log_id)
        # Some example code below to get familiar with the simulation loop

        # set turtlebot to move forward
        # turtlebot.set_velocities(leftWheelVelocity=10, rightWheelVelocity=10)
        # IMPORTANT - You need to run this command for every step in simulation

        p.stepSimulation()


        # Command to stop recording
        # p.stopStateLogging(log_id)

    p.disconnect()
"""