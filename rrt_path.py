# from simulated_rrt_tree import *
from rrt_tree import *
from CP2_startercode import *
import pybullet as p
import math
import random

MAX_ITERS = 20 #number of tries for each monte carlo run
GOAL_BIAS = 0.5 #50%
RRT_CAP = 3000 #maximum number of montecarlo runs

#action duration bounds - tune as needed. 240 samples/sec
STEP_MIN = 240 #5 
STEP_MAX = 480 #120 

#TODO: Should we allow negatives?
WHEEL_MIN = -3.0
WHEEL_MAX =  6.0



#helper function, takes in the bot, returns a list of poses to go from start->goal
def get_rrt_path(bot: Turtlebot, tree: Tree, arena):
    ARENA_XMIN, ARENA_XMAX, ARENA_YMIN, ARENA_YMAX = arena
    root_id = tree.add_node(bot.get_position(), bot.get_orientation(),
                            parent=None, action=None, steps=None, trajectory=None) #initialize tree root as bot's position at start - need to update action after first sim run

    for _ in range(RRT_CAP):  # overall RRT iterations cap 
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
            # return tree.backtrack_path(new_id) 
            return new_id

    return None
        

#gets ONE path
def do_rollout(bot: Turtlebot, start_pos, start_orn, action, steps):
    bot.set_velocities(0, 0)
    bot.teleport(start_pos, start_orn)
    p.resetBaseVelocity(bot.turtleId, [0, 0, 0], [0, 0, 0])
    p.stepSimulation()

    lw, rw = action
    bot.set_velocities(lw, rw)
    trajectory = [list(start_pos)]
    for i in range(steps):
        p.stepSimulation()
        if i % 10 == 0: # This gives a balance between resolution and performance 
            trajectory.append(list(bot.get_position()))
        if bot.collision_check():
            bot.set_velocities(0, 0)
            return None

    bot.set_velocities(0, 0)
    return (bot.get_position(), bot.get_orientation(), trajectory)

    
#calls do_rollout until we have 5 valid paths. If we make it to MAX_ITERS without 5 paths, assume the sample is bad (i.e. we are driving into a wall) and return None
#adds all nodes to tree if we have 5 valid samples from monte iteration
def monte_iter(bot:Turtlebot, tree:Tree, sample):
    closest_id = tree.nearest(sample)          
    closest = tree.get_node(closest_id)       

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

        end_pos, end_orn, trajectory = end
        candidates.append((end_pos, end_orn, action, steps, trajectory))

    if len(candidates) < 5:
        return None
    
    new_ids = []
    for end_pos, end_orn, action, steps, trajectory in candidates:
        new_id = tree.add_node(
            end_pos,
            end_orn,
            parent=closest_id,
            action=action,
            steps=steps,
            trajectory=trajectory,
        )
        new_ids.append(new_id)

    best_idx = min(
        range(len(candidates)),
        key=lambda i: math.dist((candidates[i][0][0], candidates[i][0][1]), sample),
    )
    best_id = new_ids[best_idx]
    best_node = tree.get_node(best_id)

    bot.teleport(best_node.pos, best_node.orn)
    p.stepSimulation()

    if sample != (GOAL[0], GOAL[1]):
        bot.show_intermediate_goal([sample[0], sample[1], 0])

    return best_id


