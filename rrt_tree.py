import math

class Node:
    def __init__(self, pos, orn, parent_id, action, steps, trajectory):
        self.pos = pos
        self.orn = orn
        self.parent_id = parent_id
        self.action = action
        self.steps = steps
        self.trajectory = trajectory


class Tree:
    def __init__(self):
        self.nodes = []
    
    def add_node(self, pos, orn, parent, action, steps, trajectory):
        # Add new node to tree and return its id
        node = Node(pos, orn, parent, action, steps, trajectory)
        self.nodes.append(node)
        return len(self.nodes) - 1
    
    def nearest(self, sample):
        # Find closest node to sample point using euclidean distance
        min_dist = float('inf')
        nearest_id = 0
        
        for i, node in enumerate(self.nodes):
            dist = math.sqrt((node.pos[0] - sample[0])**2 + 
                           (node.pos[1] - sample[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_id = i
        
        return nearest_id
    
    def get_node(self, node_id):
        return self.nodes[node_id]
    
    def backtrack_path(self, node_id):
        # Walk back from goal to root to get full path
        path = []
        current_id = node_id
        
        while current_id is not None:
            node = self.nodes[current_id]
            path.append((node.pos, node.orn))
            current_id = node.parent_id
        
        path.reverse()
        return path
    
    def get_action_path(self, node_id):
        # Get list of actions to execute the path - used for bonus points
        actions = []
        current_id = node_id
        
        while current_id is not None:
            node = self.nodes[current_id]
            if node.action is not None:
                actions.append((node.action, node.steps))
            current_id = node.parent_id
        
        actions.reverse()
        return actions
