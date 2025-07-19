import sys
"""
planner.py
Generates a path for the robot in the Vacuum World.

Usage:
    python3 planner.py [ algorithm ] [ world-file ]

Example:
    python3 planner.py uniform-cost tiny-1.txt

Parameters:
- algorithm (string): pathing algorithm (uniform-cost or depth-first)
- world-file (string): the txt file of the Vacuum World


Outputs to stdout:
1. Path of the robot
2. Number of nodes generated
3. Number of nodes expanded
"""


"""--------------------Path functions------------------"""

def UCS(grid, robot_location, path, rows, cols):
    # Setup optimal path for path function
    relations_dictionary = {}
    relations_dictionary[(robot_location[0],robot_location[1])] = None

    # Setup searching algorithm 
    explored = []
    queue = [robot_location]
    nodes_generated = 0
    nodes_expanded = 0
    while len(queue) != 0:
        node = queue.pop(0)
        if IsGoal(node, grid): 
            break
        explored, queue, nodes_generated,relations_dictionary = GenerateSideNodes(node, grid, rows, cols, explored, queue, nodes_generated,relations_dictionary)    
        nodes_expanded = nodes_expanded + 1

        # Breaking the infinite loop
        if nodes_expanded > 1000 or len(queue) == 0:
            return [], robot_location, grid, nodes_generated, nodes_expanded
        
    
    grid = CleanMess(node,robot_location, grid)    
    path = PathDecoder(relations_dictionary, path, node) 
    robot_location = node
    return path, robot_location, grid, nodes_generated, nodes_expanded


#
def DFS(grid, robot_location, path, rows, cols):
    # Setup optimal path for path function
    relations_dictionary = {}
    relations_dictionary[(robot_location[0],robot_location[1])] = None

    # Setup searching algorithm 
    explored = []
    stack = [robot_location]
    nodes_generated = 0
    nodes_expanded = 0
    while len(stack) != 0:
        node = stack.pop()
        
        if IsGoal(node, grid): 
            break
        
        explored, stack, nodes_generated,relations_dictionary = GenerateSideNodes(node, grid, rows, cols, explored, stack, nodes_generated,relations_dictionary)    
        nodes_expanded = nodes_expanded + 1

        # Breaking the infinite loop
        if nodes_expanded > 1000 or len(stack) == 0:
            return [], robot_location, grid, nodes_generated, nodes_expanded
        
    
    grid = CleanMess(node,robot_location, grid)    
    path = PathDecoder(relations_dictionary, path, node) 
    robot_location = node
    return path, robot_location, grid, nodes_generated, nodes_expanded

"""--------------------Helper functions------------------"""

def WithInBound(pos, rows, cols):
    if pos[0]>= cols or pos[0]< 0:
        return False
    elif pos[1]>= rows or pos[1]< 0:
        return False
    else:
        return True

def IsNotBlocked(pos, grid):
    if(grid[pos[1]][pos[0]] == '#'):
        return False
    return True

def IsGoal(pos, grid):
    if(grid[pos[1]][pos[0]] != '*'):
        return False
    return True

def GenerateSideNodes(node, grid, rows, cols, explored, queue, nodes_generated ,relations_dictionary):
    explored.append(node)
    x = node[0]
    y = node[1]
    tuple_node = (x, y)
    if WithInBound([x,y+1], rows, cols) and IsNotBlocked([x,y+1], grid) and ([x,y+1] not in explored) and ([x,y+1] not in queue):
        queue.append([x,y+1])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x,y+1)] = tuple_node
    if WithInBound([x+1,y], rows, cols) and IsNotBlocked([x+1,y], grid) and ([x+1,y] not in explored) and ([x+1,y] not in queue):
        queue.append([x+1,y])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x+1,y)] = tuple_node
    if WithInBound([x,y-1], rows, cols) and IsNotBlocked([x,y-1], grid) and ([x,y-1] not in explored) and ([x,y-1] not in queue):
        queue.append([x,y-1])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x,y-1)] = tuple_node
    if WithInBound([x-1,y], rows, cols) and IsNotBlocked([x-1,y], grid) and ([x-1,y] not in explored) and ([x-1,y] not in queue):
        queue.append([x-1,y])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x-1,y)] = tuple_node
    return explored, queue, nodes_generated,relations_dictionary

def PathDecoder(relations_dictionary, path, node):
    node = (node[0], node[1])
    path.append('V')
    next = relations_dictionary[node]
    while next is not None:
        if node[0] > next[0]:
            path.append('E')
        elif node[0] < next[0]:
            path.append('W')
        elif node[1] > next[1]:
            path.append('S')
        elif node[1] < next[1]:
            path.append('N')
        else:
            print("Error:Path Decoder")
            return []             # Something has gone very wrong
        node = next
        next = relations_dictionary[node]
    path.reverse()
    return path

def CleanMess(node,robot_location, grid):
    grid[node[1]][node[0]] = "@"
    grid[robot_location[1]][robot_location[0]] = "_"
    return grid

"""--------------------Main functions------------------"""

def main():
    if len(sys.argv) != 3:
        print("Usage: python3 planner.py <algorithm> <world-file> ")
        sys.exit(1)

    algorithm = sys.argv[1]   
    world_file = sys.argv[2]   

    #Get data from the text file
    with open(world_file) as f:
        cols = int(f.readline())
        rows = int(f.readline())
        grid = []
        for i in range(rows):
            line = []
            for x in range(cols):
                line.append(f.read(1))
            f.read(1)
            grid.append(line)

    #Find Location of Robot
    for y in range(rows):
        for x in range(cols):
            if grid[y][x] == '@':
                robot_location = [x,y]
                break
    

    #Find the location of dirty spot
    if algorithm == "uniform-cost":
        all_path=[]
        total_nodes_generated = 0
        total_nodes_expanded = 0
        i = 0
        while i != 20:
            i = i + 1            # Make sure it does not fo infinity      
            side_path = []
            side_path, robot_location, grid, nodes_generated, nodes_expanded = UCS(grid, robot_location,side_path, rows, cols)
            if len(side_path) == 0:
                break
            all_path.extend(side_path)
            total_nodes_generated = total_nodes_generated+nodes_generated
            total_nodes_expanded = total_nodes_expanded+nodes_expanded
        
    elif algorithm == "depth-first":
        all_path=[]
        total_nodes_generated = 0
        total_nodes_expanded = 0
        i = 0
        while i != 20:
            i = i + 1            # Make sure it does not fo infinity      
            side_path = []
            side_path, robot_location, grid, nodes_generated, nodes_expanded = DFS(grid, robot_location, side_path, rows, cols)
            if len(side_path) == 0:
                break
            all_path.extend(side_path)
            total_nodes_generated = total_nodes_generated+nodes_generated
            total_nodes_expanded = total_nodes_expanded+nodes_expanded

    else:
        print("Algorithm is not uniform-cost or depth-first")

    for i in all_path:
        print(i)
    print(total_nodes_generated , "nodes generated")
    print(total_nodes_expanded , "nodes expanded")


if __name__ == "__main__":
    main()
