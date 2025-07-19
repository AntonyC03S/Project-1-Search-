import sys


def UCS(grid, robot_location, path, rows, cols):
    relations_dictionary = {}
    relations_dictionary[(robot_location[0],robot_location[1])] = None
    print(robot_location)
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
        if nodes_expanded > 1000000:
            return []
        
    grid = CleanMess(node,robot_location, grid)    
    path = PathDecoder(relations_dictionary, path, node) 
    robot_location = node
    return path, robot_location


def DFS():

    print()


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
    if WithInBound([x,y+1], rows, cols) and IsNotBlocked([x,y+1], grid) and ([x,y+1] not in explored):
        queue.append([x,y+1])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x,y+1)] = tuple_node
    if WithInBound([x+1,y], rows, cols) and IsNotBlocked([x+1,y], grid) and ([x+1,y] not in explored):
        queue.append([x+1,y])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x+1,y)] = tuple_node
    if WithInBound([x,y-1], rows, cols) and IsNotBlocked([x,y-1], grid) and ([x,y-1] not in explored):
        queue.append([x,y-1])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x,y-1)] = tuple_node
    if WithInBound([x-1,y], rows, cols) and IsNotBlocked([x-1,y], grid) and ([x-1,y] not in explored):
        queue.append([x-1,y])
        nodes_generated = nodes_generated +1
        relations_dictionary[(x-1,y)] = tuple_node

    return explored, queue, nodes_generated,relations_dictionary

def PathDecoder(relations_dictionary, path, node):
    node = (node[0], node[1])
    print(node)
    print(relations_dictionary)
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
        side_path = []
        i = 0
        while i != 1:
            side_path, robot_location = UCS(grid, robot_location,side_path, rows, cols)
            if len(side_path) == 0:
                break
            i =+ 1
            
        all_path.extend(side_path)
        print(all_path)


    elif algorithm == "depth-first":
        DFS()
        print("DFS")

    else:
        print("Algorithm is not uniform-cost or depth-first")


if __name__ == "__main__":
    main()
