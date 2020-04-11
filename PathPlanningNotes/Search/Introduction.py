"""
Planning Problem:

    Given:  MAP
            START POSITION
            GOAL POSITION
            COST

    GOAL:   FIND MINIMUM COST PATH

Computing Cost:
    Suppose each step cost a unit:
    - Left lane changes cost more
    - SEARCH: Find cheapest cost

"""

grid = [[0, 0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 1, 1, 1, 0],
        [0, 0, 0, 0, 1, 0]]
init = [0, 0]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1

delta = [[-1, 0], # go up
         [ 0,-1], # go left
         [ 1, 0], # go down
         [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']

def search(grid,init,goal,cost):
    #Create an equivalent size grid, 5x6
    closed = [[0] * len(grid[0]) for i in grid]
    expansion_grid = [[-1] * len(grid[0]) for i in grid]
    action = [[' '] * len(grid[0]) for i in grid]
    expand = True
    #Set initial position to one
    closed[init[0]][init[1]] = 1
    
    
    #Initial [cost,x,y]
    x = init[0]
    y = init[1]
    g = 0

    open = [[g,x,y]]
    expansion_grid[x][y] = g
    expand_count = 0
    found = False
    resign = False

    #While you havent found end and there is no other option
    while found is False and resign is False:

        #If there are no more "branches to explore"
        if len(open) == 0:
            resign = True
            print('fail')

        else:
            open.sort() #Sort in ascending order
            open.reverse() #Reverse to grab next closest position
            next = open.pop() #Remove next location on list
            x = next[1]
            y = next[2]
            g = next[0]
            expansion_grid[x][y] = expand_count
            expand_count += 1
            print(next)

            #If next position is the goal, return Found
            if x == goal[0] and y == goal[1]:
                found = True
                print('Found')
            
            else:
                for i in range(len(delta)):
                    #Try different moves
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    #Check if out of bounds
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        #Check if it has been visited
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            # If not visited add cost and append new location
                            g2 = g + cost
                            open.append([g2,x2,y2])
                            #Save as visited
                            closed[x2][y2] = 1
                            action[x2][y2] = i
    if(expand == True):
        for i in expansion_grid:
            print(i)

    policy = [[' '] * len(grid[0]) for i in grid]
    policy[goal[0]][goal[1]] = '*'
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        x = x2
        y = y2
    for row in policy:
        print(row)

    return next

search(grid,init,goal,cost)