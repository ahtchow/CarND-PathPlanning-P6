"""
Dynamic Programming

    Given: MAP, GOAL

    OUTPUTS: Best path from anywhere.

    Why? In reality the map is changing due to external factors
         A lane change may not be possible, so the optimal path must be constantly
         calculated amongst random occurances.

    Value Function
        - Calculates each grid cells' lowest length from goal
        - Recursively calculated through
        f(x,y) = f(x',y') + cost

"""

grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0]]
goal = [len(grid)-1, len(grid[0])-1]
cost = 1 # the cost associated with moving from a cell to an adjacent one

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

delta_name = ['^', '<', 'v', '>']


def compute_value(grid,goal,cost,delta_name):

    value = [[99 for row in range(len(grid[0]))] for col in range(len(grid))]
    policy = [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]
    change = True

    while change:

        change = False

        for x in range(len(grid)):

            for y in range(len(grid[0])):

                if goal[0] == x and goal[1] == y:
                    
                    if value[x][y] > 0:
                        value[x][y] = 0
                        policy[x][y] = '*'
                        change = True
                        
                elif grid[x][y] == 0:
                    
                    for a in range(len(delta)):
                        x2 = x + delta[a][0]
                        y2 = y + delta[a][1]

                        if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                            
                            v2 = value[x2][y2] + cost
                            if v2 < value[x][y]:
                                change = True
                                value[x][y] = v2
                                policy[x][y] = delta_name[a]
    
    #Print cost
    for i in value:
        print(i)

    #Print Optimal Policy
    for j in policy:
        print(j)

    return value

compute_value(grid,goal,cost,delta_name)