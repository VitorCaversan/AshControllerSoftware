import math

balls = [[[10, 10], [20, 30, 5]], [[10, 9], [20, 40, 5]]]
mock_balls = [[20, 30, 5], [10, 40, 5], [20, 30, 7]]

dist = lambda x1,y1,x2,y2: math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def isBallsColliding(ball1: list, ball2:list):
    dist_center = dist(ball1[0], ball1[1], ball2[0], ball2[1])
    return dist_center < (ball1[2] + ball2[2])


for mock_ball in mock_balls:
    if(isBallsColliding(mock_ball, balls[1][1])):
        print("colliding")
    else:
        print("Not coliding")
        