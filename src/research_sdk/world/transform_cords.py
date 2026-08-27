import math


def world2robot(robot_position, target_position):
    '''
        input:
            Target_position: position in the world coordinate system (x,y)
            robot_position: robot pose (x, y, theta)
        output:
            t: targeted position in respect to robot coordinate system (x,y)
    '''
    if robot_position is None or target_position is None:
        return

    angle = robot_position[2]
    c = math.cos(angle)
    s = math.sin(angle)
    tx = robot_position[0]
    ty = robot_position[1]

    # Analytical inverse of the transformation matrix:
    # R^T * (target - translation)
    dx = target_position[0] - tx
    dy = target_position[1] - ty
    x =  c * dx + s * dy
    y = -s * dx + c * dy

    return (x, y)

def robot2world(r, p):
    '''
        input:
            target_coordinates: position in the robot coordinate system (x,y)
            robot_pos: robot pose (x, y, theta)
        output:
            w: position in the robot coordinate system (x,y)
    '''
    angle = p[2]
    c = math.cos(angle)
    s = math.sin(angle)

    x = c * r[0] - s * r[1] + p[0]
    y = s * r[0] + c * r[1] + p[1]

    return (x, y)
