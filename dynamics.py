import geometry as gm
import primitives as pm
import numpy as np
import math
import pdb


##
# Time of collision of a ball with a wall
def get_toc_ball_wall(obj1, obj2, aFriction=0, aColDamp=0):
    '''
        obj1: ball
        obj2: wall
    '''
    lines = obj2.get_lines()
    vel = obj1.get_velocity()
    pos = obj1.get_position()
    r = obj1.get_radius()
    tCol = np.inf
    nrmlCol = None
    ptCol = None
    for l in lines:
        # This is the nrml from the line towards the point
        nrml = l.get_normal_towards_point(pos)
        nrml.scale(-1)  # Normal from point towards line
        speed = vel.dot(nrml)
        if speed <= 0:
            continue
        # Velocity in orthogonal direction
        velOrth = vel - (speed * nrml)
        lDir = l.get_direction()
        # Find the time of collision
        ray = gm.Line(pos, pos + nrml)
        intPoint = l.get_intersection_ray(ray)
        # print l
        assert intPoint is not None, ("Intersection point cannot be none - pos, nmrl",
                                      pos.x(), pos.y(), nrml.x(), nrml.y())
        distCenter = pos.distance(intPoint)
        dist = distCenter - r

        # rounding error
        if abs(dist) < 1e-10:
            dist = round(dist, 10)
        if dist < 0:
            # It is possible that a line (but not the line segment) intersects the ball. Then
            # dist < 0, and we need to rule out such cases.
            assert distCenter >= 0
            onSegment = l.is_on_segment(intPoint)
            if onSegment:
                print "Something is amiss"
                pdb.set_trace()
            else:
                continue
        t = np.inf
        if aFriction == 0:  # constant motion
            t = dist / speed
        elif aFriction > 0:  # rolling friction
            # projected acceleration: opposite direction to velocity
            acc = -aFriction * obj1.get_mass() * (speed / vel.mag())
            # inverse of vt+0.5*at^2
            if speed * speed + 2 * dist * acc >= 0:  # will collide within this step
                t = (math.sqrt(speed * speed + 2 * dist * acc) - speed) / acc
            else:
                continue
        else:
            raise ValueError('aF should be non-negative')
        # Find the intersection point on line
        # i.e. the point
        linePoint = intPoint + (t * velOrth)
        onSegment = l.is_on_segment(linePoint)
        if not onSegment:
            continue
        if t < tCol:
            tCol = t
            nrml.scale(-1)
            nrmlCol = nrml
            ptCol = linePoint

    # pdb.set_trace()
    # print tCol, nrmlCol, ptCol
    return tCol, nrmlCol, ptCol, None


##
# Time of collision of ball with ball.
def get_toc_ball_ball(obj1, obj2, name1, name2, aFriction=0, aColDamp=0):
    # Initializations
    tCol = np.inf
    nrmlCol = None
    ptCol = None
    badResult = [np.inf, None, None, None]
    # Get the velocities of the ball.
    pos1, vel1 = obj1.get_position(), obj1.get_velocity()
    pos2, vel2 = obj2.get_position(), obj2.get_velocity()
    # print 'Before col det', name1, vel1
    # print 'Before col det', name2, vel2
    # We will go into the frame of reference of object 1
    relVel = vel2 - vel1
    # Find the direction of collision
    colDir = pos2 - pos1
    colDist = colDir.mag() - (obj1.get_radius() + obj2.get_radius())
    colDir.make_unit_norm()
    # Get the velocity along the direction of collision
    speed = -relVel.dot(colDir)
    # print "Speed is: ", speed
    if speed <= 0:
        # If the balls will not collide
        return badResult

    circ1 = gm.Circle(obj1.get_radius(), pos1)
    circ2 = gm.Circle(obj2.get_radius(), pos2)
    tCol, ptCol, nrmlCol, relDist = circ1.intersect_moving_circle(circ2, relVel)
    if ptCol is None:
        return badResult
    # print '##### NRML ####',tCol,ptCol, nrmlCol
    vOth1 = vel1.project(nrmlCol)
    vOth2 = vel2.project(nrmlCol)
    vCol1 = vel1 - vOth1
    vCol2 = vel2 - vOth2
    m1 = obj1.get_mass()
    m2 = obj2.get_mass()
    if aFriction == 0:  # constant motion
        pass
    elif aFriction > 0:  # correct tCol and new vel before collision
        # initial speed
        speed = relDist / tCol
        acc_r = [0, 0, 0, 0]
        if vel1.mag() > 0:
            acc_r[0] = vCol1.mag() / vel1.mag()
            acc_r[2] = vOth1.mag() / vel1.mag()
        if vel2.mag() > 0:
            acc_r[1] = vCol2.mag() / vel2.mag()
            acc_r[3] = vOth2.mag() / vel2.mag()

        if vCol1.dot(vCol2) <= 0:  # -> <-  or -> .
            acc = -aFriction * (m1 * acc_r[0] + m2 * acc_r[1])
        else:  # --> ->
            acc = -aFriction * (
                    (m1 * acc_r[0] - m2 * acc_r[1]) * (vCol1.mag() > vCol2.mag()) + (m2 * acc_r[1] - m1 * acc_r[0]) * (
                    vCol1.mag() <= vCol2.mag()))
        # project to relative direction
        if acc != 0:
            if speed * speed + 2 * relDist * acc >= 0:
                tCol = (math.sqrt(speed * speed + 2 * relDist * acc) - speed) / acc
                # if any stopped before collision, then "stop " event should happen first
                # use acc_r -> already check vel1.mag()=0
                if tCol * aFriction * m1 * acc_r[0] > vCol1.mag() or tCol * aFriction * m2 * acc_r[1] > vCol2.mag():
                    return badResult
            else:
                return badResult
        vCol1 = vCol1 - vCol1.get_scaled_vector(1.0) * tCol * aFriction * m1 * acc_r[0]
        vOth1 = vOth1 - vOth1.get_scaled_vector(1.0) * tCol * aFriction * m1 * acc_r[2]
        vCol2 = vCol2 - vCol2.get_scaled_vector(1.0) * tCol * aFriction * m2 * acc_r[1]
        vOth2 = vOth2 - vOth2.get_scaled_vector(1.0) * tCol * aFriction * m2 * acc_r[3]

    # pdb.set_trace()
    vCol1New = (vCol1 * (m1 - m2) + 2 * m2 * vCol2) * (1.0 / (m1 + m2))
    vCol2New = (vCol2 * (m2 - m1) + 2 * m1 * vCol1) * (1.0 / (m1 + m2))
    vel1New = (vCol1New + vOth1) * (1 - aColDamp)
    # don't set the future velocity yet... may overwrite earlier collision
    # vel2New  = vCol2New + vOth2
    # obj1.set_after_collision_velocity(vel1New)
    # obj2.set_after_collision_velocity(vel2New)
    # We dont require normal and point of collision.
    # pdb.set_trace()
    # print vel1, vel1New, vel2, vel2New, vel1New.mag(), vel2New.mag()
    # print 'After col det', name1, vel1New
    # print 'After col det', name2, vel2New
    return tCol, nrmlCol, ptCol, vel1New
