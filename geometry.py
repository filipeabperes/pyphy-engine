import numpy as np
import copy
import pdb
import math
import pdb

TOL = 1e-08  # 1e-06 works fine.


class Point:
    def __init__(self, x=0, y=0):
        self.x_ = float(x)
        self.y_ = float(y)

    @classmethod
    def from_self(cls, other):
        self = cls(x=other.x(), y=other.y())
        return self

    def x(self):
        return self.x_

    def y(self):
        return self.y_

    def x_asint(self):
        return int(round(self.x_))

    def y_asint(self):
        return int(round(self.y_))

    def mag(self):
        return np.sqrt(self.x_ * self.x_ + self.y_ * self.y_)

    def scale(self, xScale, yScale=None):
        self.x_ = xScale * self.x_
        if yScale is None:
            self.y_ = xScale * self.y_
        else:
            self.y_ = yScale * self.y_

    def get_scaled_vector(self, scale):
        other = Point.from_self(self)
        other.make_unit_norm()
        other.scale(scale)
        return other

    # Make a point unit norm
    def make_unit_norm(self):
        mag = self.mag()
        if mag > 0:
            self.scale(1.0 / mag)

    # Dot product
    def dot(self, other):
        return self.x() * other.x() + self.y() * other.y()

    # Project self on some other point.
    def project(self, other):
        otherUnit = Point.from_self(other)
        otherUnit.make_unit_norm()
        projMag = self.dot(otherUnit)
        otherUnit.scale(projMag)
        return otherUnit

    # Cosine angle between two vectors - assuming their origin to be zero.
    def cosine(self, other):
        pt1 = Point.from_self(self)
        pt2 = Point.from_self(other)
        pt1.make_unit_norm()
        pt2.make_unit_norm()
        return pt1.dot(pt2)

    # Reflect other with self as normal
    def reflect_normal(self, other):
        s = Point.from_self(self)
        s.make_unit_norm()
        prll = other.project(s)
        orth = other - prll
        # Reflect the orthogonal component
        prll.scale(-1)
        reflected = prll + orth
        return reflected

    def __add__(self, other):
        p = Point()
        if isinstance(other, Point):
            p.x_ = self.x_ + other.x_
            p.y_ = self.y_ + other.y_
        else:
            uDir = Point.from_self(self)
            uDir.make_unit_norm()
            uDir.scale(other)
            p = self + uDir
        return p

    def __sub__(self, other):
        p = Point()
        if isinstance(other, Point):
            p.x_ = self.x_ - other.x_
            p.y_ = self.y_ - other.y_
        else:
            uDir = Point.from_self(self)
            uDir.make_unit_norm()
            uDir.scale(other)
            p = self - uDir
        return p

    def __mul__(self, scale):
        p = Point()
        p.x_ = self.x_ * scale
        p.y_ = self.y_ * scale
        return p

    __rmul__ = __mul__

    def __str__(self):
        return '(%.2f, %.2f)' % (self.x_, self.y_)

    # Does a point lie on quadrant 1 if the current point is the origin
    def is_quad1(self, pt):
        return pt.x() >= self.x_ and pt.y() >= self.y_

    # Quadrant-2
    def is_quad2(self, pt):
        return pt.x() <= self.x_ and pt.y() >= self.y_

    # Quadrant-3
    def is_quad3(self, pt):
        return pt.x() <= self.x_ and pt.y() <= self.y_

    # Quadrant-4
    def is_quad4(self, pt):
        return pt.x() >= self.x_ and pt.y() <= self.y_

    # Distance
    def distance(self, pt, distType='L2'):
        if distType == 'L2':
            dist = (self - pt).mag()
        else:
            raise Exception('DistType: %s not recognized')
        return dist

    # Get the angle of the vector.
    def get_angle(self, isRadian=False):
        rad = math.atan2(self.y(), self.x())
        if isRadian:
            return rad
        else:
            theta = math.degrees(rad)
            return theta

    # Rotate the point
    def rotate_point(self, rot, isRadian=False):
        mag = self.mag()
        theta = self.get_angle(isRadian=isRadian)
        theta = theta + rot
        if not isRadian:
            rad = math.radians(theta)
        x = np.cos(rad)
        y = np.sin(rad)
        pt = Point(x, y)
        pt.make_unit_norm()
        pt.scale(mag)
        return pt

    def get_angle_between(self, other):
        pt1 = Point.from_self(self)
        pt2 = Point.from_self(other)
        pt1.make_unit_norm()
        pt2.make_unit_norm()
        cosTheta = pt1.dot(pt2)
        if cosTheta < 1 + 1e-6:
            cosTheta = min(1, cosTheta)
        theta = math.acos(cosTheta)
        return theta


def theta2dir(theta):
    '''
    theta: anti-clockwise and from the x-axis.
    in degrees
    '''
    assert -180 < theta <= 180
    theta = np.pi * (theta / 180.0)
    x = np.cos(theta)
    y = np.sin(theta)
    pt = Point(x, y)
    pt.make_unit_norm()
    return pt


class Line:
    def __init__(self, pt1, pt2):
        # The line points from st_ to en_
        self.st_ = pt1
        self.en_ = pt2
        self.make_canonical()

    @classmethod
    def from_self(cls, other):
        self = cls(other.st(), other.en())
        return self

    def make_canonical(self):
        '''
            ax + by + c = 0
        '''
        self.a_ = float(-(self.en_.y() - self.st_.y()))
        self.b_ = float(self.en_.x() - self.st_.x())
        self.c_ = float(self.st_.x() * self.en_.y() - self.st_.y() * self.en_.x())
        aMag = np.abs(self.a_)
        if aMag > TOL:
            self.a_ = self.a_ / aMag
            self.b_ = self.b_ / aMag
            self.c_ = self.c_ / aMag
        else:
            self.a_ = 0.0

    def a(self):
        return copy.deepcopy(self.a_)

    def b(self):
        return copy.deepcopy(self.b_)

    def c(self):
        return copy.deepcopy(self.c_)

    def st(self):
        return copy.deepcopy(self.st_)

    def mutable_st(self):
        return self.st_

    def en(self):
        return copy.deepcopy(self.en_)

    def mutable_en(self):
        return self.en_

    def get_direction(self):
        pt = self.en_ - self.st_
        pt.make_unit_norm()
        return pt

    def distance_to_point(self, pt):
        dist = self.a_ * pt.x() + self.b_ * pt.y() + self.c_
        dnmr = np.sqrt(np.power(self.a_, 2) + np.power(self.b_, 2))
        dist = dist / dnmr
        return dist

    # Returns the outward facing normal
    def get_normal(self):
        pt = Point(-self.a(), -self.b())
        pt.make_unit_norm()
        return pt

    # Get the normal which points in the halfspace in
    # which point pt lies.
    def get_normal_towards_point(self, pt):
        nrml = self.get_normal()
        ray = Line(pt, pt + nrml)
        intPt = self.get_intersection_ray(ray)
        if intPt is None:
            return nrml
        else:
            nrml.scale(-1)
            return nrml

    def __str__(self):
        return "(%.2f, %.2f, %.2f)" % (self.a_, self.b_, self.c_)

    # Returns the location of the point wrt a line
    def get_point_location(self, pt, tol=TOL):
        '''
            returns: 1 is point is above the line (i.e. moving counter-clockwise from the line)
                            -1 if the point is below
                             0 if on the line
        '''
        val = self.a_ * pt.x() + self.b_ * pt.y() + self.c_
        if val > tol:
            return 1
        elif val < -tol:
            return -1
        else:
            return 0

    # Determines if the two points along the lie on the same line

    # and if yes, what is their relative position.
    def get_relative_location_points(self, pt1, pt2, tol=TOL):
        '''
            returns: 0 is pt1 and pt2 donot lie on the line self
                         : 1 if pt2 is along self from pt1
                         :-1 if pt2 is the direction opposite of l1 from pt1
            Basically, we check
            x1 + \lamda l = x2
            => \lamda l   = x2 - x1 (where \lamda is a scalar constant)
        '''
        ptDir = pt2 - pt1
        ptDir.make_unit_norm()
        lDir = self.get_direction()
        cos = ptDir.cosine(lDir)
        # print "ptDir: %f, lDir: %f, cos: %f" % (ptDir.mag(), lDir.mag(), cos)
        if (cos > 1 - tol) and (cos < 1 + tol):
            return 1
        elif (cos > -1 - tol) and (cos < -1 + tol):
            return -1
        else:
            return 0

    # Determines if the point lies on the line segment
    def is_on_segment(self, pt, tol=TOL):
        d1 = self.st_.distance(pt)
        d2 = self.en_.distance(pt)
        d3 = self.st_.distance(self.en_)
        dSum = d1 + d2
        if (dSum <= d3 + tol) and (dSum >= d3 - tol):
            return True
        else:
            return False

    # Get a point along the line
    def get_point_along_line(self, pt, distance):
        lDir = self.get_direction()
        lDir.scale(distance)
        return pt + lDir

    # Intersection of two lines
    def get_intersection(self, l2):
        '''
            Point of intersection, y = (a2c1 - a1c2)/(a1b2 - a2b1)
            nr = a2c1 - a1c2
            dr = a1b2 - a2b1
        '''
        nr = l2.a() * self.c_ - self.a_ * l2.c()
        dr = self.a_ * l2.b() - l2.a() * self.b_
        # Parallel lines
        if dr == 0:
            return None
        else:
            y = nr / dr
            if self.a_ == 0:
                x = -(l2.c() + l2.b() * y) / l2.a()
            else:
                x = -(self.c_ + self.b_ * y) / self.a_
        return Point(x, y)

    # Get intersection with a line ray
    def get_intersection_ray(self, l2):
        '''
            l2 is the ray
        '''
        pt = self.get_intersection(l2)
        if pt is not None:
            relLoc = l2.get_relative_location_points(l2.st(), pt)
            # print pt, relLoc
            if relLoc != 1:
                pt = None
        return pt


##
# Circle
class Circle:
    def __init__(self, radius=20, center=Point(0, 0)):
        self.r_ = radius
        self.c_ = center

    # Given a line l, find the line parallel to l
    # that is tangent to the circle and find the
    # point of contact of this line with the circl
    def get_contact_point_pseudo_tangent(self, l):
        # Get the direction of the line from l to the center of the circle
        lToCDir = l.get_normal()
        # Scale to the radius
        lToCDir.scale(self.r_)
        # Get point of contact
        pt = self.c_ + lToCDir
        # Get equation of radius
        lr = Line(self.c_, pt)
        # Find if this line ray intersects the original line
        intersectPoint = l.get_intersection_ray(lr)
        if intersectPoint is not None:
            return pt
        else:
            # The point could be on the opposite direction
            lToCDir.scale(-1.0)
            pt = self.c_ + lToCDir
            lr = Line(self.c_, pt)
            intersectPoint = l.get_intersection_ray(lr)
            assert intersectPoint is not None
            return pt

    # find if the circle intersects with a line.
    def is_intersect_line(self, l):
        dist = np.abs(l.distance_to_point(self.c_))
        # print dist
        if dist <= self.r_:
            return True
        else:
            return False

    def intersect_moving_circle(self, circ2, v21):
        '''
            v21: velocity of 2 wrt 1
        '''
        badResult = np.inf, None, None, None
        p1, r1 = self.c_, self.r_
        p2, r2 = circ2.c_, circ2.r_
        p11 = p1 - p1
        p21 = p2 - p1
        R = r1 + r2
        dBall = p21.distance(p11)
        # print "Ball Positions", p11, p21, dBall
        if dBall >= (R - 0.1) and dBall <= R:
            dBall = (R - dBall) + dBall + 1e-6
        assert dBall >= R, "dball: %f, R: %f" % (dBall, R)
        # Make a big circle and determine if there will be a collision
        bigC = Circle(R, p11)
        isIntersect = bigC.is_intersect_line(Line(p21, p21 + v21))
        if not isIntersect:
            return badResult

        # Now intersection is guaranteed to happen
        # We have two sides of a triangle R, dBall and a third angle,
        # made by the velocity vector. We will use this to solve
        # for the point of contact.
        # Get the angle
        c1c2 = p21 - p11
        c1c2.scale(-1)
        '''
        thetaCenters = c1c2.get_angle(isRadian=True) 
        theta = v21.get_angle(isRadian=True)
        theta = thetaCenters - theta
        if theta < 0:
            theta = np.pi - np.abs(theta)
        print theta
        '''
        theta = c1c2.get_angle_between(v21)
        if np.abs(theta) > np.pi / 2:
            pdb.set_trace()
        assert np.abs(theta) <= np.pi / 2
        theta = np.abs(theta)
        if theta == 0 or theta == np.pi / 2:
            dist = dBall - R
        else:
            sinC = max(-1.0, min(1.0, dBall / (R / np.sin(theta))))
            thetaC = np.pi - math.asin(sinC)
            thetaA = np.pi - (thetaC + theta)
            # print "Theta:", theta, "thetaA:", thetaA, "thetaC:", thetaC
            assert thetaA >= 0
            dist = (R / np.sin(theta)) * np.sin(thetaA)

        speed = v21.mag()
        # print "Distance between balls: ", dist, "speed: ", speed
        # Get time to collision
        if speed == 0:
            return badResulst
        vDir = Point.from_self(v21)
        vDir.make_unit_norm()
        tCol = dist / speed

        # Get the new center after moving the circle.
        newP2 = p21 + dist * vDir
        # Line joining the centers
        # lCenters = Line(p11, newP2)
        lCenters = Line(newP2, p11)
        colNrml = lCenters.get_normal()
        newP2 = newP2 + p1
        return tCol, newP2, colNrml, dist


##
# Note this not specifically a rectangular BBox. It can be in general be 
# of any shape. 
class Bbox:
    def __init__(self, lTop, lBot, rBot, rTop):
        '''
            lTop,..rTop: of Type Point
        '''
        self.vert_ = []
        self.vert_.append(lTop)
        self.vert_.append(lBot)
        self.vert_.append(rBot)
        self.vert_.append(rTop)
        self.N_ = len(self.vert_)
        self.lines_ = []
        self.make_lines()

    @classmethod
    def from_list(cls, pts):
        pts = copy.deepcopy(pts)
        assert len(pts) == 4
        self = cls(pts[0], pts[1], pts[2], pts[3])
        return self

    # offset the bbox
    def move(self, offset):
        for i, vertex in enumerate(self.vert_):
            self.vert_[i] = vertex + offset
        self.make_lines()

    # Make lines
    def make_lines(self):
        self.lines_ = []
        for i in range(self.N_):
            self.lines_.append(Line(self.vert_[i], self.vert_[np.mod(i + 1, self.N_)]))

    def get_lines(self):
        return self.lines_

    # Determine if a point is inside the box or not
    def is_point_inside(self, pt):
        assert len(self.vert_) == 4, 'Only works for rectangles'
        # If the lines are anticlockwise
        inside = True
        for l in self.lines_:
            inLine = l.get_point_location(pt)
            inside = inside and inLine >= 0
        # Clockwise lines
        clInside = True
        for l in self.lines_:
            inLine = l.get_point_location(pt)
            clInside = clInside and inLine <= 0
        return (inside or clInside)

    # Determine if the bbox intersects with los(Line of Sight)
    def is_intersect_line(self, los):
        s = []
        isIntersect = True
        for i, v in enumerate(self.vert_):
            s.append(los.get_point_location(v))
            if i > 0:
                isIntersect = isIntersect and s[i] == s[i - 1]
        isIntersect = not (isIntersect)
        return isIntersect

    # Determine if the bbox intersects with los that is a ray
    def is_intersect_line_ray(self, los):
        intPoint, dist = self.get_intersection_with_line_ray(los)
        if intPoint is not None:
            return True
        else:
            return False

    # Find closest point
    def find_closest_interior_point(self, srcPt, pts, getIndex=False):
        '''
            from a list of Points (pts), find the point that is closest
            to srcPt and is inside the Bbox
            if all points are outside None is returned.
        '''
        intPoint = None
        dist = np.inf
        idx = None
        for i, pt in enumerate(pts):
            # No Intersection
            if pt is None:
                continue
            # Point of intersection is outside the bbox
            if not self.is_point_inside(pt):
                continue
            distTmp = srcPt.distance(pt)
            if distTmp < dist:
                intPoint = pt
                dist = distTmp
                idx = i

        if getIndex:
            return intPoint, dist, idx
        else:
            return intPoint, dist

    # Point of intersection which is closest to the
    # starting point of the line.
    def get_intersection_with_line(self, l):
        '''
            Note this function considers l as a line and not a line segment
            If a line intersects, it is not necessary that a line segment will
            also intersect.
        '''
        pts = []
        boxLines = []
        for i, v in enumerate(self.vert_):
            boxLine = Line(v, self.vert_[np.mod(i + 1, self.N_)])
            pts.append(l.get_intersection(boxLine))
            boxLines.append(boxLine)
        intPoint, dist, idx = self.find_closest_interior_point(l.st(), pts, getIndex=True)
        return intPoint, dist

    # Find which line is intersected first by the ray
    def get_line_of_first_intersection_with_ray(self, l):
        pass

    # Point of intersection which is closest to the
    # starting point of the line ray.
    def get_intersection_with_line_ray(self, l):
        '''
            Note this function considers l as a line ray and not as line segment/line
        '''
        pts = []
        for i, v in enumerate(self.vert_):
            pts.append(Line(v, self.vert_[np.mod(i + 1, self.N_)]).get_intersection_ray(l))
        return self.find_closest_interior_point(l.st(), pts)

    # Get time of collision with another bounding box.
    def get_toc_with_bbox(self, bbox, vel):
        '''
            self: is assumed to be stationary
            bbox: the other boundig bbox
            vel:  the velocity vector of bbox in frame of reference of self.
        '''
        raise Exception('This function is not ready')
        pts = []
        pts.append(self.get_intersection_with_line(gm.Line(bbox.lTop_, bbox.lTop_ + vel))[0])
        pts.append(self.get_intersection_with_line(gm.Line(bbox.lBot_, bbox.lBot_ + vel))[0])
        pts.append(self.get_intersection_with_line(gm.Line(bbox.rBot_, bbox.rBot_ + vel))[0])
        pts.append(self.get_intersection_with_line(gm.Line(bbox.rTop_, bbox.rTop_ + vel))[0])
