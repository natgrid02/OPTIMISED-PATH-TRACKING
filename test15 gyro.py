from .pathSegment import PathSegment
from .path import Path
from .pursuitController import PursuitController
#!/usr/bin/python3

from math import sqrt


class PathSegment():
    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.dx = x2 - x1
        self.dy = y2 - y1

        self.calculateStandard()
        self.length = self.getDistance(x1, y1, x2, y2)

    def calculateStandard(self):
        """
        Calculate the standard form for the line segment 
        (i.e. ax + by + c = 0)
        """
        self.a = self.y1 - self.y2
        self.b = self.x2 - self.x1
        self.c = -(self.a * self.x1 + self.b * self.y1)

    @staticmethod
    def getDistance(x1, y1, x2, y2):
        """
        Calculate the distance between two points
        """
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def dotProduct(self, x, y):
        """
        Calculate the dot product of vectors (x-x1, y-y1) and (x2-x1, y2-y1)
        """
        dx = x - self.x1
        dy = y - self.y1
        return self.dx * dx + self.dy * dy

    def findClosestPoint(self, xv, yv):
        """
        Find point on path segment closest to vehicle
        https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        """
        den = self.a**2 + self.b**2
        x = (self.b * (self.b * xv - self.a * yv) - self.a * self.c) / den
        y = (self.a * (-self.b * xv + self.a * yv) - self.b * self.c) / den

        # Calculate the percentage completed of the segment
        index = self.dotProduct(x, y) / (self.length**2)

        return (x, y, index)

    def findCircleIntersection(self, x, y, radius):
        """
        Find intersection between path segment and a circle. 
        (x, y) = closest point on path to vehicle, 
        radius = lookahead distance
        http://mathworld.wolfram.com/Circle-LineIntersection.html
        """
        x1 = self.x1 - x
        y1 = self.y1 - y
        x2 = self.x2 - x
        y2 = self.y2 - y
        dx = x2 - x1
        dy = y2 - y1
        dr2 = dx * dx + dy * dy
        det = x1 * y2 - x2 * y1

        discrim = dr2 * radius * radius - det * det
        if (discrim >= 0):
            sqrtDiscrim = sqrt(discrim)
            sign = -1 if (dy < 0) else 1

            posX = (det * dy + sign * dx * sqrtDiscrim) / dr2 + x
            posY = (-det * dx + abs(dy) * sqrtDiscrim) / dr2 + y
            negX = (det * dy - sign * dx * sqrtDiscrim) / dr2 + x
            negY = (-det * dx - abs(dy) * sqrtDiscrim) / dr2 + y

            posDot = self.dotProduct(posX, posY)
            negDot = self.dotProduct(negX, negY)

            # Return the point on the segment closest to the end
            if (posDot < 0 and negDot >= 0):
                return (negX, negY)
            elif (posDot >= 0 and negDot < 0):
                return (posX, posY)
            else:
                dPos = PathSegment.getDistance(self.x2, self.y2, posX, posY)
                dNeg = PathSegment.getDistance(self.x2, self.y2, negX, negY)
                if (dPos < dNeg):
                    return (posX, posY)
                else:
                    return (negX, negY)

        else:
            return (None, None)

#!/usr/bin/python3

from pursuit import PathSegment


class Path():

    COMPLETION_TOLERANCE = 0.98

    segments = []

    def addSegment(self, segment):
        self.segments.append(segment)

    def findGoalPoint(self, xv, yv, lookahead):
        """
        Find the goal point on the path for the current vehicle position
        """
        while (len(self.segments) > 0):
            segment = self.segments[0]

            # Find first path segment where d >= lookahead
            d = PathSegment.getDistance(xv, yv, segment.x2, segment.y2)
            if (d >= lookahead):
                # Find closest point on path segment
                (x, y, index) = segment.findClosestPoint(xv, yv)

                # Check if vehicle closer to next segment
                (x, y, index, segment) = self.checkNextSegment(xv, yv, x, y, index, segment)

                # Check if segment is complete
                self.checkSegmentComplete(index)

                # print('Closest (x, y) = (' + str(x) + ', ' + str(y) + ') Index = ' + str(index))

                # Find intersection between path and circle with radius = lookahead
                return segment.findCircleIntersection(x, y, lookahead)

            # If last segment, return the end of the segment
            elif (len(self.segments) == 1):
                # Find closest point on path segment
                (x, y, index) = segment.findClosestPoint(xv, yv)

                # Check if segment is complete
                self.checkSegmentComplete(index)

                # Goal point is the end of the segment
                return (segment.x2, segment.y2)

            # If too close to end of path, move to next segment
            else:
                print('Distance to path < lookahead. Removing segment.')
                self.segments.pop(0)

        # Done when all segments are removed
        return (None, None)

    def checkNextSegment(self, xv, yv, x, y, index, segment):
        """
        Check if the next segment is closer to the vehicle
        """
        if (len(self.segments) > 1):
            (x2, y2, index2) = self.segments[1].findClosestPoint(xv, yv)
            d1 = PathSegment.getDistance(xv, yv, x, y)
            d2 = PathSegment.getDistance(xv, yv, x2, y2)

            # Next segment is closer, so remove previous segment
            if (d2 <= d1 and index2 > 0):
                segment = self.segments[1]
                (x, y, index) = (x2, y2, index2)

                self.segments.pop(0)
                print('Next Segment is closer. Removing previous segment.')

        return (x, y, index, segment)

    def checkSegmentComplete(self, index):
        if (index >= self.COMPLETION_TOLERANCE):
            self.segments.pop(0)
            print('Segment Complete')

#!/usr/bin/python3

from math import cos, sin, radians
from control import VelocityController


class PursuitController():

    # Distance between the wheels of the vehicle
    WHEELBASE = 7
    # Ratio of velocity PID controller updates to pursuit calculations
    PID_FREQUENCY = 2

    pidIndex = 0
    done = False

    def __init__(self, path, lookahead, velocity):
        self.path = path
        self.lookahead = lookahead
        self.velocity = velocity
        self.left = VelocityController()
        self.right = VelocityController()

    def calculate(self, x, y, heading, leftVelocity, rightVelocity):
        """
        Calculate the left/right motor outputs using velocity controllers
        """
        print('')
        print('Pose = (' + str(x) + ', ' + str(y) + ')')

        left = 0
        right = 0
        # Update output velocity using pure pursuit
        if (self.pidIndex % self.PID_FREQUENCY == 0):
            (l, r) = self.update(x, y, heading)
            left = self.left.calculate(leftVelocity, l)
            right = self.right.calculate(rightVelocity, r)
            self.pidIndex = 1

        # Let velocity controller update to reach target velocity
        else:
            left = self.left.calculate(leftVelocity)
            right = self.right.calculate(rightVelocity)
            self.pidIndex += 1

        return (left, right)

    def update(self, xv, yv, heading):
        """
        Calculate the desired left/right linear velocity using pure pursuit
        https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
        http://www8.cs.umu.se/research/ifor/IFORnav/reports/rapport_MartinL.pdf
        """
        # Find goal point on path
        (xg, yg) = self.path.findGoalPoint(xv, yv, self.lookahead)
        print('(xg, yg) = (' + str(xg) + ', ' + str(yg) + ')')

        if (xg is not None and yg is not None):
            # Transform goal point relative to vehicle heading
            (xgv, ygv) = self.transformToVehicle(xv, yv, xg, yg, heading)
            print('(xgv, ygv) = (' + str(xgv) + ', ' + str(ygv) + ')')

            # Calculate left/right wheel velocity based on goal point
            return self.outputKinematics(xgv, ygv)

        else:
            self.done = True

        return (0, 0)

    def transformToVehicle(self, xv, yv, xg, yg, heading):
        """
        Transform goal point to coordinates relative to vehicle's current heading
        (i.e. treat vehicle as (0, 0) with heading as the y-axis)
        """
        heading = radians(heading)
        s = round(sin(heading), 5)
        c = round(cos(heading), 5)
        xgv = (xg - xv) * s - (yg - yv) * c
        ygv = (xg - xv) * c + (yg - yv) * s
        return (xgv, ygv)

    def outputKinematics(self, xgv, ygv):
        """
        Calculate left/right velocity based on the transformed goal point
        """
        d2 = xgv**2 + ygv**2
        deltaV = 0
        if (xgv != 0):
            radius = d2 / (2 * xgv)
            omega = self.velocity / radius
            deltaV = omega * self.WHEELBASE

        leftVelocity = 0
        rightVelocity = 0
        if (ygv >= 0):
            leftVelocity = self.velocity + deltaV
            rightVelocity = self.velocity - deltaV

        print('Velocity = (' + str(leftVelocity) + ', ' + str(rightVelocity) + ')')
        return (leftVelocity, rightVelocity)
