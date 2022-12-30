from enum import Enum
from operator import is_
from statistics import mean
import sys
from tkinter import RIGHT
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from typing import Tuple, List
import itertools

CELLROWS = 7
CELLCOLS = 14


class bcolors:
    PINK = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    END = '\033[0m'


class Rotation(Enum):
    LEFT = 0
    RIGHT = 1
    NONE = 2
    BACK = 3


class Direction(int, Enum):
    EAST = (0, 0, (+1, 0))
    NORTH = (1, 90, (0, +1))
    WEST = (2, 180, (-1, 0))
    SOUTH = (3, -90, (0, -1))

    def __new__(cls, value, angle, next):
        obj = int.__new__(cls)
        obj._value_ = value
        obj.angle = angle
        obj.next = next
        return obj

    def __eq__(self, other):
        if isinstance(other, int):
            return self.value == other
        if isinstance(other, Enum):
            return self.value == other.value
        return False


class Mapper:
    size = (49, 21)

    def __init__(self, start: Tuple[int, int], nbeacons: int) -> 'Mapper':
        self.start = start
        self.labMap = [['.']*self.size[0] for _ in range(self.size[1])]
        self.labMap[self.size[1] // 2][self.size[0] // 2] = 'I'
        self.checkpoints = {}
        self.nbeacons = nbeacons

    def print_map(self) -> None:
        for row in reversed(self.labMap):
            print(''.join(row).replace('.', '\033[104m.\033[0m'))
        print('\n' * 2)

    def check(self, x, y):
        return 0 <= x < self.size[0] and 0 <= y < self.size[1]

    def map_point(self, x, y):
        return (round(x - self.start[0]) + self.size[0] // 2,
                round(y - self.start[1]) + self.size[1] // 2)

    def visit_ground(self, x: int, y: int, ground: int) -> None:
        x, y = self.map_point(x, y)
        self.checkpoints[ground] = (x, y)

    def explore_inter(self, x: int, y: int, angle: float, line: List[int]) -> None:
        print('EXPLORE1', x, y)
        x, y = self.map_point(x, y)
        print('EXPLORE2', x, y)

        m = 1 if angle < 2 else -1
        dir = angle % 2
        self.labMap[y][x] = 'x'

        dy, dx = -dir * m, abs(dir - 1) * m
        d = [
            (y - dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
            (y + dx, x + dy),    # dyl = dxf, dxl = dyf
            (y - dx, x - dy),    # dyr = -dxf, dxr = -dyf
        ]

        if not any(line):
            py, px = d[0]
            self.labMap[py][px] = '.'

        # Connect previous intersection with new discovered intersection
        paint = False
        dir = ['-', '|'][angle % 2]
        i = 1
        ii = None
        while True:
            py, px = y + dy*i, x - dx*i
            if paint:
                print(py, px)
                if i < 1:
                    break
                self.labMap[py][px] = dir
                i -= 1
            else:
                if not self.check(px, py):
                    if ii != None:
                        paint = True
                        i = ii
                        continue
                    break
                if self.labMap[py][px] == 'I':
                    ii = i - 1
                if self.labMap[py][px] == 'x':
                    paint = True
                    i -= 1
                    continue
                i += 1

        if all(line[4:]):        # right
            py, px = d[2]
            if self.check(px, py) and self.labMap[py][px] == '.':
                self.labMap[py][px] = "*"
            # remove unnecessary *
            py2, px2 = y - 2*dx, x - 2*dy
            if self.check(px2, py2) and self.labMap[py2][px2] == 'x':
                dir = ['|', '-'][angle % 2]
                self.labMap[py][px] = dir

        if all(line[:3]):        # left
            py, px = d[1]
            if self.check(px, py) and self.labMap[py][px] == '.':
                self.labMap[py][px] = "*"
            # remove unnecessary *
            py2, px2 = y + 2*dx, x + 2*dy
            if self.check(px2, py2) and self.labMap[py2][px2] == 'x':
                dir = ['|', '-'][angle % 2]
                self.labMap[py][px] = dir

        self.print_map()

    def make_decision(self, x: int, y: int, angle: float, line: List[int]) -> None:
        print('DECISION1', x, y)
        x, y = self.map_point(x, y)
        print('DECISION2', x, y)

        m = 1 if angle < 2 else -1
        dir = angle % 2

        dy, dx = -dir * m, abs(dir - 1) * m
        d = [
            (y - dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
            (y + dx, x + dy),    # dyl = dxf, dxl = dyf
            (y - dx, x - dy),    # dyr = -dxf, dxr = -dyf
        ]

        if any(line[2:5]):
            py, px = d[0]
            if self.check(px, py) and self.labMap[py][px] == '.':
                self.labMap[py][px] = "*"
                # remove unnecessary *
                py2, px2 = y - 2*dy, x + 2*dx
                if self.check(px2, py2) and self.labMap[py2][px2] == 'x':
                    dir = ['-', '|'][angle % 2]
                    self.labMap[py][px] = dir
                self.print_map()

        py, px = d[2]
        if self.check(px, py) and self.labMap[py][px] == '*':
            return Rotation.RIGHT
        py, px = d[1]
        if self.check(px, py) and self.labMap[py][px] == '*':
            return Rotation.LEFT
        py, px = d[0]
        if self.check(px, py) and self.labMap[py][px] == '*':
            return Rotation.NONE
        return Rotation.BACK

    def save_map_to_file(self):
        with open('simulator/solution.map', 'w') as fp:
            for y in reversed(range(self.size[1])):
                for x in range(self.size[0]):
                    for b, (bx, by) in self.checkpoints.items():
                        print((x, y) , (bx, by), (x, y) == (bx, by))
                        if (x, y) == (bx, by):
                            fp.write(str(b))
                            break
                    else:
                        if y % 2 == x % 2:
                            fp.write(' ')
                        else:
                            fp.write(self.labMap[y][x].replace('.', ' '))
                fp.write('\n')

    def save_path_to_file(self):
        with open('simulator/solution.path', 'w') as fp:
            min_dist = None
            min_steps = []
            checkpoints = [v for k, v in self.checkpoints.items() if k != 0]
            print(self.checkpoints.items())
            for iteration in itertools.combinations(checkpoints, len(checkpoints)):
                iteration = list(iteration)
                iteration.append(self.checkpoints[0])
                iteration.insert(0, self.checkpoints[0])
                print(iteration, "\n"*10)

                curr_dist = 0
                curr_steps = [self.checkpoints[0]]
                for pos in range(len(iteration[:-1])):
                    cx, cy = iteration[pos]
                    gx, gy = iteration[pos + 1]
                    d, steps = a_distance(cx, cy, gx, gy, self)

                    curr_dist += d
                    curr_steps.extend(steps)

                if not min_dist or (curr_dist < min_dist and curr_dist != 0):
                    min_dist = curr_dist
                    min_steps = curr_steps
                print(curr_dist, curr_steps)

            print(min_steps)
            fx, fy = min_steps[0]
            for s in range(0, len(min_steps), 2):
                fp.write(str(min_steps[s][0] - fx) +
                         " " + str(min_steps[s][1] - fy) + "\n")


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.prev_x = self.prev_y = self.prev_a = None
        self.prev_measures = []
        self.prev_rPow = 0
        self.prev_lPow = 0
        self.prev_out = (0, 0)
        self.action = "moving"
        self.is_rotating_to: Direction = None
        self.has_plan = False

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not

    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        self.readSensors()
        self.goal = (self.measures.x, self.measures.y)
        self.gps_x, self.gps_y = self.goal
        self.last_x, self.last_y = None, None
        self.compass = self.measures.compass
        self.prev_x, self.prev_y = self.goal
        self.start = self.goal
        print("START", self.start)
        self.map = Mapper(self.start, self.nBeacons)
        self.is_rotating_to = Direction(round(
            (self.measures.compass + 360) / 90) % 4)

        try:
            while True:
                self.readSensors()

                if self.measures.endLed:
                    print(self.rob_name + " exiting")
                    quit()

                if state == 'stop' and self.measures.start:
                    state = stopped_state

                if state != 'stop' and self.measures.stop:
                    stopped_state = state
                    state = 'stop'

                if state == 'run':
                    if self.measures.visitingLed == True:
                        state = 'wait'
                    if self.measures.ground == 0:
                        self.setVisitingLed(True)
                    self.wander()
                elif state == 'wait':
                    self.setReturningLed(True)
                    if self.measures.visitingLed == True:
                        self.setVisitingLed(False)
                    if self.measures.returningLed == True:
                        state = 'return'
                    self.driveMotors(0.0, 0.0)
                elif state == 'return':
                    if self.measures.visitingLed == True:
                        self.setVisitingLed(False)
                    if self.measures.returningLed == True:
                        self.setReturningLed(False)
                    self.wander()
        except KeyboardInterrupt:
            self.map.save_map_to_file()
            self.map.save_path_to_file()
            quit()

    def update_measures(self):

        lin = mean(self.prev_out)
        #rot = self.prev_out[1] - self.prev_out[0]

        # TODO: compass vai ter noise
        #theta = self.measures.compass + rot

        self.gps_x += lin*cos(self.get_compass() * pi / 180)
        self.gps_y += lin*sin(self.get_compass() * pi / 180)

        rot = self.prev_out[1] - self.prev_out[0]
        self.compass += rot * 180 / pi
        self.compass %= 360
        if self.compass > 180:
            self.compass -= 360

    def reset_gps(self):
        x,y = self.__calculate_gps()
        print(bcolors.BOLD, bcolors.BLUE, self.last_x, self.last_y, bcolors.END)
        self.gps_x, self.gps_y = x, y
        self.last_x, self.last_y = x, y
        print(bcolors.BOLD, bcolors.BLUE, self.last_x, self.last_y, 'new', bcolors.END)

    def __calculate_gps(self):
        if self.last_x != None:
            print(self.is_rotating_to)
            if self.is_rotating_to in (Direction.EAST, Direction.WEST):
                x, y = (
                    self.last_x + round((self.gps_x - self.last_x) / 2) * 2, 
                    self.last_y)
            else:  
                x, y = (
                    self.last_x, 
                    self.last_y + round((self.gps_y - self.last_y) / 2) * 2)
        else:
            print(self.start)
            x, y = (
                self.start[0] + round(self.gps_x + .4*self.is_rotating_to.next[0] - self.start[0]),
                self.start[1] + round(self.gps_y + .4*self.is_rotating_to.next[1] - self.start[1])
            )
        return x, y

    def get_gps(self):
        # print(bcolors.RED, bcolors.BOLD, self.gps_x, self.gps_y,
        #       '==', self.measures.x, self.measures.y, bcolors.END)
        return self.gps_x, self.gps_y
        return self.measures.x, self.measures.y

    def relative_coord(self, point):
        return (point[0] - self.start[0], point[1] - self.start[1])

    def rounded_relative_coord(self, point):
        return (round(point[0] - self.start[0]), round(point[1] - self.start[1]))

    def get_measures(self):
        if len(self.prev_measures) > 2:
            self.prev_measures.pop(0)
        self.prev_measures.append([int(i) for i in self.measures.lineSensor])
        print(self.prev_measures)
        # majority vote for each line sensor point for the last measures
        return [int(sum(x) > len(self.prev_measures)/2) for x in zip(*self.prev_measures)]

    def get_compass(self):
        n1 = radians(self.compass)
        n2 = radians(self.measures.compass)
        # calculate median angle
        n = atan2(sin(n1) + sin(n2), cos(n1) + cos(n2))
        return degrees(n)

    def wander(self):
        cx, cy = self.get_gps()
        measures = self.get_measures()
        a = round((self.get_compass() + 360) / 90) % 4

        print('\n', bcolors.BOLD)
        print(measures)
        print(self.gps_x, self.gps_y)
        print(f"{self.get_compass():.1f}",
              f"{self.compass:.1f}", self.measures.compass, self.action, bcolors.END)

        if self.action == 'rotating':
            # Rotates the robot
            lPow, rPow = self.rotate()
            if self.action == 'stop':
                self.compass = self.is_rotating_to.angle
                self.prev_measures = []
                self.action = 'moving'

        elif self.action in ('moving', 'stopping', 'stop'):
            # Robot moves to the self.goal
            #
            # The action changes to stopping if it has just seen an
            # intersection and has to adjust its position to be
            # centered on the goal.
            #
            # When it is on top of the goal, the robot will stop
            # if there is any intersection it will choose which one to go

            lPow, rPow = self.__action_moving(cx, cy, a, measures)

            # Make robot visit next position and paint map
            # print(bcolors.BLUE, bcolors.BOLD, self.rounded_relative_coord((self.prev_x, self.prev_y)), self.rounded_relative_coord((self.gps_x, self.gps_y)), bcolors.END)
            # if self.rounded_relative_coord((self.prev_x, self.prev_y)) != self.rounded_relative_coord((self.gps_x, self.gps_y)):
            #     self.map.visit(cx, cy, self.measures.compass,
            #                    self.measures.ground)
            if self.measures.ground != -1 and self.close_to_cell_center(cx, cy):
                self.map.visit_ground(cx, cy, self.measures.ground)

        elif self.action in ("searching", "search_move", "search_rotate"):
            # Robot follow steps to reach the destination from search()
            #
            # It follows a list of positions. When it needs to rotate, the robot
            # calculates the angle based on the next position.
            #
            # This action when the robot has arrived to the last position in the list.

            lPow, rPow = self.__action_searching(cx, cy, a, measures)

        self.prev_x = cx
        self.prev_y = cy
        self.prev_out = (
            lPow + self.prev_out[0]) / 2, (rPow + self.prev_out[1]) / 2
        self.driveMotors(lPow, rPow)
        self.update_measures()
        print(bcolors.BOLD, "prev_out {:.3f} {:.3f} ".format(
            self.prev_out[0], self.prev_out[1]), "pow {:.3f} {:.3f}".format(lPow, rPow), bcolors.END)

    def __action_moving(self, cx, cy, a, measures) -> Tuple[float, float]:

        if not any(measures) and not self.action == 'stopping':
            # Explore dead end
            self.map.explore_inter(cx, cy, a, measures)
            self.action = "rotating"
            self.is_rotating_to = Direction((a + 2) % 4)

        if ((all(self.prev_measures[-1][:3]) or all(self.prev_measures[-1][4:]))
                and self.prev_measures[-1][3]) and not self.action == 'stopping':
            # Make robot visit intersection
            if measures != self.prev_measures[-1]:
                print(bcolors.RED, 'STOP1', bcolors.END)
                return -self.prev_out[0], -self.prev_out[1]
            px, py = self.__calculate_gps()
            self.map.explore_inter(px, py, a, measures)
            self.action = 'stopping'
            self.goal = px, py

        if self.action == 'stopping' and euclidean((cx, cy), self.goal) < 0.1:
            self.action = 'stop'

        # if not (measures[0] or measures[6]) and self.action == 'stopping':
        #     # Make robot finish intersection and stop
        #     if measures != self.prev_measures[-1]:
        #         print(bcolors.RED, 'STOP2', bcolors.END)
        #         return 0.01, 0.01
        #     self.action = 'stop'
        #     return -self.prev_out[0], -self.prev_out[1]

        if self.action not in ("stop"):
            lPow, rPow = self.follow(measures)

        if self.action == 'stop':
            self.reset_gps()
            print(bcolors.CYAN, self.measures.x, self.measures.y,
                  'is', self.gps_x, self.gps_y, bcolors.END)

            dir = self.map.make_decision(cx, cy, a, measures)

            self.action = "rotating"
            if dir == Rotation.LEFT:
                self.is_rotating_to = Direction((a + 1) % 4)
            elif dir == Rotation.RIGHT:
                self.is_rotating_to = Direction((a + 3) % 4)
            elif dir == Rotation.BACK:
                # TODO: idk if a needs to change before sending
                #self.steps = self.search()
                steps = self.search()
                print(bcolors.YELLOW, bcolors.BOLD, steps, bcolors.END)
                self.steps = self.convert_steps(steps)
                print(bcolors.YELLOW, bcolors.BOLD, self.steps, bcolors.END)
                self.action = 'searching'
                #self.is_rotating_to = Direction((a + 2) % 4)
            elif dir == Rotation.NONE:
                self.action = 'moving'
                self.is_rotating_to = Direction(a)
            lPow, rPow = 0, 0

        return lPow, rPow

    def close_to_cell_center(self, x, y):
        x -= self.start[0]
        y -= self.start[1]
        print(bcolors.YELLOW, bcolors.BOLD, x, y, abs(x - round(x))
              < 0.3 and abs(y - round(y)) < 0.3, bcolors.END)
        return abs(x - round(x)) < 0.3 and abs(y - round(y)) < 0.3

    def __action_searching(self, cx, cy, a, measures) -> Tuple[float, float]:
        # check if needs to rotate

        if len(self.steps) == 0 and not self.action == "search_rotate":
            self.action = "moving"
            # FIXME: wtf
            self.is_rotating_to = Direction(
                round((self.get_compass() + 360) / 90) % 4)
            return 0, 0

        if ((all(self.prev_measures[-1][:3]) or all(self.prev_measures[-1][4:]))
                and self.prev_measures[-1][3]) and not self.action == 'stopping':
            # Make robot visit intersection
            if measures != self.prev_measures[-1]:
                print(bcolors.RED, 'STOP1', bcolors.END)
                return -self.prev_out[0], -self.prev_out[1]
            self.goal = self.steps[0][1]

        if self.action == 'searching' and euclidean((cx, cy), self.steps[0][1]) < 0.1:
            self.action = "search_rotate"
            self.reset_gps()
            self.is_rotating_to, _ = self.steps.pop(0)
            self.goal = None

        if not self.action == "search_rotate":
            lPow, rPow = self.follow(measures)

        if self.action == 'search_rotate':
            # rotate
            print(bcolors.PINK, 'search_rotate', bcolors.END, end=' ')
            lPow, rPow = self.rotate()
            if self.action == 'stop':
                self.compass = self.is_rotating_to.angle
                print(bcolors.CYAN, self.measures.x, self.measures.y,
                      'is', self.gps_x, self.gps_y, bcolors.END)
                self.action = 'searching'

        return lPow, rPow

    def follow(self, measures) -> Tuple[float, float]:

        def follow_angle(angle: int, pow: int = 0.08) -> Tuple[float, float]:
            a2 = self.get_compass() * pi / 180
            a0 = angle - a2

            c = cos(a0)
            s = sin(a0)
            n1 = c - s
            n2 = c + s
            min_ = min(n1, n2, -1)
            rng_ = max(n1, n2, 1) - min_

            lPow = ((2*(n1 - min_) / rng_) - 1) * pow
            rPow = ((2*(n2 - min_) / rng_) - 1) * pow

            lPow = 2*lPow - self.prev_out[0]
            rPow = 2*rPow - self.prev_out[1]

            return lPow, rPow

        def follow_line(measures: List[int], pow: int = 0.08) -> Tuple[float, float]:
            w = [1, 0.2, 0.04]

            n1 = n2 = 0.12
            for i in range(0, 3):
                n2 += w[i]*measures[i]
                n1 -= w[i]*measures[i]
            for i in range(4, 7):
                n2 -= w[6-i]*measures[i]
                n1 += w[6-i]*measures[i]

            rng_ = max(abs(n1), abs(n2), 0.15)

            n1 = 2*((n1 + rng_) / (2*rng_)) - 1
            n2 = 2*((n2 + rng_) / (2*rng_)) - 1

            lPow = n1 * pow
            rPow = n2 * pow

            lPow = 2*lPow - self.prev_out[0]
            rPow = 2*rPow - self.prev_out[1]

            return lPow, rPow

        # measures = [int(i) for i in self.measures.lineSensor]
        # self.prev_measures = measures
        if len(self.prev_measures) > 1 and not self.action == 'stopping':
            measures = [(self.prev_measures[-2][i]*0.3 + self.prev_measures[-1][i]*0.7)
                        for i in range(7)]

        if self.action == 'stopping' or (self.action == 'searching' and self.goal):
            x2, y2 = self.goal
            x1, y1 = self.gps_x, self.gps_y
            dist = euclidean((x1, y1), self.goal)
            angle = atan2(y2 - y1, x2 - x1)
            return follow_angle(angle, min(dist, 0.08)) 

        if (any(measures[:2]) or any(measures[5:])) and measures[3]:
            print('follow_angle')
            # Detects line in one of the sides and in the center
            angle = round(self.get_compass() / 90) * pi / 2
            return follow_angle(angle)

            # Is moving through intersection
            x2, y2 = self.goal
            x1, y1 = self.gps_x, self.gps_y
            dist = euclidean((x1, y1), self.goal)

            # if x2 == x1 and y1 == y2:
            #     self.action = 'stop'
            #     return tuple(-a for a in self.prev_out)

            angle = atan2(y2 - y1, x2 - x1)
            return follow_angle(angle, min(dist, 0.15))

        if not (any(measures[:2]) or any(measures[5:])):
            print('follow_angle + follow_line')
            # Only detects line in the center
            lPow1, rPow1 = follow_line(measures)
            angle = round(self.get_compass() / 90) * pi / 2
            lPow2, rPow2 = follow_angle(angle)
            # print(bcolors.CYAN, "FOLLOW ANGLE", measures,  mean((lPow2, lPow2)), mean(
            #     (rPow2, rPow2)), self.get_compass(), round(self.get_compass() / 90) * 90, bcolors.END)
            return mean((lPow1, lPow2)), mean((rPow1, rPow2))

        print('follow_line')
        return follow_line(measures)

    def rotate(self) -> Tuple[float, float]:
        """
        Returns the values needed to reach the desired rotation.
        """
        angle_to = self.is_rotating_to.angle
        angle_from = self.get_compass()

        print(f"{bcolors.PINK}rot{bcolors.END} from {angle_from:.2f} to {angle_to:.2f}")

        rad_to = angle_to * pi / 180
        rad_from = angle_from * pi / 180

        a = angle_to - angle_from
        a += -360 if a > 180 else 360 if a < -180 else 0

        a = a / 180 * pi
        print(' a=', a)

        if abs(a) < 0.05:
            self.action = 'stop'
            return tuple(-a for a in self.prev_out)

        pwr = min(2*abs(a) - abs(self.prev_out[0] - self.prev_out[1]), 0.3)

        if sin(rad_from - rad_to) < 0:
            # rotate to right
            pwr *= -1

        return pwr/2, -pwr/2

    def search(self) -> List[Tuple[float, float]]:
        """Finds a new intersection to explore and returns the list of steps to reach it"""

        min_dist = None
        cx, cy = self.map.map_point(self.gps_x, self.gps_y)
        steps = []

        for i, row in enumerate(self.map.labMap):
            for j, col in enumerate(row):
                if col == '*':
                    d, s = a_distance(cx, cy, j, i, self.map)
                    if not min_dist or min_dist > d:
                        min_dist = d
                        steps = s

        if min_dist == None:
            self.map.save_map_to_file()
            self.map.save_path_to_file()
            quit()

        return [(cx, cy)] + steps

    def convert_steps(self, steps):
        # step list and the current angle of the robot
        rotations = []
        for s in range(len(steps)-1):
            x, y = steps[s]

            if self.map.labMap[y][x] == "x":
                nx, ny = steps[s + 1]

                if (nx, ny) == (x - 1, y):
                    a = Direction.WEST
                elif (nx, ny) == (x + 1, y):
                    a = Direction.EAST
                elif (nx, ny) == (x, y - 1):
                    a = Direction.SOUTH
                else:
                    a = Direction.NORTH
                print(bcolors.YELLOW, bcolors.BOLD,
                      f"appending rotation {a}", bcolors.END)

                rotations.append((a, (x - self.map.size[0] // 2 + self.start[0], y - self.map.size[1] // 2 + self.start[1])))

        return rotations


def a_distance(cx, cy, gx, gy, map) -> Tuple[float, List[Tuple[float, float]]]:
    """Returns the a* distance between the current position and the goal"""

    class Node:
        def __init__(self, parent=None, position=None, distance=0):
            self.parent = parent
            self.position = position
            self.distance = distance

    queue: List[Node] = [Node(position=(cx, cy))]
    visited: List[Node] = [Node(position=(cx, cy))]

    print(cx, cy, gx, gy)

    while queue:
        n: Node = queue.pop(0)
        d, x, y = n.distance, *n.position

        if y == gy and x == gx:
            final = []
            while n.parent != None:
                final.append(n.position)
                n = n.parent
            return d, list(reversed(final))

        # NORTH
        if map.check(x, y + 1) and map.labMap[y + 1][x] in ("-", "|", "x", "*") and (x, y + 1) not in visited:
            # TODO: change distance to take into consideration the rotations
            queue.append(Node(parent=n, position=(x, y + 1), distance=d+1))
            visited.append((x, y + 1))
        # SOUTH
        if map.check(x, y - 1) and map.labMap[y - 1][x] in ("-", "|", "x", "*") and (x, y - 1) not in visited:
            # TODO: change distance to take into consideration the rotations
            queue.append(Node(parent=n, position=(x, y - 1), distance=d+1))
            visited.append((x, y - 1))
        # EAST
        if map.check(x + 1, y) and map.labMap[y][x + 1] in ("-", "|", "x", "*") and (x + 1, y) not in visited:
            # TODO: change distance to take into consideration the rotations
            queue.append(Node(parent=n, position=(x + 1, y), distance=d+1))
            visited.append((x + 1, y))
        # WEST
        if map.check(x - 1, y) and map.labMap[y][x - 1] in ("-", "|", "x", "*") and (x - 1, y + 1) not in visited:
            # TODO: change distance to take into consideration the rotations
            queue.append(Node(parent=n, position=(x - 1, y), distance=d+1))
            visited.append((x - 1, y))
        queue.sort(key=lambda x: x.distance)
    # FIXME: somehow it reached here and resulted in an error
    print(bcolors.RED, bcolors.BOLD, "WARNING", bcolors.END)
    return 0, []


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()

        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1)]
        i = 1
        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # this line defines vertical lines
                for c in range(len(line)):
                    if (c+1) % 3 == 0:
                        if line[c] == '|':
                            self.labMap[row][(c+1)//3*2-1] = '|'
                        else:
                            None
            else:  # this line defines horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        if line[c] == '-':
                            self.labMap[row][c//3*2] = '-'
                        else:
                            None
            i = i+1


def euclidean(start, end):
    return sqrt(abs(start[0] - end[0])**2 + abs(start[1] - end[1])**2)


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv), 2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        ...
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()

    rob.run()
