from enum import Enum
from operator import is_
from statistics import mean
import sys
from tkinter import RIGHT
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from typing import Tuple, List

CELLROWS = 7
CELLCOLS = 14


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


class Mapper:
    size = (49, 21)

    def __init__(self, start: Tuple[int, int]) -> 'Mapper':
        self.start = start
        self.labMap = [['.']*self.size[0] for _ in range(self.size[1])]

    def print_map(self) -> None:
        for row in reversed(self.labMap):
            print(''.join(row).replace('.', '\033[104m.\033[0m'))
        print('\n' * 2)

    def visit(self, x: int, y: int, angle: float) -> None:
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)
        print('visit', x, y)

        dir = ['-', '|'][round(angle / 90) % 2]
        if self.labMap[y][x] != 'x':
            self.labMap[y][x] = dir

    def explore_inter(self, x: int, y: int, angle: float, line: List[int]) -> None:
        print('explore_inter', x, y, line)
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)

        print(angle)
        m = 1 if angle < 2 else -1
        dir = angle % 2
        self.labMap[y][x] = 'x'

        dy, dx = -dir * m, abs(dir - 1) * m
        d = [
            (y - dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
            (y + dx, x + dy),    # dyl = dxf, dxl = dyf
            (y - dx, x - dy),    # dyr = -dxf, dxr = -dyf
        ]

        print(d, y, x, line)

        if all(line[4:]):        # right
            py, px = d[2][0], d[2][1]
            print("rigth", self.labMap[py][px])
            if self.labMap[py][px] == '.':
                self.labMap[py][px] = "*"

        if all(line[:3]):        # left
            py, px = d[1][0], d[1][1]
            print("left", self.labMap[py][px])
            if self.labMap[py][px] == '.':
                self.labMap[py][px] = "*"

        self.print_map()

    def make_decision(self, x: int, y: int, angle: float, line: List[int]) -> None:
        print('make_decision', x, y, line)
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)

        print(angle)
        m = 1 if angle < 2 else -1
        dir = angle % 2
        self.labMap[y][x] = 'x'

        dy, dx = -dir * m, abs(dir - 1) * m
        d = [
            (y - dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
            (y + dx, x + dy),    # dyl = dxf, dxl = dyf
            (y - dx, x - dy),    # dyr = -dxf, dxr = -dyf
        ]

        print(d, y, x, line)

        if line[3]:
            print("a fazer esta merda, doubt")
            py, px = d[0][0], d[0][1]
            if self.labMap[py][px] == '.':
                self.labMap[py][px] = "*"
                self.print_map()

        py, px = d[2][0], d[2][1]
        print("rigth", self.labMap[py][px])
        if self.labMap[py][px] == '*':
            return Rotation.RIGHT

        py, px = d[1][0], d[1][1]
        print("left", self.labMap[py][px])
        if self.labMap[py][px] == '*':
            return Rotation.LEFT

        py, px = d[0][0], d[0][1]
        print("front", self.labMap[py][px])
        if self.labMap[py][px] == '*':
            return Rotation.NONE

        return Rotation.BACK


    def save_to_file(self):
        with open('simulator/solution.map', 'w') as fp:
            for y in reversed(range(self.size[1])):
                for x in range(self.size[0]):
                    if y == 10 and x == 24:
                        fp.write('I')
                    elif y % 2 == x % 2:
                        fp.write(' ')
                    else:
                        fp.write(self.labMap[y][x].replace('.', ' '))
                fp.write('\n')


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.prev_measures = [0.5]*7
        self.prev_x = self.prev_y = self.prev_a = None
        self.prev_rPow = 0
        self.prev_lPow = 0

        self.action = "moving"

        self.is_rotating_to: Direction = None

        self.prev_out = (0, 0)

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
        self.start = self.goal
        print(self.goal)
        self.map = Mapper(self.goal)
        self.prev_out = (0, 0)
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
            self.map.save_to_file()

    def wander(self):

        measures = [int(i) for i in self.measures.lineSensor]
        x, y, a = round(self.measures.x), round(self.measures.y), round(
            (self.measures.compass + 360) / 90) % 4

        print('\n', self.action, self.measures.x,
              self.measures.y, self.prev_out, self.measures.compass)

        if self.action == 'rotating':
            # Rotates the robot
            lPow, rPow = self.rotate()
            if lPow == 0 and rPow == 0:
                self.action = 'moving'
                # self.is_rotating_to = None

        elif self.action in ('moving', 'stopping'):

            # Make robot visit intersection
            # if all(measures[:3]) or all(measures[:4]):
            print(measures)
            if (all(measures[:3]) or all(measures[4:])) and not self.action == 'stopping':  # FIXME: noise
                self.map.explore_inter(
                    self.measures.x + .3*self.is_rotating_to.next[0], 
                    self.measures.y + .3*self.is_rotating_to.next[1], 
                    a, measures)
                print(self.goal, 'stoooooooooooooooooooooop ', end='')

                self.goal = (
                    self.start[0] + round(self.measures.x -
                                          self.start[0] + .3*self.is_rotating_to.next[0]),
                    self.start[1] + round(self.measures.y -
                                          self.start[1] + .3*self.is_rotating_to.next[1])
                )
                print(self.goal)
                self.action = 'stopping'
                print('STOPPING')

            # Move to goal
            lPow, rPow = self.move_to()

            # print(self.measures.lineSensor)
            if self.action == 'stop':
                dir = self.map.make_decision(  # TODO: meter mais
                    self.measures.x, 
                    self.measures.y, 
                    a, measures)

                # self.goal = round(self.measures.x), round(self.measures.y)
                self.action = "rotating"
                if dir == Rotation.LEFT:
                    self.is_rotating_to = Direction((a + 1) % 4)
                elif dir == Rotation.RIGHT:
                    self.is_rotating_to = Direction((a + 3) % 4)
                elif dir == Rotation.BACK:
                    # TODO: A*
                    # self.steps = self.search()
                    # self.action = 'searching'
                    # self.is_rotating_to = None
                    # self.goal = None
                    self.is_rotating_to = Direction((a + 2) % 4)
                elif dir == Rotation.NONE:
                    self.action = 'moving'
                    self.is_rotating_to = Direction(a)
                print('STOP')

            if not self.action in ('stopping', 'stop') and euclidean((self.measures.x, self.measures.y), self.goal) < 0.3:
                print("UPDATE GOAL")
                self.goal = (
                    self.goal[0] + 2 * self.is_rotating_to.next[0],
                    self.goal[1] + 2 * self.is_rotating_to.next[1]
                )

            # Make robot visit next position and paint map
            if self.prev_x and self.prev_x != x or self.prev_y != y:
                self.map.visit(self.measures.x, self.measures.y,
                               self.measures.compass)


        elif self.action == "searching":

            if self.goal and self.measures.x == self.goal[0] and self.measures.y == self.goal[1]:
                self.steps.pop(0)
                self.goal = None
            
            x = round(self.measures.x - self.start[0]) + int(self.map.size[0] / 2)
            y = round(self.measures.y - self.start[1]) + int(self.map.size[1] / 2)
            # check if needs to rotate
            m = 1 if a < 2 else -1
            dir = a % 2

            dy, dx = -dir * m, abs(dir - 1) * m
            d = [
                (y - dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
                (y + dx, x + dy),    # dyl = dxf, dxl = dyf
                (y - dx, x - dy),    # dyr = -dxf, dxr = -dyf
            ]
            print(self.steps)
            if len(self.steps) == 0:
                self.action = "moving"
                self.goal = x + dx + self.start[0] - int(self.map.size[0] / 2), y - dy + self.start[1] - int(self.map.size[1] / 2)
                self.is_rotating_to = Direction(round((self.measures.compass + 360) / 90) % 4)
                return
            nx, ny = self.steps[0]
            print(a, x ,y, d, self.steps, nx, ny)
            if self.is_rotating_to == None and self.goal == None:
                print(a, d, self.steps[0])
                if (ny,nx) == d[0]:
                    # does not rotate
                    x = x + dx + self.start[0] - int(self.map.size[0] / 2)
                    y = y - dy + self.start[1] - int(self.map.size[1] / 2)
                    self.goal = (x, y)

                elif (ny,nx) == d[1]:
                    # rotates left
                    self.is_rotating_to = Direction((a + 1) % 4)
                elif (ny,nx) == d[2]:
                    # rotates right
                    self.is_rotating_to = Direction((a + 3) % 4)
                else:
                    # rotate back
                    self.is_rotating_to = Direction((a + 2) % 4)

            if self.is_rotating_to != None:
                # rotate
                lPow, rPow = self.rotate()
                print("wtf", self.measures.compass, self.is_rotating_to.value)
                if self.measures.compass == self.is_rotating_to.angle:
                    print("entrei aqui crl" + "\n"*10)
                    self.is_rotating_to = None

            if self.goal != None:
                lPow, rPow = self.move_to()


        elif self.action == 'dead_end??':
            # go to nearest *
            ...

        self.prev_x = self.measures.x
        self.prev_y = self.measures.y
        self.prev_out = (
            lPow + self.prev_out[0]) / 2, (rPow + self.prev_out[1]) / 2
        print("{:6.2f} {:6.2f} ".format(lPow, rPow))
        self.driveMotors(lPow, rPow)
        return

    def follow_line(self) -> Tuple[float, float]:
        """Helps the robot staying on top of the line."""

        print("follow line")
        measures = [int(i) for i in self.measures.lineSensor]
        measures = [(self.prev_measures[i]*0.3 +
                     measures[i]*0.7)/2 for i in range(7)]
        self.prev_measures = measures

        lPow = rPow = 0.1
        # s = 0.25
        s = 0.08
        for i in range(0, 3):
            rPow += s*(3-i)*measures[i]
            lPow -= s*(3-i)*measures[i]
        for i in range(4, 7):
            rPow -= s*(i-3)*measures[i]
            lPow += s*(i-3)*measures[i]
        return lPow, rPow

    def rotate(self) -> Tuple[float, float]:
        """
        Returns the values needed to reach the desired rotation.
        """
        # TODO: ter em consideraçao os valores do power anterior de maneira a roda perfeito
        angle_to = self.is_rotating_to.angle
        angle_from = self.measures.compass

        print(f"rotating from {angle_from:5.1f} to {angle_to:5.1f}")

        rad_to = angle_to * pi / 180
        rad_from = angle_from * pi / 180

        a = angle_to - angle_from
        a += -360 if a > 180 else 360 if a < -180 else 0

        a = a / 180 * pi

        if a == 0:
            return 0, 0

        pwr = min(2*abs(a) - abs(self.prev_out[0] - self.prev_out[1]), 0.3)

        if sin(rad_from - rad_to) < 0:
            # rotate to right
            pwr *= -1

        return pwr/2, -pwr/2

    def move_to(self) -> Tuple[float, float]:

        x2, y2 = self.goal
        x1, y1 = self.measures.x, self.measures.y
        dist = euclidean((x1, y1), self.goal)

        print(f"moving from ({x1:6.1f},{y1:6.1f}) to ({x2:6.1f},{y2:6.1f})")

        if x2 == x1 and y1 == y2:
            self.action = 'stop'
            return tuple(-a for a in self.prev_out)

        a1 = atan2(y2 - y1, x2 - x1)
        a2 = self.measures.compass * pi / 180

        a0 = a1 - a2
        # print(f'a0={a0:6.2f}  cos(a0)={cos(a0):5.2f}  sin(a0)={sin(a0):5.2f}')

        c = cos(a0)
        s = sin(a0)
        n1 = c - s
        n2 = c + s
        min_ = min(n1, n2, -1)
        rng_ = max(n1, n2, 1) - min_

        lPwr = ((2*(n1 - min_) / rng_) - 1) * min(dist, 0.15)
        rPwr = ((2*(n2 - min_) / rng_) - 1) * min(dist, 0.15)

        print('\033[91m', a0, '\033[0m')
        print('\033[91m', (lPwr, rPwr), '\033[0m')
        return lPwr, rPwr

    def search(self) -> List[Tuple[float, float]]:
        #     """Finds a new intersection to explore and returns the list of steps to reach it"""

        min_dist = None
        steps = []

        cx = round(self.measures.x - self.start[0]) + int(self.map.size[0] / 2)
        cy = round(self.measures.y - self.start[1]) + int(self.map.size[1] / 2)

        for i, row in enumerate(self.map.labMap):
            for j, col in enumerate(row):
                if col == '*':
                    d, s = self.__a_distance(cx, cy, j, i)
                    if not min_dist or min_dist > d:
                        min_dist = d
                        steps = s

        return steps

    def __a_distance(self, cx, cy, gx, gy) -> Tuple[float, List[Tuple[float, float]]]:
        """Returns the a* distance between the current position and the goal"""

        queue: List[Node] = [Node(position=(cx, cy))]
        visited: List[Node] = [Node(position=(cx, cy))]

        print(cx, cy,gx,gy)

        while queue:
            n: Node = queue.pop(0)
            d, x, y = n.distance, *n.position
            print(d,x,y)

            if y == gy and x == gx:
                final = []
                while n.parent != None:
                    final.append(n.position)
                    n = n.parent
                return d, list(reversed(final))

            # NORTH
            if self.map.labMap[y + 1][x] in ("-", "|", "x", "*") and (x, y + 1) not in visited:
                # TODO: change distance to take into consideration the rotations
                queue.append(Node(parent=n, position=(x, y + 1), distance=d+1))
                visited.append((x, y + 1))
            # SOUTH
            if self.map.labMap[y - 1][x] in ("-", "|", "x", "*") and (x, y - 1) not in visited:
                # TODO: change distance to take into consideration the rotations
                queue.append(Node(parent=n, position=(x, y - 1), distance=d+1))
                visited.append((x, y - 1))
            # EAST
            if self.map.labMap[y][x + 1] in ("-", "|", "x", "*") and (x + 1, y) not in visited:
                # TODO: change distance to take into consideration the rotations
                queue.append(Node(parent=n, position=(x + 1, y), distance=d+1))
                visited.append((x + 1, y))
            # WEST
            if self.map.labMap[y][x - 1] in ("-", "|", "x", "*") and (x - 1, y + 1) not in visited:
                # TODO: change distance to take into consideration the rotations
                queue.append(Node(parent=n, position=(x - 1, y), distance=d+1))
                visited.append((x - 1, y))
            queue.sort(key=lambda x: x.distance)


class Node:
    def __init__(self, parent=None, position=None, distance=0):
        self.parent = parent
        self.position = position
        self.distance = distance


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
