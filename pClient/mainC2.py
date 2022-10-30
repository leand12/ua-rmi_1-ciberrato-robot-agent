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
    EAST = (0, 0)
    NORTH = (1, 90)
    WEST = (2, 180)
    SOUTH = (3, -90)

    def __new__(cls, value, angle):
        obj = int.__new__(cls)
        obj._value_ = value
        obj.angle = angle
        return obj


class Mapper:
    size = (49, 21)

    def __init__(self, start: Tuple[int, int]) -> 'Mapper':
        self.start = start
        self.labMap = [['.']*self.size[0] for _ in range(self.size[1])]

    def print_map(self) -> None:
        for row in reversed(self.labMap):
            print(''.join(row))
        print('\n' * 2)

    def visit(self, x: int, y: int, angle: float) -> None:
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)
        print('visit', x, y)

        dir = ['-', '|'][round(angle / 90) % 2]
        if self.labMap[y][x] != 'x':
            self.labMap[y][x] = dir
        

    def explore_inter(self, x: int, y: int, angle: float, line: List[int], action: str) -> None:
        print('explore_inter', x, y, line)
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)

        print(angle)
        m = 1 if angle < 2 else -1
        dir = angle % 2
        self.labMap[y][x] = 'x'

        ret = Rotation.BACK

        dy, dx = -dir * m, abs(dir - 1) * m
        d = [
            (y - dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
            (y + dx, x + dy),    # dyl = dxf, dxl = dyf
            (y - dx, x - dy),    # dyr = -dxf, dxr = -dyf 
        ]

        print(d, y, x, line, action)
        
        if action == "stop" and line[3]:
            print("a fazer esta merda, doubt")
            py, px = d[0][0], d[0][1]
            if self.labMap[py][px] == '.':
                ret = Rotation.NONE
                self.labMap[py][px] = "*"
            elif self.labMap[py][px] == '*':
                return Rotation.NONE

        if all(line[4:]):        # right
            py, px = d[2][0], d[2][1]
            print("rigth", self.labMap[py][px])
            if self.labMap[py][px] == '.':
                ret = Rotation.RIGHT
                self.labMap[py][px] = "*"
            elif self.labMap[py][px] == '*':
                return Rotation.RIGHT
        
        if all(line[:3]):        # left
            py, px = d[1][0], d[1][1]
            print("left", self.labMap[py][px])
            if self.labMap[py][px] == '.':
                ret = Rotation.LEFT
                self.labMap[py][px] = "*"
            elif self.labMap[py][px] == '*':
                return Rotation.LEFT

        self.print_map()
        return ret


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.prev_measures = [0.5]*7
        self.prev_x = self.prev_y = self.prev_a = None
        self.prev_rPow = 0
        self.prev_lPow = 0

        self.action = "moving"

        self.is_rotating_to: Direction =  None
        
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
        print(self.goal)
        self.map = Mapper(self.goal)

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

    def wander(self):
        
        measures = [int(i) for i in self.measures.lineSensor]
        x, y, a = round(self.measures.x), round(self.measures.y), round((self.measures.compass + 360) / 90) % 4
        
        print('\n', self.action, self.measures.x, self.measures.y, self.prev_out)

        if self.action == 'rotating':
            # Rotates the robot
            lPow, rPow = self.rotate()
            if abs(lPow) == 0 and abs(rPow) == 0:
                self.action = 'moving'
                self.is_rotating_to = None

        elif self.action == 'moving':
            # Move to goal
            lPow, rPow = self.move_to()

            # Make robot visit intersection
            # if all(measures[:3]) or all(measures[:4]):
            print(measures)
            if (all(measures[:3]) or all(measures[4:])) or self.action == 'stop': # FIXME: noise
                dir = self.map.explore_inter(self.measures.x, self.measures.y, a, measures, self.action)

                # self.goal = round(self.measures.x), round(self.measures.y)
                if not self.has_plan:
                    if dir == Rotation.LEFT:
                        self.is_rotating_to = Direction( (a + 1) % 4 )
                    elif dir == Rotation.RIGHT:
                        self.is_rotating_to = Direction( (a + 3) % 4 )
                    elif dir == Rotation.BACK:
                        self.is_rotating_to = Direction( (a + 2) % 4 )

                    self.has_plan = True

            #print(self.measures.lineSensor)
            if self.action == 'stop':
                self.has_plan = False
                #print(self.measures.lineSensor, 'done')
                
                if self.is_rotating_to != None:
                    self.action = "rotating"
                    dir = self.is_rotating_to
                else:
                    self.action = 'moving'
                    dir = Direction(a)
                
                #print(self.is_rotating_to)
                if dir.value == Direction.EAST.value:
                    self.goal = self.goal[0] + 2, self.goal[1]
                elif dir.value == Direction.WEST.value:
                    self.goal = self.goal[0] - 2, self.goal[1]
                elif dir.value == Direction.NORTH.value:
                    self.goal = self.goal[0], self.goal[1] + 2
                elif dir.value == Direction.SOUTH.value:
                    self.goal = self.goal[0], self.goal[1] - 2

            # Make robot visit next position and paint map
            if self.prev_x and self.prev_x != x or self.prev_y != y:
                self.map.visit(self.measures.x, self.measures.y, self.measures.compass)
            
        
        elif self.action == 'dead_end??':
            # go to nearest *
            ...


        self.prev_x = self.measures.x
        self.prev_y = self.measures.y
        if not self.prev_out:
            self.prev_out = lPow, rPow
        else:
            self.prev_out = (lPow + self.prev_out[0]) / 2, (rPow + self.prev_out[1]) / 2
        # print("{:7.1f} {:7.1f}      {:6.2f} {:6.2f} ".format( self.measures.x, self.measures.y, lPow, rPow), end='   ')
        self.driveMotors(lPow, rPow)
        return
        
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3
        if self.measures.irSensor[center_id] > 5.0\
           or self.measures.irSensor[left_id] > 5.0\
           or self.measures.irSensor[right_id] > 5.0\
           or self.measures.irSensor[back_id] > 5.0:
            print('Rotate left')
            self.driveMotors(-0.1, +0.1)
        elif self.measures.irSensor[left_id] > 2.7:
            print('Rotate slowly right')
            self.driveMotors(0.1, 0.0)
        elif self.measures.irSensor[right_id] > 2.7:
            print('Rotate slowly left')
            self.driveMotors(0.0, 0.1)
        else:
            measures = [int(i) for i in self.measures.lineSensor]
    
            x, y, a = round(self.measures.x), round(self.measures.y), round((self.measures.compass + 360) / 90) % 4

            # print((self.measures.compass + 360) / 90 % 4)
            
            a_err = abs((self.measures.compass + 360) / 90 % 4 - a)

            lPow = rPow = 0.125
            
            # Rotate robot on an intersection
            if self.is_rotating_to != None:
                print("\n"*4)
                print(self.measures.compass, a, ' has to go to ', self.is_rotating_to, '   err', a_err)
                lPow, rPow = self.rotate(self.is_rotating_to.angle)
                print(self.prev_lPow, self.prev_rPow)
                print(lPow, rPow)
                
                if abs(lPow) < 0.0005:
                    self.is_rotating_to = None
                    self.goal = None
                    self.moves = 0

                self.prev_lPow = lPow
                self.prev_rPow = rPow
                self.driveMotors(lPow, rPow)
                return

            
            # Make robot visit next position and paint map
            if self.prev_x and self.prev_x != x or self.prev_y != y:
                self.map.visit(x, y, self.measures.compass)
                self.has_plan = False

            # Make robot visit intersection
            # if all(measures[:3]) or all(measures[:4]):
            if (measures[0] or measures[-1]): # FIXME: noise
                dir = self.map.explore_inter(x, y, a, measures)
                print(dir)
                self.goal = round(self.measures.x), round(self.measures.y)
                if not self.has_plan:
                    if dir == Rotation.LEFT:
                        self.is_rotating_to = Direction( (a + 1) % 4 )
                    if dir == Rotation.RIGHT:
                        self.is_rotating_to = Direction( (a + 3) % 4 )
                    print(self.is_rotating_to)
                    self.has_plan = True
            
            self.driveMotors(lPow, rPow)
            print([f"{i:4.2f}" for i in measures])
    
            self.prev_x, self.prev_y = x, y
            self.prev_rPow, self.prev_lPow = rPow, lPow
            self.prev_measures = measures
    


    def follow_line(self) -> Tuple[float, float]:
        """Helps the robot staying on top of the line."""

        print("follow line")        
        measures = [int(i) for i in self.measures.lineSensor]
        measures = [(self.prev_measures[i]*0.3 + measures[i]*0.7)/2 for i in range(7)]
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

        if a == 0:
            return 0, 0

        if sin(rad_from - rad_to) < 0:
            # rotate to left
            pwr = min((-0.15 + self.prev_out[0]) / 2, a * pi / 180)
        elif sin(rad_from - rad_to) > 0: # TODO: e qnd dá 0
            # rotate to right
            pwr = min(abs((0.15 + self.prev_out[0]) / 2), abs(a * pi / 180))

        return pwr/2, -pwr/2

    def move_to(self) -> Tuple[float, float]:
        
        x2, y2 = self.goal
        x1, y1 = self.measures.x, self.measures.y
        
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

        return 0.15 * (c - s), 0.15 * (c + s)

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
