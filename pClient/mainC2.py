from enum import Enum
from operator import is_
from statistics import mean
import sys
from tkinter import RIGHT
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS = 7
CELLCOLS = 14



class Rotation(Enum):
    LEFT = 0
    RIGHT = 1


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

    def __init__(self, start: tuple[int, int]) -> 'Mapper':
        self.start = start
        self.labMap = [[' ']*self.size[0] for _ in range(self.size[1])]

    def print_map(self) -> None:
        print('\n' * 5)
        for row in reversed(self.labMap):
            print(''.join(row).replace(' ', '.'))

    def visit(self, x: int, y: int, angle: float) -> None:
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)
        print('visit', x, y)

        dir = ['-', '|'][round(angle / 90) % 2]
        self.labMap[y][x] = dir

    def explore_inter(self, x: int, y: int, angle: float, line: list[int]) -> None:
        x = round(x - self.start[0]) + int(self.size[0] / 2)
        y = round(y - self.start[1]) + int(self.size[1] / 2)
        print('explore_inter', x, y)
        self.print_map()

        m = 1 if angle < 1 else -1
        dir = angle % 2
        self.labMap[y][x] = 'x'

        dy, dx = -dir * m, abs(dir - 1) * m
        d = [
            (y + dy, x + dx),    # dyf = -1 * dir, dxf = abs(dir - 1)
            (y - dx, x + dy),    # dyl = dxf, dxl = -dyf
            (y + dx, x - dy),    # dyr = -dxf, dxr = dyf 
        ]
        #if all(line[:3]):        # left
        if line[0]:
            py, px = d[1][0], d[1][1]
            if self.labMap[py][px] == ' ':
                self.labMap[py][px] = "*"
            if self.labMap[py][px] == '*':
                return Rotation.LEFT

        if line[-1]:       
        #if all(line[4:]):        # right
            py, px = d[2][0], d[2][1]
            if self.labMap[py][px] == ' ':
                self.labMap[py][px] = "*"
            if self.labMap[py][px] == '*':
                return Rotation.RIGHT
                
        
            # if
            #   find_next()

prev_x = prev_y = prev_a = None
is_rotating_to: Direction =  None
prev_measures = [0.5]*7
has_plan = False
prev_rPow = 0
prev_lPow = 0
moves = 0

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

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
        print(self.measures.x, self.measures.y)
        self.map = Mapper((self.measures.x, self.measures.y))

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
        global prev_x, prev_y, prev_a, is_rotating_to, has_plan, prev_lPow, prev_rPow, prev_measures, moves
        
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
            if is_rotating_to != None:
                print("\n"*4)
                print(self.measures.compass, a, ' has to go to ', is_rotating_to, '   err', a_err)
                lPow, rPow = self.rotate(self.measures.compass, is_rotating_to.angle)
                print(prev_lPow, prev_rPow)
                print(lPow, rPow)
                
                if abs(lPow) < 0.0005:
                    is_rotating_to = None
                    moves = 0

                prev_lPow = lPow
                prev_rPow = rPow
                self.driveMotors(lPow, rPow)
                return

            
            # Make robot visit next position and paint map
            if prev_x and prev_x != x or prev_y != y:
                self.map.visit(x, y, self.measures.compass)
                has_plan = False

            # Make robot visit intersection
            # if all(measures[:3]) or all(measures[:4]):
            if (measures[0] or measures[-1]) and moves + 1 == 15: # FIXME: noise
                dir = self.map.explore_inter(x, y, a, measures)
                print(dir)
                if not has_plan:
                    if dir == Rotation.LEFT:
                        is_rotating_to = Direction( (a + 1) % 4 )
                    if dir == Rotation.RIGHT:
                        is_rotating_to = Direction( (a + 3) % 4 )
                    print(is_rotating_to)
                    has_plan = True
                else:
                    moves = -1
            
            self.driveMotors(lPow, rPow)
            print([f"{i:4.2f}" for i in measures])
    
            prev_x, prev_y = x, y
            prev_rPow, prev_lPow = rPow, lPow
            prev_measures = measures
            moves += 1

    
    def follow_line(self) -> tuple[float, float]:
        """ 
        Helps the robot staying on top of the line. 
        """
        global prev_measures

        print("follow line")        
        measures = [int(i) for i in self.measures.lineSensor]
        measures = [(prev_measures[i]*0.3 + measures[i]*0.7)/2 for i in range(7)]
        prev_measures = measures

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

    def rotate(self, angle_from: float, angle_to: float) -> tuple[float, float]:
        """
        Returns the values needed to reach the desired rotation.
        """

        rad_to = angle_to * pi / 180
        rad_from = angle_from * pi / 180

        a = angle_to - angle_from
        a += -360 if a > 180 else 360 if a < -180 else 0

        pwr = min(0.3, abs(a * pi / 180))

        if sin(rad_from - rad_to) > 0:
            # rotate to right
            pwr *= -1

        return -pwr/2, pwr/2

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
