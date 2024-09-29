import math, sys, time, os, random, re
#from matplotlib.patches import Arrow, Circle, Wedge, Polygon, Rectangle
from vpython import *
from functools import reduce

FLOAT = '([-+]?[0-9]*.?[0-9]+)'
INT = '([-+]?[0-9][0-9]*)'

AREA_X = 3000.
AREA_Y = 2000.

ROBOT_HEIGHT = 350.0
WALL_HEIGHT = 70.0

ROBOT_WIDTH  = 330.0
ROBOT_LENGTH = 288.5
#ROBOT_LENGTH = 210.0 #105.0*2.0
#ROBOT_X_OFFSET = ROBOT_LENGTH/2.0 - 121.5

area = [ (0.0, 0.0, -0.2), (3000.0, 2000.0, 0.2) ]
areasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , area)
area_box = box(size=vec(*areasize), color=vec(0.09, 0.38, 0.671))

scene.autoscale = 1

# all positions of robot every 5ms
save_pos = []

robot = box(color=vec(1.0, 1.0, 1.0))
robot.opacity = 0.5

opp = box(color=vec(0.7, 0.2, 0.2))
opp.opacity = 0.7

last_pos = (0.,0.,0.)

hcenter_line = curve(pos=[vec(-AREA_X/2, 0., 0.3), vec(AREA_X/2, 0., 0.3)]) 
vcenter_line = curve(pos=[vec(0., -AREA_Y/2, 0.3), vec(0., AREA_Y/2, 0.3)])

wallx = [ (0.0, 0.0, -0.5), (AREA_X+44, 22, WALL_HEIGHT) ]
wallxsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , wallx)
wallx1_box = box(pos=vec(0,-AREA_Y/2-11, WALL_HEIGHT/2), size=vec(*wallxsize), color=vec(0.78, 0.09, 0.071))
wallx2_box = box(pos=vec(0,AREA_Y/2+11, WALL_HEIGHT/2), size=vec(*wallxsize), color=vec(0.78, 0.09, 0.071))

wally = [ (0.0, 0.0, -0.5), (22, AREA_Y+44, WALL_HEIGHT) ]
wallysize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , wally)
wally1_box = box(pos=vec(-AREA_X/2-11, 0, WALL_HEIGHT/2), size=vec(*wallysize), color=vec(0.78, 0.09, 0.071))
wally2_box = box(pos=vec(AREA_X/2+11, 0, WALL_HEIGHT/2), size=vec(*wallysize), color=vec(0.78, 0.09, 0.071))


YELLOW = 0
GREEN = 1
color = YELLOW

def square(sz):
    sq = curve(pos = [vec(-sz, -sz, 0.3),
              vec(-sz, sz, 0.3),
              vec(sz, sz, 0.3),
              vec(sz, -sz, 0.3),
              vec(-sz, -sz, 0.3),])
    return sq

sq1 = square(250)
sq2 = square(500)

robot_x = 0.
robot_y = 0.
robot_a = 0.

robot_trail = curve()
robot_trail_list = []
max_trail = 500

area_objects = []

set_opp_nb = 1

def toggle_obj_disp():
    global area_objects
    if area_objects == []:
        return
    else:
        for o in area_objects:
            if o.visible:
                o.visible = 0
            else:
                o.visible = 1
				
def toggle_color():
    global color
    global GREEN, YELLOW
    if color == YELLOW:
        color = GREEN
    else:
        color = YELLOW


def set_robot():
    global robot, last_pos, robot_trail, robot_trail_list
    global save_pos, robot_x, robot_y, robot_a

    if color == YELLOW:
        tmp_x = robot_x - AREA_X/2
        tmp_y = robot_y - AREA_Y/2
        tmp_a = robot_a
    else:
        tmp_x = -robot_x + AREA_X/2
        tmp_y = -robot_y + AREA_Y/2
        tmp_a = robot_a

	
    robot.pos = vec(tmp_x, tmp_y, ROBOT_HEIGHT/2)
    axis = vec(math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)

    robot.axis = axis
    robot.size = vec(ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT)
	
    # save position
    save_pos.append((robot.pos.x, robot.pos.y, tmp_a))

    pos = vector(robot.pos.x, robot.pos.y, 0.3)
    if pos != last_pos:
        robot_trail_list.append(pos)
        last_pos = pos
    robot_trail_l = len(robot_trail_list)
    if robot_trail_l > max_trail:
        robot_trail_list = robot_trail_list[robot_trail_l - max_trail:]
    #robot_trail.append(pos=robot_trail_list)

def set_opp(x, y):
    opp.size = vector(300, 300, ROBOT_HEIGHT)
    opp.pos = vector(x, y, ROBOT_HEIGHT/2)

def graph():
    pass

def save():
    f = open("/tmp/robot_save", "w")
    for p in save_pos:
        f.write("%f %f %f\n"%(p[0], p[1], p[2]))
    f.close()

def silent_mkfifo(f):
    try:
        os.mkfifo(f)
    except:
        pass

toggle_obj_disp()
set_robot()

while True:

    silent_mkfifo("./robot_sim2dis")
    silent_mkfifo("./robot_dis2sim")

    while True:
        fr = open("./robot_sim2dis", "r")
        fw = open("./robot_dis2sim", "w")

        while True:
            # MAIN ROBOT MSGS
            m = None
            l = fr. readline()

            # parse position
            if not m:
                m = re.match("pos=%s,%s,%s"%(INT,INT,INT), l)
                if m:
                    robot_x = int(m.groups()[0])
                    robot_y = int(m.groups()[1])
                    robot_a = int(m.groups()[2])                  
                    set_robot()
                    
            # DISPLAY EVENTS
            
            #mpos = scene.mouse.project(normal=vector(0,0,1))
            #print(mpos)
            #if mpos != None:
            #    #set_opp(oppx, oppy)
            #    try:
            #        if color == YELLOW:
            #            fw.write("opp_1 %d %d"%(int(oppx + 1500), int(oppy + 1050)))
            #        else:
            #            fw.write("opp_1 %d %d"%(int(1500 - oppx), int(1050 - oppy)))
            #    except:
            #        print("not connected")
            
            #k = keysdown()
#
            ##elif k == "l":
            ##    fw.write("l")
            ##elif k == "r":
            ##    fw.write("r")
            ##elif k == "b":
            ##    fw.write("b")
            #if k == "c":
            #    robot_trail_list = []
            #    robot2_trail_list = []
            #elif k == "x":
            #    save_pos = []
            #elif k == "g":
            #    graph()
            #elif k == "s":
            #    save()
            #elif k == "h":
            #    toggle_obj_disp()
            #elif k == "i":
            #    toggle_color()
            #else:
            #    print(k)

            # EOF
            if l == "":
                break

        fr.close()
        fw.close()



