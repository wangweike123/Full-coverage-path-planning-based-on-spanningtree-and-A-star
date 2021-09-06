from Map import *
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot
import math
import queue

'''Includes all the logic functions related to generating spanning tree and robot path

'''
class PathPlanner():

    # reads in a map object and the starting position of robot in that map
    def __init__(self, m, rx, ry):
        self.map = m
        # direction in the sequence E S W N
        self.direction = [[1,0],[0,-1],[-1,0],[0,1]]
        # prints out debugging info of st(spanning tree) to console if set to True
        self.DEBUGst = False
        # prints out debugging info of draw_path to console if set to True
        self.DEBUGdp = False
        # root of the spanning tree
        self.root = Block(-1,-1)
        # starting position of the robot
        self.start = Point(-1, -1)
        self.robot_dir = 0
        # self.ST_path = []

        (start_x, start_y) = self.map.to_point_coor(rx, ry)
        self.root = Block(
            point_to_block(start_x,start_y)[0],
            point_to_block(start_x,start_y)[1])

    # # 将围绕生成树覆盖的路径加入到 ST_path中
    # def get_ST_path_list(self):
    #     self.ST_path.append((self.start.x,self.start.y))
    #     point = self.start.next
    #     while(point!=None):
    #         self.ST_path.append((point.x,point.y))
    #         point = point.next


    def spanning_tree(self):
        now = self.root
        self.map.vis[(now.x,now.y)] = 1
        dir_flag = 0
        while (True):
            if (self.DEBUGst):
                print("now at (%d, %d) %d" %(now.x, now.y, self.map.vis[(now.x,now.y)]))
            if (self.DEBUGst):
                print("current depth is %d" %(depth(self.root)))
                print("what is this then %d" %(self.map.recorder))

            # looking for blocks that have not been visited
            found = False
            for i in range(4):
                dir = (dir_flag+i)%4
                nextx = now.x + self.direction[dir][0] * 2
                nexty = now.y + self.direction[dir][1] * 2
                if (self.DEBUGst):
                    print("next is (%d, %d)" %(nextx, nexty))
                if (self.map.is_valid_block(nextx,nexty) and 
                    self.map.vis[(nextx,nexty)]<1):
                    next = Block(nextx, nexty)
                    self.map.vis[(nextx,nexty)] = self.map.vis[(nextx,nexty)] + 1
                    next.parent = now
                    now.children[dir] = next
                    now = next
                    found = True
                    self.map.recorder = self.map.recorder + 1
                    dir_flag = dir
                    break
            
            # if we cannot find a node that hasn't been visited from current node,
            # we should start tracing back to the first node satisfying condition
            if (not found): 
                if self.DEBUGst:
                    print ("tracing back")
                new_found = False

                while (True):
                    now = now.parent
                    # Edited: Rather than meaning no solution, it means path  
                    #         generating has finished 
                    if (now == None):
                        return
                        # raise Exception("no solution")
                    for i in range(4):
                        nextx = now.x + self.direction[i][0] * 2
                        nexty = now.y + self.direction[i][1] * 2
                        if (self.DEBUGst):
                            print("next is (%d, %d) %d " %(nextx, nexty,self.map.vis[(nextx,nexty)]))
                        if (self.map.is_valid_block(nextx,nexty) and 
                            self.map.vis[(nextx,nexty)]<1):

                            next = Block(nextx, nexty)
                            self.map.vis[(nextx,nexty)] = self.map.vis[(nextx,nexty)] + 1
                            next.parent = now
                            now.children[i] = next
                            now = next
                            new_found = True
                            self.map.recorder = self.map.recorder + 1
                            break
                    if new_found:
                        break

    # print out the quadtree starting from [node] to console
    # This function isn't always working as it requires many layers of recursion
    def show_sp(self, node):
        print("now at (" + str(node.x) + ", " + str(node.y) +") ")
        has_children = False
        for i in range(4):
            if(node.children[i] != None):
                has_children = True
                print(
                    "(" + str(node.x) + ", " + str(node.y) +") ->" \
                    + "(" + str(node.children[i].x) + ", " \
                    + str(node.children[i].y) +")")
                self.show_sp(node.children[i])
        if not has_children:
            print("(" + str(node.x) + ", " + str(node.y) +") ->" + "None")

    # robot in point [now] facing [dr] moves direct to the next position
    def move_direct(self, now, dr):
        # 改进后将路径点密集了
        if (self.DEBUGdp):
            print("trying to move direct ...")
        next1 =  Point();
        next2 =  Point()
        if (dr == 1 or dr == 3) :
            next1.x = now.x;
            next1.y = now.y + (-1 if dr == 1 else 1)
            next2.x = now.x;
            next2.y = now.y + (-2 if dr == 1 else 2)
        elif ( dr == 0 or dr == 2):
            next1.y = now.y;
            next1.x = now.x + (1 if dr == 0 else -1)
            next2.y = now.y;
            next2.x = now.x + (2 if dr == 0 else -2)
        else:
            raise Exception ("dr is not in the range [0,3]")

        next1.last = now;
        now.next = next1;
        next2.last = next1;
        next1.next = next2

        if (self.DEBUGdp):
            print("(%d, %d)" %(next.x, next.y))
        return next2;

    # robot in point [now] facing [dr] turns right and moves to the next position
    def turn_right(self, now, dr):
        if (self.DEBUGdp):
            print("trying to turn right...")
        toturn = Point()
        turned = Point()

        if dr == 0: 
            toturn.x = now.x + 1
            toturn.y = now.y
            turned.x = toturn.x
            turned.y = toturn.y - 2
        elif dr == 1:
            toturn.x = now.x
            toturn.y = now.y - 1
            turned.x = toturn.x - 2
            turned.y = toturn.y
        elif dr == 2:
            toturn.x = now.x - 1
            toturn.y = now.y
            turned.x = toturn.x
            turned.y = toturn.y + 2
        elif dr == 3:
            toturn.x = now.x
            toturn.y = now.y + 1
            turned.x = toturn.x + 2
            turned.y = toturn.y
        else:
            raise Exception ("dr is not in the range [0,3]")

        toturn.last = now
        now.next = toturn
        turned.last = toturn
        toturn.next = turned
        if (self.DEBUGdp):
            print("(%d, %d)" %(toturn.x, toturn.y))
            print("(%d, %d)" %(turned.x, turned.y))
        return turned

    # robot in point [now] facing [dr] turns left and moves to the next position
    def turn_left(self, now, dr):
        if (self.DEBUGdp):
            print("trying to turn left...")
        next = Point()
        if (dr == 1 or dr == 3):
            next.x = now.x + (1 if dr==1 else -1)
            next.y = now.y
        elif (dr == 0 or dr == 2):
            next.x = now.x
            next.y = now.y + (1 if dr==0 else -1)
        else:
            raise Exception ("dr is not in the range [0,3]")
        next.last = now
        now.next = next
        if (self.DEBUGdp):
            print("(%d, %d)" %(next.x, next.y))
        return next

    # robot in point [now] facing [dr] makes a turn and moves to the next position
    def turn_around(self, now, dr):
        if (self.DEBUGdp):
            print("trying to turn around...")
        turn1 = Point()
        turn2 = Point()
        turned = Point()
        if dr == 0:
            turn1.x = now.x + 1
            turn1.y = now.y
            turn2.x = turn1.x
            turn2.y = turn1.y - 1
            turned.x = turn2.x - 2
            turned.y = turn2.y
        elif dr == 1:
            turn1.x = now.x
            turn1.y = now.y - 1
            turn2.x = turn1.x - 1
            turn2.y = turn1.y
            turned.x = turn2.x
            turned.y = turn2.y + 2
        elif dr == 2:
            turn1.x = now.x - 1
            turn1.y = now.y
            turn2.x = turn1.x
            turn2.y = turn1.y + 1
            turned.x = turn2.x + 2
            turned.y = turn2.y
        elif dr == 3:
            turn1.x = now.x
            turn1.y = now.y + 1
            turn2.x = turn1.x + 1
            turn2.y = turn1.y
            turned.x = turn2.x
            turned.y = turn2.y - 2
        else:
            raise Exception ("dr is not in the range [0,3]")

        turn1.last = now
        now.next = turn1
        turn2.last = turn1
        turn1.next = turn2
        turned.last = turn2
        turn2.next = turned
        if (self.DEBUGdp):
            print("(%d, %d)" %(turn1.x, turn1.y))
            print("(%d, %d)" %(turn2.x, turn2.y))
            print("(%d, %d)" %(turned.x, turned.y))

        return turned

    # @return initialized robot position as a point object 
    #         and initialized robot direction
    # initialize the robot's position according to the first edge from root node
    # the initialized position also follows robot position rule specifed in the
    # head comment(not surpassing edge end node, wall always on robot's right side)
    def initialize_position(self):
        # 修改版
        if self.map.graph[(self.root.x, self.root.y, 0)]: # 此处的方向是朝左的
            return (Point(self.root.x+1, self.root.y), 0)
        elif self.map.graph[(self.root.x, self.root.y, 1)]:
            return (Point(self.root.x, self.root.y-1), 1)
        elif self.map.graph[(self.root.x, self.root.y, 2)]:
            return (Point(self.root.x-2, self.root.y), 2)
        elif self.map.graph[(self.root.x, self.root.y, 3)]:
            return (Point(self.root.x-1, self.root.y+1), 3)
        else :
            raise Exception ("No edge from root, Graph not initialized")

    # initialize self.map.graph, robot position, and robot direction
    # used a queue to avoid exceeding maximu recursion level
    def initialize_graph(self):
        for i in range(0, self.map.width+2):
            for j in range(0, self.map.height+2):
                for k in range(4):
                    self.map.graph[(i, j, k)] = 0
        q = queue.Queue()
        def initialize_graph_aux(now):
            for i in range(4):
                next = now.children[i]
                if (next!=None) :
                    self.map.graph[(now.x, now.y, i)] = 1
                    self.map.graph[(next.x, next.y, (i+2)%4)] = 1
                    q.put(next)
        initialize_graph_aux(self.root)
        while not q.empty() :
            initialize_graph_aux(q.get())

        (self.start, self.robot_dir) = self.initialize_position()

    def draw_path(self):
        self.initialize_graph()
        now = self.start

        if self.DEBUGdp:
            print("now at (%d, %d)" %(now.x, now.y))

        do = True
        while (now.x != self.start.x or now.y != self.start.y) or do:
            do = False
            if self.DEBUGdp:
                print("now at (%d, %d) facing %d" %(now.x, now.y, self.robot_dir))
            (block_x, block_y) = point_to_block(now.x, now.y)
            # block_x = get_block_coor(now.x)
            # block_y = get_block_coor(now.y)

            #TODO: comment 
            if self.map.graph[(block_x, block_y, (self.robot_dir+3)%4)]: #先判定左转弯，其实是 (dir-1)%4
                now = self.turn_left(now, self.robot_dir)
                self.robot_dir = (self.robot_dir+3) % 4
            elif self.map.graph[(block_x, block_y, (self.robot_dir))]: # 判定直行
                now = self.move_direct(now, self.robot_dir)
            elif self.map.graph[(block_x, block_y, (self.robot_dir+1)%4)]: #判定右转弯
                now = self.turn_right(now, self.robot_dir)
                self.robot_dir = (self.robot_dir+1) % 4
            elif self.map.graph[(block_x, block_y, (self.robot_dir+2)%4)]: # 判定回头转
                now = self.turn_around(now, self.robot_dir)
                self.robot_dir = (self.robot_dir+2) % 4
            else :
                raise Exception ("no solution")
        # self.get_ST_path_list()



    
    # convert robot's path, stored in linked list, to a python list
    def get_path(self):
        result = []
        now = self.start
        do = True
        while (now.x != self.start.x or now.y != self.start.y) or do:
            do = False
            # result.append((now.x * self.map.robot_len, now.y * self.map.robot_len))
            result.append((now.x, now.y))
            now = now.next
        return result