from PathPlanner import *
from AwayFree import *
import json

# shows a graph of borders(blocked), spanning tree(vis), and robot's path(path)
def show_graph(map, path,way_x,way_y):
    import matplotlib.pyplot as plt
    plt.figure(1)
    blocked_X = []
    blocked_Y = []
    blocked_x = []
    blocked_y = []
    for i in range(0, map.width+1):
        for j in range(0, map.height+1):
            if (map.blocked[(i,j)]):
                # print (i,j)
                blocked_X.append(i * map.robot_len + map.min_x)
                blocked_Y.append(j * map.robot_len + map.min_y)
                blocked_x.append(i)
                blocked_y.append(j)
    plt.plot(blocked_X, blocked_Y, '.')

    path_X = []
    path_Y = []
    path_x = []
    path_y = []
    for i in path:
        path_X.append(i[0] * map.robot_len + map.min_x)
        path_Y.append(i[1] * map.robot_len + map.min_y)
        path_x.append(i[0])
        path_y.append(i[1])
    plt.plot(path_X, path_Y, '.-y')
    # for i in range(len(path)+1):
    #     plt.plot([pathx[i], pathx[i+1]], [pathy[i],pathy[i+1]], '.-r')
    #     plt.pause(0.001)

    way_X = []
    way_Y = []
    for i in range(len(way_x)):
        way_X.append(way_x[i]*map.robot_len + map.min_x)
        way_Y.append(way_y[i]*map.robot_len + map.min_y)
    plt.plot(way_X,way_Y,'.-r')
    plt.show()

     # 将 blocked_x / blocked_y (障碍物点)、path_x / path_y (覆盖书路径)、way_x/way_y(生成的周围路径) 、
     # 保存到本地，均为（i,j）格子坐标，方便避障调试
    Date = {'blocked_x':blocked_x,'blocked_y':blocked_y,\
            'path_x':path_x,'path_y':path_y,\
            'way_x':way_x,'way_y':way_y}
    file_name = "./my_date.json"
    with open(file_name,'w') as file_obj:
        json.dump(Date,file_obj)
        # print("写入json文件",Date)


def show_turn_num(planner):
    # 统计转弯次数
    now = planner.start
    print(now.x,now.y)
    num = 0
    total = 0
    next1 = now.next
    next2 = next1.next
    while now.next!=None:
        total=total+1
        now=now.next
    now = planner.start
    while next2 != None:
        print(next2.x,next2.y)
        if (now.x==next1.x==next2.x) or (now.y==next1.y==next2.y):
            now = next2
            next1 = now.next
            next2 = next1.next
            continue
        now = now.next
        next1 = now.next
        next2 = next1.next
        num = num+1
    print("转弯"+str(num)+"次，共计"+str(total)+"个节点")


def main():

    map_path = "v2.smap"
    my_map = Map()
    my_map.initialize_map(map_path) # 初始化地图对象
    obstacles = my_map.obstacles # 存储障碍物列表
    planner = PathPlanner(my_map, 20, 10)
    planner.spanning_tree()
    # planner.show_sp(planner.root)
    planner.draw_path()
    result = planner.get_path()  # result 存储围绕生成树的路径

    way_x,way_y = away_free_space((planner.start.x,planner.start.y), obstacles, result) # 消除剩余空间的路径
    show_graph(my_map, result,way_x,way_y)
    # show_turn_num(planner) # 显示沿生成树转过的次数


if __name__ == "__main__":
    main()
