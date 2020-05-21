# coding=utf-8
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import time
def straight_distance(point_a, point_b):
    x1,y1 = point_a
    x2,y2 = point_b
    return ( (x1-x2)**2 + (y1-y2)**2 )**0.5
def find_nearest_obstacle_distance(array,value):#这个速度快一点
    dis_array = (array-value)**2
    min_distance = (dis_array[:,1]+dis_array[:,0]).min()
    return min_distance**0.5
'''
vertex：顶点集
adjacency_mat：邻接矩阵
start_index：一般是0 代表顶点集第一个是起点
goal_index：一般是1 代表顶点集第二个是起点
'''
def A_star_list(vertex,#采样点；或者图中的点
                self_index,#起始点
                goal_index,#终止点
                parent_index,
                historic_cost):
    ## A*算法的路径代价表计算工具
    historic_cost = historic_cost
    heuristic_cost = straight_distance(vertex[self_index],
                                       vertex[goal_index])
    total_cost = historic_cost + heuristic_cost
    A_list = np.mat([self_index, parent_index, total_cost])
    return A_list
def A_star_algorithm(vertex, adjacency_mat, start_index, goal_index):
    ## A*搜索算法
    num_sample = len(vertex) - 2
    # 搜索起点，数组依次为：[顶点集，起点索引，目标点索引，父节点索引，历史代价]
    A_list = A_star_list(vertex, start_index, goal_index, -1, 0)
    close_list = np.mat([[-1, -1, -1]])
    open_list = A_list
    # 如果开表非空
    while open_list.shape[0] > 0:
        # 获取开表中代价最小点及其索引
        min_cost_index = np.argmin(open_list[:, 2])
        n_list = open_list[min_cost_index, :]
        # 将最小点转移到闭表,闭表存放的是最短路径
        close_list = np.append(close_list, open_list[min_cost_index, :], axis=0)
        open_list = np.delete(open_list, min_cost_index, 0)
        # 如果不是目标点（通过索引判断）
        if n_list[0, 0] != goal_index:
            # 查找子节点的索引
            sub_list = np.array([], dtype=int)
            for i in range(num_sample + 2):
                if adjacency_mat[int(n_list[0, 0]), i] == 1:
                    sub_list = np.append(sub_list, i)
            for i in sub_list:  # i为顶点集索引
                # 如果子节点在开表中, 比较并更新
                if i in open_list[:, 0]:
                    # 获取开表中已有点的索引
                    exist_index = np.argwhere(open_list[:, 0] == i)[0, 0]
                    # 比较代价，若新点更小，则替换
                    A_list = A_star_list(vertex, i, 1, n_list[0, 0], n_list[0, 2])
                    if A_list[0, 2] < open_list[exist_index, 2]:
                        open_list[exist_index, :] = A_list[0, :]

                # 如果子节点不在开表与闭表中，将其加入开表
                elif not ((i in open_list[:, 0]) or (i in close_list[:, 0])):
                    A_list = A_star_list(vertex, i, 1, n_list[0, 0], n_list[0, 2])
                    open_list = np.append(open_list, A_list, axis=0)
        else:
            print('找到最优路径，结束搜索')
            return vertex, adjacency_mat, close_list, True

    else:
        print('没有找到合理的路径')
        return vertex, adjacency_mat, close_list, False
def A_star_plot(map_img, vertex, adjacency_mat, close_list):
    ## 绘制A*求解结果图
    # 画点
    point_size = 5
    point_color = (0, 127, 0)  # BGR
    thickness = 4
    # 将矩阵转化为数组，再转化为元组，以供cv2使用
    vertex = np.array(vertex)
    vertex_tuple = tuple(map(tuple, vertex))
    for point in vertex_tuple:
        cv2.circle(map_img, point, point_size, point_color, thickness)

    for i in range(vertex.shape[0]):
        for j in range(vertex.shape[0]):
            if adjacency_mat[i, j] == 1:
                cv2.line(map_img, vertex_tuple[i], vertex_tuple[j], (255, 150, 150), 2)

    # 回溯绘制最优路径
    path = []
    point_a_index = 1
    while point_a_index != 0:
        path.append((vertex[point_a_index][0],vertex[point_a_index][1]))
        exist_index = int(np.argwhere(close_list[:, 0] == point_a_index)[0, 0])
        point_b_index = int(close_list[exist_index, 1])
        cv2.line(map_img, vertex_tuple[point_a_index], vertex_tuple[point_b_index], (0, 0, 255), 4)
        point_a_index = point_b_index
    # 显示图片
    path.append((vertex[0][0],vertex[0][1]))
    cv2.imshow("map", map_img)  # 转为RGB显示
    cv2.waitKey()
    cv2.destroyAllWindows()
    return path,map_img

'''
只对inflacted之后的区域计算静态APF以及针对某个点计算,现在代码外没有进行影响半径的设置
所以我打算在这里进行全局变量的设置影响半径D_MAX
'''
D_MAX = 35
OBSTACLE_MAX = 255
APF_WAY = 3
'''
这个会有终点不可达效应:https://blog.csdn.net/junshen1314/article/details/50472410
D = [10,35) 1/34~1/10 - 1/35 = 1/ (34*35) ~ 1/14 /D**2
'''
def APF_function(D):
    if D > D_MAX:
        return 0
    elif D > 0:
        return OBSTACLE_MAX * (1.0 / D - 1.0 / D_MAX) / (D**2)
    else:
        return OBSTACLE_MAX
#这个会有终点不可达效应
 # 3 rou的时候接近0
ROU = D_MAX/3
def my_APF_function(D):
    if D > D_MAX:
        return 0
    else:
        return OBSTACLE_MAX * np.exp(- D ** 2 / (2 * ROU ** 2))

'''解决了终点不可达效应,但是在动态障碍物的情况下显得有一些贪心,时常会发生碰撞'''
'''improved_APF_function的障碍物影响力下降的太快,不能很好的避障'''
def improved_APF_function(D,D_goal):
    if D > D_MAX:
        return 0
    elif D > 0:
        uo =  (1.0 / D - 1.0 / D_MAX)  / (D ** 2) * (D_goal ** 2)
        uo +=  ((1.0 / D - 1.0 / D_MAX )**2) * D_goal
        return uo
    else:
        return OBSTACLE_MAX

def my_improved_APF_function(D,D_goal):
    if D > D_MAX:
        return 0
    else:
        uo =  np.exp(- D ** 2 / (2 * ROU ** 2))  / (D ** 2) * (D_goal ** 2)
        uo += np.exp(- D ** 2 / (2 * ROU ** 2))**2 * D_goal
        return uo


# if __name__ == "__main__":
#     D_goal = 5
#     D = np.arange(100)+1
#     print(D)
#     begin = time.time()
#     result_list = []
#     for i in range(1,101):
#         if APF_WAY == 0:
#             result = APF_function(D[i-1])  # 3 rou的时候接近0
#         elif APF_WAY == 1:
#             result = my_APF_function(D[i-1])  # 3 rou的时候接近0
#         elif APF_WAY == 2:
#             result = improved_APF_function(D[i-1], abs(D_goal-D[i-1]))
#         elif APF_WAY == 3:
#             result = my_improved_APF_function(D[i-1],abs(D_goal-D[i-1]))
#         result_list.append(result)
#     end = time.time()
#     print("time", (end - begin) * 500)
#     plt.scatter(D, result_list)
#     plt.show()



