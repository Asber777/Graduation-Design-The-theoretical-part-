# coding=utf-8
import random
from  my_motion_planning_toolbox import *
import os
import matplotlib.pyplot as mp
from mpl_toolkits.mplot3d import axes3d
# https://www.jianshu.com/p/eaa1bf01b3a6
import yaml
def generate_yaml_doc(yaml_file,py_object):
    file = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(py_object, file)
    file.close()
def get_yaml_data(yaml_file):
    # 打开yaml文件
    file = open(yaml_file, 'r', encoding="utf-8")
    file_data = file.read()
    file.close()
    # 将字符串转化为字典或列表
    data = yaml.safe_load(file_data)
    print(data)
    return data
# Load Global Variable
current_path = os.path.abspath(".")
yaml_path = os.path.join(current_path, "my_motion_roadmap.yaml")
data = get_yaml_data(yaml_path)
CIRCLE_R = data['CIRCLE_R']  # 机器人以及动态障碍物的半径(假设都是圆)
POINT_GOAL = tuple(data['POINT_GOAL'])  # 全局目标
POINT_START = tuple(data['POINT_START'])  # 全局起始点
DYNAMIC_DEFAULT_COLOR = tuple(data['DYNAMIC_DEFAULT_COLOR'])  # 动态障碍物的颜色
DYNAMIC_VISIBLE_R = data['DYNAMIC_VISIBLE_R']  # 人的可视范围(假设是方形区域)的半径
REACH_GOAL_THRESHOLD = data['REACH_GOAL_THRESHOLD']
ROBOT_SPEED = data['ROBOT_SPEED']  # 机器人的速度
ROBOT_COLOR = tuple(data['ROBOT_COLOR'])  # 机器人的颜色
ROBOT_VISIBLE_R = data['ROBOT_VISIBLE_R']  # 机器人的可视范围(假设是方形区域)的半径
num_key = data['num_key']
ROBOT_CONTROL = data['ROBOT_CONTROL']
STEP_REWARD = data['STEP_REWARD']  # 机器人走一步的代价
COLLISION_REWARD = data['COLLISION_REWARD']  # 机器人发生碰撞的代价
REACH_GOAL_REWARD = data['REACH_GOAL_REWARD']  # 机器人到达终点的奖励
MAX_ATTRACTIVE = data['MAX_ATTRACTIVE']
'''
step一般是用于计算动力学方程更新参数和reward的，是物理引擎，返回
  observation
  reward
  done :判断是否到了重新设定(reset)环境
  info :用于调试的诊断信息，有时也用于学习，但智能体（agent ）在正式的评价中不允许使用该信息进行学习。
render是图像引擎用来显示环境中的物体图像
  from gym.envs.classic_control import rendering
  # 这一句导入rendering模块，利用rendering模块中的画图函数进行图形的绘制
  self.viewer = rendering.Viewer(600, 400)   # 600x400 是画板的长和框
  line1 = rendering.Line((100, 300), (500, 300))
  line2 = rendering.Line((100, 200), (500, 200))
  # 给元素添加颜色
  line1.set_color(0, 0, 0)
  line2.set_color(0, 0, 0)
  # 把图形元素添加到画板中
  self.viewer.add_geom(line1)
  self.viewer.add_geom(line2)
  return self.viewer.render(return_rgb_array=mode == 'rgb_array')
  gym rendering 画图模块
  具体参考：https://www.jianshu.com/p/bb5a7116d189
  这里我们先使用cv2,在之后再重构
进程通过调用reset()来启动，它返回一个初始observation
'''
# WARNNIG:由于简单的使用opencv作为显示,所以所有位置x,y 以索引格式得到图片上相应像素的时候需要 img[y,x]
# 为什么会产生这个问题尚未明了
class Dynamic(object):
    def __init__(self,route):
        self.point_color = DYNAMIC_DEFAULT_COLOR
        self.point_size = CIRCLE_R
        self.thickness = -1  # 可以为 0 、4、8 边框宽度 -1为填充
        self.route = route
        self.speed = len(self.route)
        self.current_step = 0
        self.dynamic_visible_r = DYNAMIC_VISIBLE_R
    def step(self):
        self.current_step = self.current_step + 1
        self.current_step = self.current_step % self.speed
        observation = self.getLocation()
        reward = 0
        done = False
        info = None
        return [observation,reward,done,info]
    # WARNNIG:由于简单的使用opencv作为显示,所以所有位置x,y 以索引格式得到图片上相应像素的时候需要 img[y,x]
    # 貌似调用opencv circle line函数都是(x,y) 维度索引是反的
    def render(self,mr):
        assert isinstance(mr,MotionRoadmap),"fuck u ,mr need to be MotionRoadMap"
        cv2.circle(mr.get_current_map(), self.getLocation(), self.point_size, \
                   self.point_color, self.thickness)
        x,y = self.getLocation()
        x_1,x_2,y_1,y_2 = mr.range_in_map(x-self.dynamic_visible_r,x+self.dynamic_visible_r,\
                                          y-self.dynamic_visible_r,y+self.dynamic_visible_r)
        return mr.get_current_map()[y_1:y_2,x_1:x_2]
    def getLocation(self):
        x, y = self.route[self.current_step]
        return (int(x),int(y))
    def reset(self):
        self.current_step = 0
        return self.getLocation()
class Circle_man(Dynamic):
    def __init__(self, r, a, b, speed):
        self.r = r
        self.a, self.b = (a, b)
        self.speed = speed
        self.theta = np.arange(0, 2 * np.pi, 2 * np.pi/speed)
        self.route = [(int(self.a + self.r * np.cos(i)), int(self.b + self.r * np.sin(i))) \
                      for i in self.theta]
        Dynamic.__init__(self, self.route)
class Linear_man(Dynamic):
    def __init__(self, start, end, speed):
        self.x1, self.y1 = start
        self.x2, self.y2 = end
        self.speed = speed
        half_speed = int(speed/2) #半个周期就要走完两个点,然后再折返,否则会闪现
        x_gap = (self.x2 - self.x1)/ half_speed
        y_gap = (self.y2 - self.y1)/ half_speed
        if x_gap!=0:
            x = np.arange(self.x1 , self.x2, x_gap)
        else:
            x = np.ones(self.speed)*self.x1
        if y_gap!=0:
            y = np.arange(self.y1 , self.y2,y_gap)
        else:
            y = np.ones(self.speed)*self.y1
        self.route = [ (x[i], y[i]) for i in range(half_speed)]
        self.route += [ (x[i], y[i]) for i in range(half_speed-1,-1,-1)]
        Dynamic.__init__(self, self.route)
class MotionRoadmap(object):
    def __init__(self, map_img):
        ## 初始化实例，需要输入一张 bmp 格式的地图
        self.static_map = map_img.copy()
        self.current_map = map_img.copy()
        # 对静态地图进行膨胀,时间复杂度较高，不建议经常调用(这里建议地图的静态障碍物是黑色的,因为下面碰撞检测是检测黑色)
        KERNEL = np.ones(((CIRCLE_R-1)*2, (CIRCLE_R-1)*2), np.uint8)  # 膨胀 20
        self.dilated_map = cv2.erode(map_img, KERNEL)
        # 读取图像尺寸
        self.size = self.static_map.shape
        self.x_max = self.size[0]
        self.y_max = self.size[1]
        # 运动规划的起点
        self.point_start = POINT_START
        # 运动规划的终点
        self.point_goal = POINT_GOAL
    def get_static_map(self):
        return self.static_map.copy()  # 转为RGB显示
    def get_dilated_map(self):
        return self.dilated_map  # 转为RGB显示
    def get_current_map(self):
        return self.current_map
    def set_current_map(self,img):
        self.current_map = img
    def reach_goal(self,point):
        if straight_distance(point,self.point_goal) <= REACH_GOAL_THRESHOLD:
            return True
        return False
    def point_in_map(self,point):
        x, y = point
        if x < self.x_max and y < self.y_max and x >= 0 and y >= 0:
            return True
        else:
            return False
    def range_in_map(self,x_start,x_end,y_start,y_end):
        if x_start < 0:
            x_start = 0
        if x_start >= self.x_max:
            x_start = self.x_max - 1
        if x_end < 0:
            x_end = 0
        if x_end >= self.x_max:
            x_end = self.x_max - 1
        if y_start < 0:
            y_start = 0
        if y_start >= self.y_max:
            y_start = self.y_max - 1
        if y_end < 0:
            y_end = 0
        if y_end >= self.y_max:
            y_end = self.y_max - 1
        return x_start,x_end,y_start,y_end
    def static_collision_detection(self, point):
        x, y = point
        feasibility = True
        if self.point_in_map(point):
            # 这里输入的是x,y 但是图片坐标是反过来的
            r,g,b = self.dilated_map[y, x]
            if  r == 0 and g == 0 and b == 0 :# 二值化之后黑色(障碍物)为0(这里最好该一下,如果别人不用黑色表示障碍物就会出错)
                feasibility = False
        else:
            feasibility = False
        return feasibility
    def static_check_path(self,point_current, point_other):
        x1, y1 = point_current
        x2, y2 = point_other
        '''路径检查的采样点数，取路径横向与纵向长度的较大值，保证每个像素都能验证到'''
        step_length = int(max(abs(x1 - x2), abs(y1 - y2)))
        path_x = np.linspace(x1, x2, step_length + 1)
        path_y = np.linspace(y1, y2, step_length + 1)
        for i in range(int(step_length + 1)):
            if not self.static_collision_detection([math.ceil(path_x[i]), math.ceil(path_y[i])]):
                return False
        return True
class Robot(Dynamic):
    def __init__(self,point_start,point_goal,speed = ROBOT_SPEED,visible_r = ROBOT_VISIBLE_R):
        Dynamic.__init__(self, [point_start])
        self.route = [point_start]
        self.point_start = point_start
        self.point_goal = point_goal
        self.RobotLocation = point_start
        self.point_color = ROBOT_COLOR
        self.speed = speed
        self.visible_r = visible_r
    def step(self,action):
        x, y = self.getLocation()
        if isinstance (action,int):# 一般使用cv2.waitkey得到的数字(其实对应的是按键的ascii)
            delta_x,delta_y = ROBOT_CONTROL.get(action,(0,0))#如果按到其他没有的key 则返回（0,0）
        elif isinstance(action, tuple):
            delta_x,delta_y = action  # 如果按到其他没有的key 则返回（0,0）
        else:
            raise("Error type")
        x = x + delta_x * self.speed
        y = y + delta_y * self.speed
        self.RobotLocation = (x, y)
        self.route.append(self.RobotLocation)
        self.current_step = self.current_step + 1
        observation = self.RobotLocation
        reward = 0
        done = False
        if (x,y)==self.point_goal:
            done = True
        info = None
        return [observation,reward,done,info]
    def getLocation(self):
        return self.RobotLocation
    def reset(self):
        self.RobotLocation = self.point_start
        self.route = [self.point_start]
        self.current_step = 0
        return self.RobotLocation
    def getRoute(self):
        return self.route
class DynamicEnv(MotionRoadmap):
    motion_direction = [(1,0),(0,1),(-1,0),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1),(0,0)]
    def __init__(self,map_img,crowd_list,robot):
        MotionRoadmap.__init__(self,map_img)
        self.crowd_list = crowd_list
        self.Robot = robot
        self.current_map = self.render()
        self.done = False
        self.reward = 0
        self.APF_static = np.array([])
        self.APF_WAY = APF_WAY
    def crowd_density_point(self,point,radius):
        density = 0
        for person in self.crowd_list:
            if straight_distance(person.getLocation(),point)<radius:
                density = density + 1
        return density
    def crowd_collision_detection(self,point):
        for person in self.crowd_list:
            if straight_distance(point,person.getLocation()) < (self.Robot.point_size + person.point_size):
                return False
        return True
    def crowd_check_path(self,point_current, point_other):
        x1, y1 = point_current
        x2, y2 = point_other
        ## 路径检查的采样点数，取路径横向与纵向长度的较大值，保证每个像素都能验证到
        step_length = int(max(abs(x1 - x2), abs(y1 - y2)))
        path_x = np.linspace(x1, x2, step_length + 1)
        path_y = np.linspace(y1, y2, step_length + 1)
        for i in range(step_length + 1):
            if not self.crowd_collision_detection([math.ceil(path_x[i]), math.ceil(path_y[i])]):
                return False
        return True
    def render(self):#图像引擎的计算，得到全局的图像current_map
        self.set_current_map(self.get_static_map())
        for person in self.crowd_list:
            person.render(self)
        self.Robot.render(self)
        img_copy = self.get_current_map().copy()
        cv2.circle(img_copy, self.point_start, 10, (255,200,0), -1)
        cv2.circle(img_copy, self.point_goal, 10, (255, 100, 0), -1)
        return img_copy
    #step可以传入键盘的key int num,也可以传入tuple
    def step(self, action):#作出一个动作，进行物理计算，并且得到相应的done，reward，observation，（info）
        for person in self.crowd_list:
            person.step()
        self.Robot.step(action)#这里之后需要修改成依照算法决策，而不是环境的step给的
        r_x,r_y = self.Robot.getLocation()
        #检查是否和动态或者静态障碍物接触，接触了就-10分
        if not self.static_collision_detection((r_x,r_y)) or not self.crowd_collision_detection((r_x,r_y)):
            if not self.static_collision_detection((r_x,r_y)):
                print("撞墙!")
            if not self.crowd_collision_detection((r_x,r_y)):
                print("撞人!")
            done = True
            reward = COLLISION_REWARD
        #检查是否到达重点
        elif self.reach_goal((r_x,r_y)):
            done = True
            reward = REACH_GOAL_REWARD
        else:
            done = False
            reward = STEP_REWARD
        observation = None
        self.reward += reward
        self.done = done
        info = None
        return [observation,reward,done,info]
    def reset(self):#重置所有状态，并且返回初始的observation
        for person in self.crowd_list:
            person.reset()
        self.Robot.reset()
        self.render()
        self.done = False
        self.reward = 0
        self.APF_static = np.array([])
        return None

    def prm_planning(self, **param):
        print('开始 PRM 路径规划，请等待,PRM对象是Full Observed的地图（Static+Dynamic Person）...')
        # 关键字参数处理
        num_sample = 100
        distance_neighbor = 200
        if 's' in param:
            num_sample = param['s']
        if 'n' in param:
            distance_neighbor = param['n']
        if not ('p' in param):
            param['p'] = True
        ## 构造地图
        # 采样并添加顶点
        vertex = [self.point_start, self.point_goal]
        while len(vertex) < (num_sample + 2):
            x = random.randint(0,self.x_max)
            y = random.randint(0,self.y_max)
            if self.static_collision_detection((x,y)) and self.crowd_collision_detection((x,y)):
                vertex.append((x,y))
        ## 构造邻接矩阵
        adjacency_mat = np.zeros((num_sample + 2, num_sample + 2))
        for i in range(num_sample + 2):
            for j in range(num_sample + 2):
                # 如果距离小于 distance_neighbor 且路径不碰撞
                if straight_distance(vertex[i], vertex[j]) <= distance_neighbor and \
                            self.static_check_path(vertex[i],vertex[j]) and \
                        self.crowd_check_path(vertex[i],vertex[j]):
                    adjacency_mat[i, j] = 1  # 邻接矩阵置为1
        ## A*算法搜索最佳路径
        self.vertex, self.adjacency_mat, self.close_list, find = A_star_algorithm(vertex, adjacency_mat, 0, 1)
        ## 根据关键字确定是否绘图
        if not (param['p'] == 'None'):
            if (find == True):
                return True,A_star_plot(self.render(), self.vertex, self.adjacency_mat, self.close_list)
            else:
                print('没有找到解，无法绘图！')
        return  False,[],None
    def del_apf_static(self):
        self.APF_static = np.array([])
        my_file = 'static_apf.out'
        if os.path.exists(my_file):
            os.remove(my_file)
            return True
        else:
            print('no such file:%s' % my_file)
            return False

    '''https://blog.csdn.net/junshen1314/article/details/50472410'''
    def apf_static(self, **param):
        if self.APF_static.size!=0 :
            return self.APF_static
        else:
            try:
                result =  np.loadtxt('static_apf.out')
                self.APF_static = result
                print('人工势场在系统中已存在')
                return result
            except IOError:
                print('开始人工势场法路径规划并保存系统，请等待...')
                # 地图灰度二值化 膨胀
                img_gray = cv2.cvtColor(self.static_map, cv2.COLOR_BGR2GRAY)
                ret, img_binary = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
                APF_KERNEL = np.ones((D_MAX*2, D_MAX*2), np.uint8)
                dilated_map = cv2.erode(img_binary, APF_KERNEL)
                # 得到有斥力作用的非obstacle点
                inflacted_img = np.ones([self.x_max,self.y_max])*255 - img_gray + dilated_map
                # 创建障碍物坐标集 and  创建斥力坐标集 and 创建静态障碍物的斥力势场图
                postion_obs = np.argwhere(img_binary == [0])#这里得到的坐标是反的表示，即(y,x)
                inflacted_zones = np.argwhere(inflacted_img == [0])#这里得到的坐标是反的表示，即(y,x)
                img_potential = np.zeros([self.x_max,self.y_max])
                # 对障碍物外的斥力范围计算静态排斥力
                print("对障碍物外的斥力范围计算静态排斥力")
                begin = time.time()
                for (y,x) in inflacted_zones:
                    # 障碍物的斥力场:uo  d_min:最近障碍物的距离 也就是距离的最小值
                    # 因为postion_obs是(y,x),所以比较的点也要是(y,x)
                    d_min = find_nearest_obstacle_distance(postion_obs,(y,x))
                    if self.APF_WAY == 0:
                        uo = APF_function(d_min)
                    elif self.APF_WAY == 1:
                        uo = my_APF_function(d_min)
                    elif self.APF_WAY == 2:
                        uo = improved_APF_function(d_min,straight_distance((x,y), self.point_goal))
                    elif self.APF_WAY == 3:
                        uo = my_improved_APF_function(d_min, straight_distance((x,y), self.point_goal))
                    img_potential[x, y] = uo
                end = time.time()
                print(" 对障碍物外的斥力范围计算静态排斥力势场图创建完毕,花费",end-begin,"s")
                # 对障碍物内的斥力范围给予恒定排斥力
                for (y, x) in postion_obs:
                    img_potential[x, y] = OBSTACLE_MAX
                np.savetxt('static_apf.out',img_potential)
                # plt.imshow(img_potential, cmap=plt.cm.hot, vmin=255, vmax=0)
                # plt.colorbar()
                # plt.show()
                self.APF_static = img_potential
                return img_potential
    def apf_dynamic(self,point):
        Repulsive = []
        # 障碍物的斥力场 uo  d_min:最近障碍物的距离 也就是距离的最小值
        for person in self.crowd_list:
            D = straight_distance(person.getLocation(),point)
            uo = OBSTACLE_MAX
            if D > CIRCLE_R:# 在CIRCLE_R*2之内的都是OBSTACLE_MAX
                if self.APF_WAY == 0:
                    uo = APF_function(D)  # 3 rou的时候接近0
                elif self.APF_WAY == 1:
                    uo = my_APF_function(D)  # 3 rou的时候接近0
                elif self.APF_WAY == 2:
                    uo = improved_APF_function(D, straight_distance(point, self.point_goal))
                elif self.APF_WAY == 3:
                    uo = my_improved_APF_function(D, straight_distance(point, self.point_goal))
            Repulsive.append(uo)
        return Repulsive
    def apf_goal(self,point, goal):
        return straight_distance(point,goal)
    def apf_cul(self,location,goal):
        if self.static_collision_detection(location):# 没有发生静态碰撞
            repulsive = mr.apf_static()[location[0],location[1]] + sum(mr.apf_dynamic(location))
            attractive = self.apf_goal(location,goal)/MAX_ATTRACTIVE * OBSTACLE_MAX
            return  attractive + repulsive
        else:
            return OBSTACLE_MAX
    '''得到location这一点周围梯度最大的下一点,并且输出'''
    def apf_next_guide(self,location,next_goal):
        potential_current = self.apf_cul(location, next_goal)
        D = (0,0)#如果陷入局部最优 有一定概率要随机游走
        for d in self.motion_direction:  # 选出梯度最大的那一个点
            index_x = location[0] + d[0]
            index_y = location[1] + d[1]
            if ((index_x < 0) or (index_x >= self.x_max) or
                    (index_y < 0) or (index_y >= self.y_max)):
                potential_next = float('inf')
            else:
                potential_next = self.apf_cul((index_x, index_y), next_goal)
            if potential_current > potential_next:
                potential_current = potential_next
                D = d
            if D == (0,0):
                if random.random() >0.4:
                    D = random.choice(self.motion_direction)
        return D


PRM_POINT_NUM = 150
PRM_DISTANCE = 150
foldname = "animFram/22/"
step_num = 0
collision_time = 0
reward = 0
IMAGE_PATH = "map_1.bmp"
if __name__ == "__main__":
    # 图像路径 我画的图片静态障碍物是（100,100）（400,300）的矩形 圆的r=15
    image_path = IMAGE_PATH
    img = cv2.imread(image_path)  # np.ndarray BGR uint8
    img = cv2.resize(img, (500, 500))
    crowd_list = []
    crowd_list.append(Linear_man((350,90),(400,15),20))#横轴是x 从左到右变大 竖轴是y 从上到下变大
    crowd_list.append(Linear_man((100,400),(150,350),10))#start end speed
    crowd_list.append(Circle_man(25,50,75,25))# r a,b speed
    crowd_list.append(Circle_man(25,150,45,50))
    crowd_list.append(Circle_man(25,250,45,15))
    crowd_list.append(Circle_man(25,400,400,20))
    robot = Robot(POINT_START, POINT_GOAL)
    mr = DynamicEnv(img, crowd_list, robot)
    mr.reset()
    sign,path_and_img = mr.prm_planning(s=PRM_POINT_NUM,n=PRM_DISTANCE)
    if sign:
        path = path_and_img[0]
        img = path_and_img[1]
        cv2.imwrite(foldname + "prm_planning.jpg", img)
        print(path)
        mr.del_apf_static()
        k = len(path) - 2
        mr.point_goal=path[k]
        mr.apf_static()
        print("game begin")
        for i in range(700):
            #  visualization and save img
            img = mr.render()
            x, y = mr.Robot.getLocation()
            for h in range(x-50,x+50):
                for u in range(y-50,y+50):
                    apf = mr.apf_cul((h, u),path[k])
                    cv2.circle(img,(h,u),1,(apf,apf,apf),-1)
            if reward == COLLISION_REWARD:
                cv2.circle(img, (x, y), 10, (100, 200, 150), -1)
            else:
                cv2.circle(img, (x, y), 10, (255, 255, 0), -1)
            cv2.circle(img,path[k],5,(255,100,0),1)
            cv2.imshow("img", img)
            cv2.imwrite(foldname+"img"+str(i)+".jpg",img)
            # detect shutdown signal
            key = cv2.waitKey(1)#1000ms=1s
            if key == 27:
                break
            # get_robot_APF_guide and step
            observation,reward, done,info = mr.step(mr.apf_next_guide((x,y),path[k]))
            step_num += 1
            # detect whether have a collision or reach local goal
            if reward == COLLISION_REWARD:
                collision_time += 1
                print("collision!",collision_time)
            if reward == REACH_GOAL_REWARD :

                k -= 1
                # if k == -1 than reach the end
                if k == -1:
                    break
                # or reach the local goal ,keep going
                print("reach local goal", k)
                mr.del_apf_static()
                mr.point_goal = path[k]
                mr.apf_static()
                mr.reward-=REACH_GOAL_REWARD
            if collision_time>30:
                break
        py_object={
            'Reward':mr.reward,
            'Step_num':step_num,
            'Collision_time':collision_time,
            'PRM_POINT_NUM':PRM_POINT_NUM,
            'PRM_DISTANCE':PRM_DISTANCE,
            'APF_WAY':APF_WAY,
            'route':mr.Robot.getRoute()
        }
        generate_yaml_doc(foldname+"setting_and_result.yaml",py_object)
        print(py_object)








