# Graduation-Design-The-theoretical-part
my_motion_planning_toolbox.py	   
my_motion_planning_toolbox.yaml	   
my_motion_roadmap.py	   
my_motion_roadmap.yaml    
是四个主要的工程，my_motion_planning_toolbox.py和my_motion_planning_toolbox.yaml主要是负责一些通用函数，比如距离，以及APF的斥力函数和配置等东西
而my_motion_planning_toolbox.py是主要的文件，实现了Dynamic以及其子类Circle_man、Linear_man、Robot；实现了MotionRoadmap以及其子类DynamicEnv（实现baseline）、DE2(DynamicEnv)（这个DE2实现的是改进的A* 算法），创建了9个测试环境，分别是world0-8,实现了向量比较的类Vector。
