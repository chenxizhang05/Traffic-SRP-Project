"""
Created on Tue Sep 13 19:59:21 2022

@author: calvi
"""

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math 

# this is a vector class to store the xy position 
class Location: 
    x = 0.0
    y = 0.0
    
    
    def __init__(self, x_val , y_val):
        self.x = x_val
        self.y = y_val 
        
    def set(self, x_val , y_val):
        self.x = x_val
        self.y = y_val 
        
    def get_x(self):
        return self.x 
    
    def get_y(self):
        return self.y 


# this is a class to store the index of where a road starts and ends 
class Road: 
    
    def __init__(self, s , e):
        self.start = s
        self.end = e

# this is a node to store the position of itself, and which index it is , and which roads are connected to it 
class Node: 

    def __init__(self, i , x_val , y_val) :
       self.index = i 
       self.pos = Location(x_val, y_val)
       self.link_nodes = [] 

    def add_road_links(self, node) :
        if (self.index != node.index):
            if not self.is_node_in_list(node):
                self.link_nodes.append(node)

    def is_node_in_list(self, node):
        for nd in self.link_nodes:
            if nd.pos.x == node.pos.x and nd.pos.y == node.pos.y:
                return True
        return False

    def is_intersection(self):
        return len(self.link_nodes)>2

    def neighbouring_nodes(self):
        return len(self.link_nodes)

    def distance(self, point):
        d = math.sqrt((point.x - self.pos.x)**2 + (point.y - self.pos.y)**2)
        return d
    
    def distance_nodes(self):
        dist=[]
        for node in self.link_nodes:
            distance_ = math.sqrt((node.pos.x - self.pos.x)**2 + (node.pos.y - self.pos.y)**2)
            dist.append(distance_)
        return sum(dist)
    
    def angle_nodes(self):
        angles = []
        for node in self.link_nodes:
            angle = math.atan(node.pos.y/node.pos.x)
            if angle < 0:
                angle+= 2*math.pi
            angles.append([angle,node])
        angles.sort()
        angles2=[]
        for index in range(0, len(angles)-1):
            index1 = index + 1
            if index + 1 == len(angles):
                index1 = 0
            cos1 = ((angles[index][1].pos.x-self.pos.x)**2 + (angles[index][1].pos.y-self.pos.y)**2 + (angles[index1][1].pos.x-self.pos.x)**2 + (angles[index1][1].pos.y-self.pos.y)**2- (angles[index1][1].pos.x - angles[index][1].pos.x)**2 +(angles[index1][1].pos.y-angles[index][1].pos.y)**2)/(2*math.sqrt((angles[index][1].pos.x-self.pos.x)**2 +(angles[index][1].pos.y-self.pos.y)**2)*math.sqrt((angles[index1][1].pos.x-self.pos.x)**2 +(angles[index1][1].pos.y-self.pos.y)**2))
            if -1 < cos1 < 1:
                angles2.append(math.acos(cos1))
        anglessquared=[]
        for x in angles2:
            anglessquared.append(math.sin(2*x)**2)
        return sum (anglessquared)

    def danger_index(self):
        danger_index = 0.615385 * self.distance_nodes() + 0.307692 * self.neighbouring_nodes() + 0.076923 * self.angle_nodes()
        return danger_index
    
    def intersection(self, road1, road2):
        if road1.end.pos.x != road1.start.pos.x and road2.end.pos.x != road2.start.pos.x:
            g1 = (road1.end.pos.y-road1.start.pos.y)/(road1.end.pos.x-road1.start.pos.x)
            g2 = (road2.end.pos.y-road2.start.pos.y)/(road2.end.pos.x-road2.start.pos.x)
            x = ((-g2*road2.end.pos.x + road2.end.pos.y)-(-g1*road1.end.pos.x + road1.end.pos.y))/(g1-g2)
            y = g2*x - g2 * road2.end.pos.x + road2.end.pos.y
            point = Location(x, y)
        elif road1.end.pos.x == road1.start.pos.x and road2.end.pos.x != road2.start.pos.x:
            g2 = (road2.end.pos.y-road2.start.pos.y)/(road2.end.pos.x-road2.start.pos.x)
            x = road1.end.pos.x
            y = g2*x - g2 * road2.end.pos.x + road2.end.pos.y
            point = Location(x, y)
        elif road1.end.pos.x != road1.start.pos.x and road2.end.pos.x == road2.start.pos.x:
            g1 = (road1.end.pos.y-road1.start.pos.y)/(road1.end.pos.x-road1.start.pos.x)
            x = road2.end.pos.x
            y = g1*x - g1 * road1.end.pos.x + road1.end.pos.y
            point = Location(x, y)
        else:
            print('the lines are paralle1')
        return point
    
    
    def operation(self):
        times = 100
        if len(self.link_nodes) == 3:
            for times in range(100): 
                times -= 1
                n1 = self.link_nodes[0]
                n2 = self.link_nodes[1]
                n3 = self.link_nodes[2]
                r1 = Road(self, n1)
                r2 = Road(self, n2)
                r3 = Road(self, n3)
                r4 = Road(n1, n2)
                r5 = Road(n1, n3)
                r6 = Road(n2, n3)
                final1 = self.intersection(r1, r6)
                final2 = self.intersection(r2, r5)
                final3 = self.intersection(r3, r4)
                w1 = self.distance(final1)/(self.distance(final1) + self.distance(final2) + self.distance(final3))
                w2 = self.distance(final2)/(self.distance(final1) + self.distance(final2) + self.distance(final3))
                w3 = self.distance(final3)/(self.distance(final1) + self.distance(final2) + self.distance(final3))
                final_x = final1.x * w1 + final2.x * w2 + final3.x * w3
                final_y = final1.y * w1 + final2.y * w2 + final3.y * w3
                final = Location(final_x, final_y)
                self.pos = final
             
        
### this is a network class to hold everything together 
class Road_Network:
    
    def __init__(self) :
        self.nodes = [] 
        self.roads = [] 
        
    def add_new_node(self, i, x_val, y_val):
        new_node = Node(i, x_val, y_val)
        self.nodes.append(new_node) 
        
    def add_new_road(self, start_index, end_index):
        r = Road(start_index, end_index)
        self.roads.append(r)
        self.nodes[start_index].add_road_links(self.nodes[end_index])
        self.nodes[end_index].add_road_links(self.nodes[start_index])

    def find_index(self, x_val, y_val):
        for node in self.nodes:
            if node.pos.x == x_val and node.pos.y == y_val:
                return node.index
        return 0

line_count = 0
point_count = 0  
road_count = 0 

        
road_network = Road_Network() # road network class 
        
file = open("/Users/awesomebunny1/Desktop/SRP/street network/data files/roads.txt", "r")
for line in file:    
    if line[-1:] == "\n":
        line = line[:-1]

    if len(line)!=0:
        line_count += 1 
        linetemp = line.split(' ') # this becomes a list of strs 
        float_list = [float(i) for i in linetemp] # this becomes a list of floats 

        if (line_count == 1) :
            point_count = int(float_list[0])
        elif (line_count == 2) : 
            road_count = int(float_list[0])
    
        # reading in the positions 
        if (line_count > 2 ) and (line_count < 2 + point_count + 1 ): 
           road_network.add_new_node(line_count-2, float_list[0], float_list[1])
      
        # reading in the road linkages 
        if (line_count > 2 + point_count ): 
            road_network.add_new_road(int(float_list[0]), int(float_list[1]))
            

for node in road_network.nodes:
    neighbourhood = []
    list_nodes1 = node.link_nodes.copy()
    danger_neighbourhood = []
    danger_nodes = []
    if node.is_intersection():
        list_nodes1.append(node)
        for node in list_nodes1: 
            danger_neighbourhood.append(node.danger_index()**2)
        if node.danger_index() > sum(danger_neighbourhood)/len(danger_neighbourhood):
            danger_nodes.append(node)
            node.operation()

for node in road_network.nodes:
    #print(road.start,road.end)
    if node.is_intersection():
        for node1 in node.link_nodes:
            xpoints1 = [node.pos.x, node1.pos.x]
            ypoints1 = [node.pos.y, node1.pos.y]
            plt.plot(xpoints1, ypoints1, color='red')
    else:
        for node2 in node.link_nodes:
            xpoints2 = [node.pos.x, node2.pos.x]
            ypoints2 = [node.pos.y, node2.pos.y]
            plt.plot(xpoints2, ypoints2, color='green')    
plt.show()
    #print(node.index, node.pos.x,node.pos.y,node.link_roads)

"""
Created on Tue Sep 13 19:59:21 2022

@author: calvi
"""

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math 

# this is a vector class to store the xy position 
class Location: 
    x = 0.0
    y = 0.0
    
    
    def __init__(self, x_val , y_val):
        self.x = x_val
        self.y = y_val 
        
    def set(self, x_val , y_val):
        self.x = x_val
        self.y = y_val 
        
    def get_x(self):
        return self.x 
    
    def get_y(self):
        return self.y 


# this is a class to store the index of where a road starts and ends 
class Road: 
    
    def __init__(self, s , e):
        self.start = s
        self.end = e

# this is a node to store the position of itself, and which index it is , and which roads are connected to it 
class Node: 

    def __init__(self, i , x_val , y_val) :
       self.index = i 
       self.pos = Location(x_val, y_val)
       self.link_nodes = [] 

    def add_road_links(self, node) :
        if (self.index != node.index):
            if not self.is_node_in_list(node):
                self.link_nodes.append(node)

    def is_node_in_list(self, node):
        for nd in self.link_nodes:
            if nd.pos.x == node.pos.x and nd.pos.y == node.pos.y:
                return True
        return False

    def is_intersection(self):
        return len(self.link_nodes)>2

    def neighbouring_nodes(self):
        return len(self.link_nodes)

    def distance(self, point):
        d = math.sqrt((point.x - self.pos.x)**2 + (point.y - self.pos.y)**2)
        return d
    
    def distance_nodes(self):
        dist=[]
        for node in self.link_nodes:
            distance_ = math.sqrt((node.pos.x - self.pos.x)**2 + (node.pos.y - self.pos.y)**2)
            dist.append(distance_)
        return sum(dist)
    
    def angle_nodes(self):
        angles = []
        for node in self.link_nodes:
            angle = math.atan(node.pos.y/node.pos.x)
            if angle < 0:
                angle+= 2*math.pi
            angles.append([angle,node])
        angles.sort()
        angles2=[]
        for index in range(0, len(angles)-1):
            index1 = index + 1
            if index + 1 == len(angles):
                index1 = 0
            cos1 = ((angles[index][1].pos.x-self.pos.x)**2 + (angles[index][1].pos.y-self.pos.y)**2 + (angles[index1][1].pos.x-self.pos.x)**2 + (angles[index1][1].pos.y-self.pos.y)**2- (angles[index1][1].pos.x - angles[index][1].pos.x)**2 +(angles[index1][1].pos.y-angles[index][1].pos.y)**2)/(2*math.sqrt((angles[index][1].pos.x-self.pos.x)**2 +(angles[index][1].pos.y-self.pos.y)**2)*math.sqrt((angles[index1][1].pos.x-self.pos.x)**2 +(angles[index1][1].pos.y-self.pos.y)**2))
            if -1 < cos1 < 1:
                angles2.append(math.acos(cos1))
        anglessquared=[]
        for x in angles2:
            anglessquared.append(math.sin(2*x)**2)
        return sum (anglessquared)

    def danger_index(self):
        danger_index = 0.615385 * self.distance_nodes() + 0.307692 * self.neighbouring_nodes() + 0.076923 * self.angle_nodes()
        return danger_index
    
    def intersection(self, road1, road2):
        if road1.end.pos.x != road1.start.pos.x and road2.end.pos.x != road2.start.pos.x:
            g1 = (road1.end.pos.y-road1.start.pos.y)/(road1.end.pos.x-road1.start.pos.x)
            g2 = (road2.end.pos.y-road2.start.pos.y)/(road2.end.pos.x-road2.start.pos.x)
            x = ((-g2*road2.end.pos.x + road2.end.pos.y)-(-g1*road1.end.pos.x + road1.end.pos.y))/(g1-g2)
            y = g2*x - g2 * road2.end.pos.x + road2.end.pos.y
            point = Location(x, y)
        elif road1.end.pos.x == road1.start.pos.x and road2.end.pos.x != road2.start.pos.x:
            g2 = (road2.end.pos.y-road2.start.pos.y)/(road2.end.pos.x-road2.start.pos.x)
            x = road1.end.pos.x
            y = g2*x - g2 * road2.end.pos.x + road2.end.pos.y
            point = Location(x, y)
        elif road1.end.pos.x != road1.start.pos.x and road2.end.pos.x == road2.start.pos.x:
            g1 = (road1.end.pos.y-road1.start.pos.y)/(road1.end.pos.x-road1.start.pos.x)
            x = road2.end.pos.x
            y = g1*x - g1 * road1.end.pos.x + road1.end.pos.y
            point = Location(x, y)
        else:
            print('the lines are paralle1')
        return point
    
    
    def operation(self):
        times = 0
        if len(self.link_nodes) == 3:
            while times > 0: 
                times -= 1
                n1 = self.link_nodes[0]
                n2 = self.link_nodes[1]
                n3 = self.link_nodes[2]
                r1 = Road(self, n1)
                r2 = Road(self, n2)
                r3 = Road(self, n3)
                r4 = Road(n1, n2)
                r5 = Road(n1, n3)
                r6 = Road(n2, n3)
                final1 = self.intersection(r1, r6)
                final2 = self.intersection(r2, r5)
                final3 = self.intersection(r3, r4)
                w1 = self.distance(final1)/(self.distance(final1) + self.distance(final2) + self.distance(final3))
                w2 = self.distance(final2)/(self.distance(final1) + self.distance(final2) + self.distance(final3))
                w3 = self.distance(final3)/(self.distance(final1) + self.distance(final2) + self.distance(final3))
                final_x = final1.x * w1 + final2.x * w2 + final3.x * w3
                final_y = final1.y * w1 + final2.y * w2 + final3.y * w3
                final = Location(final_x, final_y)
                self.pos = final
                return final 
             
        
### this is a network class to hold everything together 
class Road_Network:
    
    def __init__(self) :
        self.nodes = [] 
        self.roads = [] 
        
    def add_new_node(self, i, x_val, y_val):
        new_node = Node(i, x_val, y_val)
        self.nodes.append(new_node) 
        
    def add_new_road(self, start_index, end_index):
        r = Road(start_index, end_index)
        self.roads.append(r)
        self.nodes[start_index].add_road_links(self.nodes[end_index])
        self.nodes[end_index].add_road_links(self.nodes[start_index])

    def find_index(self, x_val, y_val):
        for node in self.nodes:
            if node.pos.x == x_val and node.pos.y == y_val:
                return node.index
        return 0

line_count = 0
point_count = 0  
road_count = 0 

        
road_network = Road_Network() # road network class 
        
file = open("/Users/awesomebunny1/Desktop/SRP/street network/data files/roads.txt", "r")
for line in file:    
    if line[-1:] == "\n":
        line = line[:-1]

    if len(line)!=0:
        line_count += 1 
        linetemp = line.split(' ') # this becomes a list of strs 
        float_list = [float(i) for i in linetemp] # this becomes a list of floats 

        if (line_count == 1) :
            point_count = int(float_list[0])
        elif (line_count == 2) : 
            road_count = int(float_list[0])
    
        # reading in the positions 
        if (line_count > 2 ) and (line_count < 2 + point_count + 1 ): 
           road_network.add_new_node(line_count-2, float_list[0], float_list[1])
      
        # reading in the road linkages 
        if (line_count > 2 + point_count ): 
            road_network.add_new_road(int(float_list[0]), int(float_list[1]))
            

for node in road_network.nodes:
    neighbourhood = []
    list_nodes1 = node.link_nodes.copy()
    danger_neighbourhood = []
    danger_nodes = []
    if node.is_intersection():
        list_nodes1.append(node)
        for node in list_nodes1: 
            danger_neighbourhood.append(node.danger_index()**2)
        if node.danger_index() > sum(danger_neighbourhood)/len(danger_neighbourhood):
            danger_nodes.append(node)
            print(node.operation()
)

for node in road_network.nodes:
    #print(road.start,road.end)
    if node.is_intersection():
        for node1 in node.link_nodes:
            xpoints1 = [node.pos.x, node1.pos.x]
            ypoints1 = [node.pos.y, node1.pos.y]
            plt.plot(xpoints1, ypoints1, color='red')
    else:
        for node2 in node.link_nodes:
            xpoints2 = [node.pos.x, node2.pos.x]
            ypoints2 = [node.pos.y, node2.pos.y]
            plt.plot(xpoints2, ypoints2, color='green')    
plt.show()
    #print(node.index, node.pos.x,node.pos.y,node.link_roads)


