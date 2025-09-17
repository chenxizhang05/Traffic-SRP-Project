import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math
from shapely.geometry import Polygon, Point

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

    def is_node_in_list(self, node):
        for nd in self.link_nodes:
            if nd.pos.x == node.pos.x and nd.pos.y == node.pos.y:
                return True
        return False
       
    def add_road_links(self, node) :
        if (self.index != node.index):
            if not self.is_node_in_list(node):
                self.link_nodes.append(node)
                
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
        
    def call_node_from_index(self, index):
        for node in self.nodes:
            if node.index == index:
                return node
            
    def pick_ten_equal_points(self, road):
        point1x = (10*self.call_node_from_index(road.start).pos.x + self.call_node_from_index(road.end).pos.x)/11
        point1y = (10*self.call_node_from_index(road.start).pos.y + self.call_node_from_index(road.end).pos.y)/11
        point2x = (9*self.call_node_from_index(road.start).pos.x + 2*self.call_node_from_index(road.end).pos.x)/11
        point2y = (9*self.call_node_from_index(road.start).pos.y + 2*self.call_node_from_index(road.end).pos.y)/11
        point3x = (8*self.call_node_from_index(road.start).pos.x + 3*self.call_node_from_index(road.end).pos.x)/11
        point3y = (8*self.call_node_from_index(road.start).pos.y + 3*self.call_node_from_index(road.end).pos.y)/11
        point4x = (7*self.call_node_from_index(road.start).pos.x + 4*self.call_node_from_index(road.end).pos.x)/11
        point4y = (7*self.call_node_from_index(road.start).pos.y + 4*self.call_node_from_index(road.end).pos.y)/11
        point5x = (6*self.call_node_from_index(road.start).pos.x + 5*self.call_node_from_index(road.end).pos.x)/11
        point5y = (6*self.call_node_from_index(road.start).pos.y + 5*self.call_node_from_index(road.end).pos.y)/11
        point6x = (5*self.call_node_from_index(road.start).pos.x + 6*self.call_node_from_index(road.end).pos.x)/11
        point6y = (5*self.call_node_from_index(road.start).pos.y + 6*self.call_node_from_index(road.end).pos.y)/11
        point7x = (4*self.call_node_from_index(road.start).pos.x + 7*self.call_node_from_index(road.end).pos.x)/11
        point7y = (4*self.call_node_from_index(road.start).pos.y + 7*self.call_node_from_index(road.end).pos.y)/11
        point8x = (3*self.call_node_from_index(road.start).pos.x + 8*self.call_node_from_index(road.end).pos.x)/11
        point8y = (3*self.call_node_from_index(road.start).pos.y + 8*self.call_node_from_index(road.end).pos.y)/11
        point9x = (2*self.call_node_from_index(road.start).pos.x + 9*self.call_node_from_index(road.end).pos.x)/11
        point9y = (2*self.call_node_from_index(road.start).pos.y + 9*self.call_node_from_index(road.end).pos.y)/11
        point10x = (self.call_node_from_index(road.start).pos.x + 10*self.call_node_from_index(road.end).pos.x)/11
        point10y = (self.call_node_from_index(road.start).pos.y + 10*self.call_node_from_index(road.end).pos.y)/11
        empty_set = [Point(point1x, point1y), Point(point2x, point2y), Point(point3x, point3y), Point(point4x, point4y), Point(point5x, point5y), Point(point6x, point6y), Point(point7x, point7y), Point(point8x, point8y), Point(point9x, point9y), Point(point10x, point10y)]
        return empty_set
    
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
            
class Building_Network:
    def __init__(self):
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
        
    def buildingrectangles(self, connected):
        maximum_x = -1000000000
        maximum_y = -1000000000
        minimum_x = 1000000000
        minimum_y = 1000000000
        for i in connected:
            maximum_x = max(i.pos.x, maximum_x)
            maximum_y = max(i.pos.y, maximum_y)
            minimum_x = min(i.pos.x, minimum_x)
            minimum_y = min(i.pos.y, minimum_y)
        p1 = (maximum_x, maximum_y)
        p2 = (maximum_x, minimum_y)
        p3 = (minimum_x, minimum_y)
        p4 = (minimum_x, maximum_y)
        coords = [p1, p2, p3, p4]
        poly = Polygon(coords)
        return poly
    
    def in_area_of_polygon(self, polygona, set_of_ten_equal_points):
        for i in set_of_ten_equal_points:
            i.within(polygona)

for roads in road_network.roads:
    for i in range(len(buildinglist)):
        if building_network. in_area_of_polygon(buildinglist[i], road_network.pick_ten_equal_points(roads)) != False:
            ##break node.operation()
            
line_count1 = 0
point_count1 = 0  
road_count1 = 0 
   
building_network = Building_Network() # road network class
        
file1 = open("/Users/awesomebunny1/Desktop/SRP/street network/data files/buildings.txt", "r")
for line1 in file1:    
    if line1[-1:] == "\n":
        line1 = line1[:-1]
        
    if len(line1)!=0:
        line_count1 += 1 
        linetemp1 = line1.split(' ') # this becomes a list of strs 
        float_list1 = [float(i) for i in linetemp1] # this becomes a list of floats 

        if (line_count1 == 1) :
            point_count1 = int(float_list1[0])
        elif (line_count1 == 2) : 
            road_count1 = int(float_list1[0])
    
        # reading in the positions 
        if (line_count1 > 2 ) and (line_count1 < 2 + point_count1 + 1 ): 
           building_network.add_new_node(line_count1-2, float_list1[0], float_list1[1])
      
        # reading in the road linkages 
        if (line_count1 > 2 + point_count1 ): 
            building_network.add_new_road(int(float_list1[0]), int(float_list1[1]))
            
cc1 = [building_network.nodes[0], building_network.nodes[29], building_network.nodes[45], building_network.nodes[209], building_network.nodes[210], building_network.nodes[211]]
cc2 = [building_network.nodes[1], building_network.nodes[2], building_network.nodes[17], building_network.nodes[166]]
cc3 = [building_network.nodes[3], building_network.nodes[4], building_network.nodes[12], building_network.nodes[13], building_network.nodes[37], building_network.nodes[212], building_network.nodes[213], building_network.nodes[244]]
cc4 = [building_network.nodes[5], building_network.nodes[27], building_network.nodes[28], building_network.nodes[214]]
cc5 = [building_network.nodes[6], building_network.nodes[7], building_network.nodes[113], building_network.nodes[115]]
cc6 = [building_network.nodes[8], building_network.nodes[116], building_network.nodes[118], building_network.nodes[173]]
cc7 = [building_network.nodes[9], building_network.nodes[10], building_network.nodes[11], building_network.nodes[69], building_network.nodes[70], building_network.nodes[161], building_network.nodes[162], building_network.nodes[163]]
cc8 = [building_network.nodes[14], building_network.nodes[20], building_network.nodes[36], building_network.nodes[236]]
cc9 = [building_network.nodes[15], building_network.nodes[16], building_network.nodes[71], building_network.nodes[165], building_network.nodes[231]]
cc10 = [building_network.nodes[18], building_network.nodes[19], building_network.nodes[30], building_network.nodes[38]]
cc11 = [building_network.nodes[21], building_network.nodes[25], building_network.nodes[207], building_network.nodes[208]]
cc12 = [building_network.nodes[22], building_network.nodes[23], building_network.nodes[24], building_network.nodes[61], building_network.nodes[235], building_network.nodes[237], building_network.nodes[238], building_network.nodes[245]]
cc13 = [building_network.nodes[26], building_network.nodes[31], building_network.nodes[46], building_network.nodes[246]]
cc14 = [building_network.nodes[32], building_network.nodes[33], building_network.nodes[148], building_network.nodes[149], building_network.nodes[155], building_network.nodes[164], building_network.nodes[189]]
cc15 = [building_network.nodes[34], building_network.nodes[39], building_network.nodes[62], building_network.nodes[63]]
cc16 = [building_network.nodes[35], building_network.nodes[73], building_network.nodes[152], building_network.nodes[153], building_network.nodes[154], building_network.nodes[160]]
cc17 = [building_network.nodes[40], building_network.nodes[41], building_network.nodes[42], building_network.nodes[43], building_network.nodes[44], building_network.nodes[48], building_network.nodes[49], building_network.nodes[50], building_network.nodes[54], building_network.nodes[58]]
cc18 = [building_network.nodes[47], building_network.nodes[60], building_network.nodes[96], building_network.nodes[122]]
cc19 = [building_network.nodes[51], building_network.nodes[53], building_network.nodes[170], building_network.nodes[218], building_network.nodes[240], building_network.nodes[268]]
cc20 = [building_network.nodes[52], building_network.nodes[85], building_network.nodes[117], building_network.nodes[151]]
cc21 = [building_network.nodes[55], building_network.nodes[92], building_network.nodes[94], building_network.nodes[131]]
cc22 = [building_network.nodes[56], building_network.nodes[66], building_network.nodes[93], building_network.nodes[100], building_network.nodes[127], building_network.nodes[133], building_network.nodes[169], building_network.nodes[176]]
cc23 = [building_network.nodes[57], building_network.nodes[95], building_network.nodes[128], building_network.nodes[205]]
cc24 = [building_network.nodes[59], building_network.nodes[88], building_network.nodes[183], building_network.nodes[243]]
cc25 = [building_network.nodes[64], building_network.nodes[91], building_network.nodes[167], building_network.nodes[230]]
cc26 = [building_network.nodes[65], building_network.nodes[87], building_network.nodes[98], building_network.nodes[99]]
cc27 = [building_network.nodes[67], building_network.nodes[68], building_network.nodes[76], building_network.nodes[97]]
cc28 = [building_network.nodes[72], building_network.nodes[74], building_network.nodes[75], building_network.nodes[232]]
cc29 = [building_network.nodes[77], building_network.nodes[83], building_network.nodes[187], building_network.nodes[199]]
cc30 = [building_network.nodes[78], building_network.nodes[80], building_network.nodes[105], building_network.nodes[106], building_network.nodes[110], building_network.nodes[174], building_network.nodes[182], building_network.nodes[223]]
cc31 = [building_network.nodes[79], building_network.nodes[112], building_network.nodes[114], building_network.nodes[184]]
cc32 = [building_network.nodes[81], building_network.nodes[82], building_network.nodes[186], building_network.nodes[200]]
cc33 = [building_network.nodes[84], building_network.nodes[137], building_network.nodes[143], building_network.nodes[144]]
cc34 = [building_network.nodes[86], building_network.nodes[134], building_network.nodes[145], building_network.nodes[196]]
cc35 = [building_network.nodes[89], building_network.nodes[129], building_network.nodes[130], building_network.nodes[222]]
cc36 = [building_network.nodes[90], building_network.nodes[233], building_network.nodes[239], building_network.nodes[242]]
cc37 = [building_network.nodes[101], building_network.nodes[123], building_network.nodes[126], building_network.nodes[241]]
cc38 = [building_network.nodes[102], building_network.nodes[109], building_network.nodes[111], building_network.nodes[126]]
cc39 = [building_network.nodes[103], building_network.nodes[104], building_network.nodes[193], building_network.nodes[201]]
cc40 = [building_network.nodes[107], building_network.nodes[108], building_network.nodes[146], building_network.nodes[198]]
cc41 = [building_network.nodes[119], building_network.nodes[138], building_network.nodes[139], building_network.nodes[192]]
cc42 = [building_network.nodes[120], building_network.nodes[121], building_network.nodes[190], building_network.nodes[191]]
cc43 = [building_network.nodes[124], building_network.nodes[125], building_network.nodes[158], building_network.nodes[185]]
cc44 = [building_network.nodes[132], building_network.nodes[175], building_network.nodes[215], building_network.nodes[216]]
cc45 = [building_network.nodes[135], building_network.nodes[136], building_network.nodes[168], building_network.nodes[194]]
cc46 = [building_network.nodes[140], building_network.nodes[177], building_network.nodes[195], building_network.nodes[220]]
cc47 = [building_network.nodes[141], building_network.nodes[142], building_network.nodes[147], building_network.nodes[197]]
cc48 = [building_network.nodes[150], building_network.nodes[159], building_network.nodes[188], building_network.nodes[204]]
cc49 = [building_network.nodes[156], building_network.nodes[157], building_network.nodes[202], building_network.nodes[203]]
cc50 = [building_network.nodes[171], building_network.nodes[172], building_network.nodes[217], building_network.nodes[219], building_network.nodes[224], building_network.nodes[225], building_network.nodes[227], building_network.nodes[228], building_network.nodes[247], building_network.nodes[248], building_network.nodes[249], building_network.nodes[250], building_network.nodes[251], building_network.nodes[252], building_network.nodes[253], building_network.nodes[254], building_network.nodes[255], building_network.nodes[256], building_network.nodes[257], building_network.nodes[258], building_network.nodes[259], building_network.nodes[260], building_network.nodes[261], building_network.nodes[262], building_network.nodes[263], building_network.nodes[264], building_network.nodes[265], building_network.nodes[266], building_network.nodes[267], building_network.nodes[269], building_network.nodes[270], building_network.nodes[271], building_network.nodes[272], building_network.nodes[273], building_network.nodes[274], building_network.nodes[275], building_network.nodes[276], building_network.nodes[277], building_network.nodes[278], building_network.nodes[279], building_network.nodes[280], building_network.nodes[281], building_network.nodes[282], building_network.nodes[283], building_network.nodes[284], building_network.nodes[285], building_network.nodes[286], building_network.nodes[287], building_network.nodes[288], building_network.nodes[289], building_network.nodes[290], building_network.nodes[291], building_network.nodes[292], building_network.nodes[293], building_network.nodes[294], building_network.nodes[295], building_network.nodes[296], building_network.nodes[297], building_network.nodes[298], building_network.nodes[299], building_network.nodes[300], building_network.nodes[301], building_network.nodes[302], building_network.nodes[303], building_network.nodes[304], building_network.nodes[305], building_network.nodes[306], building_network.nodes[307], building_network.nodes[308], building_network.nodes[309], building_network.nodes[310], building_network.nodes[311], building_network.nodes[312], building_network.nodes[313], building_network.nodes[314], building_network.nodes[315], building_network.nodes[316], building_network.nodes[317], building_network.nodes[318], building_network.nodes[319]]
cc51 = [building_network.nodes[178], building_network.nodes[179], building_network.nodes[180], building_network.nodes[181]]
cc52 = [building_network.nodes[206], building_network.nodes[221], building_network.nodes[229], building_network.nodes[234]]
bb4 = building_network.buildingrectangles(cc4)
bb5 = building_network.buildingrectangles(cc5)
bb6 = building_network.buildingrectangles(cc6)
bb8 = building_network.buildingrectangles(cc8)
bb10 = building_network.buildingrectangles(cc10)
bb11 = building_network.buildingrectangles(cc11)
bb12 = building_network.buildingrectangles(cc12)
bb13 = building_network.buildingrectangles(cc13)
bb15 = building_network.buildingrectangles(cc15)
bb18 = building_network.buildingrectangles(cc18)
##bb20 = building_network.buildingrectangles(cc20)
bb21 = building_network.buildingrectangles(cc21)
bb23 = building_network.buildingrectangles(cc23)
bb25 = building_network.buildingrectangles(cc25)
bb26 = building_network.buildingrectangles(cc26)
bb27 = building_network.buildingrectangles(cc27)
bb28 = building_network.buildingrectangles(cc28)
bb29 = building_network.buildingrectangles(cc29)
bb31 = building_network.buildingrectangles(cc31)
bb32 = building_network.buildingrectangles(cc32)
bb33 = building_network.buildingrectangles(cc33)
bb34 = building_network.buildingrectangles(cc34)
bb35 = building_network.buildingrectangles(cc35)
bb36 = building_network.buildingrectangles(cc36)
bb37 = building_network.buildingrectangles(cc37)
bb38 = building_network.buildingrectangles(cc38)
bb40 = building_network.buildingrectangles(cc40)
bb42 = building_network.buildingrectangles(cc42)
bb43 = building_network.buildingrectangles(cc43)
bb44 = building_network.buildingrectangles(cc44)
bb45 = building_network.buildingrectangles(cc45)
bb46 = building_network.buildingrectangles(cc46)
bb47 = building_network.buildingrectangles(cc47)
bb48 = building_network.buildingrectangles(cc48)
bb49 = building_network.buildingrectangles(cc49)
bb52 = building_network.buildingrectangles(cc52)
bb2 = Polygon([(-3.4549, 1.32292), (-3.3203, 0.582518), (-2.7241, 0.582518), (-2.7241, 1.32292)])
bb24 = Polygon([(-0.0702996, 2.84211), (-0.0702996, 1.53441), (0.0643003, 1.53441), (0.0643003, 2.57291)])
bb39 = Polygon([(1.8528, -1.69639), (1.4105, -1.69639), (1.4105, 0.092118), (1.6028, 0.092118)])
bb41 = Polygon([(1.3051, -2.63779), (1.0451, -2.63779), (1.0451, -1.85779), (1.2751, -1.85779)])
bb51 = Polygon([(1.9489, 1.11132), (1.9489, 0.601718), (0.698904, 0.601718), (0.698904, 2.45751)])
bb1 = Polygon([(-2.8395, 3.61131), (-2.8395, 2.07291), (-3.628, 2.07291), (-3.628, 1.66901), (-2.503, 1.66901), (-2.503, 3.61131)])
bb3 = Polygon([(-1.8203, 1.93831), (-1.9549, 1.82291), (-1.9549, 1.61131), (-1.8203, 1.51521), (-1.1664, 1.51521), (-1.0318, 1.61131), (-1.0318, 1.82291), (-1.1664, 1.93831)])
bb7 = Polygon([(-3.234, -1.16749), (-3.234, -1.36749), (-3.0615, -1.36749), (-3.0615, -1.38749), (-2.9665, -1.38749), (-2.9665, -1.36749), (-2.854, -1.36749), (-2.854, -1.16749)])
bb9 = Polygon([(-3.2337, 0.130618), (-3.2337, -0.138682), (-3.1472, -1.02329), (-2.8876, -1.02329), (-2.8876, 0.130618)])
bb14 = Polygon([(-2.7241, -1.16749), (-2.7241, -1.53969), (-2.9491, -1.53969), (-2.9491, -1.69639), (-2.0895, -1.69639), (-2.0895, -1.16749)])
bb16 = Polygon([(-3.939, -1.19629), (-3.939, -1.73629), (-3.149, -1.73629), (-3.149, -1.58229), (-3.349, -1.58229), (-3.349, -1.19629)])
bb19 = Polygon([(2.1413, 0.957518), (2.1413, 0.784418), (2.2759, 0.769818), (2.4105, 0.515218), (3.7951, 0.515218), (3.7951, 0.957518)])
bb22 = Polygon([(-0.8395, -0.465582), (-0.8395, -0.994387), (0.468204, -0.994387), (0.468204, -0.465582), (0.1605, -0.465582), (0.1605, -0.811687), (-0.6472, -0.811687), (-0.6472, -0.465582)])
bb30 = Polygon([(2.5066, -0.167482), (2.5066, -0.273282), (2.1605, -0.273282), (2.1605, -0.427082), (2.5066, -0.427082), (2.5066, -0.504082), (2.6509, -0.50408), (2.6509, -0.167482)])
buildinglist = [bb1, bb2, bb3, bb4, bb5, bb6, bb7, bb8, bb9, bb10, bb11, bb12, bb13, bb14, bb15, bb16, bb18, bb19, bb21, bb22, bb23, bb24, bb25, bb26, bb27, bb28, bb29, bb30, bb31, bb32, bb33, bb34, bb35, bb36, bb37, bb38, bb39, bb40, bb41, bb42, bb43, bb44, bb45, bb46, bb47, bb48, bb49, bb51, bb52]
for roads in road_network.roads:
    for i in range(len(buildinglist)):
        if building_network. in_area_of_polygon(buildinglist[i], road_network.pick_ten_equal_points(roads)) != 0:
            ##break node.operation()
        
for node in building_network.nodes:
    for node3 in node.link_nodes:
        xpoints3 = [node.pos.x, node3.pos.x]
        ypoints3 = [node.pos.y, node3.pos.y]
        plt.plot(xpoints3, ypoints3, color='black')
plt.show()
          

