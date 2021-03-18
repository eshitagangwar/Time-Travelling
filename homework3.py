#!/usr/bin/env python
# coding: utf-8

# In[88]:


import numpy as np
class Node:
    instances = []
    def __init__(self, year, x, y):
        self.x = x
        self.y = y
        self.year = year
        self.visited = False
        self.parent = None
        self.frontier = False
        self.explored = False
        Node.instances.append(self) 
        self.cost = 0
        self.move_cost = 0

from queue import PriorityQueue


def check_goal(node , goal):
    if (node.year == goal.year) and (node.x == goal.x) and (node.y == goal.y):
        return True
    else:
        return False
    
    
def get_cost(node,child):
    if (node.x != child.x and node.y != child.y and child.year == node.year):
        return 14
    elif (node.year != child.year):
        return(abs(node.year-child.year))
    else:
        return 10
    
    
def check_node(g,year,x,y):
    a ='_'.join([str(year),str(x),str(y)])
    if a in g.keys():
        return 1
    else:
        return 0
    
def child_node(parent_node,W,H,g,jaunt,alg):
    #print('here')
    x = parent_node.x 
    y= parent_node.y 
    year =parent_node.year
    l = []
    b = ''
    # checking for jaunting lines
    s_key = '_'.join([str(year),str(x),str(y)])
    
    if s_key in jaunt.keys():
        j_l = jaunt[s_key]
        
        for i in j_l:
            if x < W-1 and y < H-1 and x>=0 and y>=0:
                if  check_node(g,i,x,y) == 0:
                    A = Node(int(i), x,y)
                    l.append(A)
                    g[str(i)+'_'+str(x)+'_'+str(y)] = A
                else:
                    if alg == 1:
                        l.append(g[str(i)+'_'+str(x)+'_'+str(y)])
                    
                
            
    
      
     # creation of south west
    if x-1 < W and y-1 < H  and x-1>=0 and y-1 >=0:
        if  check_node(g,year,x-1,y-1) == 0:
            A = Node(year, x-1,y-1)
            l.append(A)
            g[str(year)+'_'+str(x-1)+'_'+str(y-1)] = A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x-1)+'_'+str(y-1)]
                l.append(b)
                
    # creation of west node           
    if x-1 < W and y < H  and x-1>=0 and y >=0:
        if  check_node(g,year,x-1,y) == 0:
            A = Node(year, x-1,y)  
            l.append(A)
            g[str(year)+'_'+str(x-1)+'_'+str(y)] = A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x-1)+'_'+str(y)]
                l.append(b)
                
    # creation of nort west node           
    if x-1 < W and y+1 < H  and x -1>=0 and y+1 >=0:
        if check_node(g,year,x-1,y+1) == 0:
            A = Node(year, x-1,y+1)
            l.append(A)
            g[str(year)+'_'+str(x-1)+'_'+str(y+1)] = A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x-1)+'_'+str(y+1)]
                l.append(b)
    # creation of south node       
    if x < W and y-1 < H  and x>=0 and y-1 >=0:
        if check_node(g,year,x,y-1) == 0:
            A = Node(year, x,y-1)
            l.append(A)
            g[str(year)+'_'+str(x)+'_'+str(y-1)] = A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x)+'_'+str(y-1)]
                l.append(b)
                
    #creation of north node          
    if x < W and y+1 < H  and x>=0 and y+1 >=0:
        if check_node(g,year,x,y+1) == 0:
            A = Node(year, x,y+1)
            l.append(A)
            g[str(year)+'_'+str(x)+'_'+str(y+1)] =A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x)+'_'+str(y+1)]
                l.append(b)
    #creation of southeast node            
    if x+1 < W and y-1 < H and x+1>=0 and y-1 >=0:
        if  check_node(g,year,x+1,y-1) == 0 :
            A = Node(year, x+1,y-1)
            l.append(A)
            g[str(year)+'_'+str(x+1)+'_'+str(y-1)] =A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x+1)+'_'+str(y-1)]
                l.append(b)
    #creation of east node            
    if x+1 < W and y < H  and x+1>=0 and y >=0:
        if  check_node(g,year,x+1,y) == 0:
            A = Node(year, x+1,y)  
            l.append(A)
            g[str(year)+'_'+str(x+1)+'_'+str(y)] = A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x+1)+'_'+str(y)]
                l.append(b)
    #creation of north east            
    if x+1 < W and y+1 < H  and x+1>=0 and y+1 >=0:
        if check_node(g,year,x+1,y+1) == 0:
            A = Node(year, x+1,y+1)
            l.append(A)
            g[str(year)+'_'+str(x+1)+'_'+str(y+1)] = A
        else:
            if alg == 1:
                b = g[str(year)+'_'+str(x+1)+'_'+str(y+1)]
                l.append(b)
                
    return((l,g))
    

def n_d( year,x, y):
    visited = str(year)+'_'+str(x)+'_'+str(y)
    return (visited)


def check_goal(node , goal):
    if (node.year == goal.year) and (node.x == goal.x) and (node.y == goal.y):
        return True
    else:
        return False


def rebuild(q,node):
    count = 0
    for i in q.queue:
        if i[2] == node:
            break
            
        count = count +1
    

    q.queue.pop(count)
    return(q)

def get_cost(node,child):
    if (node.x != child.x and node.y != child.y and child.year == node.year):
        return 14
    elif (node.year != child.year):
        return(abs(node.year-child.year))
    else:
        return 10
def get_heu(node,i):

    u = np.sqrt((node.x - i.x)**2 + (node.y - i.y)**2)
    return u   



def bfs(start,goal,jaunt,h,w):
    if check_goal(goal,start):
        l = []
        l.append((goal.year,goal.x,goal.y,1))
        
        return (0,l)
    queue = [start]
    start.cost = 0
    start.visited = True

    
    level = 0
    flag = 0
    generated = dict()
    while queue:
        
        node = queue[0]
        queue = queue[1:]
        child , generated = child_node(node, h,w , generated,jaunt,alg=0)
        cost = node.cost
        for i in child:
            if  i.visited == False :
                queue.append(i)
                i.move_cost = 1
                i.cost = cost+1
                i.parent = node
                i.visited = True
                
            if i.x == goal.x and i.y == goal.y and i.year == goal.year :
                flag = 1
                goal.parent = node
                goal.cost = cost+1
                break
                
        if flag == 1:
            break
        
        level = level +1        

    if flag == 0:
        return('FAILS')
    #print('out')
    back = goal
    path = []
    
    
    i = 0
    while back != start:
        i = i+1
        path.append((back.year,back.x,back.y,back.cost))
        back = back.parent
    path.append((start.year,start.x,start.y,0))    
    path.reverse()
    cost = len(path)
    return((cost,path))



    

import time   
import numpy as np
def ucs(start,goal,jaunt,h,w):
    if check_goal(goal,start):
        return ((0,1,(goal.year,goal.x,goal.y)))
    front = PriorityQueue()
    
    
    generated = dict()
    flag = 0
    generated[str(start.year)+'_'+str(start.x)+'_'+str(start.y)] = start
    generated[str(goal.year)+'_'+str(goal.x)+'_'+str(goal.y)] = goal
    start.frontier = True
    front.put((start.cost,1,start))
    
    count = 1
    while front.empty() != True:
        
        node =front.get()[2]
        node.frontier = False
        if node.explored == False:
            node.explored = True
            
        if (node.year == goal.year) and (node.x == goal.x) and (node.y == goal.y):
            flag = 1
            break
            

        
        
        child , generated = child_node(node, h,w , generated,jaunt,alg=1)
        o = 0
       
        for i in child:
            
            if (i.frontier == True) and (i.explored == False):
                prev_cost = i.cost
                new_cost =  node.cost + get_cost(node,i) 
                ch_cost = min(prev_cost,new_cost)
                if ch_cost < prev_cost:
                    i.parent = node
                    i.cost = ch_cost
                    i.move_cost = get_cost(node,i)
                    front = rebuild(front,i)
                    o = i.cost
                    count = count+1
                    front.put((o,count,i))
                    
                    
                
                
                
            
            elif i.explored == True and (i.frontier == False):
                i.explore = True
                
               
            
            else:
                
                i.cost = node.cost + get_cost(node,i)
                
                i.parent = node
                i.move_cost = get_cost(node,i)
                o = i.cost
                count = count+1
                front.put((o,count,i))
                
                i.frontier = True
                
                
    
    if flag == 0:
        return('FAILS')
    
    back = goal
    path = []
    
    
    goal_cost =  0
    while back != start:
        goal_cost = goal_cost + back.move_cost
        path.append((back.year,back.x,back.y,back.move_cost))
        back = back.parent
    path.append((start.year,start.x,start.y,start.move_cost))    
    path.reverse()
    
    return((goal_cost,path))            
        
        
def get_heu(node,i):

    u = np.sqrt((node.x - i.x)**2 + (node.y - i.y)**2)
    return u

def a(start,goal,jaunt,h,w):
    if check_goal(goal,start):
        return ((0,1,(goal.year,goal.x,goal.y)))
    front = PriorityQueue()
    
    
    generated = dict()
    flag = 0
    generated[str(start.year)+'_'+str(start.x)+'_'+str(start.y)] = start
    generated[str(goal.year)+'_'+str(goal.x)+'_'+str(goal.y)] = goal
    start.frontier = True
    front.put((start.cost,1,start))
    
    count = 1
    while front.empty() != True:
        
        node =front.get()[2]
        node.frontier = False
        if node.explored == False:
            node.explored = True
            
        if (node.year == goal.year) and (node.x == goal.x) and (node.y == goal.y):
            flag = 1
            break
            

        
        
        child , generated = child_node(node, h,w , generated,jaunt,alg=1)
        o = 0
       
        for i in child:
            
            if (i.frontier == True) and (i.explored == False):
                prev_cost = i.cost
                new_cost =  node.cost + get_cost(node,i) + get_heu(goal,i)
                ch_cost = min(prev_cost,new_cost)
                if ch_cost < prev_cost:
                    i.parent = node
                    i.cost = ch_cost
                    i.move_cost = get_cost(node,i)
                    front = rebuild(front,i)
                    o = i.cost
                    count = count+1
                    front.put((o,count,i))
                    #time.sleep(0.0000001)
                    
                
                
                
            
            elif i.explored == True and (i.frontier == False):
                i.explore = True
                
               
            
            else:
                
                i.cost = node.cost + get_cost(node,i) + get_heu(goal,i)
                
                i.parent = node
                i.move_cost = get_cost(node,i)
                o = i.cost
                count = count+1
                front.put((o,count,i))
                #time.sleep(0.0000001)
                i.frontier = True
                
                
    
    if flag == 0:
        return('FAILS')
    
    back = goal
    path = []
    
    
    goal_cost =  0
    while back != start:
        goal_cost = goal_cost + back.move_cost
        path.append((back.year,back.x,back.y,back.move_cost))
        back = back.parent
    path.append((start.year,start.x,start.y,start.move_cost))    
    path.reverse()
    
    return((goal_cost,path))            
file1 = open("output.txt","w")       
f = open('input.txt', 'r')
algo = f.readline().rstrip().upper()
h, w = list(map(int,f.readline().rstrip().split()))
s_year, s_x , s_y  = list(map(int,f.readline().rstrip().split()))
g_year, g_x , g_y = list(map(int,f.readline().rstrip().split()))
noOfJauntingLine = int(f.readline().rstrip())
jaunt = dict()
for i in range(0,noOfJauntingLine):
    s  = f.readline().rstrip().split()
    s_key = '_'.join(s[:len(s)-1])
    #print(s_key)
    s_value = s[len(s)-1]
    #print(s_value)
    if s_key in jaunt.keys():
        jaunt[s_key].append(s_value)
    else:
        jaunt[s_key] = [s_value]
    
        
    
start = Node(s_year, s_x , s_y)
goal= Node(g_year, g_x , g_y)
import time
start_time =  time.time()
flag =0
if algo == 'A*':
    ns =a(start, goal , jaunt,h,w)
    if ns != 'FAILS':
        flag = 1
        
elif algo == 'BFS':
    
    ns = bfs(start, goal , jaunt,h,w)
    if ns != 'FAILS':
        flag = 1
    
    
elif algo == 'UCS':
    
    ns =a(start, goal , jaunt,h,w)
    if ns != 'FAILS':
        flag = 1
    
        
    
    
else:
    file1.write('FAIL')

if flag == 1:
    file1.write(str(ns[0])+'\n')
    file1.write(str(len(ns[1])))
    for i in ns[1]:        
        file1.write('\n'+str(i[0]) + ' '+str(i[1]) + ' '+str(i[2])+ ' ' +str(i[3]) )
else:
    file1.write('FAIL')
  




