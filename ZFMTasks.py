import osmnx as ox
import pandas as pd
import networkx as nx
import numpy as np
import random
import math
from tqdm import tqdm
from menucl import main_menu1,menu2
import time
import shutil
import os,sys
import folium
import csv
from folium.features import DivIcon
import folium.plugins as plugins

import codecs
import geopy
from geopy.distance import VincentyDistance
# from folium.plugins import MarkerCluster

import time
from sklearn.model_selection import train_test_split
from sklearn import metrics
from sklearn import tree

from sklearn.ensemble import GradientBoostingClassifier


name_city='luxembourg city'
ox.config(use_cache=True)
#name_city='morlupo'
ray=50
max_dist=1500
decision='y'
type_net='walk'
num_usr=2000
days=2
min_speed=1.0
max_speed=1.5
grid_distance=500
max_osmid=0
newosmid=0
max_walk_time=40
maxlat=0
maxlong=0
minlat=999999999
minlong=999999999
maxlen=0
cud=0
list_group=[]
m_r=80

'''
    Parameters: big osmnx graph, number of tasks, days of the simulation
    Returns:    list of lists [id_task, latitude, longitude, day, hour, minute, duration, resource required, distance factor]
    Notes:      1 - Task duration is fixed in minutes
                2 - Resource required is the CPU support for performing the task (in percentage)
                3 - Distance factor is the coverage radius of the task in metres (df parameter)
    Created by: Yueqian Zhang
                Jan 10 2019
'''
'''
    attack_locations_generator:
            input: G_big + number of attack locations
            output: randomly generated center positions of attack locations
'''
def attack_locations_generator(big_graph, num_att):
    al = []
    l = list(big_graph.nodes())
    for i in range(num_att):
        # ind = random.randint(0,lengthl-1)
        # y=l[ind]['y']
        # x=l[ind]['x']
        index = random.choice(range(len(l)))
        y=big_graph.node[l[index]]['y']
        x=big_graph.node[l[index]]['x']
        grid_num=convert_location(big_graph,y,x)
        al.append([float(y),float(x),grid_num])
    return al

'''
    task_generator:
        To generate tasks of specific number
        90% of the tasks are ligitimate
        10% of them are iligitimate:
            duration: 70%-> 40, 50, 60 mins 30%-> 10, 20, 30 mins
            hour: 80%->peak hours (7am~13pm)
            resources: 50%->7%~10% CPU

            In the radius (200m) of center of attack locations
'''

def task_generator(big_graph, num_tasks, days, df, ligi, on_peak, attackLocations, num_att):
    attack_radius = 50
    tl = []
    l = []
    l = list(big_graph.nodes())
    # lengthl = len(l)
    for i in range(num_tasks):
        day = random.randint(0,days-1)
        # if(day == 0):
        #     h = random.randint(12,23)
        # else:
        #     h = random.randint(0,23)
        if(random.randint(1,10)<2):
            h = random.randint(0,5)
        else:
            h = random.randint(6,23)
        m = random.randint(0,59)

        # df = random.randint(30,100)
        rnum1 = random.randint(1,100)
        # 90% tasks are ligitimate
        # if a task is iligitimate:
        #   70% the duration between 40 and 60
        #   80% it's on peak

        if(rnum1 < 11):
            ligi = False
            # rnum2 = random.randint(1,10)
            if(random.randint(1,10)<4):
                dur = random.randint(1,3)*10 #30% the duraion between 10 and 30
            else:
                dur = random.randint(4,6)*10
            # rnum3 = random.randint(1,10)
            if(random.randint(1,10)>2):
                # if(day!=0):
                h = random.randint(7,11)# 80% on peak hours
            else:
                h = random.randint(12,17)
            # else:
            #     h = random.randint(*random.choice([(0,6),(14,23)]))
            # rnum4 = random.randint(1,10)
            if(random.randint(1,10)>2):
                r = random.randint(7,10)
            else:
                r = random.randint(1,6)
            # Select random attack locations
            rnum5 = random.randint(0,num_att-1)
            y_a = attackLocations[rnum5][0]
            x_a = attackLocations[rnum5][1]
            # Select random distance from 0m to x00m from the attack location center
            # Change!
            distance = random.randint(0,attack_radius)*0.001
            # origin_a = geopy.Point(y_a, x_a)
            # Select random bearing
            origin_a = (y_a, x_a)
            bearing = random.randint(0,360)
            destination_a = VincentyDistance(kilometers=distance).destination(origin_a, bearing)
            y, x = destination_a.latitude, destination_a.longitude  #got the iligitimate location

        else:
            ligi = True
            dur = random.randint(1,6)*10
            r = random.randint(1,10)
            index = random.choice(range(len(l)))
            y=big_graph.node[l[index]]['y']
            x=big_graph.node[l[index]]['x']
            # y=big_graph.node[random.choice(l)]['y']
            # x=big_graph.node[random.choice(l)]['x']
            # ind = random.randint(0,lengthl-1)
            # y=l[ind]['y']
            # x=l[ind]['x']

        #            num_move = dur/10;
        #            for i in range(num_move):
        # need remaining time

        remaining_t = dur
        grid_num=convert_location(big_graph,y,x)
        if(h in range(7,11)):
            on_peak = True
        else:
            on_peak = False
        tl.append([i+1, float(y), float(x), day, h, m, dur, remaining_t, r, df,ligi,on_peak,grid_num])

        #tl.append([i+1, float(big_graph.node[random.choice(l)]['y']), float(big_graph.node[random.choice(l)]['x']), day, h, m, dur, r, df,ligi,on_peak])#big_graph: G_big

    return tl
'''
    function to generate the movements of a task
'''
def task_movement(tl):
    taskMovementl=[]
    for i in range(0,len(tl)):
        y = tl[i][1]
        x = tl[i][2]
        day = tl[i][3]
        h = tl[i][4]
        m = tl[i][5]
        dur = tl[i][6]
        remaining_t = tl[i][7]
        r = tl[i][8]
        df = tl[i][9]
        ligi = tl[i][10]
        on_peak = tl[i][11]
        grid_num = tl[i][12]

        num_move = int(dur/10)
        each_r = float(r)/float(num_move)
        #print(num_move)
        for j in range(0,num_move):
            remaining_t = tl[i][7]
            day = tl[i][3]
            h = tl[i][4]
            m = tl[i][5]
            distance = random.randint(10,m_r)*0.001
            # origin = geopy.Point(y, x)
            origin = (y, x)
            # Select random bearing
            bearing = random.randint(0,360)
            destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
            y, x = destination.latitude, destination.longitude  #got the destinations
            m = m+j*10
            if(m>59):
                h = h+1
                m = m%60
                if(h>23):
                    day = day+1
                    h = h%24
            if(h in range(7,11)):
                on_peak = True
            else:
                on_peak = False
            grid_num=convert_location(G_big,y,x)
            remaining_t = remaining_t - j*10
            taskMovementl.append([i+1, float(y), float(x), day, h, m, dur,remaining_t, each_r, df,ligi,on_peak,grid_num])
    return taskMovementl



'''
    convert_location function is to convert the latitude and longitude of the tasks into grid
'''
# This is the function to convert the longtitude and latitude into the grid
def convert_location(big_graph,y,x):
    global maxlat
    global maxlong
    global minlat
    global minlong
    for n,d in big_graph.nodes(data=True):
        x_g=d['x']
        y_g=d['y']
        if(y_g<minlat):
            minlat=y_g
        if(y_g>maxlat):
            maxlat=y_g
        if(x_g<minlong):
            minlong=x_g
        if(x_g>maxlong):
            maxlong=x_g
    g_length=ox.great_circle_vec(maxlat,minlong,maxlat,maxlong)#length
    g_width=ox.great_circle_vec(minlat,maxlong,maxlat,maxlong)#width
    #grid_long=math.ceil(g_length/100)#number of grid on x
    #grid_lat=math.ceil(g_width/100)#number of grid on y
    grid_long=int(g_length/600)#number of grid on x
    grid_lat=int(g_width/600)#number of grid on y
    #grid_long=int(g_length/grid_distance)#number of grid on x
    #grid_lat=int(g_width/grid_distance)#number of grid on y

    if(grid_lat!=0):
        diff_ver=(maxlat-minlat)/grid_lat#more than one grid so calculate the length of each grid on y

    else:
        diff_ver=maxlat
        grid_lat=1

    if(grid_long !=0):
        diff_or=(maxlong-minlong)/grid_long
    else:
        diff_or=maxlong
        grid_long=1
    row=math.ceil((y-minlat)/diff_ver)
    col=math.ceil((x-minlong)/diff_or)
    grid_num = grid_long*(row-1)+col
    '''
    row=int((y-minlat)/diff_ver)
    col=int((x-minlong)/diff_or)
    if(col==0):
        col =1
    if(col == grid_long-1):
        col = grid_long
    grid_num = grid_long*row+col
    '''

    return grid_num


    #for i in range(0,len(tasks)):
        #y = tasks[i][1]
        #x = tasks[i][2]

def checkcontact(point,listpoints):
    count=0
    ngh=0
    for itm in listpoints:
        if(itm[2]!=point[2]):
            if(itm[2]!=ngh):
                dist=ox.great_circle_vec(point[0],point[1],itm[0],itm[1])
                if(dist<ray):
                    count+=1
                    ngh=itm[2]



    return count
def add_points(G3,min_dist):

    global maxlen
    global maxlat
    global maxlong
    global minlat
    global minlong
    dictdist={}
    # epsg:4326 Commonly used by organizations that provide GIS data for the entire globe or many countries. CRS used by Google Earth
    G2 = nx.MultiDiGraph(name='G2', crs={'init':'epsg:4326'})
    for  n,d  in G.nodes(data=True):
        x=d['x']
        y=d['y']
        G3.node[n]['group']=-1

        if(y<minlat):
            minlat=y
        if(y>maxlat):
            maxlat=y
        if(x<minlong):
            minlong=x
        if(x>maxlong):
            maxlong=x

# to calculate the great-circle distance between two points or between vectors of points, using haversine
    dist_or=ox.great_circle_vec(maxlat,minlong,maxlat,maxlong)#length
    dist_ver=ox.great_circle_vec(minlat,maxlong,maxlat,maxlong)#width

    grid_long=int(dist_or/grid_distance)#number of grid on x
    grid_lat=int(dist_ver/grid_distance)#number of grid on y

    #listlong=[]
    #long1= minlong *math.pi /180
    #long2=long1
    #lat1= minlat *math.pi /180
    #lat2= maxlat *math.pi /180
    #bearing1 = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
    #bearing1 = math.degrees(bearing)
    #bearing1 = (bearing + 360) % 360

    #bearing2 = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
    #bearing2 = math.degrees(bearing)
    #bearing2 = (bearing + 360) % 360
    #for j in range (0,grid_long):



    #list_group.append(list())

    if(grid_lat!=0):
        diff_ver=(maxlat-minlat)/grid_lat#more than one grid so calculate the length of each grid on y

    else:
        diff_ver=maxlat
        grid_lat=1

    if(grid_long !=0):
        diff_or=(maxlong-minlong)/grid_long
    else:
        diff_or=maxlong
        grid_long=1

    ng=grid_lat*grid_long#grid number
    for i in range(0,(grid_long*grid_lat)):
        list_group.append(list())

    global max_osmid,newosmid
    err=min_dist/2
    max_osmid=max(G3.nodes())+1
    newosmid=max_osmid
    ed_osmid=0
    r=0
    totlen=0
    nedges=0




    list_edges=list(G3.edges(data=True))
    # means process

    pbar = tqdm(total=len(list_edges))
    d=float(float(min_dist)/1000)

    for u1,v1,dat in list_edges:

        if dat['length']> maxlen:
            maxlen=dat['length']

        pbar.update(1);

        flag=0
        r=r+1
        i=0

        R=6371009

        x1=G3.node[u1]['x']
        x2=G3.node[v1]['x']
        y1=G3.node[u1]['y']
        y2=G3.node[v1]['y']


        #To find the row and col of the node


        if(y1==maxlat):
            row=grid_lat-1
        else:
            row=int((y1-minlat)/diff_ver)

        if(x1==maxlong):
            col=grid_long-1
        else:
            col=int((x1-minlong)/diff_or)

        groupu=(grid_long*row)+col



        ##list_group[groupu].append(u1)

        nodeu = {}
        nodeu['y'] = y1
        nodeu['x'] = x1
        nodeu['osmid'] =  G3.node[u1]['osmid']
        nodeu['u']=u1
        nodeu['v']=u1
        nodeu['d']=-1
        nodeu['dec']=0
        nodeu['du']=0
        nodeu['dv']=0
        nodeu['group']=groupu

        ##G2.add_node(G3.node[u1]['osmid'],nodeu)


        lat1 = y1 * math.pi / 180
        lat2 = y2 * math.pi / 180

        long1 = x1 * math.pi / 180
        long2 = x2 * math.pi / 180



        dista=ox.great_circle_vec(y1,x1,y2,x2)

        totlen= totlen + dista
        nedges=nedges+1
        if dista< min_dist or (y1==y2 and x1==x2):
            continue

        rel=int(dista/min_dist)
        ty=y1
        tx=x1


        bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360

        for k in range (0,rel):
            distance= d*(k+1)
            origin = geopy.Point(y1, x1)
            destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
            y3, x3 = destination.latitude, destination.longitude



            dist=ox.great_circle_vec(ty,tx,y3,x3)

            du=ox.great_circle_vec(y1,x1,y3,x3)
            dv=ox.great_circle_vec(y2,x2,y3,x3)
            dec=math.sqrt( (x3 - tx)**2 + (y3 - ty)**2 )
            node = {}
            node['y'] = y3
            node['x'] = x3
            node['osmid'] = newosmid
            node['u']=u1
            node['v']=v1
            node['d']=dist
            node['dec']=dec
            node['du']=du
            node['dv']=dv
            node2 = {}
            if(y3==maxlat):
                row=grid_lat-1
            else:
                row=int((y3-minlat)/diff_ver)
            if(x3==maxlong):
                col=grid_long-1
            else:
                col=int((x3-minlong)/diff_or)
            group=(grid_long*row)+col

            node['group']=group
            list_group[group].append(newosmid)

            G2.add_node(newosmid,y=y3,x=x3,osmid=newosmid,u=u1,v=v1,d=dist,dec=dec,du=du,dv=dv,group=group)
            deglen=110.25
            x = ty - y3
            y = (tx - x3)*math.cos(math.radians(y3))
            #x = ty - y3
            #y =  (tx - x3)*math.cos(math.radians(y3))


            tx=x3
            ty=y3

            #if i==1:
                #G2.add_edge(u=u1,v=newosmid,key=0,highway='unclassified',length=dist,oneway=False,osmid=ed_osmid)
            #else:
                #G2.add_edge(u=(newosmid-1),v=newosmid,key=0,highway='unclassified',length=dist,oneway=False,osmid=ed_osmid)
            ed_osmid=ed_osmid+1
            i=i+1
            newosmid=newosmid+1

        #if i>1:
            #dist=ox.great_circle_vec(ty,tx,y2,x2)
            #G2.add_edge(u=(newosmid-1),v=v1,key=0,highway='unclassified',length=dist,oneway=False,osmid=ed_osmid)
            #G2.remove_edge(u1,v1)
    pbar.close()
    print ("Average Lenght of Edges : ",totlen/nedges)
    return G2


with open('Setup.txt', 'r') as data:
    count=0
    for line in data:
        if count==1:
            p=line.split()
            days=int(p[4])


        if count==4:
            r=line.split()
            num_usr=int(r[4])

        if count==22:
            r=line.split()
            default=int(r[5])
            print ('\nDefault choice = ',default)

        if count==18:
            r=line.split()
            antenna_decision=int(r[5])
        if count==9:
            r=line.split()
            numhours=int(r[4])
        if count==12:
            r=line.split()
            endhour=int(r[4])

        count+=1

if default==0:
    shutil.copy2('./Inputs/saved/DefaultList/route_usr_1day_0.html','/var/www/html/CrowdSenSim/route_usr_1day_0.html')
    sys.exit(0)

saved=[]
with open('./Inputs/SavedList.txt', 'r') as data:
    for line in data:

        listsaved={}
        p=line.split()
        listsaved['id']=int(p[0])
        listsaved['name']=p[1]
        listsaved['us']=int(p[2])
        listsaved['days']=int(p[3])
        saved.append(listsaved)

try:
    saved,name_city,num_usr,days=main_menu1(saved,num_usr,days)
except SystemExit:
    print ('Exit from CrowdSenSim')
    sfile = open('./Inputs/SavedList.txt', 'w')
    for item1 in saved:
        sfile.write(" %s %s %s %s\n" % (item1['id'],item1['name'],item1['us'],item1['days']))

    sfile.close()
    sys.exit(1)


aFile='Setup.txt'
shutil.move( aFile, aFile+"~" )

destination= open( aFile, "w" )
source= open( aFile+"~", "r" )
count=0
for line in source:

    if count ==4:
        destination.write("|Number of users| = "+str(num_usr) + "\n" )
    elif count==1:
        destination.write("|Days of simulation| = "+str(days)+ "\n" )
    else:
        destination.write( line )
    count+=1

source.close()
destination.close()


if  name_city!='no' :







    while(True):

        print ('Antenna choice = ',antenna_decision )
        print ('Number of Days = ',days)
        print ('Number of Users = ',num_usr)


        try:
            print ('Downloading map of ***',name_city,'*** ................')
            G = ox.graph_from_place(name_city,network_type=type_net,simplify=False)
            break
        except:
            print ('Wrong city name, please retry')
            time.sleep(2)
            name_city=menu2()


    while(True):
        choice=input('\nDo you want to save it? (y/n)     ')
        ch = choice.lower()
        if ch!='y' and ch !='n':
            print ('Wrong selection, please retry')
        else :
            break
    if ch=='y':
        if len(saved)>0:
            index=int(saved[-1]['id'])
        else:
            index=-1
        index+=1
        try:
            os.makedirs('./Inputs/saved/'+str(index)+'list')
        except:
            pass
        list_eve_to_save={}
        list_eve_to_save['id']=index
        list_eve_to_save['name']=name_city.replace(" ","_")
        list_eve_to_save['us']=num_usr
        list_eve_to_save['days']=days
        saved.append(list_eve_to_save)

        setfile = open('./Inputs/saved/'+str(index)+'list/Setup.txt', 'w')
        setfile.write("%s %s " % (num_usr,days))
        setfile.close()



    #ox.plot_graph(G)

    G_und = G.to_undirected()

    print ('Elaborating Map................')
    start = time.time()


    G_big=add_points(G_und,3)
    print ("Number nodes after algorithm: ",len(G_big.nodes()))

    end = time.time()
    print((end - start),'  <-----Algorithm  Time (seconds) '     )

    G_und=None
    antennal = []
    if(antenna_decision==1):
        c=0
        afile = open('./Inputs/CoordinatesAntennas.txt', 'w')
        afile.write("/ID-Antenna/-/Lat/-/Long/\n")
        for i in range(0,len(list_group)):

            if len(list_group[i])!=0:
                c+=1
                antenna=random.choice(list_group[i])
                afile.write("%s %s %s\n" % (c,float(G_big.node[antenna]['y']), float(G_big.node[antenna]['x'])))
                #afile.write("%s %s %s\n" % (c,float(G_big.node[antenna]['y']), float(G_big.node[antenna]['x'])))
    afile.close()
    if ch=='y':
        shutil.copy2('./Inputs/CoordinatesAntennas.txt', './Inputs/saved/'+str(index)+'list/CoordinatesAntennas.txt')


   # wfile = open('punti.txt', 'w')
    #for item1 in G_big.nodes():
       # wfile.write(" %s \n" % [G_big.node[item1]['x'],G_big.node[item1]['y'],G_big.node[item1]['group']])

    #wfile.close()



    #with open('punti.txt', 'r') as data:
        #list_node=[]
        #for line in data:
            #list_node.append(line)




    listhours=np.loadtxt("./Inputs/hours.txt")
    perc_hour=[]

    hr=0
    hr=hr+((days-1)*24)
    sumconts=0
    random0_or_tracce1=0

    if random0_or_tracce1==0:
        for i in range(0,hr):
            listhours[i]=15


    for i in range(0,hr):
        sumconts+=listhours[i]

    perctot=0.0
    perc_hour.append(0)


    for i in range(0,hr):
        perc=float(listhours[i])/float(sumconts)
        perctot+=perc
        perc_hour.append(perc*100)





    perc_hour[0]=((float(perctot)/float(hr))*100)


    '''
        THE CODE HAS BEEN CHANGED HERE!!!!!!!!!!!!!!!!!!!!!!
        We generate a new file with a series of tasks
        The definition task_generator is called here (the task duration is in minutes)
    '''

    #
    TPR = 0
    TNR = 0
    PPV = 0
    NPV = 0
    ACC = 0
    f1T = 0
    f1F = 0
    for run_num in range(1,11):
        #Attack locations
        num_att = 6
        attackLocations = attack_locations_generator(G_big, num_att)
        # attackLocations = ([49.785900519853207,-92.757295113443519],[49.786155099411985,-92.757181876589542],[49.786247014083202,-92.756431519220342],[49.785727626173042,-92.756565667165887],[49.785943973840489,-92.757243066719184],[49.782233552181076,-92.824056265838237])#dryden
        # attackLocations = ([43.190922434521539,-80.384271186606398],[43.103332740357232,-80.437914548966830],[43.100762341628233,-80.437528435286708],[43.204569848019709, -80.393321084072937],[43.191805331223968,-80.382187961233839],[43.068070576944656,-80.270124067809576])
        # attackLocations = ([45.525989744290115,-75.259123096736957],[45.524404475876544,-75.257465804536565],[45.531369893393418,-75.288606111620979],[45.52564652714934,-75.257939924522262],[45.437553850855259,-75.154134169719157],[45.465510726679724,-75.264172499250975])#55u
        # attackLocations = ([45.530669817166121,-75.229775385328367],[45.526219485369282,-75.259054527518344],[45.437061526345431,-75.154209730000346],[45.437209891081480,-75.152757335831680],[45.437237758876542,-75.155320244335030],[45.543885511924692,-75.267860652562874])#55r
        # attackLocations = ([45.440715879260736,-75.196143223305072],[45.526257039101928,-75.260386658202918],[45.437747130676222,-75.141705822337912],[45.440117000888385,-75.195740399873543],[45.532052069592545,-75.233576301519605],[45.438400455434191,-75.257574445539149])#1112u
        # attackLocations = ([45.492131931007009,-75.266888723307616],[45.532600419399792,-75.288399619035886],[45.434390572086848,-75.156209634205354],[45.525260483657917,-75.262412085096145],[ 45.441274167262414,-75.196633268978317],[45.437809707805286,-75.257499108961639])
        # attackLocations = ([48.475549150194681,-81.340015953015040],[48.472465539209772,-81.386810012366340],[48.476021458387066,-81.339398610954149],[48.479199545211138,-81.298707580595050],[48.477017645542674,-81.321903320777864],[48.475830415709176,-81.320738146699497])
        # attackLocations = ([48.476926106723212,-81.325482099366454],[48.473456309997850,-81.339482131781807],[48.479420297681521,-81.300985379625374],[48.476438287278185,-81.330099331736534],[48.475295779234486,-81.328448560724851],[48.475174144262503,-81.343032177230413])
        # attackLocations = ([48.472410663610397,-81.374659324671597],[48.480347168378344,-81.313312517706876],[48.476985508795572,-81.325526831767746],[48.476212048549073,-81.299634933529688],[48.479038720365715,-81.324465723706282],[48.466318116408857,-81.442909251234340])
        # attackLocations = ([48.477806252073940,-81.325534875897205],[48.479609256185931,-81.311710891791634],[48.479464484135271,-81.313095201767126],[48.476558650141641,-81.312776058455412],[48.479620321625454,-81.312182678351931],[48.477904218731872,-81.287616131877002])
        attackLo_file = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/AttackLocations.txt','w')
        attackLo_file.write("/ID-AttackLoations/    -/Latitude/ -/Longitude/\n")
        for i in range(0,len(attackLocations)):
            #attackLo_file.write("{}\t {}\t {}\t {}\n".format(i+1,attackLocations[i][0],attackLocations[i][1],attackLocations[i][2]))
            attackLo_file.write("{}\t {}\t {}\n".format(i+1,attackLocations[i][0],attackLocations[i][1]))
        attackLo_file.close()
        num_tsk = int(num_usr/10)
        # num_tsk = num_usr
        distance_factor = 100      # coverage radius of the task (in metres)
        ligi = True #ligitimacy of the task
        on_peak = True  #if the task is on peak hours
        tasks = task_generator(G_big, num_tsk, days, distance_factor, ligi, on_peak, attackLocations, num_att)
        #
        '''
        csvfile = open('./Inputs/Mobility/Tasks.csv','w')
        with csvfile:
            titles = ['ID','Latitude','Longitude','Day','Hour','Minute','Duration','Resources','Coverage','Ligimacy','OnPeakHours','GridNumber']
            writer = csv.DictWriter(csvfile, fieldnames=titles)
            writer.writeheader()
            for i in range(0,len(tasks)):
                if(tasks[i][3]==0):
                    continue
                writer.writerow({'ID':tasks[i][0],'Latitude':tasks[i][1],'Longitude':tasks[i][2],'Day':tasks[i][3],'Hour':tasks[i][4],'Minute':tasks[i][5],'Duration':tasks[i][6],'Resources':tasks[i][7],'Coverage':tasks[i][8],'Ligimacy':tasks[i][9],'OnPeakHours':tasks[i][10],'GridNumber':tasks[i][11]})

        '''
        # task txt file
        task_file = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/Tasks.txt', 'w')
        task_file.write("/ID-Task/  -/Lat/  -/Long/ -/Day/  -/Hour/ -/Minute/   -/Duration/ -/Remaining time/ -/Resources/    -/Coverage/ -/Ligitimacy/ -/on peak hour/ -/grid_number\n")
        for i in range(0,len(tasks)):
            task_file.write("{}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\n".format(tasks[i][0], tasks[i][1], tasks[i][2], tasks[i][3], tasks[i][4], tasks[i][5], tasks[i][6], tasks[i][7], tasks[i][8], tasks[i][9], tasks[i][10], tasks[i][11], tasks[i][12]))

        task_file.close()

        # task csv file
        csvfile = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/Tasks.csv','w')
        with csvfile:
            titles = ['ID','Latitude','Longitude','Day','Hour','Minute','Duration','RemainingTime','Resources','Coverage','Ligitimacy','OnPeakHours','GridNumber']
            writer = csv.DictWriter(csvfile, fieldnames=titles)
            writer.writeheader()
            for i in range(0,len(tasks)):
                # if(tasks[i][3]==0):
                #     continue
                writer.writerow({'ID':tasks[i][0],'Latitude':tasks[i][1],'Longitude':tasks[i][2],'Day':tasks[i][3],'Hour':tasks[i][4],'Minute':tasks[i][5],'Duration':tasks[i][6],'RemainingTime':tasks[i][7],'Resources':tasks[i][8],'Coverage':tasks[i][9],'Ligitimacy':tasks[i][10],'OnPeakHours':tasks[i][11],'GridNumber':tasks[i][12]})

        #
        taskMovementl = task_movement(tasks)

        csvfile = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/TasksMovement.csv','w')
        # csvfile = open('./Inputs/Mobility/test/timmins/TasksIncludeMovements_'+str(num_usr)+str(name_city)+str(m_r)+str(num_att)+'.csv','w')
        with csvfile:
            titles = ['ID','Latitude','Longitude','Day','Hour','Minute','Duration','RemainingTime','Resources','Coverage','Ligitimacy','OnPeakHours','GridNumber']
            writer = csv.DictWriter(csvfile, fieldnames=titles)
            writer.writeheader()
            for i in range(0,len(taskMovementl)):
                # if(taskMovementl[i][3]==0):
                #     continue
                writer.writerow({'ID':taskMovementl[i][0],'Latitude':taskMovementl[i][1],'Longitude':taskMovementl[i][2],'Day':taskMovementl[i][3],'Hour':taskMovementl[i][4],'Minute':taskMovementl[i][5],'Duration':taskMovementl[i][6],'RemainingTime':taskMovementl[i][7],'Resources':taskMovementl[i][8],'Coverage':taskMovementl[i][9],'Ligitimacy':taskMovementl[i][10],'OnPeakHours':taskMovementl[i][11],'GridNumber':taskMovementl[i][12]})

        # taskm_file = open('./Inputs/Mobility/test/timmins/TasksMovement_'+str(num_usr)+str(name_city)+str(m_r)+str(num_att)+'.txt','w')
        taskm_file = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/TasksMovement.txt','w')
        taskm_file.write("/ID-Task/  -/Lat/  -/Long/ -/Day/  -/Hour/ -/Minute/   -/Duration/ -/Remaining time/ -/Resources/    -/Coverage/ -/Ligitimacy/ -/on peak hour/ -/grid_number\n")
        for i in range(0,len(taskMovementl)):
            # if(taskMovementl[i][3]==0):
            #     continue
            taskm_file.write("{}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\t {}\n".format(i+1, taskMovementl[i][0],taskMovementl[i][1], taskMovementl[i][2], taskMovementl[i][3], taskMovementl[i][4], taskMovementl[i][5], taskMovementl[i][6], taskMovementl[i][7], taskMovementl[i][8], taskMovementl[i][9], taskMovementl[i][10], taskMovementl[i][11], taskMovementl[i][12]))

        taskm_file.close()

        csvfile = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/Illegitimate_'+str(name_city)+str(m_r)+'.csv','w')
        with csvfile:
            titles = ['ID','Latitude','Longitude','Day','Hour','Minute','Duration','RemainingTime','Resources','Coverage','Ligitimacy','OnPeakHours','GridNumber']
            writer = csv.DictWriter(csvfile, fieldnames=titles)
            writer.writeheader()
            for i in range(0,len(tasks)):
                # x = math.cos(tasks[i][1])*math.cos(tasks[i][2])
                # y = math.cos(tasks[i][1])*math.sin(tasks[i][2])
                if(tasks[i][10]==True):
                    continue
                writer.writerow({'ID':tasks[i][0],'Latitude':tasks[i][1],'Longitude':tasks[i][2],'Day':tasks[i][3],'Hour':tasks[i][4],'Minute':tasks[i][5],'Duration':tasks[i][6],'RemainingTime':tasks[i][7],'Resources':tasks[i][8],'Coverage':tasks[i][9],'Ligitimacy':tasks[i][10],'OnPeakHours':tasks[i][11],'GridNumber':tasks[i][12]})




    #     '''
    #     Apply machine learning algorithm (gradient boosting)
    #     Input:
    #         Tasks including their movement;
    #         Traning duration: 1, 2, 3, 4, 5 days;
    #         Test set: 2nd, 3nd, 4th, 5th days;
    #     Outputs:
    #         Task ID for these tasks considered as the illegitimate by machine learning
    #     '''
    #     data = pd.read_csv('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/Tasks.csv', index_col=False)
    #     testdata = pd.read_csv('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/Tasks.csv', index_col=False)
    #     # data = data.set_index("ID")
    #     # print(data.head(5))
    #
    #     # print(data['Ligitimacy'].dtypes)
    #
    #     # Convert true and false into numeric
    #     data = data.drop(data[data['GridNumber'] < 1].index)
    #     testdata = testdata.drop(testdata[testdata['GridNumber'] < 1].index)
    #     #data = data[data.GridNumber>0]
    #
    #
    #
    #     training_day = [1,2,3,4,5]
    #     # random_state_l=[8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096]
    #     #clf = tree.DecisionTreeClassifier()
    #     # for i in random_state_l:
    #         #X_train, X_test, y_train, y_test = train_test_split(data[used_features].values, data['Ligitimacy'], test_size=0.3, random_state=i)
    #         # D_train,D_test = train_test_split(data, test_size=0.3, random_state=i)
    #
    #
    #         #
    #     TN = 0
    #     FN = 0
    #     TP = 0
    #     FP = 0
    #     IllegitimateEstimate=[]
    #     for day in training_day:
    #         trainingSet = data[data['Day']<day]
    #         testSet = testdata[testdata['Day']==day]
    #         # print(testSet.head(5))
    #
    #         count_row = testSet.shape[0]# number of row in test set
    #
    #         D_train = trainingSet
    #             # # D_train = trainingSet.sample(n=trainingSet.shape[0],random_state=i)
    #             # print(D_train.shape[0]
    #         used_features =["Hour","Duration","Resources","GridNumber"]
    #         clf = GradientBoostingClassifier(n_estimators=100, learning_rate=0.1,max_depth=3, random_state=1).fit(D_train[used_features].values, D_train["Ligitimacy"])
    #         predAll = clf.predict(testSet[used_features])
    #         cmatrix=metrics.confusion_matrix(testSet["Ligitimacy"], predAll)
    #
    #         # creport = pandas_classification_report(testSet["Ligitimacy"], predAll)
    #         # print(creport)
    #         #
    #         # creport = pd.DataFrame(creport)
    #         # f_fscore = creport.iloc[0]['f1-score'] + f_fscore
    #         # t_fscore = creport.iloc[1]['f1-score'] + t_fscore
    #         # print(cmatrix)
    #
    #         TN = cmatrix[0][0] + TN
    #         FN = cmatrix[1][0] + FN
    #         TP = cmatrix[1][1] + TP
    #         FP = cmatrix[0][1] + FP
    #         for i in range(count_row):
    #             row = testSet.iloc[[i]]
    #             # print(row)
    #             pred = clf.predict(row[used_features])
    #             if(pred==False):
    #                 # print(row['ID'])
    #                 IllegitimateEstimate.append([row["ID"].values[0]])
    #                 # IllegitimateEstimate.append([row["ID"].values[0],row["Ligitimacy"].values[0]])
    #                 # print(row)
    #     iTask = open('./Inputs/Mobility/differentradius/50/'+str(run_num)+'/EstimatedIllegitimateTask.txt', 'w')
    #     # iTask.write("/ID-Task/\n")
    #     for i in range(0,len(IllegitimateEstimate)):
    #         iTask.write("{}\n".format(IllegitimateEstimate[i][0]))
    #
    #     iTask.close()
    #     # print(IllegitimateEstimate)
    #
    #     # Sensitivity, hit rate, recall, or true positive rate
    #     TPR_new = TP/(TP+FN)
    #
    #     TNR_new = TN/(TN+FP)
    #
    #     PPV_new = TP/(TP+FP)
    #
    #     NPV_new = TN/(TN+FN)
    #
    #     TPR = TPR + TPR_new
    #     # Specificity or true negative rate
    #     TNR = TNR + TNR_new
    #     # Precision or positive predictive value
    #     PPV = PPV + PPV_new
    #     # Negative predictive value
    #     NPV = NPV + NPV_new
    #
    #     ACC = ACC + (TP+TN)/(TP+FP+FN+TN)
    #
    #     f1T = f1T + 2*((PPV_new*TPR_new)/(PPV_new+TPR_new))
    #
    #     f1F = f1F + 2*((NPV_new*TNR_new)/(NPV_new+TNR_new))
    # TPR = TPR/10
    # TNR = TNR/10
    # PPV = PPV/10
    # NPV = NPV/10
    # ACC = ACC/10
    # f1T = f1T/10
    # f1F = f1F/10
    # print("Legitimate Recall: ",TPR,"\nIllegitimate Recall: ",TNR,"\nLegitimate Precision: ",PPV,"\nIllegitimate Precision: ",NPV,"\nOverall accuracy: ",ACC,"\nLeigitmate F1: ",f1T,"\nIllegitimate F1: ",f1F)
    #
    #



else:




    print ('Event List loaded *****')

sfile = open('./Inputs/SavedList.txt', 'w')
for item1 in saved:
    sfile.write(" %s %s %s %s\n" % (item1['id'],item1['name'],item1['us'],item1['days']))

sfile.close()
