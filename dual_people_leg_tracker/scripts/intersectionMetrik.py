# -*- coding: utf-8 -*-

# Generates random line seqments and defines a metrik on the intersections

import random
import matplotlib.pyplot as plt
import itertools
import numpy as np

# Close existing figures
plt.close('all')

class LineSegment:
    def __init__(self, id_):
        self.id_ = id_
        self.point0 = np.array([random.uniform(0, 1),random.uniform(0, 1), 0.0])
        self.point1 = np.array([random.uniform(0, 1),random.uniform(0, 1), 0.0])
        
        self.length = np.linalg.norm((self.point1 - self.point0))
        self.dirVec = (self.point1 - self.point0)
             
        
    def __str__(self):
        return "[" + str(self.id_) + "]" + str(self.point0) + " ~ " + str(self.point1);        

def drawLineSegment(lineSegment):
    plt.plot([lineSegment.point0[0], lineSegment.point1[0]], [lineSegment.point0[1], lineSegment.point1[1]], 'k-')
    circle0=plt.Circle((lineSegment.point0[0],lineSegment.point0[1]),.01,color='g')    
    circle1=plt.Circle((lineSegment.point1[0],lineSegment.point1[1]),.01,color='r')
    
    fig = plt.gcf()
    fig.gca().add_artist(circle0)
    fig.gca().add_artist(circle1)
    
    #plt.text(lineSegment.point0[0], lineSegment.point0[1], str(lineSegment.id_))
    
def crossingMetric(lineSegment0, lineSegment1):
    
    D = np.dot(lineSegment0.dirVec, lineSegment1.dirVec)
    
    if(D == 0):
        print "The lines are parallel"
        return 0.0


    ax = lineSegment0.point0[0]
    ay = lineSegment0.point0[1]
    
    bx = lineSegment0.dirVec[0]
    by = lineSegment0.dirVec[1]
    
    cx = lineSegment1.point0[0]
    cy = lineSegment1.point0[1]
    
    dx = lineSegment1.dirVec[0]
    dy = lineSegment1.dirVec[1]
    
    
    u=(cy*bx-ay*bx-cx*by+ax*by)/(dx*by-dy*bx)
    t=(cx+dx*u-ax)/bx

    
    #t = np.cross((lineSegment0.point0 - lineSegment1.point0),lineSegment0.dirVec) / np.cross(lineSegment1.dirVec,lineSegment0.dirVec);
 
    
    if(u<=1 and u>=0 and t>=0 and t<=1):
        
        dist = (pow(1-abs(0.5-u)*2,2) + pow(1-abs(0.5-t)*2,2))/2

        print "u= " + str(u)        
        print "t= " + str(t)
        print "dist= " + str(dist)
        
        crossingPoint = lineSegment1.point0 + u*lineSegment1.dirVec;
    
        circle0=plt.Circle((crossingPoint[0],crossingPoint[1]),dist/10.0,color='b')    
    
        fig = plt.gcf()
        fig.gca().add_artist(circle0)
        
        print "Lines cross"
        return dist
        
    print "Lines do not cross"
    return 0.0
    

# Number of lines:
N = 5;

lineSegments = [LineSegment(i) for i in range(N)]


meanCrossingError = 0.0
# Check the crossing value
for perm in itertools.combinations(lineSegments,2):
    #print "Checking combination " + str(perm[0].id_) + " - " + str(perm[1].id_)
    meanCrossingError += crossingMetric(perm[0],perm[1])
    
plt.title('MeanCrossingError = ' + str(meanCrossingError/N))

# Draw plot
for lineSegment in lineSegments:
    #print lineSegment
    drawLineSegment(lineSegment)

plt.axis('equal')
axes = plt.gca()
axes.set_xlim([-0.1,1.1])
axes.set_ylim([-0.1,1.1])

plt.show()
    
