import networkx as nx
from box2dframework  import *
from math import sqrt
import numpy
import math
import pyglet
import subprocess
import pygame
pygame.init()


def playSound():
    try:
        pygame.mixer.music.load("sounds/discovery1.wav")
        pygame.mixer.music.play()
    except:
        raise RuntimeError("donwload ALL sounds from http://www.sounds-resource.com/pc_computer/wogoo/sound/1485/ ")
    #pygame.event.wait()



#pyglet.app.run()
# import os
# import urllib
# import zipfile

# def getunzipped(theurl, thedir):
#   name = os.path.join(thedir, 'temp.zip')
#   try:
#     name, hdrs = urllib.urlretrieve(theurl, name)
#   except IOError, e:
#     print "Can't retrieve %r to %r: %s" % (theurl, thedir, e)
#     return
#   try:
#     z = zipfile.ZipFile(name)
#   except zipfile.error, e:
#     print "Bad zipfile (from %r): %s" % (theurl, e)
#     return
#   for n in z.namelist():
#     dest = os.path.join(thedir, n)
#     destdir = os.path.dirname(dest)
#     if not os.path.isdir(destdir):
#       os.makedirs(destdir)
#     data = z.read(n)
#     f = open(dest, 'w')
#     f.write(data)
#     f.close()
#   z.close()
#   os.unlink(name)


# loc = ""+__file__
# bn = os.path.basename(__file__)
# loc = loc[0:len(loc)-len(bn)]+"/dlsounds/"
# getunzipped('http://www.sounds-resource.com/download/1482/PC Computer - World of Goo - General Sound Effects 12.zip',loc)


# not yet used
class GooBase(object):
    def __init__(self):
        pass
# not yet used
class CommonGoo(object):
    def __init__(self):
        super(CommonGoo,self).__init__()

        # connections
        self.maxDegree = 99999
        self.maxConnectDegree = 2
        # user interactivity
        self.removable = False
        # NODE PYSICS
        self.density = 1.0
        self.noseRadius = 1.0
        # EDGE PHYSICS
        self.jointType = 'distance' # / 'rope'
        self.stiffness = 1.0
        self.edgeLength = 1.0
        self.edgeStability = 1.0


class GooParam(object):
    def __init__(self, scale = 1.0):
        self.gooDensity = 0.1
        self.gooRadius = 1.0 * scale
        self.maxGooDist = self.gooRadius * 14.0
        self.addAsEdgeDist = 2.0 * self.gooRadius
        self.maxGooDegree = 10000
        self.maxConnectTo = 2
        self.maxReactionForce = 14.0
        self.edgeSpringHz = 2.0


class GooGraph(nx.Graph):
    def __init__(self, world, param):
        super(GooGraph, self).__init__()
        self.world = world
        self.param = param


        self.gooFixture=b2FixtureDef(shape=b2CircleShape(radius=(self.param.gooRadius)),
                            density=self.param.gooDensity, friction=100.2)


    def addGooNode(self, pos):  
        goo = self.world.CreateDynamicBody(position=pos,fixtures=self.gooFixture)
        self.add_node(goo)
        return goo

    def connectNodes(self, gooA, gooB):
        dfn=b2DistanceJointDef(
                           frequencyHz=self.param.edgeSpringHz,
                           dampingRatio=0.1,
                           bodyA=gooA,bodyB=gooB,
                           anchorA=gooA.position,
                           anchorB=gooB.position
                       )
        j=self.world.CreateJoint(dfn)
        self.add_edge(gooA, gooB, joint=j)
        playSound()

    def nodeDist(self,a,b):
            return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def manageClick(self, pos ,draggedGoo = None):
        if draggedGoo is not None:
            if self.degree(draggedGoo)>0:
                return
        numGoo = len(self)
        print numGoo
        #pos = b2Vec2(*pos)
        if numGoo == 0 :
            self.addGooNode(pos)
        else: 
            candidates = []
            # find the nearest k goos
            for otherGoo in self:
                degree = self.degree(otherGoo)
                #print 'degree', degree
                if(degree < self.param.maxGooDegree):
                    oPos = otherGoo.position
                    distance = self.nodeDist(pos, oPos)
                    if distance < self.param.maxGooDist and distance > self.param.gooRadius*1.9:
                        # TODO check if an edge is better
                        candidates.append((otherGoo, distance, oPos))
            def keyF(t):
                return t[1]

            sortedCandidates = sorted(candidates,key=keyF)
            numC = len(sortedCandidates)
            if len(sortedCandidates) >= self.param.maxConnectTo:
                sortedCandidates = sortedCandidates[0:self.param.maxConnectTo]
            numC = len(sortedCandidates)

            if(numC == 1 and numGoo ==1):
                newGoo = self.addGooNode(pos)
                self.connectNodes(sortedCandidates[0][0], newGoo)
            elif numC >= 2 :
                # add goo to graph and add edge
                if self.param.maxConnectTo ==2 :

                    gooA = sortedCandidates[0][0]
                    gooB = sortedCandidates[1][0]

                    # check if there is an edge between
                    # those goos
                    if(self.has_edge(gooA, gooB)):

                        if draggedGoo is None :
                            newGoo = self.addGooNode(pos) 
                        else:
                            newGoo = draggedGoo
                        for oldGoo, dist, oldGooPos in sortedCandidates :

                            self.connectNodes(oldGoo, newGoo)
                    else:
                        self.connectNodes(gooA, gooB)
                else :
                    raise RuntimeError("not impolemented yet")


class GalaxyOfGooTester(Framework):
    name="GalaxyOfGooTester"
    description="This demonstrates a soft distance joint. Press: (b) to delete a body, (j) to delete a joint"
    bodies=[]
    joints=[]
    def __init__(self):
        super(GalaxyOfGooTester, self).__init__()

        self.param = GooParam(scale=1.0)
        self.gooGraph = GooGraph(world=self.world, param=self.param)

        #self.maxCreationDist = 15.0
        #self.minCreationDist = 7.0
        #self.maxGooDegree   = 10
        #self.connectNewToMax = 5
   


        ground=self.world.CreateStaticBody(
            position=(0,0),
            shapes=b2PolygonShape(box=(100,1)),
            friction=100.0
            )

        
        self.gooMode = True

    def numGoo(self):
        return len(self.gooGraph)

                    


     
    def Keyboard(self, key):
        if key==Keys.K_g:
            self.gooMode = True

        if key==Keys.K_n:
            self.gooMode = False


    def MouseDown(self, p):
        if self.gooMode:
            self.gooGraph.manageClick(p)

        return super(GalaxyOfGooTester,self).MouseDown(p)

    def MouseUp(self, p):
        """
        Left mouse button up.
        """     
        if self.mouseJoint:
            self.gooGraph.manageClick(p, self.mouseJoint.bodyB)
            self.world.DestroyJoint(self.mouseJoint)
            self.mouseJoint = None

        if self.bombSpawning:
            self.CompleteBombSpawn(p)




    def Step(self, param):
        Framework.Step(self, param)
        #print "\n\n"
        for gooA,gooB in self.gooGraph.edges():
            joint =self.gooGraph[gooA][gooB]['joint']
            jReacForce = joint.GetReactionForce(30)
            jReacForce=(numpy.array(jReacForce)**2).sum()**(0.5)
            #print jReacForce.round(4)

            if(jReacForce>self.param.maxReactionForce):
                self.gooGraph.remove_edge(gooA, gooB)
                self.world.DestroyJoint(joint)

if __name__=="__main__":
     main(GalaxyOfGooTester)
