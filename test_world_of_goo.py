import networkx as nx
from box2dframework  import *
from math import sqrt
import numpy
import math





class GooParam(object):
    def __init__(self, scale = 1.0):
        self.gooRadius = 1.0 * scale
        self.maxGooDist = self.gooRadius * 14.0
        self.addAsEdgeDist = 2.0 * self.gooRadius
        self.maxGooDegree = 10000
        self.maxConnectTo = 2




class GooGraph(nx.Graph):
    def __init__(self, world, param):
        super(GooGraph, self).__init__()
        self.world = world
        self.param = param


        self.gooFixture=b2FixtureDef(shape=b2CircleShape(radius=(self.param.gooRadius)),
                            density=0.1, friction=100.2)


    def addGooNode(self, pos):  
        goo = self.world.CreateDynamicBody(position=pos,fixtures=self.gooFixture)
        self.add_node(goo)
        return goo

    def connectNodes(self, gooA, gooB):
        dfn=b2DistanceJointDef(
                           frequencyHz=2.0,
                           dampingRatio=0.1,
                           bodyA=gooA,bodyB=gooB,
                           anchorA=gooA.position,
                           anchorB=gooB.position
                       )
        j=self.world.CreateJoint(dfn)
        self.add_edge(gooA, gooB, joint=j)


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
                    if distance < self.param.maxGooDist and distance > self.param.gooRadius*3.0:
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

        self.maxCreationDist = 15.0
        self.minCreationDist = 7.0
        self.maxGooDegree   = 10
        self.connectNewToMax = 5
        self.gooRadius = 1.0
        self.maxReactionForce = 16.0
        boxSize = 1.0
        gridShape = [4,2]
        boxDistance  = [10.0,10.0]
        bd=boxDistance[0]
        increment = [boxSize*2 +boxDistance[0], 
                     boxSize*2 +boxDistance[1]]


        ground=self.world.CreateStaticBody(
            position=(0,-boxSize),
            shapes=b2PolygonShape(box=(100,1)),
            friction=100.0
            )

        

        self.gooFixture=b2FixtureDef(shape=b2CircleShape(radius=(self.gooRadius)),
                            density=0.05, friction=100.2)




        for x in range(gridShape[0]):
            for y in range(gridShape[1]):
                pos = (increment[0]*x,increment[1]*y)
                #self.addGoo(pos)

        for x in range(gridShape[0]-1):
            for y in range(gridShape[1]-1):
                pos = (increment[0]*x+bd/2,increment[1]*y +bd/2)
                #self.addGoo(pos)

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

            if(jReacForce>self.maxReactionForce):
                self.gooGraph.remove_edge(gooA, gooB)
                self.world.DestroyJoint(joint)

if __name__=="__main__":
     main(GalaxyOfGooTester)
