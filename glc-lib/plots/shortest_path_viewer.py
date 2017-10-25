import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
from matplotlib import cm
from matplotlib.patches import Circle


#Get the graph
path = np.transpose(np.genfromtxt('shortest_path.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('shortest_path_nodes.txt', delimiter=','))

##prep for plotting
#path = np.transpose(path)
#euc_vert = np.transpose(euc_vert)
#land_vert = np.transpose(land_vert)
#dijk_vert = np.transpose(dijk_vert)
#num_edges=np.size(sink)/2-1
#path_edges=np.size(path)/2



#### Dijkstra Search Results

fig = plt.figure(figsize=(10,10))
ax = plt.gca()
ax.set_xlim([0,10])
ax.set_ylim([0,10])
plt.plot(path[0],path[1])
plt.scatter(nodes[0],nodes[1],s=0.5)

ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([10.0, 10.0],0.5, facecolor="green",alpha=0.3,edgecolor="none"))
plt.show()
plt.save("shortest_path.png")
#ax.add_patch(Rectangle((8.0,1.0), 1.0, 8.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((3.0,5.5), 4.0, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((2.0,8.0), 2.5, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((5.5,8.0), 2.5, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((2.0,1.0), 6.0, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))



##for i in xrange(0,num_edges-1,1):
    ##plt.plot([ source[i][0], sink[i][0] ],[ source[i][1], sink[i][1] ], color='blue',lw=0.1,zorder=0)

#plt.plot(path[0],path[1],lw=3,c='red')

##Plot edges of PRM
##for i in xrange(0,num_edges,1):
##    plt.plot([ source[i][0], sink[i][0] ],[ source[i][1], sink[i][1] ], color='blue',lw=0.1,zorder=0)

##plot examined vertices  
#plt.scatter(dijk_vert[0],dijk_vert[1],c=(dijk_vert[2]),cmap='jet',zorder=2,s=10,lw = 0)
#plt.savefig("../bug_trap_dijk.png")

#### Euclidean A* Search Results

#plt.figure(2)
#fig = plt.figure(figsize=(25,25))
#ax = plt.gca()
#ax.set_xlim([0,10])
#ax.set_ylim([0,10])

#ax.add_patch(Rectangle((1.0,1.0), 1.0, 8.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((8.0,1.0), 1.0, 8.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((3.0,5.5), 4.0, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((2.0,8.0), 2.5, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((5.5,8.0), 2.5, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((2.0,1.0), 6.0, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
##for i in xrange(0,num_edges-1,1):
    ##plt.plot([ source[i][0], sink[i][0] ],[ source[i][1], sink[i][1] ], color='blue',lw=0.1,zorder=0)

#plt.plot(path[0],path[1],lw=3,c='red')

##plot examined vertices 
#plt.scatter(euc_vert[0],euc_vert[1],c=(euc_vert[2]),cmap='jet',zorder=2,s=10,lw = 0)
#plt.savefig("../bug_trap_euc.png")


#### Euclidean A* Search Results

#plt.figure(3)
#fig = plt.figure(figsize=(25,25))
#ax = plt.gca()
#ax.set_xlim([0,10])
#ax.set_ylim([0,10])

#ax.add_patch(Rectangle((1.0,1.0), 1.0, 8.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((8.0,1.0), 1.0, 8.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((3.0,5.5), 4.0, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((2.0,8.0), 2.5, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((5.5,8.0), 2.5, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
#ax.add_patch(Rectangle((2.0,1.0), 6.0, 1.0, facecolor="black",alpha=0.7,edgecolor="none"))
##for i in xrange(0,num_edges-1,1):
    ##plt.plot([ source[i][0], sink[i][0] ],[ source[i][1], sink[i][1] ], color='blue',lw=0.1,zorder=0)

#plt.plot(path[0],path[1],lw=3,c='red')

##plot examined vertices 
#plt.scatter(land_vert[0],land_vert[1],c=(land_vert[2]),cmap='jet',zorder=2,s=10,lw = 0)
#plt.savefig("../bug_trap_land.png")

##plt.show()

