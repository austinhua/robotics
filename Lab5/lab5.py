import math
from klampt import vectorops,so2

#CONFIGURATION VARIABLES

environment = 'normal'
#environment = 'fancy'

#method = 'accumulator'
#method = 'occupancy_grid'
#method = 'robust_occupancy_grid'
method = 'probabilistic_occupancy_grid'

noisy_sensor = True

def init():
    pass

def depthImageToPointCloud(scan):
    """In:
    - scan: a 1D array of quantized depth values
    Out:
    - a list of 2D points giving the point cloud in the
      sensor-local frame.  y axis is forward, x axis is to the right.
      units are in meters.
    """
    #TODO: implement me.  This return array is incorrect
    pointCloud = []

    for i in range(len(scan)):
        if scan[i] == 0:
            continue
        distance = 4 * float(scan[i])/256
        width = distance * 2 * math.tan(math.pi/3)
        x = i * width/100 - width/2
        pointCloud.append((x,distance))

    return pointCloud

def localToWorld(localPC,robotState,cameraExtrinsics):
    """Convert a local point cloud to world coordinates.
    In:
    - localPC: a point cloud in camera-local coordinates
      with x axis to the right, y axis forward.
    - robotState: an (x,y,theta) tuple
    - cameraExtrinsics: the position of the camera, in robot-
      local coordinates.
    Out:
    - a point cloud in world x-y coordinates.
    """
    #TODO: implement me.  This return value is incorrect, it just
    #adds on the local camera displacement
    worldPC = []

    for p in localPC:
        xRobot = p[1] + cameraExtrinsics[0]
        yRobot = -p[0] + cameraExtrinsics[1]

        a = so2.apply(robotState[2], (xRobot, yRobot))
        worldPt = (robotState[0] + a[0], robotState[1] + a[1])
        worldPC.append(worldPt)
    return worldPC


class PointCloudMap:
    """A 2D map consisting of a list of points"""

    def __init__(self,points):
        self.points = points

class GridMap:
    """A 2D map consisting of a grid of occupancy levels.
    - array: a 2D array of numeric values, taking the range from 0 (unoccupied) to 1
      (occupied).  The array is laid out so array[i][j] is in the i'th cell in the
      x-direction and the j'th cell in the y-direction
    - bounds: a pair of tuples [(xmin,ymin),(xmax,ymax)] giving the rectangle
      holding the map
    - vmin, vmax: the minimum and maximum values in the array. Automatically computed
      (usually 0 and 1).
    """

    def __init__(self,array,bounds):
        self.array = array
        self.bounds = bounds
        self.vmin = 0
        self.vmax = 1
        for row in array:
            for v in row:
                self.vmin = min(self.vmin,v)
                self.vmax = max(self.vmax,v)

class MapperBase:
    """By default, just keeps the last scans as a point cloud. """
    def __init__(self):
        self.points = []
    def addScan(self,Tsensor,pointList):
        """Adds the point list to the map.
        - Tsensor: a rigid transform of the sensor in world coordinates
          (Y axis points forward, X axis points to the right, Z points up)
        - pointList: a list of 2-tuples giving the point cloud in world
          coordinates
        """
        self.points = pointList
    def getMap(self):
        """Returns a PointCloudMap or a GridMap describing the world."""
        return PointCloudMap(self.points)

class AccumulatorMapper(MapperBase):
    """Simply adds new scans to a point cloud. """
    def __init__(self):
        self.points = []
    def addScan(self,Tsensor,pointList):
        self.points += pointList
    def getMap(self):
        return PointCloudMap(self.points)

class OccupancyGridMapper(MapperBase):
    """Adds new scans to an occupancy grid.  You will need to implement
    the addScan() method.

    Attributes:
    - bounds: a pair [bmin,bmax] where bmin is the lower left corner and
      bmax is the upper right corner of the range of the map.
    - res: a pair (xres,yres) giving the resolution of the grid in the x and
      y directions.
    - grid: a 2D array (list of lists) containing occupancies (0 for empty,
      1 for filled)
    """
    def __init__(self,width,height,xres,yres):
        self.bounds = [(-0.5*width,-0.5*height),(0.5*width,0.5*height)]
        self.res = (xres,yres)
        self.grid = [[0]*yres for i in range(xres)]
    def addScan(self,Tsensor,pointList):
        #TODO: implement me
        width = self.bounds[1][0]*2
        height = self.bounds[1][1]*2
        xBlockSize = width/float(self.res[0])
        yBlockSize = height/float(self.res[1])

        for p in pointList:
            x = int((p[0]-self.bounds[0][0])/xBlockSize)
            y = int((p[1]-self.bounds[0][1])/yBlockSize)
            self.grid[x][y] = 1
    def getMap(self):
        return GridMap(self.grid,self.bounds)

class RobustOccupancyGridMapper(MapperBase):
    """Adds new scans to an occupancy grid.  You will need to implement
    the addScan() method.

    Attributes:
    - bounds: a pair [bmin,bmax] where bmin is the lower left corner and
      bmax is the upper right corner of the range of the map.
    - res: a pair (xres,yres) giving the resolution of the grid in the x and
      y directions.
    - grid: a 2D array (list of lists) containing *counts*.
    """
    def __init__(self,width,height,xres,yres):
        self.bounds = [(-0.5*width,-0.5*height),(0.5*width,0.5*height)]
        self.res = (xres,yres)
        self.grid = [[0]*yres for i in range(xres)]
        #TODO: tune me
        self.threshold = 0.1
        self.minPoints = 6
    def addScan(self,Tsensor,pointList):
        #TODO: implement me
        width = self.bounds[1][0]*2
        height = self.bounds[1][1]*2
        xBlockSize = width/float(self.res[0])
        yBlockSize = height/float(self.res[1])

        for p in pointList:
            x = int((p[0]-self.bounds[0][0])/xBlockSize)
            y = int((p[1]-self.bounds[0][1])/yBlockSize)
            self.grid[x][y] += 1
        pass
    def getMap(self):
        #TODO: you may wish to produce a new grid rather than the grid of
        #counts
        xres = self.res[0]
        yres = self.res[1]
        map = [[0]*yres for i in range(xres)]
        adjX = [-1, 0, 1, 1,  1,  0, -1, -1]
        adjY = [ 1, 1, 1, 0, -1, -1, -1,  0]

        for i in range(xres):
            for j in range(yres):
                count = self.grid[i][j]
                if count <= self.minPoints:
                    map[i][j] = 0
                    continue
                maxNeighbor = 0
                for k in range(len(adjX)):
                    nX = i + adjX[k]
                    nY = j + adjY[k]
                    if nX < 0 or nX >= xres or nY < 0 or nY >= yres:
                        continue
                    nCount = self.grid[nX][nY]
                    maxNeighbor = max(maxNeighbor, nCount)
                if count < self.threshold * maxNeighbor:
                    map[i][j] = 0
                    continue
                map[i][j] = 1

        return GridMap(map,self.bounds)


class ProbabilisticOccupancyGridMapper(MapperBase):
    """Adds new scans to an occupancy grid.  You will need to implement
    the addScan() method.

    Attributes:
    - bounds: a pair [bmin,bmax] where bmin is the lower left corner and
      bmax is the upper right corner of the range of the map.
    - res: a pair (xres,yres) giving the resolution of the grid in the x and
      y directions.
    - occupancyCounts, freeCounts: a 2D array (list of lists) containing
      *counts*.
    """
    def __init__(self,width,height,xres,yres):
        self.bounds = [(-0.5*width,-0.5*height),(0.5*width,0.5*height)]
        self.res = (xres,yres)
        occupancyPrior = 3 # c
        freePrior = 3 # d
        self.occupancyCounts = [[occupancyPrior]*yres for i in range(xres)]
        self.freeCounts = [[freePrior]*yres for i in range(xres)]
    def addScan(self,Tsensor,pointList):
        #TODO: implement me
        width = self.bounds[1][0]*2
        height = self.bounds[1][1]*2
        xBlockSize = width/float(self.res[0])
        yBlockSize = height/float(self.res[1])

        xCoord = Tsensor[1][0]
        yCoord = Tsensor[1][1]

        for p in pointList:
            visited = []
            x = int((p[0]-self.bounds[0][0])/xBlockSize)
            y = int((p[1]-self.bounds[0][1])/yBlockSize)
            self.occupancyCounts[x][y] += 1
            visited.append((x,y))

            u = 0
            while u < 1:
                rayPt = vectorops.interpolate((xCoord, yCoord), p, u)
                x = int((rayPt[0]-self.bounds[0][0])/xBlockSize)
                y = int((rayPt[1]-self.bounds[0][1])/yBlockSize)

                if (x, y) not in visited:
                    self.freeCounts[x][y] += 1
                    visited.append((x,y))
                u += .05

    def getMap(self):
        #TODO: you may wish to produce a new grid rather than the grid of
        #counts
        xres = self.res[0]
        yres = self.res[1]
        map = [[0]*yres for i in range(xres)]

        for i in range(xres):
            for j in range(yres):
                if (float(self.occupancyCounts[i][j]))/(float(self.freeCounts[i][j] + self.occupancyCounts[i][j])) > 0.5:
                    map[i][j] = 1

        return GridMap(map,self.bounds)


def get_control(t):
    """Used by the server to determine how the robot should drive. Return value is
    a tuple (vfwd,vleft,turnrate)"""
    if t % 2 < 1.5:
        return (1,0,3)
    else:
        return (1,0,-3)


# WRITTEN ANSWERS:
# (B.1): The drawback of accumulating points is that the number of points in the point cloud increases over time so eventually you will run out of memory.
# (B.3): The advantages of the occupancy grid technique over an accumulator is that you use a fixed amount of memory to represent the occupancy grid. The drawback is that it is only accurate up to a certain resolution.
# (C.1): The map is very inaccurate because a single incorrect value will result in the occupancy grid square showing up as filled. These errors accumulate over time resulting in more and more grid spaces incorrectly identified as filled due to the noisy sensor data.
