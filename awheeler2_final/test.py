def findFrontierCell(self, grid):
    for i in range(len(grid)):  # height should be set to height of grid
        if (grid[i] > 0):
            continue
        x = i % self.width
        y = (i - x) / self.width
        if (self.validateFrontier(x, y, grid)):
            return x, y
    def validateFrontier(self, x, y, grid):
        flag = False
        for i in range(x - 1, x + 2):
            for j in range(y - 1, y + 2): #test greater than zero
                if (grid[j * self.width + i] == 0):
                    flag = True
        if (grid[y * self.width + x] == -1 and flag == True):  # convert from x,y to grid cell number
            return True
        else:
            return False

        self.complete = False
        self.backup = False

    def readBumper(self, msg):
        self.backup=True

        hile(map.complete == False and not rospy.is_shutdown()):  # change to map not complete

        if (map.backup == True):
            print "Backing Up"
            turtle.backup()
            map.backup = False
        w