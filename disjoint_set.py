class DisjointSet:
    def __init__(self, size):
        self.vertex = [i for i in range(size)]
        self.weight = [1] * size
        self._size = size

    def validate(self, v1):
        #Check if v1 is a valid index
        return self._size > v1 and v1 >= 0

    def size(self, v1):
        #return the size of the set v1 belongs to
        if self.validate(v1): #check to see if it's a valid index
            return self.weight[self.find(v1)] #return size
        return 0    

    def parent(self, v1):
        #return the parent of v1... if v1 is the root of a tree, return the negative size of the tree for which v1 is the root
        if self.validate(v1): #check to see if it's a valid index
            if self.vertex[v1] == v1:
                return -self.weight[v1]
            return self.vertex[v1]

    def isConnected(self, v1, v2):
        #check if given 2 vertex are connected
        if self.validate(v1) and self.validate(v2):
            #find the root of both sets
            parent1 = self.find(v1)
            parent2 = self.find(v2)
            #if v1 and v2 have the same roots, they're connected
            return parent1 == parent2

    def find(self, v1):
        #returns the root of the set v1 belongs to
        if self.validate(v1):
            #hold the index of the parent
            prev = v1
            parent = self.parent(v1)
            while parent >= 0:
                prev = parent
                parent = self.parent(parent)
            return prev
       
    def unionByWeight(self, v1, v2):
        #connects two elements v1 and v2 together based on weight
        root1 = self.find(v1)
        root2 = self.find(v2)
        if self.validate(v1) and self.validate(v2) and v1 != v2 and root1 != root2:
            #if weight v1 > v2 then v2 merges into v1; root of v2 merges with root of v1
            if self.size(v1) > self.size(v2):
                self.weight[root1] += self.size(v2) #increase the weight of the root
                self.vertex[root2] = root1 #change the parent of root2 to be root1
            else:
                self.weight[root2] += self.size(v1) #increase the weight of the root
                self.vertex[root1] = root2
