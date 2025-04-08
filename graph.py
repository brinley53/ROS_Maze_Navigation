import sys
from disjoint_set import DisjointSet

class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]
        self.marked = [] #make a list to keep track of vertices we've marked for traversals

    def add_edge(self, u, v, w):
        self.graph[u][v] = w
        self.graph[v][u] = w

    def dfs(self, start): 
        self.marked = [] #make the marked list empty
        return self._dfs(start)

    def _dfs(self, current): #recursive private function for depth first search
        #first, mark the current node
        self.marked.append(current)
        #now, search through every neighbor
        for neighbor in self.graph[current]:
            #check to see if we've already visited that neighbor
            if neighbor not in self.marked:
                self._dfs(neighbor)
        return self.marked
    
    def bfs(self, start): 
        self.marked = [] #nothing is marked
        queue = [start] #have the start initially on the queue
        current = start #initialize a current vertex
        while len(queue) != 0: #while the queue is not empty
            current = queue.pop(0) #pop the front
            self.marked.append(current) #mark the current one
            for neighbor in self.graph[current]: 
                if neighbor not in self.marked and neighbor not in queue:
                    queue.append(neighbor) #add every neighbor we haven't already done to the queue
        return self.marked

    def dijkstra(self, src):
        # Stores shortest distance.
        dist = [sys.maxsize] * self.V
        # Shortest distance to the same node is 0.
        dist[src] = 0

        #create fringe to hold changeable nodes
        fringe = []
        for i in range(self.V):
            fringe.append([i,-1])

        while len(fringe) != 0: #go through each element in the fringe and evaluate them
            shortest = fringe.pop(0)
            if shortest[0] != src: #skip the source node
                dist[shortest[0]] = shortest[1] #store the final value of the shortest path
            #check all the edges that the shortest has
            index = 0
            while index < self.V:
                edge_dist = self.graph[shortest[0]][index]
                if edge_dist != 0:
                    edge_dist += max(0, shortest[1])
                    #compare the value to the value in the fringe
                    for pair in fringe:
                        if pair[0] == index:
                            if edge_dist < pair[1] or pair[1] == -1:
                                pair[1] = edge_dist
                index+=1
            fringe = self.reorder_fringe(fringe)
                    
        # You have to call print_solution by passing dist.
        # In this way everyone's output would be standardized.
        self.print_dijkstra(dist)
    
    def reorder_fringe(self, fringe):
        #helper function to reorder the fringe for dijkstra
        new_fringe = [[-1,-1]]
        for tup in fringe:
            if tup[1] == -1:
                new_fringe.append(tup)
            else:
                i = 0
                while tup[1] > new_fringe[i][1]:
                    if new_fringe[i][1] == -1:
                        break
                    i += 1
                new_fringe.insert(i, tup)
        new_fringe.remove([-1,-1])
        return new_fringe

    def print_dijkstra(self, dist):
        print("Vertex \t Distance from Source")
        for node in range(self.V):
            print(f"{node} \t->\t {dist[node]}")

    def prim(self):
        # Store the resulting graph.
        # where result[i] keeps the source vertex.
        # See the example output for expected result.
        result = [None] * self.V
        
        #create fringe to hold changeable nodes
        fringe = []
        for i in range(self.V):
            fringe.append([i,-1])

        while len(fringe) != 0: #go through each element in the fringe and evaluate them
            #arbitrarily start at a node
            node = fringe.pop(0)[0]

            #check all the edges that the node has
            index = 0
            while index < self.V:
                edge_dist = self.graph[node][index]
                if edge_dist != 0:
                    #compare the value to the value in the fringe
                    for pair in fringe:
                        if pair[0] == index:
                            if edge_dist < pair[1] or pair[1] == -1:
                                pair[1] = edge_dist
                                result[index] = node
                index+=1
            fringe = self.reorder_fringe(fringe)
        
        # You have to call print_solution by passing the output graph.
        # In this way everyone's output would be standardized.
        self.print_prim(result)

    def print_prim(self, result):
        print("Edge \t Weight")
        for i in range(1, self.V):
            print(f"{result[i]} - {i} \t {self.graph[i][result[i]]}")

    def kruskal(self):

        ds = DisjointSet(self.V)
        # Your code.
        result = []
        
        #create fringe to hold changeable nodes
        fringe = []

        i = 0
        while i < self.V:
            j = 0
            while j < self.V:
                if self.graph[i][j] != 0:
                    fringe.append([i, j, self.graph[i][j]])
                j += 1
            i += 1
        
        fringe = self.reorder_fringe_k(fringe)

        while len(fringe) != 0: #go through each element in the fringe and evaluate them
            #get shortest path
            node = fringe.pop(0)
            if not ds.isConnected(node[0], node[1]):
                result.append(node)
                ds.unionByWeight(node[0], node[1])

            fringe = self.reorder_fringe_k(fringe)

        # Similar to the previous e.g. print your
        # resulting graph.
        self.print_kruskal(result)

    def isConnected(self, node, result):
        #determines whether two nodes are connected

        return False

    def print_kruskal(self, result):
        print("Edge \t Weight")
        # Note that the below code is slightly different than the Prim's.
        # You can change this print code according to your choice, but
        # you have to display your graph in (vertex->vertex weight) format.
        for edge in result:
            print(f"{edge[0]} -> {edge[1]} \t {edge[2]}")

    def reorder_fringe_k(self, fringe):
        #helper function to reorder the fringe for kruskal
        new_fringe = [[-1,-1,-1]]
        for tup in fringe:
            i = 0
            while tup[2] > new_fringe[i][2]:
                if new_fringe[i][2] == -1:
                    break
                i += 1
            new_fringe.insert(i, tup)
        new_fringe.remove([-1,-1,-1])
        return new_fringe