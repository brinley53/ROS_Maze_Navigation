import sys
from disjoint_set import DisjointSet

class Graph:
    def __init__(self):
        # len(self.graph) = vertices
        # self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]
        self.graph = {} # key = node, val = dictionary of neighbor and weight
        self.marked = [] #make a list to keep track of vertices we've marked for traversals

    def add_edge(self, u, v, w):
        # u and v are vertices, w is weight
        if u not in self.graph:
            self.graph[u] = {}
        if v not in self.graph:
            self.graph[v] = {}
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
        dist = {node: sys.mazsize for node in self.graph}
        # Shortest distance to the same node is 0.
        dist[src] = 0

        #create fringe to hold changeable nodes
        fringe = [[node, dist[node]] for node in self.graph]

        while fringe: #go through each element in the fringe and evaluate them
            fringe.sort(key=lambda x: x[1])
            current, current_dist = fringe.pop(0)

            for neighbor in self.graph[current]:
                edge_weight = self.graph[current][neighbor]
                new_dist = current_dist + edge_weight
                if new_dist < dist[neighbor]:
                    dist[neighbor] = new_dist
                    for i in range(len(fringe)):
                        if fringe[i][0] == neighbor:
                            fringe[i][1] = new_dist
                            break
                    
        # You have to call print_solution by passing dist.
        # In this way everyone's output would be standardized.
        self.print_dijkstra(dist)

    def print_dijkstra(self, dist):
        print("Vertex \t Distance from Source")
        for node in dist:
            print(f"{node} \t->\t {dist[node]}")

