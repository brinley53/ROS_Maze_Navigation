import sys

#DISJOINT SET CLASS FROM LAB 5
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

class Graph:
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for _ in range(vertices)] for _ in range(vertices)]

    def add_edge(self, u, v, w):
        self.graph[u][v] = w
        self.graph[v][u] = w

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

def main():
    # Create a graph with 21 vertices.
    graph = Graph(21)

    # Add edges and their weights.
    graph.add_edge(0, 1, 4)
    graph.add_edge(0, 2, 1)
    graph.add_edge(1, 3, 3)
    graph.add_edge(2, 4, 2)
    graph.add_edge(3, 5, 2)
    graph.add_edge(4, 6, 2)
    graph.add_edge(5, 7, 2)
    graph.add_edge(7, 8, 2)
    graph.add_edge(6, 8, 2)

    graph.add_edge(8, 9, 5)
    graph.add_edge(8, 10, 4)
    graph.add_edge(9, 11, 3)
    graph.add_edge(10, 11, 1)

    graph.add_edge(11, 12, 1)
    graph.add_edge(12, 13, 1)
    graph.add_edge(13, 14, 1)

    graph.add_edge(14, 15, 1)
    graph.add_edge(14, 16, 10)
    graph.add_edge(15, 17, 1)
    graph.add_edge(16, 20, 1)
    graph.add_edge(17, 18, 1)
    graph.add_edge(18, 19, 1)
    graph.add_edge(19, 20, 1)

    # Run Dijkstra's algorithm from source vertex 0.
    graph.dijkstra(0)

    # Find and print the Prim's Minimum Spanning Tree (MST).
    graph.prim()

    # Find and print the Kruskal's Minimum Spanning Tree (MST).
    graph.kruskal()

main()
