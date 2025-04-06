class TreeNode:
  def __init__(self, value):
    self.value = value
    self.left = None
    self.right = None

class BinaryTree:
    def __init__(self):
        self.root = None
    
    def __preorder_traversal(self, node_list, cur_node):
        '''Preorder traversal'''
        if cur_node == None:
            return node_list
        #add the current node to the list then traverse the left then the right
        node_list.append(cur_node.value)
        self.__preorder_traversal(node_list, cur_node.left)
        self.__preorder_traversal(node_list, cur_node.right)
        return node_list


    def preorder_traversal(self) -> list:
        """Implement preorder traversal."""
        return self.__preorder_traversal([], self.root) #call private method
    
    def __inorder_traversal(self, node_list, cur_node):
        if cur_node == None:
            return node_list
        #traverse the left, add the current node, then traverse the right
        self.__inorder_traversal(node_list, cur_node.left)
        node_list.append(cur_node.value)
        self.__inorder_traversal(node_list, cur_node.right)
        return node_list

    def inorder_traversal(self) -> list:
        """Implement inorder traversal."""
        return self.__inorder_traversal([], self.root) #call private method

    def __postorder_traversal(self, node_list, cur_node):
        if cur_node == None:
            return node_list
        #traverse the left, then the right, then add the current node to the list
        self.__postorder_traversal(node_list, cur_node.left)
        self.__postorder_traversal(node_list, cur_node.right)
        node_list.append(cur_node.value)
        return node_list

    def postorder_traversal(self) -> list:
        """Implement postorder traversal."""
        return self.__postorder_traversal([], self.root) #call private method
    
class Graph:
    def __init__(self, vertices):
        self.vertices = vertices
        self.adjacency_list = [[] for _ in range(vertices)]
        self.marked = [] #make a list to keep track of vertices we've marked for traversals

    def add_edge(self, u, v):
        """Add an edge between vertices u and v."""
        self.adjacency_list[u].append(v)
        self.adjacency_list[v].append(u)

    def dfs(self, start): 
        self.marked = [] #make the marked list empty
        return self._dfs(start)

    def _dfs(self, current): #recursive private function for depth first search
        #first, mark the current node
        self.marked.append(current)
        #now, search through every neighbor
        for neighbor in self.adjacency_list[current]:
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
            for neighbor in self.adjacency_list[current]: 
                if neighbor not in self.marked and neighbor not in queue:
                    queue.append(neighbor) #add every neighbor we haven't already done to the queue
        return self.marked


    
def main():
    # Create a binary tree
    bt = BinaryTree()
    bt.root = TreeNode(1)
    bt.root.left = TreeNode(2)
    bt.root.right = TreeNode(3)
    bt.root.left.left = TreeNode(4)
    bt.root.left.right = TreeNode(5)
    bt.root.right.left = TreeNode(6)
    bt.root.right.right = TreeNode(7)
    bt.root.left.left.left = TreeNode(8)
    bt.root.left.left.right = TreeNode(9)
    bt.root.right.right.right = TreeNode(10)

    # Test the traversals
    print("Preorder Traversal:", bt.preorder_traversal())
    print("Inorder Traversal:", bt.inorder_traversal())    
    print("Postorder Traversal:", bt.postorder_traversal())  

    # Create a graph with 20 vertices
    graph = Graph(20)

    # Add edges (change as needed)
    graph.add_edge(0, 1)
    graph.add_edge(0, 2)
    graph.add_edge(1, 3)
    graph.add_edge(1, 4)
    graph.add_edge(2, 5)
    graph.add_edge(2, 6)
    graph.add_edge(3, 7)
    graph.add_edge(3, 8)
    graph.add_edge(4, 9)
    graph.add_edge(4, 10)
    graph.add_edge(5, 11)
    graph.add_edge(5, 12)
    graph.add_edge(6, 13)
    graph.add_edge(6, 14)
    graph.add_edge(7, 15)
    graph.add_edge(7, 16)
    graph.add_edge(8, 17)
    graph.add_edge(8, 18)
    graph.add_edge(9, 19)

    # Test DFS and BFS from a source vertex
    print("DFS from vertex 0:", graph.dfs(0))  
    print("BFS from vertex 0:", graph.bfs(0))

    # Create a graph with 4 vertices
    graph = Graph(4)
    graph.add_edge(0, 1)
    graph.add_edge(0, 2)
    graph.add_edge(1, 2)
    graph.add_edge(2, 0)
    graph.add_edge(2, 3)
    graph.add_edge(3, 3)

    # Test DFS and BFS from a source vertex
    print("DFS from vertex 2:", graph.dfs(2))
    print("BFS from vertex 2:", graph.bfs(2)) 

main()
