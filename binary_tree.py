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