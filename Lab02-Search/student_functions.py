import numpy as np
from queue import Queue
import heapq
from queue import PriorityQueue
import math
def DFS(matrix, start, end):
    """
    DFS algorithm:
    Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer  
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes, each key is a visited node,
        each value is the adjacent node visited before it.
    path: list
        Founded path
    """
    # TODO: 
    
    path=[]
    visited={}
  
    #Hàm đệ quy DFS
    def dfs_rec(node):
        visited[node] = True
        path.append(node)
    #Check nếu là node end thì return true
        if node == end:
            return True
    #Chạy vòng lặp quét cả ma trận rồi đệ quy
        for i in range(len(matrix)):
            if matrix[node][i] != 0 and not visited.get(i):
                if dfs_rec(i):
                    return True
        path.pop()
        return False
    #Sử dụng hàm đệ quy với tham số là node start
    dfs_rec(start)
    return visited, path

def BFS(matrix, start, end):
    """
    DFS algorithm
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited 
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """

    # TODO: 
    path=[]
    visited={}
    #khởi tạo hàng đợi cho BFS
    queue = Queue()
    #Bỏ node đầu tiên vào hàng đợi và đánh dấu đã thăm
    queue.put(start)
    visited[start] = None
    #Tiến hành duyệt bằng BFS 
    while queue:
        current_node = queue.get()
        #check xem node hiện tại có phải node end hay chưa
        if current_node == end:
            break
        #lần lượt duyệt các node kề của node hiện tại bằng cách duyệt ma trận
        for neighbor in range(len(matrix)):
            if matrix[current_node][neighbor] != 0 and neighbor not in visited:
                queue.put(neighbor)
                visited[neighbor] = current_node
    #tạo đường đi bằng mảng visited
    node = end
    while node is not None:
        path.append(node)
        node = visited[node]
    path.reverse()
    return visited, path



def UCS(matrix, start, end):
    """
    Uniform Cost Search algorithm
     Parameters:visited
    ---------------------------
    matrix: np array
        The graph's adjacency matrix
    start: integer
        starting node
    end: integer
        ending node
    
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:  
    path=[]
    visited={}
    # Hàng đợi ưu tiên lưu các node chưa được duyệt
    heap = []  
    # Đưa nút start vào hàng đợi ưu tiên với chi phí là 0
    heapq.heappush(heap, (0, start))    

    while heap:
        # Lấy ra nút có chi phí thấp nhất trong hàng đợi ưu tiên
        cost, node = heapq.heappop(heap)  
        # Nếu nút đó là nút kết thúc thì trả về kết quả  
        if node == end:     
            path = [end]
            while end != start:
                end = visited[end]
                path.append(end)
            path.reverse()
            return visited, path
        if node not in visited:
            # Đánh dấu node là đã duyệt
            visited[node] = None  
        for neighbor in range(len(matrix[node])):
                neighbor_cost = matrix[node][neighbor]
                if neighbor_cost != 0 and neighbor not in visited:
                    # Thêm các node kề vào hàng đợi ưu tiên với chi phí tương ứng
                    heapq.heappush(heap, (cost + neighbor_cost, neighbor))   
                    # Lưu trữ node kề vào visited với node trước đó là node hiện tại
                    visited[neighbor] = node    

    return visited, path     



def GBFS(matrix, start, end):
    """
    Greedy Best First Search algorithm 
    heuristic : edge weights
     Parameters:
    ---------------------------
    matrix: np array 
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
   
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    path=[]
    visited={}
    pq = PriorityQueue()
    pq.put((0, start))
    path ={}
    while pq:
        # Lấy phần tử với cost nhỏ nhất
        min_cost_node = pq.get()
        cost, node = min_cost_node
        # Nếu tìm thấy node end thì tạo path
        if node == end:
            path_list = []
            while node in path:
                path_list.append(node)
                node = path[node]
            path_list.append(start)
            path_list.reverse()
            return visited, path_list
        # Đánh dấu node hiện tại là đã thăm rồi
        visited[node] = True 
        # Thêm các node họ hàng chưa được thăm của node hiện tại vào hàng đợi ưu tiên
        for neighbor in range(len(matrix[node])):
            neighbor_cost = matrix[node][neighbor]
            if neighbor_cost > 0 and neighbor not in visited:
                path[neighbor] = node
                pq.put((neighbor_cost, neighbor))
    return visited, path  

def Astar(matrix, start, end, pos):
    """
    A* Search algorithm
    heuristic: Euclidean distance based on positions parameter
     Parameters:
    ---------------------------
    matrix: np array UCS
        The graph's adjacency matrix
    start: integer 
        starting node
    end: integer
        ending node
    pos: dictionary. keys are nodes, values are positions
        positions of graph nodes
    Returns
    ---------------------
    visited
        The dictionary contains visited nodes: each key is a visited node, 
        each value is the key's adjacent node which is visited before key.
    path: list
        Founded path
    """
    # TODO:
    visited = {}
    path = []
    path = {}
    queue = []
    path[start] = None
    #hàm heuristic
    def euclidean_distance(pos1, pos2):
         return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
     
    #f_score = g_score + h_score
    #g_scores
    g_scores = {node: float('inf') for node in range(len(matrix))}
    g_scores[start] = 0
    #f_score
    f_scores = {node: float('inf') for node in range(len(matrix))}
    f_scores[start] = euclidean_distance(pos[start], pos[end])

    #thêm vào hàng đợi ưu tiên với f_score ở node start
    heapq.heappush(queue, (f_scores[start], start))

    while queue:
        current = heapq.heappop(queue)[1]
        #Nếu node hiện tại là end thì trả về đường đi
        if current == end:
            path_list = []
            while current is not None:
                path_list.append(current)
                current = path[current]
            path_list.reverse()
            return visited, path_list
        #đánh dấu là đã thăm node này
        visited[current] = True

        for neighbor in range(len(matrix[current])):
            if matrix[current][neighbor] > 0 and neighbor not in visited:
                current_g_score = g_scores[current] + matrix[current][neighbor]
                if current_g_score < g_scores[neighbor]:
                    path[neighbor] = current
                    g_scores[neighbor] = current_g_score
                    h = euclidean_distance(pos[neighbor], pos[end])
                    f_scores[neighbor] = g_scores[neighbor] + h
                    heapq.heappush(queue, (f_scores[neighbor], neighbor))

    return visited, path


