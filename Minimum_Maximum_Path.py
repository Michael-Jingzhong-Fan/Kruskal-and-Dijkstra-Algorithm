from collections import defaultdict
import numpy as np

#Get-Content input2.txt | python3 planet_solution.py

def get_input(fileName):
    '''
    Gets coordinates of zearth and vertices and the number of vertices
    from a textfile. 
    
    Arguments: 
        fileName: txt file containing coordinates
    
    Returns: 
        a tuple containing (zearth, vertices)
            zearth: coordinates of Zearth
            vertices: An array containing coordinates of vertices
    '''
    lines = open(fileName).read().split('\n')    
    
    #Get number of vertices
    vertexNumber = int(lines[1])
    #Sanity Check
    if (vertexNumber < 1) or (vertexNumber > 500):
        raise Exception('Number of stations should be within 1 and 500')
        
    earth = [0.0, 0.0, 0.0]
    zearth = process_raw_coordinates(lines[0])
    
    vertices = []
    vertices.append(earth)
    vertices.append(zearth)
    
    for i in range(vertexNumber):
        #Get coordinates excluding vertexNumbers and zearth
        coord = process_raw_coordinates(lines[i+2])    
        vertices.append(coord)
    
    return vertices, vertexNumber + 1 + 1 #Add Earth and Zearth

def process_raw_coordinates(line):
    '''
    Extracts coordinates from strings read from a txt file.

    Arguments: 
        line: string read from a txt file
    
    Returns:
        coordinate: coordinate value extracted from string
    '''
    coordinate = []
    #Split string in terms of whitespace
    string = line.split()
    #Convert string to floats
    for num in string:
        num = float(num)
        
        #Check for values
        if (num < -10000.0) | (num > 10000.0):
            raise Exception('Coordinate values must be within -10000, 10000')
        
        coordinate.append(num)
        
    return coordinate
    
def get_distance(coord1, coord2):
    ''' 
    Calculate euclidean distance between two vertices.

    Arguments: 
        coord1: coordinate

        coord2: coordinate
    
    Returns:
        coordinate: coordinate value extracted from string
    '''
    #To numpy array form as list for math operations
    coord1 = np.asarray(coord1)
    coord2 = np.asarray(coord2)
    
    return np.linalg.norm(coord1 - coord2)


class Graph():
    '''
    Class to represent graph
    '''
    def __init__(self, stationNumber):
        #For Kruskal's Algorithm
        self.n = stationNumber #Number of stations
        self.graph = []
        #For Dijkstra's Algorithm
        self.connections = defaultdict(list)
        self.weights={}
        
    def add_connection(self, vertex1, vertex2, weight):
        self.graph.append([vertex1, vertex2, weight])
        
    
    def get_parent_vertices(self,parent,vertex):
        '''
        Find parent vertices of a vertex. 
        '''
        if parent[vertex]== vertex:
            #No parent
            return vertex
        #Iterate
        return self.get_parent_vertices(parent, parent[vertex])
    
    def union(self, parent, rank, x, y):
        xroot = self.get_parent_vertices(parent,x)
        yroot = self.get_parent_vertices(parent,y)
        
        #Attach smaller rank tree under root of
        #high rank tree
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]: #Function to find set the parent of a vertexyroot]:
            parent[yroot] = xroot
        else:
            #If ranks are the same, then make one 
            #a root and increment its rank by one
            parent[yroot] = xroot
            rank[xroot] += 1
    
    def get_minimum_spanning_tree(self):
        '''
        Creates a minimum spanning tree for a given graph
        and returns the minimum spanning tree
            
        Returns: 
            result: A list containing a list of [vertex1, vertex2, weight]
                    values that belong to the minimum spanning tree
        '''
        result = [] #Store the resultant minimum spanning tree
        i,j = 0,0 #index variables for sorted 
                  #connections and result respectively
                  
        #Sort graph acording to connection size
        self.graph = sorted(self.graph,key=lambda item:item[2])
        
        parent, rank = [], []
        
        #Create n subsets with single elements
        for vertex in range(self.n):
            parent.append(vertex)
            rank.append(0)
        
        #Number of connections to be taken is n-1
        while j < self.n - 1:
            #Pick the smallest connection
            vertex1,vertex2,weight = self.graph[i]
            #Increment index to consider next largest connection
            i += 1
            x = self.get_parent_vertices(parent, vertex1)
            y = self.get_parent_vertices(parent, vertex2)
            
            #If including this connection doesn't cause 
            #a cycle, include it in result
            if x is not y:
                j+=1
                result.append([vertex1,vertex2,weight])
                self.union(parent, rank, x, y)
            #else discard the connection
            
        return result
    
    def get_shortest_path(self, start, end, mst):
        '''
        Find the shortest weight path in a graph using Dijkstra's algorithm.
        
        Args: 
            start: Start vertex
            end: End vertex
            mst: Graph in which to find shortest path
        
        Returns: 
            path: Shortest path from start to end
            
        '''
        def _add_connection(mst):
            for connection in mst:
                vertex1 = connection[0]
                vertex2 = connection[1]
                weight = connection[2]
                self.connections[vertex1].append(vertex2)
                self.connections[vertex2].append(vertex1)
                self.weights[(vertex1,vertex2)] = weight
                self.weights[(vertex2,vertex1)] = weight
        
        #Initialise graph for dijkstra implementation
        _add_connection(mst)
        shortestPaths = {start: (None, 0)}
        currentVertex = start
        visited = set()
        weight = 0
        
        while currentVertex != end:
            visited.add(currentVertex)
            destinations = self.connections[currentVertex]
            weightToCurrentVertex = shortestPaths[currentVertex][1]
            
            for nextVertex in destinations:
                weight = self.weights[(currentVertex, nextVertex)] + weightToCurrentVertex
                if nextVertex not in shortestPaths: 
                    shortestPaths[nextVertex] = (currentVertex, weight)
                else:
                    current_shortest_weight = shortestPaths[nextVertex][1]
                    if current_shortest_weight > weight:
                        shortestPaths[nextVertex] = (currentVertex, weight)
                        
            next_destinations = (
                    {vertex: shortestPaths[vertex] for vertex in shortestPaths if vertex not in visited}
                    )
            if not next_destinations:
                return "route not possible"
            
            #Next vertex is the destination with the lowest weight
            currentVertex = min(next_destinations, key=lambda k:next_destinations[k][1])
            
        path = []
        while currentVertex is not None:
            path.append(currentVertex)
            nextVertex = shortestPaths[currentVertex][0]
            currentVertex = nextVertex
        #Reverse path
        path = path[::-1]
        
        return path
    
    def get_max_weight(self, path):
        '''
        Find maximum weight between two vertices in a path. 
        
        Args: 
            path: minimum weight connection in a path
            
        Returns:
            maxWeight: maximum weight between two vertices
        '''
        maxWeight = 0.0
        for i in range(len(path)-1):
            vertex1 = path[i]
            vertex2 = path[i+1]
            weight = self.weights[(vertex1,vertex2)]
            if weight > maxWeight:
                maxWeight = weight
                
        return maxWeight

    

if __name__ == "__main__":
    fileName = 'input.txt'
    lines = get_input(fileName)
    
    #Unpack
    coordinates, vertexNumbers = get_input(fileName)
    
    #Initialise Graph object for constructing minimum spanning tree
    mst = Graph(vertexNumbers)
    
    #Create completed graph by loading all possible connections
    #Vertex 0, 1 = Earth, Zearth
    for i, coord1 in enumerate(coordinates): 
        for j, coord2 in enumerate(coordinates):
            if (i != j) & (i<j): 
                weight = get_distance(coord1, coord2)
                mst.add_connection(i,j,weight)
    
    result = mst.get_minimum_spanning_tree()
    route = mst.get_shortest_path(0,1,result)
    max_weight = mst.get_max_weight(route)
    
    #print("best route is: {}".format(route))
    print("%.2f" % max_weight)
    
    
    
