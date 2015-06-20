#!/usr/bin/env python

__author__ = "Varban Metodiev"

__credits__ = [ " 'Dijkstra's Algorithm - Illustrated Explanation' by Eoin Bailey" ]

__license__ = "GPL"
__version__ = "0.99b"
__maintainer__ = "Varban Metodiev"
__email__ = "varban.metodiev@gmail.com"
__status__ = "Working"

class Node(object):
    
    __node_id = None

    previous_id = None
    visited_flag = None
    distance_from_node = None

    neighbor_cost = {}

    def __init__(self, node_id):
        self.__node_id = node_id

    def setPreviousId(self, prev_id):
        self.previous_id = prev_id

    def getPreviousId(self):
        return self.previous_id

    def getNodeId(self):
        return self.__node_id

    def getVisitedFlag(self):
        return self.visited_flag

    def setVisitedFlag(self):
        self.visited_flag = 1

    def getDistanceFromNode(self):
        return self.distance_from_node

    def setDistanceFromNode(self, dist):
        self.distance_from_node = dist

    def setNeighborCost(self, neigh_cost_dict):
        self.neighbor_cost = neigh_cost_dict

    def getNeighborCost(self, node_id):
        node_id_value = int()
        if node_id in self.neighbor_cost.keys():
            node_id_value = int(self.neighbor_cost[node_id])
            return node_id_value


class Vertex(object):
    
    pair_left = None
    pair_right = None
    cost = None

    def __init__(self, left, right, cost):
        self.pair_left = left
        self.pair_right = right
        self.cost = cost

    def getVertexLeftPair(self):
        return self.pair_left

    def getVertexRightPair(self):
        return self.pair_right

    def getVertexCost(self):
        return self.cost

    def printVertexPair(self):
        p_left = self.pair_left.getNodeId()
        p_right = self.pair_right.getNodeId()
        print p_left, p_right, self.cost
        
class Graph(object):
    
    __graph_name = None
    __graph_description = None

    last_vertex = None
    vertex_list = []

    def __init__(self, name, description):
        self.__graph_name = name
        self.__graph_description = description

    def addVertex(self, left, right, cost):
        self.last_vertex = Vertex(left, right, cost)
        self.vertex_list.append(self.last_vertex)
        return self.last_vertex

    def printVertexes(self):
        for counter in range(len(self.vertex_list)):
            self.vertex_list[counter].printVertexPair()
            print "==========="

class Dijkstra(object):

    vertexes = []
    neigh_dict = dict()

    start_node = None

    def __init__(self, graph):
        self.vertexes = graph.vertex_list

    def createNeighborNodes(self):

        adj_array = []
        listed_nodes = []
        
        for counter_left in range(len(self.vertexes)):
            left =  self.vertexes[counter_left].getVertexLeftPair().getNodeId()

            if left not in listed_nodes:

                listed_nodes.append(left)
                dictionary = dict()
                
                for counter_right in range(len(self.vertexes)):
                    left_inner = self.vertexes[counter_right].getVertexLeftPair().getNodeId()
                    if (left_inner == left):
                        value = self.vertexes[counter_right].getVertexRightPair()
                        dictionary.update({value.getNodeId():self.vertexes[counter_right].getVertexCost()})
                        print "Dictionary is:", dictionary
                        adj_array.append(value)

                self.neigh_dict.update({left : adj_array})
                adj_array = list()
                self.vertexes[counter_left].getVertexLeftPair().setNeighborCost(dictionary)


    def findSpfDijkstra(self, start_node, inf_distance):

        #Check for starting point
        starting_points = self.neigh_dict.keys()
        print "Starting points are:", starting_points
        
        if start_node not in starting_points:
            print "Cannot start from that node!"
        else:
            print "Starting from:", start_node

        #Mark start point Dist = 0, all other Dist = "INF"
        nodes_list = []
        dijkstra_nodes_list = []

        for key in starting_points:
            index = self.neigh_dict[key]
            print "key:", key, "index", index

            for element in index:
                if (element.getNodeId() == start_node):
                    element.setDistanceFromNode(int(0))

                else:
                    element.setDistanceFromNode(int(inf_distance))
                    
                if element not in nodes_list:
                    nodes_list.append(element)

        for check in nodes_list:
            print "Nodes consistency check:", check.getNodeId(), "with dist:", check.getDistanceFromNode()
             
        #Dijskstra
        while (len(nodes_list) > 0):
            node_with_min_dist = min(nodes_list, key=lambda(item): item.distance_from_node)
            node_with_min_dist.setVisitedFlag()
            distance = node_with_min_dist.getDistanceFromNode()

            print "=============================================================="

            if (node_with_min_dist.getNodeId() in starting_points):
                for node in self.neigh_dict[node_with_min_dist.getNodeId()]:
                    for element in nodes_list:
                        if (element.getNodeId() == node.getNodeId()):
                            if(element.getVisitedFlag != 1):
                                abc = node_with_min_dist.getNeighborCost(element.getNodeId())
                                if (element.getDistanceFromNode() > distance + node_with_min_dist.getNeighborCost(element.getNodeId())):
                                    element.setDistanceFromNode(distance + node_with_min_dist.getNeighborCost(element.getNodeId()))
                                    element.setPreviousId(node_with_min_dist.getNodeId())

                if (node_with_min_dist.getVisitedFlag != 1):
                    nodes_list.remove(node_with_min_dist)
                    dijkstra_nodes_list.append(node_with_min_dist)
                    starting_points.remove(node_with_min_dist.getNodeId())

            else:
                nodes_list.remove(node_with_min_dist)
                dijkstra_nodes_list.append(node_with_min_dist)

        return dijkstra_nodes_list

    def showPathFromNode(self, end_node, nodes_list):

 #       for element in nodes_list:
 #          print element.getNodeId(), element.getPreviousId()

        current_node = end_node
        
        while (current_node.getPreviousId() != None):
            current_node = filter(lambda node: node.getNodeId() == current_node.getPreviousId(), nodes_list)[0]
            print current_node.getNodeId()

    def showNeighborNodes(self):
        print "From showNeighborNodes method:\n", self.neigh_dict


#Create nodes
node1 = Node(1)
node2 = Node(2)
node3 = Node(3)
node4 = Node(4)
node5 = Node(5)
node6 = Node(6)
node7 = Node(7)
node8 = Node(8)
node9 = Node(9)


#Create Graph
tmpGraph = Graph("TheGraph", "WeightedGraph")

tmpGraph.addVertex(node1, node2, 7)
tmpGraph.addVertex(node2, node1, 7)

tmpGraph.addVertex(node1, node3, 4)
tmpGraph.addVertex(node3, node1, 4)

tmpGraph.addVertex(node2, node3, 2)
tmpGraph.addVertex(node3, node2, 2)

tmpGraph.addVertex(node1, node4, 5)
tmpGraph.addVertex(node4, node1, 5)

tmpGraph.addVertex(node2, node5, 25)
tmpGraph.addVertex(node5, node2, 25)

tmpGraph.addVertex(node3, node8, 9)
tmpGraph.addVertex(node8, node3, 9)

tmpGraph.addVertex(node4, node6, 9)
tmpGraph.addVertex(node6, node4, 9)

tmpGraph.addVertex(node5, node7, 10)
tmpGraph.addVertex(node7, node5, 10)

tmpGraph.addVertex(node6, node8, 20)
tmpGraph.addVertex(node8, node6, 20)

tmpGraph.addVertex(node7, node9, 2)
tmpGraph.addVertex(node9, node7, 2)

tmpGraph.addVertex(node8, node9, 3)
tmpGraph.addVertex(node9, node8, 3)

print "Graph:"
tmpGraph.printVertexes()

#Set node distances, choose start node
tmpDijkstra = Dijkstra(tmpGraph)
tmpDijkstra.createNeighborNodes()
tmpDijkstra.showNeighborNodes()

#SPF - Setting Max distance to 555
spf_list = tmpDijkstra.findSpfDijkstra(1, 555)
print "Showing path from node7 (passed as method argument) to the start node:"
tmpDijkstra.showPathFromNode(node7, spf_list);
