import networkx as nx
import numpy as np

"""
This class is responsible for managing obstacle clusters.
It creates a graph of obstacles and finds connected components.
It stores the centroids of the obstacle clusters.
"""
class ObstacleClusterManager:

    """
    This method initializes the ObstacleClusterManager class.
    It initializes the obstacle clusters centroids and obstacle points.
    """
    def __init__(self):

        
        self._obstacle_clusters_centroids = np.array([])
        self._obs_points = None

    """
    This method creates a graph of obstacles and finds connected components.
    It adds nodes to the graph based on the obstacle points and adds edges based on the euclidean distance.
    It computes the centroids of the obstacle clusters by calculating the mean of the x and y coordinates.
    It stores the centroids in the obstacle clusters centroids list.
    """    
    def obstacle_clustering(self):


        # Crea un grafo vuoto
        G = nx.Graph()

        # Aggiungi nodi al grafo
        for i in range(len(self._obs_points)):
            G.add_node(i)

        # Aggiungi archi tra i nodi in base alla distanza euclidea
        for i in range(len(self._obs_points)): 
            for j in range(i+1, len(self._obs_points)):
                distance = np.linalg.norm(self._obs_points[i] - self._obs_points[j])
                if distance <= 0.65:
                    G.add_edge(i, j)
    
        # Trova le componenti connesse
        connected_components = list(nx.connected_components(G))
        
        # Stampa le componenti connesse
        for i, component in enumerate(connected_components):
            sumx=0
            sumy=0
            for j in component:
                sumx+=self._obs_points[0][j]
                sumy+=self._obs_points[1][j]
            self._obstacle_clusters_centroids.append([sumx/len(component),sumy/len(component),len(component)])
        

    """
    This method returns the obstacle clusters centroids.
        
    :return: The obstacle clusters centroids.
    """        
    def get_obstacle_clusters_centroids(self):

        return self._obstacle_clusters_centroids

    """
        This method sets the obstacle points.
        
        :param obs_points: The obstacle points.
    """
    def set_obstacle_points(self, obs_points):

        self._obs_points = obs_points