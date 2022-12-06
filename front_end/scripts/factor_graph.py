
from utils import *


class FactorGraph:
    def __init__(self) -> None:
        self.vertexes = []
        self.edges = []

    def add_vertex(self, index, vertex):
        self.edges.append(adjacent_edge(self.vertexes[index], vertex))
        self.vertexes.append(vertex)

    def add_adjacent_vertex(self, vertex):
        self.add_vertex(-1, vertex)

    def save_graph(self):
        pass