from utils import *
import transformation


class FactorGraph:
    def __init__(self) -> None:
        self.vertexes = []
        self.edges = []

    def add_adjacent_vertex(self, vtx):
        if len(self.vertexes) == 0:
            self.vertexes.append(vtx)
            return self.vertexes[-1], None
        pair = self.vertexes.size + [-1, 0]
        self.edges.append(np.r_[pair, pair_to_edge(self.vertexes[-1], vtx)])
        self.vertexes.append(vtx)
        return self.vertexes[-1], self.edges[-1]

    def add_edge(self, idx1, idx2, homo):
        self.edges.append(np.r_[[idx1, idx2], matrix_to_tf(homo)])
        return self.edges[-1]

    def save_graph(self):
        pass