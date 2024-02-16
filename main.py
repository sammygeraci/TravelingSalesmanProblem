import time

import numpy as np


class Graph:
    def __init__(self):
        self.size = 0
        self.adjacency = np.array([[]])

    def __str__(self):
        return f"size: {self.size}\nadjacency matrix:\n{np.array2string(self.adjacency)}"

    def read_graph(self, filename):
        est = time.process_time()
        st = time.time()
        print(f"Reading nodes of graph from file \"{filename}\"")
        f = open(filename, "r")
        rows_str = f.read().split("\n")
        rows_str.pop()
        rows = []
        for row_str in rows_str:
            rows.append(row_str.split())
        self.size = len(rows[-1])
        list_for_arr = []
        for row in rows:
            weights = []
            for element in row:
                weights.append(int(element))
            zeros_list = [0] * (self.size - len(row))
            arr_row = weights + zeros_list
            list_for_arr.append(arr_row)
        arr = np.array(list_for_arr)
        for i in range(self.size):
            for j in range(i + 1, self.size):
                arr[i, j] = arr[j, i]
        self.adjacency = arr
        eet = time.process_time()
        et = time.time()
        res = st - et
        eres = eet - est
        print(f"Execution Time: {res}")
        print(f"CPU Execution Time: {eres} seconds\n\n")

    def read_graph_section(self, filename, section_size):
        est = time.process_time()
        st = time.time()
        print(f"Reading first {section_size} nodes of graph from file \"{filename}\"")
        f = open(filename, "r")
        rows_str = f.read().split("\n")
        if section_size >= len(rows_str):
            print(f"Selection size {section_size} cannot be greater than original graph size. ")
            return
        else:
            rows_str = rows_str[:section_size]
        rows = []
        for row_str in rows_str:
            rows.append(row_str.split())
        self.size = section_size
        list_for_arr = []
        for row in rows:
            weights = []
            for element in row:
                weights.append(int(element))
            zeros_list = [0] * (self.size - len(row))
            arr_row = weights + zeros_list
            list_for_arr.append(arr_row)
        arr = np.array(list_for_arr)
        for i in range(self.size):
            for j in range(i + 1, self.size):
                arr[i, j] = arr[j, i]
        self.adjacency = arr
        eet = time.process_time()
        et = time.time()
        res = st - et
        eres = eet - est
        print(f"Execution Time: {res}")
        print(f"CPU Execution Time: {eres} seconds\n\n")

    def brute_force_tsp(self):
        est = time.process_time()
        st = time.time()
        print(f"Using brute force to solve traveling salesman problem for current graph")
        min_dist = -1
        nodes = [node for node in range(self.size)]
        path = []
        for node in nodes:
            print(f"Progress at {(node / self.size) * 100}%")
            remaining = nodes.copy()
            remaining.remove(node)
            curr_path = self.brute_force_recurse(node, remaining, min_dist)
            if curr_path[0]:
                min_dist = curr_path[1]
                path = curr_path[2]
            # print(min_dist)

        eet = time.process_time()
        et = time.time()
        res = et - st
        eres = eet - est
        print(f"Execution Time: {res} seconds")
        print(f"CPU Execution Time: {eres} seconds\n\n")
        return [min_dist, path]

    def brute_force_recurse(self, origin, remaining, dist_budget):
        if len(remaining) == 1:
            if self.adjacency[origin, remaining[0]] < dist_budget or dist_budget == -1:
                return [True, self.adjacency[origin, remaining[0]], [origin, remaining[0]]]
            else:
                return [False]

        else:
            min_rem_dist = dist_budget
            new_min = False
            new_path = []
            for next_node in remaining:
                dist_to_next = self.adjacency[origin, next_node]

                if dist_to_next < min_rem_dist or min_rem_dist == -1:
                    new_dist_budget = -1 if min_rem_dist == -1 else min_rem_dist - dist_to_next
                    next_remaining = remaining.copy()
                    next_remaining.remove(next_node)
                    curr_path = self.brute_force_recurse(next_node, next_remaining, new_dist_budget)

                    if curr_path[0]:
                        min_rem_dist = dist_to_next + curr_path[1]
                        new_min = True
                        new_path = [origin] + curr_path[2]

            if new_min:
                return [True, min_rem_dist, new_path]
            else:
                return [False]

    def nearest_neighbor(self):
        est = time.process_time()
        st = time.time()
        print("Using nearest neighbor to find traveling salesman problem solution for current graph")
        nodes = [node for node in range(self.size)]
        min_path_dist = -1
        path = []
        for node in nodes:
            print(f"Progress at {(node / self.size) * 100}%")
            new_path = [node]
            path_dist = 0
            remaining = nodes.copy()
            remaining.remove(node)
            while len(remaining) > 0 and (path_dist < min_path_dist or min_path_dist == -1):
                closest_next = -1
                closest_dist = -1
                for next_node in remaining:
                    if self.adjacency[new_path[-1], next_node] < closest_dist or closest_next == -1:
                        closest_dist = self.adjacency[new_path[-1], next_node]
                        closest_next = next_node
                path_dist += closest_dist
                if path_dist < min_path_dist or min_path_dist == -1:
                    new_path.append(closest_next)
                    remaining.remove(closest_next)
                else:
                    break
            if path_dist < min_path_dist or min_path_dist == -1:
                path = new_path
                min_path_dist = path_dist
        eet = time.process_time()
        et = time.time()
        res = et - st
        eres = eet - est
        print(f"Execution Time: {res} seconds")
        print(f"CPU Execution Time: {eres} seconds\n\n")
        return [min_path_dist, path]

    def nearest_neighborhood(self, neighborhood_size):
        est = time.process_time()
        st = time.time()
        print("Using nearest neighborhood to find traveling salesman problem solution for current graph")
        nodes = [node for node in range(self.size)]
        min_path_dist = -1
        path = []
        for node in nodes:
            print(f"Progress at {(node / self.size) * 100}%")
            new_path = [node]
            path_dist = 0
            remaining = nodes.copy()
            remaining.remove(node)
            while len(new_path) < self.size and (path_dist < min_path_dist or min_path_dist == -1):
                origin = new_path[-1]
                new_path.remove(origin)
                if min_path_dist == -1:
                    nh = self.nearest_neighborhood_recurse(origin, remaining.copy(), -1, neighborhood_size)
                else:
                    dist_budget = min_path_dist - path_dist
                    nh = self.nearest_neighborhood_recurse(origin, remaining.copy(), dist_budget, neighborhood_size)
                if nh[0]:
                    path_dist += nh[1]
                    new_path = new_path + nh[2]
                    for n in nh[2]:
                        if not n == origin:
                            remaining.remove(n)
                else:
                    break

            if (path_dist < min_path_dist or min_path_dist == -1) and len(new_path) == self.size:
                path = new_path
                min_path_dist = path_dist
        eet = time.process_time()
        et = time.time()
        res = et - st
        eres = eet - est
        print(f"Execution Time: {res} seconds")
        print(f"CPU Execution Time: {eres} seconds\n\n")
        return [min_path_dist, path]

    def nearest_neighborhood_recurse(self, origin, remaining, dist_budget, level):
        if len(remaining) == 1:
            if self.adjacency[origin, remaining[0]] < dist_budget or dist_budget == -1:
                return [True, self.adjacency[origin, remaining[0]], [origin, remaining[0]]]
            else:
                return [False]
        elif level == 1:
            closest_next = -1
            closest_dist = -1
            for next_node in remaining:
                if self.adjacency[origin, next_node] < closest_dist or closest_next == -1:
                    closest_dist = self.adjacency[origin, next_node]
                    closest_next = next_node
            if closest_dist < dist_budget or dist_budget == -1:
                return [True, closest_dist, [origin, closest_next]]
            else:
                return [False]
        else:
            min_rem_dist = dist_budget
            new_min = False
            new_path = []
            for next_node in remaining:
                dist_to_next = self.adjacency[origin, next_node]

                if dist_to_next < min_rem_dist or min_rem_dist == -1:
                    new_dist_budget = -1 if min_rem_dist == -1 else min_rem_dist - dist_to_next
                    next_remaining = remaining.copy()
                    next_remaining.remove(next_node)
                    curr_path = self.nearest_neighborhood_recurse(next_node, next_remaining, new_dist_budget, level - 1)
                    if curr_path[0]:
                        min_rem_dist = dist_to_next + curr_path[1]
                        new_min = True
                        new_path = [origin] + curr_path[2]
            if new_min:
                return [True, min_rem_dist, new_path]
            else:
                return [False]


def main():
    g = Graph()
    g.read_graph_section("Size150.graph", 15)
    print(g)
    print(g.brute_force_tsp())
    print(g.nearest_neighbor())
    print(g.nearest_neighborhood(10))


main()
