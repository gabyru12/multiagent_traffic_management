from scipy.spatial import distance
import random

import heapq
from collections import deque

class Network:
    def __init__(self):
        self.nodes = {} # nodeID -> Node
        self.roads = {} # roadID -> Road
        self.lanes = {} # laneID -> Lane
        self.next_lane_uid = 0

        self.spawn_nodes = []

    def create_network(self, grid_size, lane_count, road_length):
        spacing = road_length
        grid_size = grid_size
        lane_count = lane_count
        node_id = 0
        road_id = 0

        # Create nodes
        for i in range(grid_size):
            for j in range(grid_size):
                node_id += 1
                x = -spacing + j * spacing
                y = spacing - i * spacing
                self.nodes[node_id] = Node(self, node_id, [x, y], "traffic_light")

        # Create bidirectional roads
        for i in range(grid_size):
            for j in range(grid_size):
                node_index = i * grid_size + j + 1
                # Horizontal roads
                if j < grid_size - 1:
                    right_node = node_index + 1
                    for direction in [(self.nodes[node_index], self.nodes[right_node]), (self.nodes[right_node], self.nodes[node_index])]:
                        road_id += 1
                        e = Road(road_id, lane_count, *direction)
                        self.roads[road_id] = e
                        for lane in e.lanes.values():
                            self.lanes[lane.UID] = lane

                # Vertical roads
                if i < grid_size - 1:
                    below_node = node_index + grid_size
                    for direction in [(self.nodes[node_index], self.nodes[below_node]), (self.nodes[below_node], self.nodes[node_index])]:
                        road_id += 1
                        e = Road(road_id, lane_count, *direction)
                        self.roads[road_id] = e
                        for lane in e.lanes.values():
                            self.lanes[lane.UID] = lane

        # Create spawn nodes around the border 
        all_x = [node.pos[0] for node in self.nodes.values()]
        all_y = [node.pos[1] for node in self.nodes.values()]
        min_x, max_x = min(all_x), max(all_x)
        min_y, max_y = min(all_y), max(all_y)

        spawn_offset = spacing  # how far spawn nodes sit from the grid

        # Add spawners around each outer node
        for node in list(self.nodes.values()):
            x, y = node.pos
            is_left = x == min_x
            is_right = x == max_x
            is_top = y == max_y
            is_bottom = y == min_y

            # Left border
            if is_left:
                node_id += 1
                spawn = Node(self, node_id, [x - spawn_offset, y], "spawn")
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

            # Right border
            if is_right:
                node_id += 1
                spawn = Node(self, node_id, [x + spawn_offset, y], "spawn")
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

            # Top border
            if is_top:
                node_id += 1
                spawn = Node(self, node_id, [x, y + spawn_offset], "spawn")
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

            # Bottom border
            if is_bottom:
                node_id += 1
                spawn = Node(self, node_id, [x, y - spawn_offset], "spawn")
                self.spawn_nodes.append((spawn, node))
                self.nodes[node_id] = spawn

        for spawn, main_node in self.spawn_nodes:
            for direction in [(spawn, main_node), (main_node, spawn)]:
                road_id += 1
                e = Road(road_id, lane_count, *direction)
                self.roads[road_id] = e
                for lane in e.lanes.values():
                    self.lanes[lane.UID] = lane

        self.spawn_nodes = [node[0] for node in self.spawn_nodes]

        self.update_flow_roads()
        self.make_lane_connections()

        print(f"Grid built with {len(self.nodes)} nodes, {len(self.roads)} roads.")

    def update_flow_roads(self):
        for road in self.roads.values():
            for inFlowRoad in road.startNode.inRoads:
                road.inFlowRoads.append(inFlowRoad)
            for outFlowRoad in road.endNode.outRoads:
                road.outFlowRoads.append(outFlowRoad)
            common_road = [e for e in road.inFlowRoads if e in road.outFlowRoads]
            road.parallelRoad = common_road[0]

    def make_lane_connections(self):
        for road in self.roads.values():
            for inFlowRoad in road.inFlowRoads:
                if road.vector_direction == inFlowRoad.vector_direction:
                    road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                    road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                elif inFlowRoad.vector_direction == (0,-1):
                    if road.vector_direction == (-1,0):
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                    elif road.vector_direction == (1,0):
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                elif inFlowRoad.vector_direction == (0,1):
                    if road.vector_direction == (1,0):
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                    elif road.vector_direction == (-1,0):
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                elif inFlowRoad.vector_direction == (-1,0):
                    if road.vector_direction == (0,1):
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                    elif road.vector_direction == (0,-1):
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                elif inFlowRoad.vector_direction == (1,0):
                    if road.vector_direction == (0,-1):
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[0])
                        road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1])
                    elif road.vector_direction == (0,1):
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1])
                        road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[0])
                if abs(inFlowRoad.vector_direction[0] - road.vector_direction[0]) == 2 or abs(inFlowRoad.vector_direction[1] - road.vector_direction[1]) == 2:
                    road.parallelRoad = inFlowRoad
                    road.lanes[1].inFlowLanes.append(inFlowRoad.lanes[1]) 
                    road.lanes[0].inFlowLanes.append(inFlowRoad.lanes[1]) 
            for outFlowRoad in road.outFlowRoads:
                if road.vector_direction == outFlowRoad.vector_direction:
                    road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                    road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (0,-1):
                    if road.vector_direction == (-1,0):
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (1,0):
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (0,1):
                    if road.vector_direction == (1,0):
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (-1,0):
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (-1,0):
                    if road.vector_direction == (0,1):
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (0,-1):
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                elif outFlowRoad.vector_direction == (1,0):
                    if road.vector_direction == (0,-1):
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                        road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])
                    elif road.vector_direction == (0,1):
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[0])
                        road.lanes[0].outFlowLanes.append(outFlowRoad.lanes[1])
                if abs(outFlowRoad.vector_direction[0] - road.vector_direction[0]) == 2 or abs(outFlowRoad.vector_direction[1] - road.vector_direction[1]) == 2:
                    road.parallelRoad = outFlowRoad
                    road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[1])
                    road.lanes[1].outFlowLanes.append(outFlowRoad.lanes[0])

    def generate_route(self, start_road_ID: int, start_lane_UID: int, end_node_ID: int):
        """
        Dijkstra shortest-path on lanes.
        start_road: the road where the car currently is
        start_lane: the lane the car is currently in
        end_node: the target node
        Returns deque of (road, lane)
        """

        routes = []
        startRoad = self.roads[start_road_ID]
        startLane = self.lanes[start_lane_UID]
        
        # Priority queue: (cost, counter, lane)
        pq = []
        parent = {}        # lane → previous lane
        dist = {}          # lane → distance
        visited = set()
        counter = 0        # unique increasing ID (tie-breaker)

        # Initialize Dijkstra with the car's current lane
        dist[startLane] = 0
        parent[startLane] = None
        heapq.heappush(pq, (0, counter, startLane))
        counter += 1
        goal_lane = None
        path_cost = None

        # ----- Dijkstra search -----
        while pq:
            current_cost, _, lane = heapq.heappop(pq)

            if lane in visited:
                continue
            visited.add(lane)

            road = lane.road

            # Goal condition: the lane arrives at the desired node
            if road.endNode.ID == end_node_ID:
                goal_lane = lane
                path_cost = current_cost
                break

            # Relax neighbors (the lanes you can drive into next)
            for next_lane in lane.outFlowLanes:
                new_cost = current_cost + next_lane.length

                if next_lane not in dist or new_cost < dist[next_lane]:
                    dist[next_lane] = new_cost
                    parent[next_lane] = lane
                    heapq.heappush(pq, (new_cost, counter, next_lane))
                    counter += 1

        # ----- No route found -----
        if goal_lane is None:
            return deque(), path_cost

        # ----- Reconstruct the lane-by-lane route -----
        path = deque()
        lane = goal_lane

        while lane is not None:
            path.appendleft((lane.road.ID, lane.UID))
            lane = parent[lane]

        return path, path_cost

    def yen_k_shortest_paths(self, start_road_ID, start_lane_ID, end_node_ID, K):
        """
        Computes the K shortest loopless paths using Yen’s algorithm.
        Uses self.generate_route() which returns (path, cost).
        Each path is a deque of (roadID, laneID).
        """

        start_lane_UID = self.roads[start_road_ID].lanes[start_lane_ID].UID

        # ---- First shortest path (Dijkstra) ----
        first_path, first_cost = self.generate_route(start_road_ID, start_lane_UID, end_node_ID)
        if not first_path:
            return []   # no path exists
        A = [(first_path, first_cost)]   # shortest paths found
        B = []                           # candidate paths: (cost, path)

        # ---- Loop to find paths 2..K ----
        for k in range(1, K):
            prev_path, prev_cost = A[k-1]

            prev_path_list = list(prev_path)
            num_nodes = len(prev_path_list)

            # Spur at every node except the last
            for i in range(num_nodes - 1):

                # ---- Root path ----
                root_path = prev_path_list[:i]

                if not root_path:
                    spur_road_ID  = start_road_ID
                    spur_lane_UID  = start_lane_UID
                else:
                    spur_road_ID, spur_lane_UID = root_path[-1]

                # ---- Temporarily remove roads that recreate previously found paths ----
                removed_roads = {}  
                for p, _ in A:
                    p_list = list(p)
                    if len(p_list) > i and p_list[:i] == root_path:
                        banned_road_ID, banned_lane_UID = p_list[i]
                        lane = self.lanes[banned_lane_UID]
                        if lane not in removed_roads:             
                            removed_roads[lane] = lane.outFlowLanes
                        lane.outFlowLanes = []       

                # ---- Compute spur path ----
                spur_path, spur_cost = self.generate_route(spur_road_ID, spur_lane_UID, end_node_ID)

                # ---- If spur path exists, build full path candidate ----
                if spur_path:
                    spur_list = list(spur_path)
                    if spur_list and root_path:
                        # drop the first lane of the spur, because it repeats root's last lane
                        spur_list = spur_list[1:]

                    new_full_path = deque(root_path + spur_list)
                    new_full_cost = sum(self.lanes[laneUID].length for _, laneUID in new_full_path) - self.roads[new_full_path[0][0]].length

                    heapq.heappush(B, (new_full_cost, new_full_path))

                # ---- Restore removed outflows ----
                for lane, original in removed_roads.items():
                    lane.outFlowLanes = original

            # ---- No more candidates ----
            if not B:
                break

            # ---- Choose the shortest remaining candidate ----
            cost, path = heapq.heappop(B)
            A.append((path, cost))

        for j in range(len(A)):
            A[j] = list(A[j])
            for i in range(len(A[j][0])):
                A[j][0][i] = list(A[j][0][i])
                A[j][0][i][1] = self.lanes[A[j][0][i][1]].ID
                A[j][0][i] = tuple(A[j][0][i])
            A[j][0].popleft()
            A[j] = tuple(A[j])
        return A

class Node:
    def __init__(self, network, ID: int, pos: list[int, int], type_of_intersection: str):
        self.network = network
        self.ID = ID
        self.pos = pos  # (x, y)
        self.type = type_of_intersection  # e.g. tl=traffic_light, s=stop, etc.
        self.neighbourNodes = []
        self.inRoads = []
        self.outRoads = []

class Road:
    def __init__(self, ID: int, laneCount: int, startNode: "Node", endNode: "Node"):
        self.ID = ID
        self.laneCount = laneCount
        self.lanes = {}
        self.startNode = startNode
        self.endNode = endNode
        self.nodeSet = sorted([startNode.ID, endNode.ID])
        self.parallelRoad = None
        self.length = None
        self.angle_sin = None
        self.angle_cos = None
        self.vector_direction = None
        self.inFlowRoads = []
        self.outFlowRoads = []
        self.init_properties()

    # Lane connections between roads
    def init_properties(self):
        self.network = self.startNode.network
    
        # Update node properties
        self.startNode.outRoads.append(self)
        self.endNode.inRoads.append(self)
        self.startNode.neighbourNodes.append(self.endNode)

        # Road properties
        self.length = int(distance.euclidean(self.startNode.pos, self.endNode.pos))
        self.angle_sin = (self.endNode.pos[1]-self.startNode.pos[1]) / self.length
        self.angle_cos = (self.endNode.pos[0]-self.startNode.pos[0]) / self.length
        self.vector_direction = ((self.endNode.pos[0] - self.startNode.pos[0])/self.length, (self.endNode.pos[1] - self.startNode.pos[1])/self.length)
        for i in range(self.laneCount):
            laneID = i
            self.lanes[i] = Lane(laneID, self.network.next_lane_uid, self)
            self.network.next_lane_uid += 1

class Lane:
    def __init__(self, ID: int, UID: int, road: "Road"):
        self.ID = ID
        self.UID = UID
        self.road = road
        self.length = self.road.length
        self.parallelLanes = []
        self.vehicles = deque()
        self.inFlowLanes = []
        self.outFlowLanes = []

    def init_properties(self):
        for i in self.road.lanes.keys():
            laneID = self.road.lanes[i].ID
            if laneID != self.ID:
                self.parallelLanes.append(self.road.lanes[i])
