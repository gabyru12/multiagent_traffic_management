from spade.agent import Agent
from spade.behaviour import OneShotBehaviour, CyclicBehaviour
from spade.template import Template
from spade.message import Message

from collections import deque
import json
import asyncio
import heapq


class TrafficLightAgent(Agent):
    """
    Each TrafficLightAgent controls a road segment and manages lane capacities.
    It responds to cars' requests about current signal state and lane capacities,
    and coordinates with neighbouring TLs.
    """

    def __init__(self, jid: str, password: str, road_id: int, network: "Network"):
        super().__init__(jid, password)
        self.road_id = road_id
        self.network = network
        self.yellow_light_time = 0.5

        self.intersection = None
        self.road_capacity = None
        self.n_lanes = None
        self.TL_group = []

        # Dynamic attributes
        self.lane_capacities = None
        self.cars_in_lane = None
        self.signal = "red"
        self.tolerance = 0
        self.received_proposals = {}
        self.best_proposal = None

    def init_properties(self):
        self.intersection = self.network.roads[self.road_id].endNode.ID
        self.road_capacity = self.network.roads[self.road_id].length
        self.n_lanes = self.network.roads[self.road_id].laneCount

        self.lane_capacities = {i: 0 for i in range(self.n_lanes)} 
        self.cars_in_lane = {i: deque() for i in range(self.n_lanes)}

        for road in self.network.nodes[self.intersection].inRoads:
            if road.ID != self.road_id:
                tl_responsible = f"tl{road.ID}@localhost"
                self.TL_group.append(tl_responsible)

    def dijkstra(self, start_road_ID: int, start_lane_UID: int, end_node_ID: int):
        """
        Dijkstra shortest-path on lanes.
        start_road: the road where the car currently is
        start_lane: the lane the car is currently in
        end_node: the target node
        Returns deque of (road, lane)
        """

        routes = []
        startRoad = self.network.roads[start_road_ID]
        startLane = self.network.lanes[start_lane_UID]
        
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

    def generate_routes(self, start_road_ID, start_lane_ID, end_node_ID, K):
        """
        Computes the K shortest loopless paths using Yen’s algorithm.
        Uses self.generate_route() which returns (path, cost).
        Each path is a deque of (roadID, laneID).
        """

        start_lane_UID = self.network.roads[start_road_ID].lanes[start_lane_ID].UID

        # ---- First shortest path (Dijkstra) ----
        first_path, first_cost = self.dijkstra(start_road_ID, start_lane_UID, end_node_ID)
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
                        lane = self.network.lanes[banned_lane_UID]
                        if lane not in removed_roads:             
                            removed_roads[lane] = lane.outFlowLanes
                        lane.outFlowLanes = []       

                # ---- Compute spur path ----
                spur_path, spur_cost = self.dijkstra(spur_road_ID, spur_lane_UID, end_node_ID)

                # ---- If spur path exists, build full path candidate ----
                if spur_path:
                    spur_list = list(spur_path)
                    if spur_list and root_path:
                        # drop the first lane of the spur, because it repeats root's last lane
                        spur_list = spur_list[1:]

                    new_full_path = deque(root_path + spur_list)
                    new_full_cost = sum(self.network.lanes[laneUID].length for _, laneUID in new_full_path) - self.network.roads[new_full_path[0][0]].length

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
                A[j][0][i][1] = self.network.lanes[A[j][0][i][1]].ID
                A[j][0][i] = tuple(A[j][0][i])
            A[j][0].popleft()
            A[j] = tuple(A[j])
        return A

    async def setup(self):
        print(f"[{self.jid}] Traffic light for road {self.road_id} started.")
        self.init_properties()
        self.add_behaviour(self.ReceiveMessageBehaviour())

    # -------------
    # Behaviours 
    # -------------
    class ReceiveMessageBehaviour(CyclicBehaviour):
        async def run(self):
            tl = self.agent
            msg = await self.receive(timeout=5)

            if not msg:
                return

            performative = msg.metadata.get("performative")
            onthology = msg.metadata.get("onthology")
            action = msg.metadata.get("action")
            sender = str(msg.sender)

            if performative == "request" and onthology == "traffic-management" and action == "get-signal":
                message = tl.signal
                await self.send_message(to=sender, performative="inform", onthology="traffic-management", action="send-signal", body=message)
                
            elif performative == "request" and onthology == "traffic-management" and action == "get-lane-capacity":
                data = json.loads(msg.body)
                road = data["road"]
                lane = data["lane"]
                current_capacity = tl.lane_capacities[lane]
                total_capacity = tl.road_capacity

                message = json.dumps({"road": data["road"], "lane": data["lane"], "current_capacity": current_capacity, "total_capacity": total_capacity})
                await self.send_message(to=sender, performative="inform", onthology="traffic-management", action="send-lane-capacity", body=message)

            elif performative == "inform" and onthology == "traffic-management" and action == "update-lane":
                data = json.loads(msg.body)
                car = data["car"]
                affected_lane = data["lane"]
                event = data["event"]

                if event == "+":
                    tl.lane_capacities[affected_lane] += 1
                    tl.cars_in_lane[affected_lane].append(car)
                
                elif event == "-":
                    tl.lane_capacities[affected_lane] -= 1
                    tl.cars_in_lane[affected_lane].popleft()
                    await self.inform_car_has_left(affected_lane)

            elif performative == "cfp" and onthology == "traffic-management" and action == "asking-for-proposal":
                n_cars_in_road = 0
                for lane, n_cars_in_lane in tl.lane_capacities.items():
                    n_cars_in_road += n_cars_in_lane 

                message = json.dumps({"tolerance": tl.tolerance, "n_cars_in_road": n_cars_in_road})
                await self.send_message(to=sender, performative="propose", onthology="traffic-management", action="send-proposal", body=message)

            elif performative == "propose" and onthology == "traffic-management" and action == "send-proposal":
                proposal_data = json.loads(msg.body)
                tl.received_proposals[sender] = proposal_data

            elif performative == "accept-proposal" and onthology == "traffic-management" and action == "accept-proposal":
                tl.add_behaviour(tl.SignalTimeBehaviour())

            elif performative == "reject-proposal" and onthology == "traffic-management" and action == "reject-proposal":
                return

            elif performative == "inform" and onthology == "traffic-management" and action == "car-waiting":
                tl.tolerance += 1

            elif performative == "request" and onthology == "traffic-management" and action == "get-routes":
                data = json.loads(msg.body)
                road_id, lane_id, end_node_id = data["current_road"], data["current_lane"], data["end_node"]

                if tl.network.roads[data["current_road"]].endNode.ID == data["end_node"]:
                    await self.send_message(to=sender, performative="inform", onthology="traffic-management", action="reached-destination", body="You've arrived at your destination")
                    return


                routes = tl.generate_routes(road_id, lane_id, end_node_id, 5)
                for i in range(len(routes)):
                    routes[i] = list(routes[i])
                    routes[i][0] = list(routes[i][0])
                    routes[i] = tuple(routes[i])

                message = json.dumps(routes)
                await self.send_message(to=sender, performative="inform", onthology="traffic-management", action="send-routes", body=message)

        async def send_message(self, to, performative, onthology, action, body):
            tl = self.agent 

            msg = Message(to=to, metadata={"performative": performative, "onthology": onthology, "action": action}, body=body)
            await self.send(msg)
            
            #print(f"[{tl.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def inform_car_has_left(self, affected_lane):
            tl = self.agent

            for car in tl.cars_in_lane[affected_lane]:
                await self.send_message(to=car, performative="inform", onthology="traffic-management", action="lane-cleared", body="")

    class SignalTimeBehaviour(OneShotBehaviour):
        async def run(self):
            tl = self.agent

            # Turn light to green
            green_light_time = self.set_green_light_time()
            tl.signal = "green"
            await asyncio.sleep(green_light_time)

            for neighbouring_tl in tl.TL_group:
                await self.send_message(to=neighbouring_tl, performative="cfp", onthology="traffic-management", action="asking-for-proposal", body="")

            # Turn light to yellow
            tl.signal = "yellow"
            await asyncio.sleep(tl.yellow_light_time)

            best_proposal = None

            if len(tl.received_proposals) > 0:
                best_proposal = self.choose_best_proposal()

            # Turn light to red
            tl.signal = "red"

            for neighbouring_tl in tl.TL_group:
                if neighbouring_tl == best_proposal[0]:
                    winner = best_proposal[0]
                    await self.send_message(to=neighbouring_tl, performative="accept-proposal", onthology="traffic-management", action="accept-proposal", body="")
            
                else:
                    await self.send_message(to=neighbouring_tl, performative="reject-proposal", onthology="traffic-management", action="reject-proposal", body="")
            
            tl.received_proposals = {}
            tl.best_proposal = None
            tl.tolerance = 0

        async def send_message(self, to, performative, onthology, action, body):
            tl = self.agent

            msg = Message(to=to, metadata={"performative": performative, "onthology": onthology, "action": action}, body=body)
            await self.send(msg)

            #print(f"[{tl.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        def set_green_light_time(self):
            tl = self.agent

            minimum_time = 1
            maximum_time = 2
            green_light_time = round(minimum_time + (maximum_time - minimum_time) * (sum(tl.lane_capacities.values())/(tl.n_lanes * tl.road_capacity)), 1)

            return green_light_time

        def choose_best_proposal(self):
            tl = self.agent
            
            # Sorts senders by the tolerance score in their proposal in descending order
            sorted_proposals = sorted(tl.received_proposals.items(), key=lambda item: item[1]["tolerance"], reverse=True)
            best_proposal = sorted_proposals[0]

            return best_proposal


