from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.message import Message
from spade.template import Template
import networkx as nx
import json


class RoutingAgent(Agent):
    """
    Handles routing and rerouting for cars.
    It uses a graph (e.g., a road network) to compute routes between nodes.
    """

    def __init__(self, jid, password, graph: "Graph"):
        super().__init__(jid, password)
        self.graph = graph

    async def setup(self):
        print(f"[{self.jid}] Routing agent ready.")

        # --- 1. Car requesting initial spawn route ---
        t1 = Template(metadata={"type": "request_spawnNewRoute"})
        self.add_behaviour(self.SpawnRouteResponder(), t1)

        # --- 2. Car requesting reroute due to blockage ---
        t2 = Template(metadata={"type": "request_newRoute"})
        self.add_behaviour(self.ReRouteResponder(), t2)

    # ---------------------------------------------------------------
    # Behaviour 1: Respond to new cars asking for an initial route
    # ---------------------------------------------------------------
    class SpawnRouteResponder(CyclicBehaviour):
        """Respond to initial route request when a car spawns."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            start_node, end_node = msg.body.split()
            start_edge = self.agent.graph.nodes[start_node].outEdges[0].ID 
            start_lane = self.agent.graph.nodes[start_node].outEdges[0].lanes[0].ID 

            # call your Dijkstra → returns list of (edge, lane)
            route = self.agent.generate_route(start_edge, start_lane, end_node)
            reply = msg.make_reply()
            reply.set_metadata("type", "info_route")
            reply.body = json.dumps(list(route))   # serialize as list
            await self.send(reply)

    # ---------------------------------------------------------------
    # Behaviour 2: Handle reroute requests from cars
    # ---------------------------------------------------------------
    class ReRouteResponder(CyclicBehaviour):
        """When a car gets stuck, find new alternative routes."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            start_edge_ID, start_lane_ID, end_node_ID = msg.body.split()
            routes = []
            # call your Dijkstra → returns list of (edge, lane)
            for outFlowLane in self.agent.graph.edges[start_edge_ID].lanes[start_lane_ID].outFlowLanes:
                routes.append(self.agent.generate_route(outFlowLane.edge.ID, outFlowLane.ID, end_node_ID))
                reply = msg.make_reply()
                reply.set_metadata("type", "info_possibleRoutes")
                reply.body = json.dumps(list(routes))   # serialize as list
                await self.send(reply)
