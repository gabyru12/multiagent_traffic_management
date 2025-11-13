from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, OneShotBehaviour
from spade.template import Template
from spade.message import Message
from collections import deque
import json

class CarAgent(Agent):
    def __init__(self, jid, password, start_node, end_node):
        super().__init__(jid, password)
        self.start_node = start_node
        self.end_node = end_node
        self.routing_agent_JID = "routingAgent@localhost"
        
        # dynamic state
        self.max_fuel = 50
        self.fuel = self.max_fuel
        self.fuel_consumption = self.fuel_consumption
        self.route = None
        self.possibleRoutes = None
        self.current_road = None
        self.current_lane = None
        self.next_road = None
        self.next_lane = None
        self.current_tl_jid = None
        self.current_tl_signal = None
        self.total_lane_capacity = None
        self.current_lane_capacity = None
        self.position = None
        self.canStillMoveTo = None

    async def setup(self):
        #print(f"[{self.jid}] Car starting...")

        # first, request initial route
        spawn_bh = self.SpawnCar()
        self.add_behaviour(spawn_bh)
        await spawn_bh.join()

        # start the unified controller
        self.add_behaviour(self.DriveController(period=0.1))

    class SpawnCar(OneShotBehaviour):
        """Car requests a route from its routing agent based on its starting and ending node."""
        async def run(self):
            msg = Message(to=self.agent.routing_agent_JID)
            msg.set_metadata("type", "request_spawnNewRoute")
            msg.body = f"{self.agent.start_node} {self.agent.end_node}"
            await self.send(msg)
            
            response = await self.receive(timeout=5)
            if not response:
                print(f"[{self.agent.jid}] No route received on spawn.")
                return
            
            route = json.loads(response.body)
            self.agent.route = deque(route[0])
            self.agent.current_road, self.agent.current_lane = self.agent.route.popleft()
            self.agent.position = 1
            self.agent.current_tl_jid = f"tl{self.agent.current_road}@localhost"
            self.agent.canStillMoveTo = 1

            # Notify TL that car spawned in lane
            msg = Message(to=self.agent.current_tl_jid)
            msg.set_metadata("type", "update_laneCapacitiesOnSpawn")
            msg.body = str(self.agent.current_lane)
            await self.send(msg)
            #print(f"[{self.agent.jid}] Spawned at road {self.agent.current_road}, lane {self.agent.current_lane}")

    class DriveController(PeriodicBehaviour):
        """Main driving logic — handles movement, communication, and rerouting."""
        async def run(self):
            # 1. Update traffic light signal and lane capacity
            await self.update_signal_and_capacity()
            if self.agent.current_tl_signal is None:
                return  # no info yet, skip tick

            # 2. Move the car based on the current signal and capacity
            await self.move_once()

        async def update_signal_and_capacity(self):
            msg = Message(to=self.agent.current_tl_jid)
            msg.set_metadata("type", "request_signalAndCapacity")
            msg.body = str(self.agent.current_lane)
            await self.send(msg)

            response = await self.receive(timeout=1)
            if response:
                self.agent.current_tl_signal, self.agent.current_lane_capacity, self.agent.total_lane_capacity = map(int, response.body.split())
                # Debug print
                # print(f"[{self.agent.jid}] Signal={self.agent.current_tl_signal}, Cap={self.agent.current_lane_capacity}/{self.agent.total_lane_capacity}")

        async def move_once(self):
            # Red light
            if self.agent.current_tl_signal == 0:
                if self.agent.position + 1 <= self.agent.total_lane_capacity and self.agent.position + 1 <= self.agent.canStillMoveTo:
                    self.agent.position += 1

            # Green light
            elif self.agent.current_tl_signal == 1:
                if self.agent.position + 1 <= self.agent.total_lane_capacity:
                    self.agent.position += 1
                    self.agent.canStillMoveTo += 1
                else:
                    await self.try_change_road()

        async def try_change_road(self):
            """Try to move to the next road, or reroute if blocked."""
            if not self.agent.route:
                print(f"[{self.agent.jid}] No route left — destination reached?")
                return

            self.agent.next_road, self.agent.next_lane = self.agent.route[0]

            # Ask next TL for capacity
            msg = Message(to=self.agent.current_tl_jid)
            msg.set_metadata("type", "request_nextLaneCapacity")
            msg.body = f"{self.agent.next_road} {self.agent.next_lane}"
            await self.send(msg)

            reply = await self.receive(timeout=2)
            if not reply:
                print(f"[{self.agent.jid}] No next lane capacity info.")
                return

            nextLaneCapacity, nextLaneTotalCapacity = map(int, reply.body.split())

            if nextLaneCapacity < nextLaneTotalCapacity:
                await self.change_road(nextLaneCapacity, nextLaneTotalCapacity)
            else:
                found_new_route = await self.find_new_route()
                if found_new_route:
                    nextLaneCapacity, nextLaneTotalCapacity = found_new_route
                    self.change_road(nextLaneCapacity, nextLaneTotalCapacity)
                else:
                    return

        async def change_road(self, nextLaneCapacity, nextLaneTotalCapacity):
            """Perform the road change when space is available."""
            self.agent.position = 1
            self.agent.canStillMoveTo = nextLaneTotalCapacity - nextLaneCapacity
            self.agent.next_road, self.agent.next_lane = self.agent.route[0]

            # Notify TLs to update lane capacities
            msg = Message(to=self.agent.current_tl_jid)
            msg.set_metadata("type", "update_laneCapacitiesAfterChaningRoads_fromCar")
            msg.body = f"{self.agent.current_lane} {self.agent.next_road} {self.agent.next_lane}"
            await self.send(msg)

            self.agent.current_road, self.agent.current_lane = self.agent.route.popleft()
            self.agent.current_tl_jid = f"tl{self.agent.current_road}@localhost"

        async def find_new_route(self):
            """
            Ask routing agent for a new possible route.
            Returns: nextLaneCapacity and nextLaneTotalCapacity 
            """
            msg = Message(to=self.agent.routing_agent_JID)
            msg.set_metadata("type", "request_newRoute")
            msg.body = f"{self.agent.current_road} {self.agent.current_lane} {self.agent.end_node}"
            await self.send(msg)

            response = await self.receive(timeout=3)
            if not response:
                #print(f"[{self.agent.jid}] No reroute info received.")
                return

            routes_list = json.loads(response.body)
            possibleRoutes = [(deque(route[0]), route[1]) for route in routes_list]
            possibleRoutes = sorted(possibleRoutes, key=lambda x: (x[1] if x[1] is not None else float("+inf")))

            for route, _ in possibleRoutes:
                self.agent.next_road, self.agent.next_lane = route[0]
                next_cap = await self.request_next_lane_capacity()
                if next_cap and next_cap[0] < next_cap[1]:
                    self.agent.route = route
                    #print(f"[{self.agent.jid}] Found new route via road {self.agent.next_road}, lane {self.agent.next_lane}")
                    return next_cap
            return None

        async def request_next_lane_capacity(self):
            """Helper to query next lane capacity."""
            msg = Message(to=self.agent.current_tl_jid)
            msg.set_metadata("type", "request_nextLaneCapacity")
            msg.body = f"{self.agent.next_road} {self.agent.next_lane}"
            await self.send(msg)

            reply = await self.receive(timeout=1)
            if reply:
                return tuple(map(int, reply.body.split()))
            return None
