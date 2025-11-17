from spade.agent import Agent
from spade.behaviour import PeriodicBehaviour, OneShotBehaviour, CyclicBehaviour
from spade.template import Template
from spade.message import Message

from collections import deque
import json
import asyncio

class CarAgent(Agent):
    def __init__(self, jid, password, starting_road, end_node, remove_callback=None):
        super().__init__(jid, password)
        self.starting_road = starting_road
        self.starting_lane = 0
        self.end_node = end_node
        self.remove_callback = remove_callback
        self.fuel_consumption = {"moving": 1,"idle": 0.2}
        self.travel_weight = 0.5
        self.capacity_weight = 0.5
        
        # dynamic state
        self.wasted_fuel = 0
        self.current_road = self.starting_road
        self.current_lane = self.starting_lane
        self.current_tl_jid = f"tl{self.current_road}@localhost"
        self.current_tl_signal = None
        self.current_lane_capacity = None
        self.total_lane_capacity = None

        self.position = 1
        self.can_still_move_to = None

        self.possible_routes = None
        self.n_received_capacities = 0
        self.received_capacities = {}

    async def setup(self):
        print(f"[{self.jid}] Car starting...")
        self.add_behaviour(self.ReceiveMessageBehaviour())

        self.add_behaviour(self.SpawnCar())

    class SpawnCar(OneShotBehaviour):
        """Set initial attributes for car"""
        async def run(self):
            car = self.agent

            await self.request_lane_capacity()
            await self.signal_to_update_lane_capacity()

            car.add_behaviour(car.DriveController(period=0.2))

        async def send_message(self, to: str, performative: str, onthology: str, action: str, body: str):
            car = self.agent

            msg = Message(to=to, metadata={"performative": performative, "onthology": onthology, "action": action}, body=body)
            await self.send(msg)
            #print(f"[{car.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def request_lane_capacity(self):
            car = self.agent

            message = json.dumps({"road": car.current_road, "lane": car.current_lane})
            await self.send_message(to=car.current_tl_jid, performative="request", onthology="traffic-management", action="get-lane-capacity", body=message)

            while (car.current_road, car.current_lane) not in car.received_capacities:
                await asyncio.sleep(0.01)
            
            car.current_lane_capacity = car.received_capacities[(car.current_road, car.current_lane)][0]
            car.total_lane_capacity = car.received_capacities[(car.current_road, car.current_lane)][1]
            car.can_still_move_to = car.total_lane_capacity - car.current_lane_capacity

            car.received_capacities = {}
            car.n_received_capacities = 0

        async def signal_to_update_lane_capacity(self):
            car = self.agent

            message = json.dumps({"car": str(car.jid), "lane": car.current_lane, "event": "+"})
            await self.send_message(to=car.current_tl_jid, performative="inform", onthology="traffic-management", action="update-lane", body=message)

    class DriveController(PeriodicBehaviour):
        """Main driving logic"""
        async def run(self):
            car = self.agent

            if car.position < car.can_still_move_to:
                car.position += 1
                return

            elif car.position == car.total_lane_capacity:
                await self.request_signal()

                if car.current_tl_signal == "green":
                    await self.request_new_routes()
                    await self.request_capacities()
                    entering_road, entering_lane = self.choose_route()

                    if entering_road != None and entering_lane != None:
                        leaving_road, leaving_lane = car.current_road, car.current_lane
                        await self.update_car_attributes(entering_road, entering_lane)
                        await self.signal_to_update_lane_capacities(leaving_road, entering_road, leaving_lane, entering_lane)
                        car.current_tl_signal = None
                        car.possible_routes = None
                        car.n_received_capacities = 0
                        car.received_capacities = {}
                        return
                    else:
                        return

                elif car.current_tl_signal == "red":
                    await self.send_message(to=car.current_tl_jid, performative="inform", onthology="traffic-management", action="car-waiting", body="")
                    car.current_tl_signal = None
                    return

            elif car.position == car.can_still_move_to and car.position >= car.total_lane_capacity - 10:
                await self.request_signal()
                if car.current_tl_signal == "red":
                    await self.send_message(to=car.current_tl_jid, performative="inform", onthology="traffic-management", action="car-waiting", body="")
                    car.current_tl_signal = None
                return

        async def send_message(self, to: str, performative: str, onthology: str, action: str, body: str):
            car = self.agent

            msg = Message(to=to, metadata={"performative": performative, "onthology": onthology, "action": action}, body=body)
            await self.send(msg)
            #print(f"[{car.jid}] → Sent to {to}: ({performative}, {action}) {body}")

        async def request_signal(self):
            car = self.agent

            # Request traffic light for current signal
            await self.send_message(to=car.current_tl_jid, performative="request", onthology="traffic-management", action="get-signal", body="")

            # Wait for message to arrive
            while car.current_tl_signal == None:
                await asyncio.sleep(0.01)

        async def request_new_routes(self):
            car = self.agent

            # Request routing agent for possible routes to destination
            message = json.dumps({"current_road": car.current_road, "current_lane": car.current_lane, "end_node": car.end_node})
            await self.send_message(to=car.current_tl_jid, performative="request", onthology="traffic-management", action="get-routes", body=message)

            # Wait for possible routes to arrive
            while car.possible_routes == None:
                await asyncio.sleep(0.01)

        async def request_capacities(self):
            car = self.agent

            next_road_lanes = set()
            for route in car.possible_routes:
                path = route[0]
                cost = route[1]
                for road_lane in path:
                    if road_lane not in next_road_lanes:
                        next_road_lanes.add(road_lane)

            length_next_road_lanes = len(next_road_lanes)

            for next_road_lane in next_road_lanes:
                road = next_road_lane[0]
                lane = next_road_lane[1]
                send_to = f"tl{road}@localhost"
                message = json.dumps({"road": road, "lane": lane})
                await self.send_message(to=send_to, performative="request", onthology="traffic-management", action="get-lane-capacity", body=message)

            while car.n_received_capacities != length_next_road_lanes:
                await asyncio.sleep(0.01)

        def choose_route(self):
            car = self.agent

            for route in car.possible_routes:
                temp_capacity = 0
                path = route[0]
                for road_lane in path:
                    temp_capacity += car.received_capacities[road_lane][0]
                route.append(temp_capacity)

            for route in car.possible_routes:
                travel_cost = route[1]
                capacity_cost = route[2]
                total_cost = (car.travel_weight*travel_cost) + (car.capacity_weight*capacity_cost)
                route.append(total_cost)

            car.possible_routes.sort(key=lambda route: route[3])

            for route in car.possible_routes:
                path = route[0]
                first_road_lane = path[0]
                first_lane_current_capacity =  car.received_capacities[first_road_lane][0]
                first_lane_total_capacity =  car.received_capacities[first_road_lane][1]
                if first_lane_current_capacity < first_lane_total_capacity:
                    return first_road_lane

            return (None, None)

        async def update_car_attributes(self, entering_road, entering_lane):
            car = self.agent

            car.current_road, car.current_lane = entering_road, entering_lane
            car.current_tl_jid = f"tl{car.current_road}@localhost"
            car.position = 1
            currentCapacity, totalCapacity = car.received_capacities[(car.current_road, car.current_lane)]
            car.current_lane_capacity, car.total_lane_capacity = currentCapacity, totalCapacity
            car.can_still_move_to = car.total_lane_capacity - car.current_lane_capacity

        async def signal_to_update_lane_capacities(self, leaving_road, entering_road, leaving_lane, entering_lane):
            car = self.agent

            # Request traffic light to update lane capacities for the current road and previous road
            previous_road_tl_jid = f"tl{leaving_road}@localhost"
            message = json.dumps({"car": str(car.jid), "lane": leaving_lane, "event": "-"})
            await self.send_message(to=previous_road_tl_jid, performative="inform", onthology="traffic-management", action="update-lane", body=message)

            next_road_tl_jid = f"tl{entering_road}@localhost"
            message = json.dumps({"car": str(car.jid), "lane": entering_lane, "event": "+"})
            await self.send_message(to=next_road_tl_jid, performative="inform", onthology="traffic-management", action="update-lane", body=message)
       
    class ReceiveMessageBehaviour(CyclicBehaviour):
        async def run(self):
            car = self.agent
            msg = await self.receive(timeout=5)

            if not msg:
                return

            performative = msg.metadata.get("performative")
            onthology = msg.metadata.get("onthology")
            action = msg.metadata.get("action")
            sender = str(msg.sender)

            if performative == "inform" and onthology == "traffic-management" and action == "lane-cleared":
                car.can_still_move_to = min(car.total_lane_capacity, car.can_still_move_to + 1)

            elif performative == "inform" and onthology == "traffic-management" and action == "send-lane-capacity":
                data = json.loads(msg.body)
                road = data["road"]
                lane = data["lane"]
                current_capacity = data["current_capacity"]
                total_capacity = data["total_capacity"]
                car.received_capacities[(road,lane)] = (current_capacity, total_capacity)
                car.n_received_capacities += 1

            elif performative == "inform" and onthology == "traffic-management" and action == "send-signal":
                signal = msg.body
                car.current_tl_signal = signal

            elif performative == "inform" and onthology == "traffic-management" and action == "send-routes":
                routes = json.loads(msg.body)

                for route in routes:
                    route[0] = deque(tuple(x) for x in route[0])
                    route = tuple(route)

                car.possible_routes = routes

            elif performative == "inform" and onthology == "traffic-management" and action == "reached-destination":
                print(f"[{car.jid}] has reached it's destination")

                message = json.dumps({"car": str(car.jid), "lane": car.current_lane, "event": "-"})
                await self.send_message(to=car.current_tl_jid, performative="inform", onthology="traffic-management", action="update-lane", body=message)

                if car.remove_callback:
                    car.remove_callback()

                await car.stop()
