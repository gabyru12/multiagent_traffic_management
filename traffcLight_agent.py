from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.template import Template
from spade.message import Message


class TrafficLightAgent(Agent):
    """
    Each TrafficLightAgent controls a road segment and manages lane capacities.
    It responds to cars' requests about current signal state and lane capacities,
    and coordinates with neighbouring TLs.
    """

    def __init__(self, jid, password, road_id, intersection, road_capacity, n_lanes):
        super().__init__(jid, password)
        self.road_id = road_id
        self.intersection = intersection
        self.road_capacity = road_capacity
        self.n_lanes = n_lanes
        self.lane_capacities = [0 for _ in range(n_lanes)]  # cars currently in lane
        self.signal = 0  # 0=red, 1=green (for simplicity)

    async def setup(self):
        #print(f"[{self.jid}] Traffic light for road {self.road_id} started.")

        # --- 1. Respond to cars asking about signal & current lane capacity
        t1 = Template(metadata={"type": "request_signalAndCapacity"})
        self.add_behaviour(self.SignalResponder(), t1)

        # --- 2. Handle cars requesting the next lane capacity (indirect request)
        t2 = Template(metadata={"type": "request_nextLaneCapacity"})
        self.add_behaviour(self.NextLaneCapacityRedirector(), t2)

        # --- 3. Respond to redirected requests from another TL
        t3 = Template(metadata={"type": "redirectedRequest_nextLaneCapacity"})
        self.add_behaviour(self.NextLaneCapacityResponder(), t3)

        # --- 4. Pass info back to the car via the original TL
        t4 = Template(metadata={"type": "info_nextLaneCapacity"})
        self.add_behaviour(self.NextLaneInfoBackToCar(), t4)

        # --- 5. Update lane capacity when a car spawns
        t5 = Template(metadata={"type": "update_laneCapacitiesOnSpawn"})
        self.add_behaviour(self.UpdateLaneCapacitiesOnCarSpawn(), t5)

        # --- 6. Handle capacity updates when cars change roads
        t6 = Template(metadata={"type": "update_laneCapacitiesAfterChaningRoads_fromCar"})
        self.add_behaviour(self.UpdateLaneCapacitiesSignalFromCar(), t6)

        t7 = Template(metadata={"type": "update_laneCapacitiesAfterChaningRoads_fromTL"})
        self.add_behaviour(self.UpdateLaneCapacitiesSignalFromTL(), t7)

    # ---------------------------------------------------------------
    # Behaviour 1: Respond to car asking about signal & lane capacity
    # ---------------------------------------------------------------
    class SignalResponder(CyclicBehaviour):
        """Send current signal and lane capacity back to the requesting car."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                lane = int(msg.body)
            except ValueError:
                return

            if lane < 0 or lane >= self.agent.n_lanes:
                #print(f"[{self.agent.jid}] Invalid lane ID {lane}")
                return

            reply = msg.make_reply()
            reply.set_metadata("type", "info_signalCapacity")
            reply.body = f"{self.agent.signal} {self.agent.lane_capacities[lane]} {self.agent.road_capacity}"
            await self.send(reply)

    # ---------------------------------------------------------------
    # Behaviour 2: Redirect car's next-lane request to next TL
    # ---------------------------------------------------------------
    class NextLaneCapacityRedirector(CyclicBehaviour):
        """Receive car's request and forward it to the TL of the next road."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                next_edge, next_lane = msg.body.split()
            except ValueError:
                return

            carJID = str(msg.sender)
            next_tl_jid = f"tl{next_edge}@localhost"

            redirect_msg = Message(to=next_tl_jid)
            redirect_msg.set_metadata("type", "redirectedRequest_nextLaneCapacity")
            redirect_msg.body = f"{next_lane} {carJID}"
            await self.send(redirect_msg)

    # ---------------------------------------------------------------
    # Behaviour 3: Respond to redirected request from another TL
    # ---------------------------------------------------------------
    class NextLaneCapacityResponder(CyclicBehaviour):
        """Respond to TL asking for a lane’s current and total capacity."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                lane, carJID = msg.body.split()
                lane = int(lane)
            except ValueError:
                return

            if lane < 0 or lane >= self.agent.n_lanes:
                return

            reply = msg.make_reply()
            reply.set_metadata("type", "info_nextLaneCapacity")
            reply.body = f"{self.agent.lane_capacities[lane]} {self.agent.road_capacity} {carJID}"
            await self.send(reply)

    # ---------------------------------------------------------------
    # Behaviour 4: Pass next-lane info back to the car
    # ---------------------------------------------------------------
    class NextLaneInfoBackToCar(CyclicBehaviour):
        """Forward next-lane capacity info to the requesting car."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                nextLaneCapacity, nextLaneTotalCapacity, carJID = msg.body.split()
            except ValueError:
                return

            msgToCar = Message(to=carJID)
            msgToCar.set_metadata("type", "info_nextLaneCapacity")
            msgToCar.body = f"{nextLaneCapacity} {nextLaneTotalCapacity}"
            await self.send(msgToCar)

    # ---------------------------------------------------------------
    # Behaviour 5: Increment lane capacity when a car spawns
    # ---------------------------------------------------------------
    class UpdateLaneCapacitiesOnCarSpawn(CyclicBehaviour):
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                laneToUpdate = int(msg.body)
            except ValueError:
                return

            if 0 <= laneToUpdate < self.agent.n_lanes:
                self.agent.lane_capacities[laneToUpdate] += 1
                # print(f"[{self.agent.jid}] Car spawned: lane {laneToUpdate} now has {self.agent.lane_capacities[laneToUpdate]} cars")

    # ---------------------------------------------------------------
    # Behaviour 6: Update capacity when car leaves to another road
    # ---------------------------------------------------------------
    class UpdateLaneCapacitiesSignalFromCar(CyclicBehaviour):
        """Decrease current lane and notify next TL to increase its lane."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                laneFrom, nextRoad, nextLane = msg.body.split()
                laneFrom = int(laneFrom)
                nextLane = int(nextLane)
            except ValueError:
                return

            # Car left: free one spot in this TL’s lane
            if 0 <= laneFrom < self.agent.n_lanes:
                self.agent.lane_capacities[laneFrom] = max(0, self.agent.lane_capacities[laneFrom] - 1)

            # Notify next TL
            next_tl_jid = f"tl{nextRoad}@localhost"
            msgToNext = Message(to=next_tl_jid)
            msgToNext.set_metadata("type", "update_laneCapacitiesAfterChaningRoads_fromTL")
            msgToNext.body = str(nextLane)
            await self.send(msgToNext)

    # ---------------------------------------------------------------
    # Behaviour 7: Update capacity when notified by previous TL
    # ---------------------------------------------------------------
    class UpdateLaneCapacitiesSignalFromTL(CyclicBehaviour):
        """Increase lane capacity when another TL tells us a car arrived."""
        async def run(self):
            msg = await self.receive(timeout=1)
            if not msg:
                return

            try:
                laneToUpdate = int(msg.body)
            except ValueError:
                return

            if 0 <= laneToUpdate < self.agent.n_lanes:
                self.agent.lane_capacities[laneToUpdate] += 1
                # print(f"[{self.agent.jid}] Car arrived: lane {laneToUpdate} now has {self.agent.lane_capacities[laneToUpdate]} cars")
