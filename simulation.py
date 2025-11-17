import asyncio
import random
import traceback
from spade import run

from network import Network, Node, Road, Lane
from car_agent import CarAgent
from trafficLight_agent import TrafficLightAgent

async def simulation():
    # -------------------------------------------------------
    # 1. Load your network
    # -------------------------------------------------------
    network = Network()
    network.create_network(2, 2, 25)

    # -------------------------------------------------------
    # 2. Start all traffic lights
    # -------------------------------------------------------
    traffic_lights = {}
    traffic_light_groups = {}
    for road_id, road in network.roads.items():
        tl_jid = f"tl{road_id}@localhost"
        tl_pwd = "tl"

        tl_agent = TrafficLightAgent(
            jid=tl_jid,
            password=tl_pwd,
            road_id=road_id,
            network=network,
        )

        await tl_agent.start()
        traffic_lights[road_id] = tl_agent
        traffic_light_groups.setdefault(road.endNode, []).append(tl_agent)

        print(f"[SIM] Started TrafficLightAgent for road {road_id}")

    # ------------------------------------------------------------------------------------------
    # 2. Choose a traffic light for each intersection to have green light from start
    # ------------------------------------------------------------------------------------------
    for intersection, tl_group in traffic_light_groups.items():
        if len(tl_group) == 1:
            tl_group[0].signal = "green"
            continue
        chosen_tl = random.choice(tl_group)
        chosen_tl.add_behaviour(chosen_tl.SignalTimeBehaviour())

    # -------------------------------------------------------
    # 3. Spawn cars every 0.5s
    # -------------------------------------------------------
    car_counter = 0

    print("\n[SIM] Starting main loop… Cars spawn every 0.5 seconds.\n")
    active_cars = set()

    while True:
        temp = list(network.spawn_nodes)
        starting_node = random.choice(temp)
        starting_road = starting_node.outRoads[0].ID
        temp.remove(starting_node)
        end_node = random.choice(temp).ID

        car_jid = f"rivenisbadchamp{car_counter}@localhost"
        car_pwd = "car"

        print(f"[SIM] Spawning {car_jid} at road {starting_road} → node {end_node}")

        car_agent = CarAgent(
            jid=car_jid,
            password=car_pwd,
            starting_road=starting_road,
            end_node=end_node,
            remove_callback=lambda jid=car_jid: active_cars.remove(jid)
        )
        active_cars.add(car_agent)

        await car_agent.start()

        car_counter += 1

        await asyncio.sleep(0.5)  # spawn interval


# Run simulation
if __name__ == "__main__":
    run(simulation())

