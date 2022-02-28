import logging
import sys

import droneapp.models.drone_manager

logging.basicConfig(level=logging.INFO, stream=sys.stdout)

if __name__ == '__main__':
    drone = droneapp.models.drone_manager.DroneManager()

    while True:
        args = str(input())
        if args == "q":
            break

        drone.send_command(args)
