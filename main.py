import logging
import sys

import droneapp.models.drone_manager

logging.basicConfig(level=logging.INFO, stream=sys.stdout)

if __name__ == '__main__':
    drone = droneapp.models.drone_manager.DroneManager()

    #TODO
    #コンソール上からコマンドを入力し、ドローン操作したい

    drone.send_command('up')
