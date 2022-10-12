import numpy as np
import time

EXEC_TIME = 5
FREQ = 200
DT = 1/FREQ

if __name__ == "__main__":
    start = time.time()
    t = 0.0

    # Print "ROB311 @UM-ROBOTICS" for 5 seconds @200Hz

    while time.time()<(start+EXEC_TIME):
        goaltime = time.time()+DT
        while time.time()<goaltime:
            pass
        print("ROB311 @UM-ROBOTICS")