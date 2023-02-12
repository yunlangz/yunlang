import picar_4wd as fc
import time

def turn_left(power, seconds):
    fc.turn_left(power)
    time.sleep(seconds)
    fc.stop()
