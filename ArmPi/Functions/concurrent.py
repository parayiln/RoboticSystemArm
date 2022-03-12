# week 5

import sys
from sensing_process import Sensing
import rossros as ros
from sensing_control_ultasonic import SensingUltra, InterpretationUltra, ControllerUltra



if __name__=='__main__':
    Sensor = Sensing()
    Control = Controller()


    sense_bus=ros.Bus()
    process_bus=ros.Bus()
    termination_bus = ros.Bus()

    delay=.01

    sense_p = ros.Producer(Sensor.sense, sense_bus, delay, termination_bus, 'sense line')
    process_cp = ros.ConsumerProducer(Sense.process, sense_bus, process_bus, delay, termination_bus, 'process line')
    control_c= ros.Consumer(Control.move, process_bus, delay, termination_bus, 'control line follower')


    time = ros.Timer(termination_bus, 5, 0.01, term_bus, "Timer")

    try:
        ros.runConcurrently([sense_p, process_cp, control_c, time.timer()])
    except:
        print("Someting is wrong, exiting code")
