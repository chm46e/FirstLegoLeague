# LEGO type:advanced slot:0 autostart

# Micropython: (built in) <https://lego.github.io/MINDSTORMS-Robot-Inventor-hub-API/index.html>
import hub
# Filesystem: <https://micropython-lego-ri5.readthedocs.io/en/latest/library/index.html>
import runtime
import util
import version
import math

from util import movement
from utime import sleep

## These two next are fail safes for competitions.
# Forcefully stop taking gyro values
BOMB_GYRO = False
# Forcefully stop taking color sensor values
BOMB_COLOR = False

# TODO: Change the wheel diameter according to your robot.
WHEEL_DIAMETER = 56 # mm
WHEEL_CIRROT = ((56 / 10) * math.pi) # 1 rot = x cm

CLOCKWISE = 1
COUNTER_CLOCKWISE = -1

EXIT_EXEC = False

# TODO: Change the port values according to your robot.
# This is a crucial step and cannot be avoided.
class PORT:
    RDM = "F" # Right Driving Motor
    LDM = "E" # Left Driving Motor
    RMM = "D" # Right Medium Motor
    LMM = "C" # Left Medium Motor
    RCS = "B" # Right Color Sensor
    LCS = "A" # Left Color Sensor

class API:
    def __init__(self, vm):
        self.shub = self.Hub(vm)
        self.vm = vm

        # TODO: If you'd like, you can tweak the PID values.
        # PID constants
        self.Kp = 0.6    # proportional weight
        self.Ki = 0 # integral weight
        self.Kd = 0.3    # derivative weight
        self.Kc = 0.03   # color reflectance weight

    class Hub:
        def __init__(self, vm):
            self.vm = vm
            self.timer = self.Timer()
            self.gyro = self.Gyro() # gyro sensor
            self.lCs = self.Color(PORT.LCS) # left Color sensor
            self.rCs = self.Color(PORT.RCS) # right Color sensor

            # TODO: If the motor is going the wrong way, change CLOCKWISE to COUNTER_CLOCKWISE
            self.lMm = self.Motor(PORT.LMM, CLOCKWISE, vm, self.timer) # left Medium motor
            self.rMm = self.Motor(PORT.RMM, CLOCKWISE, vm, self.timer) # right Medium motor
            self.bDm = self.MotorPair(PORT.LDM, PORT.RDM, vm) # both Driving motors
            self.lDm = self.Motor(PORT.LDM, COUNTER_CLOCKWISE, vm, self.timer) # left Driving motor
            self.rDm = self.Motor(PORT.RDM, CLOCKWISE, vm, self.timer) # right Driving motor

        def led(self, color):
            # 0.off 1.pink 2.violet 3.blue 4.turquoise
            # 5.lgreen 6.green 7.yellow 8.orange
            # 9.red 10.white >.dim white
            hub.led(color)

        async def beep(self, freq=100, time=500):
            # freq - Hz (100 - 10000)
            hub.sound.volume(2) # 1 -> 10
            await self.vm.sound.beep_async(freq, time)

        def write_info(self):
            print("Execute Main: Spike Hub "+version.__version__+" "+hub.__version__)
            print("Capacity "+str(hub.battery.capacity_left())+"%, Voltage "+str(hub.battery.voltage())+"mV, Charger "+str(hub.battery.charger_detect()))
            print("Startup Performance "+str(self.timer.get())+"ms")

        def is_button(self, side):
            if side == "left":
                return hub.button.left.is_pressed()
            elif side == "right":
                return hub.button.right.is_pressed()
            elif side == "center":
                return hub.button.center.is_pressed()
            return False
        
        # make sure len < 10 or otherwise it will crash and burn
        def display(self, output: int):
            font = ["0999009090090900909009990",
                    "0090009900009000090009990",
                    "0999000090099900900009990",
                    "0999000090099900009009990",
                    "0909009090099900009000090",
                    "0999009000099900009009990",
                    "0999009000099900909009990",
                    "0999000090009000900009000",
                    "0999009090099900909009990",
                    "0999009090099900009009990"]
            hub.display.align(hub.FRONT)
            self.vm.system.display.show( 
                hub.Image( 
                    util.scratch.convert_image( 
                        font[output],
                        self.vm.store.display_brightness())), 
                clear=False)

        # ONLY medium motors drive with this class (see motor_stop -> hold position)
        # Driving motors are allowed to only access encoder methods
        class Motor:
            def __init__(self, port, direction, vm, timer):
                self.port = port
                self.direction = direction
                self.vm = vm
                self.encoder = 0
                self.timer = timer

            def settings(self):
                # Stall detection. If stall, stops motor.
                self.vm.store.motor_stall(self.port, False)
                # No Medium motor acceleration, as the front lift is too heavy
                self.vm.store.motor_acceleration(self.port, (0, 0))
                # Make sure it comes back to the set degree, if it overshoots (only for mediums)
                # 0 coast, 1 brake, 2 hold position
                self.vm.store.motor_stop(self.port, 0)

            def start(self, speed):
                self.settings()
                (acceleration, deceleration) = self.vm.store.motor_acceleration(self.port) 
                self.vm.system.motors.on_port(self.port).run_at_speed(
                    self.direction * speed,
                    stall = self.vm.store.motor_stall(self.port),
                    acceleration = acceleration,
                    deceleration = deceleration
                )
            
            def stop(self):
                self.settings()
                self.vm.system.motors.on_port(self.port).stop(self.vm.store.motor_stop(self.port))

            def run_for_degrees(self, degrees, speed):
                self.reset_encoder()
                clock = self.timer.get()
                last_enc = 0

                self.start(speed)
                while True:
                    global EXIT_EXEC
                    if (hub.button.center.is_pressed() or EXIT_EXEC):
                        EXIT_EXEC = True
                        self.stop()
                        return

                    self.get_encoder()
                    # Stall Protection
                    if (clock + 500 <= self.timer.get()):
                        if (last_enc == abs(self.encoder)):
                            print("STALL: motor "+self.port+" "+str(degrees)+"deg "+str(speed))
                            self.stop()
                            break
                        last_enc = self.encoder
                        clock = self.timer.get()
                    if (abs(self.encoder) >= abs(degrees) or hub.button.center.is_pressed()):
                        self.stop()
                        break
                    self.start(speed)
            
            def get_encoder(self):
                self.encoder = util.sensors.get_sensor_value(self.port, 2, 0)
                return self.encoder

            def reset_encoder(self):
                self.encoder = 0
                self.vm.system.motors.on_port(self.port).preset(0)
            
        # Only allowed for Driving motors
        class MotorPair:
            def __init__(self, lport, rport, vm):
                self.lport = lport
                self.rport = rport
                self.distance = 0
                self.vm = vm
            
            def settings(self):
                # make sure it uses the correct ports
                self.vm.store.move_pair((self.lport, self.rport))
                # small default acceleration (on top of programs)
                # to make accidental bad startups less common (the gyro could fix them to some degree)
                self.vm.store.move_acceleration((1000, 1000)) 
                self.vm.store.move_speed(50) # just in case
                self.vm.store.move_stop(0) # just coast (remove power from motors)

            def start(self, steering, speed):
                self.settings()
                movement.move_start(
                    self.vm, movement.from_steering(steering, speed),
                ) 

            def stop(self):
                self.settings()
                movement.move_stop(self.vm)

            # average of Driving motor encoders converted to cm
            # (56mm / 10 * pi) * (((e1 + e2) / 2) / 360)
            def get_distance(self):
                lencoder = util.sensors.get_sensor_value(self.lport, 2, 0)
                rencoder = util.sensors.get_sensor_value(self.rport, 2, 0)
                average = abs((-lencoder + rencoder)) / 2
                self.distance = WHEEL_CIRROT * (average / 360)
                return self.distance

            def reset_encoders(self):
                self.distance = 0
                self.vm.system.motors.on_port(self.lport).preset(0)
                self.vm.system.motors.on_port(self.rport).preset(0)
            
        class Gyro:
            def __init__(self):
                self.value = 0

            def angle(self):
                if (BOMB_GYRO):
                    return 0
                self.value = hub.motion.yaw_pitch_roll()[0]
                return self.value
            
            def reset(self):
                self.value = 0
                hub.motion.yaw_pitch_roll(0)
        
        class Color:
            def __init__(self, port):
                self.value = 0
                self.port = port

            # reflectance measurement (0black - 100white)
            def get(self):
                if (BOMB_COLOR):
                    return 0
                self.value = util.sensors.get_sensor_value(self.port, 1, 0)
                return self.value
        
        class Timer:
            def __init__(self):
                self.value = 0
            
            def get(self):
                self.value = util.time.get_time()
                return self.value
            
            def start(self):
                util.time.start_time()

            def stop(self):
                util.time.stop_time()

            def reset(self):
                self.value = 0
                util.time.reset_time()
    
    def reset_for_start(self):
        self.shub.bDm.reset_encoders()
        self.shub.gyro.reset()

    # Drive straight using gyro PID controller with acceleration/deceleration
    # The target is in cm. Motors also have default acceleration.
    def straight (self, target, speed = 60, accel = False, decel = False):
        speed_change_rate = speed * 0.01005 # this can be changed
        decel_distance = 12 + (12 * ((speed - 50) / 100))

        proportional = 0
        integral = 0
        derivative = 0

        lasterr = 0
        timer = speed
        phase = 1 # phase 0 (accel), 1 (full), 2 (decel)

        if (accel):
            timer = 0
            phase = 0

        self.reset_for_start() 
        while True:
            global EXIT_EXEC
            if (self.shub.is_button("center") or EXIT_EXEC):
                EXIT_EXEC = True
                self.shub.bDm.stop()
                return

            heading = self.shub.gyro.angle()
            if (speed > 0):
                proportional = (0 - heading)
            elif (speed < 0):
                proportional = (heading - 0)
            else:
                return
            integral += proportional
            derivative = proportional - lasterr
            steering = self.Kp*proportional + self.Ki*integral + self.Kd*derivative
            lasterr = proportional

            if steering > 100 : steering = 100
            if steering < -100 : steering = -100
            if speed < 0 : steering *= 2

            if (phase == 0):
                timer += speed_change_rate
            elif (phase == 2):
                if (timer > 15):
                    timer -= speed_change_rate

            if (phase == 0 and timer >= speed):
                phase = 1
                timer = speed
            if (phase == 1 and decel and target - self.shub.bDm.get_distance() <= decel_distance):
                phase = 2
            
            self.shub.bDm.start(steering, timer)

            if (self.shub.bDm.get_distance() >= abs(target)):
                self.shub.bDm.stop()
                return
            
    # Turn x degrees using gyro sensor with acceleration/deceleration
    # The target is in degrees. (Doesn't quite match up with the real world)
    def turn(self, target, speed = 60, accel = False, decel = False):
        speed_change_rate = speed * 0.006 # this can be changed
        decel_distance = 8 * (speed / 50)

        timer = speed
        phase = 1 # phase 0 (accel), 1 (full), 2 (decel)

        if (accel):
            timer = 0
            phase = 0

        self.reset_for_start()
        while True:
            global EXIT_EXEC
            if (self.shub.is_button("center") or EXIT_EXEC):
                EXIT_EXEC = True
                self.shub.bDm.stop()
                return

            if (phase == 0):
                timer += speed_change_rate
            elif (phase == 2):
                if (timer > 15):
                    timer -= speed_change_rate

            if (phase == 0 and timer >= speed):
                phase = 1
                timer = speed
            if (phase == 1 and decel and abs(target) - abs(self.shub.gyro.angle()) <= decel_distance):
                phase = 2

            if (target > 0):
                self.shub.lDm.start(timer)
            elif (target < 0):
                self.shub.rDm.start(timer)
            else:
                return

            if (abs(self.shub.gyro.angle()) >= abs(target)):
                self.shub.bDm.stop()
                return
    
    # Turn mediums motors for x degrees (PORT.LMM, PORT.RMM)
    def medium(self, target, speed = 100, port = PORT.RMM):
        if (port == PORT.LMM):
            self.shub.lMm.run_for_degrees(target, speed)
        elif (port == PORT.RMM):
            self.shub.rMm.run_for_degrees(target, speed)
        else:
            # i don't want to admit, how many times i've made this error hehe
            print("INCORRECT PORT: medium "+port+" "+str(target)+"deg "+str(speed))
            return

    # Line assisted gyro steering (the most precise movement, but more uncommon)
    # Drive straight using gyro PID controller with additional color sensor adjustments with acceleration/deceleration
    # This deals with gyro drift over long distances (the robot has to be over a black line)
    def lags(self, target, speed = 60, portside = "rr", accel = False, decel = False):
        speed_change_rate = speed * 0.01005 # this value can be tweaked
        decel_distance = 12 + (12 * ((speed - 50) / 100))
        perfect_reflectance = 50

        proportional = 0
        integral = 0
        derivative = 0
        reflectance = 0

        lasterr = 0
        timer = speed
        phase = 1 # phase 0 (accel), 1 (full), 2 (decel)

        if (accel):
            timer = 0
            phase = 0

        self.reset_for_start() 
        while True:
            global EXIT_EXEC
            if (self.shub.is_button("center") or EXIT_EXEC):
                EXIT_EXEC = True
                self.shub.bDm.stop()
                return

            heading = self.shub.gyro.angle()
            if (speed > 0):
                proportional = (0 - heading)
            elif (speed < 0):
                proportional = (heading - 0)
            else:
                return
            integral += proportional
            derivative = proportional - lasterr
            if (portside == "rr"): # right motor, right side
                reflectance = (self.shub.rCs.get() - perfect_reflectance)
            elif (portside == "rl"): # right motor, left side
                reflectance = (perfect_reflectance - self.shub.rCs.get())
            elif (portside == "ll"): # left motor, left side
                reflectance = (perfect_reflectance - self.shub.lCs.get())
            elif (portside == "lr"): # left motor, right side
                reflectance = (self.shub.lCs.get() - perfect_reflectance)
            else:
                return

            steering = self.Kp*proportional + self.Ki*integral + self.Kd*derivative + self.Kc*reflectance
            lasterr = proportional

            if steering > 100 : steering = 100
            if steering < -100 : steering = -100

            if (phase == 0):
                timer += speed_change_rate
            elif (phase == 2):
                if (timer > 15):
                    timer -= speed_change_rate

            if (phase == 0 and timer >= speed):
                phase = 1
                timer = speed
            if (phase == 1 and decel and target - self.shub.bDm.get_distance() <= decel_distance):
                phase = 2

            self.shub.bDm.start(steering, timer)

            if (self.shub.bDm.get_distance() >= abs(target)):
                self.shub.bDm.stop()
                return

async def main(vm, stack):
    api = API(vm)
    api.shub.write_info()

    loop = True
    screen = 0

    # It is allowed during the competition to start the program, if it doesn't move.
    # competitionmode toggles: starting the 0 program right away. (we didn't use it)
    # (during competition) We just entered the program before the match started.
    competitionmode = False

    LED_OFF = 0 # off
    LED_START = 6 # green
    LED_EXIT = 9 # red
    LED_MOVE = 7 # yellow

    while loop:
        global EXIT_EXEC
        if (EXIT_EXEC):
            return

        api.shub.led(LED_OFF)
        if (competitionmode):
            api.shub.led(LED_START)
            await exec_0(api)
            screen = 1
            competitionmode = False

        # Works fine without using timers
        if (api.shub.is_button("right") and api.shub.is_button("left")):
            api.shub.led(LED_EXIT)
            yield 250
            if (api.shub.is_button("right") and api.shub.is_button("left")):
                yield 750
                if (api.shub.is_button("right") and api.shub.is_button("left")):
                    vm.stop()
            api.shub.led(LED_OFF)

        if (api.shub.is_button("right")):
            api.shub.led(LED_MOVE)
            if (screen < 9):
                screen+=1
            elif (screen >= 9):
                screen = 0
            sleep(0.2)
        elif (api.shub.is_button("left")):
            api.shub.led(LED_MOVE)
            screen = abs(screen - 1)
            sleep(0.2)
        # As the program is in Advanced mode, I can remap the center,
        # without it exiting the program.
        elif (api.shub.is_button("center")):
            api.shub.led(LED_START)
            sleep(0.4) # so it doesn't autoexit
            if (screen == 0):
                await exec_0(api)
            elif (screen == 1):
                await exec_1(api)
            elif (screen == 2):
                await exec_2(api)
            elif (screen == 3):
                await exec_3(api)
            elif (screen == 4):
                await exec_4(api)
            elif (screen > 4):
                loop = False
            global EXIT_EXEC
            EXIT_EXEC = False
            screen+=1
            sleep(0.2)
        
        api.shub.display(screen)
    vm.stop()

# TODO: start your robot movement from here
async def exec_0(api):
    # EXAMPLES how to use:
    #(you can add optional parameters (like accel = False))
    #(have a look at the function definitions)
    #api.straight(40, 60)
    #api.turn(90, 60)
    #api.medium(50, -100, port=PORT.RMM)
    #api.medium(50, 100, port=PORT.LMM)
    #sleep(1)
    #api.lags(40, 60)
    api.straight(40, 60)
    pass

async def exec_1(api):
    pass

async def exec_2(api):
    pass

async def exec_3(api):
    pass

async def exec_4(api):
    pass

async def exec_5(api):
    pass

# Entry point
def setup(rpc, system, stop):
    vm = runtime.VirtualMachine(rpc, system, stop, "vm")
    vm.register_on_start("main", main)
    return vm
