#!/usr/bin/env python3

"""
Class for testing control of 12 servos. It assumes ros-i2cpwmboard has been
installed
"""
import rclpy
from rclpy.node import Node
import sys, select, termios, tty
from i2cpwm_board.msg import Servo, ServoArray, ServoConfig
from i2cpwm_board.srv import ServosConfig

numServos = 1

msg = """
Servo Control Module for 12 Servos.

Enter one of the following options:
-----------------------------
quit: stop and quit the program
oneServo: Move one servo manually, all others will be commanded to their center position
allServos: Move all servo's manually together

Keyboard commands for One Servo Control
---------------------------
   q            t   y   u
            f   g       j   k
                b   n   m

  q: Quit current command mode and go back to Option Select
  t: Command servo min value
  y: Command servo center value
  u: Command servo max value
  f: Manually decrease servo command value by 10
  g: Manually decrease servo command value by 1
  j: Manually increase servo command value by 1
  k: Manually increase servo command value by 10
  b: Save new min command value
  n: Save new center command value
  m: Save new max command value


  anything else : Prompt again for command


CTRL-C to quit
"""
keyDict = {
    'q': None,
    't': lambda x: x.set_value(x._min),
    'y': lambda x: x.set_value(x._center),
    'u': lambda x: x.set_value(x._max),
    'f': lambda x: x.set_value(x.value-0.01),
    'g': lambda x: x.set_value(x.value-.1),
    'j': lambda x: x.set_value(x.value+.1),
    'k': lambda x: x.set_value(x.value+0.01),
    'b': lambda x: x.set_min(x.value),
    'n': lambda x: x.set_center(x.value),
    'm': lambda x: x.set_max(x.value),
}

validCmds = ['quit','oneServo','allServos']

class ServoConvert():
    def __init__(self, id=1, center_value=0, direction=1):
        self.value      = center_value
        self._center    = center_value
        self._min       = -1
        self._max       = 1
        self._dir       = direction
        self.id         = id

    def set_value(self, value_in):
        if False:
            print('Servo value not in range [0,4095]')
        else:
            self.value = value_in

    def set_center(self, center_val):
        if False:
            print('Servo value not in range [0,4095]')
        else:
            self._center = center_val
            print('Servo %2i center set to %4i' % (self.id+1, center_val))

    def set_max(self, max_val):
        if False:
            print('Servo value not in range [0,4095]')
        else:
            self._max = max_val
            print('Servo %2i max set to %4i' % (self.id+1, max_val))

    def set_min(self, min_val):
        if False:
            print('Servo value not in range [0,4095]')
        else:
            self._min = min_val
            print('Servo %2i min set to %4i' % (self.id+1, min_val))

class SpotMicroServoControl(Node):
    def __init__(self):
        super().__init__('spot_micro_servo_control')

        self._servo1_config = ServoConfig()
        self._servo1_config.center = 300
        self._servo1_config.range = 400
        self._servo1_config.servo = 1
        self._servo1_config.direction = 1

        self.get_logger().info("> Waiting for config_servos service...")
        print('test1')

        self.servo_config_client = self.create_client(ServosConfig, 'config_servos')
        while not self.servo_config_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for service')
                return
            self.get_logger().info('Waiting for config_servos service...')

        self.get_logger().info("> Config_servos service found!")
        print('test2')

        request = ServosConfig.Request()
        request.servos.append(self._servo1_config)

        try:
            future = self.servo_config_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            if response is not None:
                print("Config servos done!!, returned success: %s" % str(response.success))
            else:
                print("Service call failed: no response")
        except Exception as e:
            print("Service call failed: %s" % str(e))

        self.get_logger().info("Setting Up the Spot Micro Servo Control Node...")

        self.servos = {}
        for i in range(numServos):
            self.servos[i] = ServoConvert(id=i)
        self.get_logger().info("> Servos correctly initialized")

        self._servo_msg = ServoArray()
        for i in range(numServos):
            self._servo_msg.servos.append(Servo())

        self.ros_pub_servo_array = self.create_publisher(ServoArray, "/servos_proportional", 1)
        self.get_logger().info("> Publisher correctly initialized")

        self.get_logger().info("Initialization complete")

        self.settings = termios.tcgetattr(sys.stdin)

    def send_servo_msg(self):
        for servo_key, servo_obj in self.servos.items():
            self._servo_msg.servos[servo_obj.id].servo = servo_obj.id+1
            self._servo_msg.servos[servo_obj.id].value = servo_obj.value

        self.ros_pub_servo_array.publish(self._servo_msg)

    def reset_all_servos_center(self):
        for s in self.servos:
            self.servos[s].value = self.servos[s]._center

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        self.reset_all_servos_center()
        self.send_servo_msg()

        try:
            while rclpy.ok():
                print(msg)
                userInput = input("Command?: ")

                if userInput not in validCmds:
                    print('Valid command not entered, try again...')
                else:
                    if userInput == 'quit':
                        print("Ending program...")
                        print('Final Servo Values')
                        print('--------------------')
                        for i in range(numServos):
                            print('Servo %2i:   Min: %1.2f,   Center: %1.2f,   Max: %1.2f' % (i, self.servos[i]._min, self.servos[i]._center, self.servos[i]._max))
                        break

                    elif userInput == 'oneServo':
                        self.reset_all_servos_center()
                        self.send_servo_msg()

                        nSrv = -1
                        while True:
                            userInput = input('Which servo to control? Enter a number 1 through 12: ')
                            try:
                                val = int(userInput)
                                if val not in range(1, numServos+1):
                                    print("Invalid servo number entered, try again")
                                else:
                                    nSrv = val - 1
                                    break
                            except ValueError:
                                print("Invalid servo number entered, try again")

                        print('Enter command, q to go back to option select: ')
                        while True:
                            userInput = self.getKey()

                            if userInput == 'q':
                                break
                            elif userInput not in keyDict:
                                print('Key not in valid key commands, try again')
                            else:
                                keyDict[userInput](self.servos[nSrv])
                                print('Servo %2i cmd: %1.2f' % (nSrv, self.servos[nSrv].value))
                                self.send_servo_msg()

                    elif userInput == 'allServos':
                        self.reset_all_servos_center()
                        self.send_servo_msg()

                        print('Enter command, q to go back to option select: ')
                        while True:
                            userInput = self.getKey()

                            if userInput == 'q':
                                break
                            elif userInput not in keyDict:
                                print('Key not in valid key commands, try again')
                            elif userInput in ('b','n','m'):
                                print('Saving values not supported in all servo control mode')
                            else:
                                for s in self.servos.values():
                                    keyDict[userInput](s)
                                print('All Servos Commanded')
                                self.send_servo_msg()

                rate = self.create_rate(10)
                rate.sleep()
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == "__main__":
    rclpy.init()
    smsc = SpotMicroServoControl()
    smsc.run()
    rclpy.shutdown()
