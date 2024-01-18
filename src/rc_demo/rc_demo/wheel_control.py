from time import sleep
import os
import rclpy                                     # ROS2 Python Client Library
from rclpy.node import Node                      # ROS2 Node
from sensor_msgs.msg import Joy                  # ROS2 standard Joy Message
import serial
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from pydub import AudioSegment
from pydub.playback import play
import can
import RPi.GPIO as GPIO



class WheelControl(Node):

    mode = 0  # 0: Null, 1: Relative Position Control, 2: Aboslute Position Control, 3: Velocity Control, 4: Torque Control
    state = 7  # 0: Null, 5: Emergency Stop, 6: Clear Alarm, 7: Disable, 8: Enable

    motors_x = 0
    motors_y = 0
    left_motor_param = 0
    right_motor_param = 0

    state_enable = [False, 0, 0]  # [state_enable, previous_button_pressed, button_pressed]
    state_emerge = 0

    max_speed = 30
    twist_ratio = 0.5

    sleep_time = 0.02

    air_pump = 22
    air_valve = 23
    ac_input = 27

    axis_state = [0, 0]

    valve_state = [False, 0, 0]

    # arm_enable = "1\n"
    # arm_disable = "2\n"
    # arm_down = "3\n"
    # arm_up = "4\n"

    def __init__(self,name):
        super().__init__(name)
        # self.hillside = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/hillside.mp3', format='mp3')
        # self.initialized = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Initialized.mp3', format='mp3')
        # self.enabled = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Enabled.mp3', format='mp3')
        # self.disabled = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Disabled.mp3', format='mp3')
        # self.emergency = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/EmergencyStop.mp3', format='mp3')
        # self.stopped = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Stopped.mp3', format='mp3')
        # self.fence = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Fence.mp3', format='mp3')
        # self.stairs = AudioSegment.from_file('/home/skippy/Sundries-Wiper/src/wheel_motors/wheel_motors/audio/Stairs.mp3', format='mp3')
        # self.get_logger().info("Audio loaded.")

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.air_pump, GPIO.OUT)
        GPIO.setup(self.air_valve, GPIO.OUT)
        GPIO.setup(self.ac_input, GPIO.IN)
        self.get_logger().info("Air pump and valve initialized.")

        self.rs485_1 = modbus_rtu.RtuMaster(serial.Serial(port="/dev/ttySC0", baudrate=115200, bytesize=8, parity='N', stopbits=1, xonxoff=0))
        self.rs485_1.set_timeout(0.5)
        self.rs485_1.set_verbose(True)
        self.get_logger().info("RS485-1 initialized.")
        sleep(1)
        self.esc_clear_alarm()
        self.esc_disable()
        self.esc_velocity_control()
        self.get_logger().info("ESC initialized.")

        self.can0 = can.interface.Bus(channel = 'can0', bustype = 'socketcan')
        self.platform_raise = can.Message(arbitration_id=0x00000100, data=[0xf6, 0x00, 0x03, 0xe8, 0x0f, 0xa0, 0x00, 0x6b], is_extended_id=True)
        self.platform_lower = can.Message(arbitration_id=0x00000100, data=[0xf6, 0x01, 0x03, 0xe8, 0x0f, 0xa0, 0x00, 0x6b], is_extended_id=True)
        self.platform_stop = can.Message(arbitration_id=0x00000100, data=[0xf6, 0x00, 0x03, 0xe8, 0x00, 0x00, 0x00, 0x6b], is_extended_id=True)
        self.get_logger().info("CAN initialized.")

        # self.arduino = serial.Serial('/dev/ttyUSB2', 9600, timeout=0.1)
        # self.get_logger().info("Arduino initialized.")

        # play(self.initialized)

        self.sub_joy = self.create_subscription(
            Joy, 
            "joy", 
            self.joy_recv, 
            10)
        
    def esc_clear_alarm(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=6)
        # sleep(self.sleep_time)

    def esc_emergency_stop(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=5)
        # sleep(self.sleep_time)

    def esc_enable(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=8)
        # sleep(self.sleep_time)

    def esc_disable(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200e), output_value=7)
        # sleep(self.sleep_time)

    def esc_torque_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=4)
        # sleep(self.sleep_time)

    def esc_velocity_control(self):
        self.rs485_1.execute(1, cst.WRITE_SINGLE_REGISTER, int(0x200d), output_value=3)
        # sleep(self.sleep_time)

    def esc_motor_velocity(self, left_target, right_target):
        self.rs485_1.execute(1, cst.WRITE_MULTIPLE_REGISTERS, int(0x2088), output_value=[left_target, -right_target])
        # sleep(self.sleep_time)

            

    def joy_recv(self,msg):
        input = {
            'left_joy_x'              : -msg.axes[0]     ,
            'left_joy_y'              :  msg.axes[1]     ,
            'right_joy_x'             : -msg.axes[3]     ,
            'right_joy_y'             :  msg.axes[4]     ,
            'dpad_x'                  : -msg.axes[6]     ,
            'dpad_y'                  :  msg.axes[7]     ,
            'left_trigger'            : -msg.axes[2]     ,
            'right_trigger'           :  msg.axes[5]     ,
            'left_shoulder_button'    :  msg.buttons[4]  ,
            'right_shoulder_button'   :  msg.buttons[5]  ,
            'a_button'                :  msg.buttons[0]  ,
            'b_button'                :  msg.buttons[1]  ,   
            'x_button'                :  msg.buttons[2]  ,
            'y_button'                :  msg.buttons[3]  ,
            'select_button'           :  msg.buttons[6]  ,
            'start_button'            :  msg.buttons[7]  ,
            'menu_button'             :  msg.buttons[8]  ,
            'left_joy_button'         :  msg.buttons[9]  ,
            'right_joy_button'        :  msg.buttons[10]  
        } 
        self.input_processor(input)

    def input_processor(self,input):
        self.axis_state[0] = self.axis_state[1]
        self.axis_state[1] = input['dpad_y']
        if self.axis_state[0] == 0 and self.axis_state[1] == 1:
            self.can0.send(self.platform_raise)
        if self.axis_state[0] == 0 and self.axis_state[1] == -1:
            self.can0.send(self.platform_lower)
        if abs(self.axis_state[0]) == 1 and self.axis_state[1] == 0:
            self.can0.send(self.platform_stop)

        self.state_emerge = input['left_shoulder_button']
        if self.state_emerge == 1:
            self.esc_emergency_stop()
            self.state_enable[0] = False
            # play(self.emergency)
            self.get_logger().info("Emergency Stop")
        else:
            self.state_enable[1] = self.state_enable[2]
            self.state_enable[2] = input['right_shoulder_button']
            self.motors_x = input['left_joy_x']
            self.motors_y = input['left_joy_y']

            if self.state_enable[1] == 0 and self.state_enable[2] == 1:
                self.state_enable[0] = not self.state_enable[0]
                if self.state_enable[0] == True:
                    self.esc_clear_alarm()
                    self.esc_velocity_control()
                    self.esc_enable()
                    self.get_logger().info("Motors Enabled")
                    # play(self.enabled)
                else:
                    self.esc_clear_alarm()
                    self.esc_velocity_control()
                    self.esc_disable()
                    self.get_logger().info("Motors Disabled")
                    # play(self.stopped)

            if self.state_enable[0] == True:
                self.left_motor_param = int(self.max_speed * (self.motors_y + self.motors_x * self.twist_ratio))
                self.right_motor_param = int(self.max_speed * (self.motors_y - self.motors_x * self.twist_ratio))

                self.esc_motor_velocity(self.left_motor_param, self.right_motor_param)
                self.get_logger().info("Left Motor: %d, Right Motor: %d" % (self.left_motor_param, self.right_motor_param))

            # if input['left_shoulder_button_2'] == 1:
                # play(self.fence)

            # if input['right_shoulder_button_2'] == 1:
                # play(self.stairs)

            # if input['a_button'] == 1:
                # play(self.hillside)

            if input['y_button'] == 1:
                GPIO.output(self.air_pump, GPIO.HIGH)
            else:
                GPIO.output(self.air_pump, GPIO.LOW)

            self.valve_state[1] = self.valve_state[2]
            self.valve_state[2] = input['x_button']
            if self.valve_state[1] == 0 and self.valve_state[2] == 1:
                self.valve_state[0] = not self.valve_state[0]
            if self.valve_state[0] == True:
                GPIO.output(self.air_valve, GPIO.HIGH)
            else:
                GPIO.output(self.air_valve, GPIO.LOW)

            # if input['dpad_x'] == -1:
            #     self.arduino.write(self.arm_enable.encode())
            # if input['dpad_x'] == 1:
            #     self.arduino.write(self.arm_disable.encode())
            # if input['select_button'] == 1:
            #     self.arduino.write(self.arm_down.encode())
            # if input['start_button'] == 1:
            #     self.arduino.write(self.arm_up.encode())


def main(args=None):
    rclpy.init(args=args)
    node = WheelControl('wheel_control')
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
