import sys
import time
import threading
import traceback
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC


stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
],
                              dtype=float)

stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
],
                                dtype=float)

dt = 0.002
runing_time = 0.0
crc = CRC()

class JointStatePub(Node):
    def __init__(self):
        super().__init__('topicddsbridge_jointst_pub')
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

class PIDGainPub(Node):
    def __init__(self):
        super().__init__('topicddsbridge_pidgain_pub')
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'pid_gain',
            10
        )

class JointStateSub(Node):
    def __init__(self):
        super().__init__('topicddsbridge_jointst_sub')
        self.subscription = self.create_subscription(
            JointState,
            '/low_level_info/joint_states',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to joint state topic.')
        self.jointAngle_data = {}
        self.jointVelocity_data = {}

        self.dof_pos = []
        self.dof_vel = []

    def joint_state_callback(self, msg):
        joint_names = msg.name
        positions = msg.position
        velocities = msg.velocity

        for i, name in enumerate(joint_names):
            self.jointAngle_data[name] = positions[i]
            self.jointVelocity_data[name] = velocities[i]

    def get_jointAngle_data(self):
        self.dof_pos = [self.jointAngle_data['flh'],  self.jointAngle_data['flu'],  self.jointAngle_data['fld'],  
                        self.jointAngle_data['frh'],  - self.jointAngle_data['fru'],  - self.jointAngle_data['frd'], 
                        -self.jointAngle_data['rlh'],  self.jointAngle_data['rlu'],  self.jointAngle_data['rld'],    
                        -self.jointAngle_data['rrh'],  - self.jointAngle_data['rru'],  - self.jointAngle_data['rrd']]
    
        self.dof_vel = [self.jointVelocity_data['flh'],  self.jointVelocity_data['flu'],  self.jointVelocity_data['fld'],  
                        self.jointVelocity_data['frh'],  - self.jointVelocity_data['fru'],  - self.jointVelocity_data['frd'], 
                        -self.jointVelocity_data['rlh'],  self.jointVelocity_data['rlu'],  self.jointVelocity_data['rld'],    
                        -self.jointVelocity_data['rrh'],  - self.jointVelocity_data['rru'],  - self.jointVelocity_data['rrd']]
    
        return self.dof_pos, self.dof_vel

class LinarASub(Node):
    def __init__(self):
        super().__init__('Imu_linear_acceleration_subscriber')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/acceleration_hr',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to linear acceleration topic.')
        self.linear_acceleration = None

    def joint_state_callback(self, msg):
        self.linear_acceleration = msg.vector
        # self.get_logger().info(f"Received linear acceleration: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

    def get_Imu_data(self):
        return self.linear_acceleration

class AngularVSub(Node):
    def __init__(self):
        super().__init__('Imu_angular_velocity_subscriber')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/angular_velocity_hr',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to angular velocity topic.')
        self.angular_velocity = None

    def joint_state_callback(self, msg):
        self.angular_velocity = msg.vector
        # self.get_logger().info(f"Received angular velocity: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

    def get_Imu_data(self):
        return self.angular_velocity

class QuaternionSub(Node):
    def __init__(self):
        super().__init__('Imu_quaternion_subscriber')
        self.subscription = self.create_subscription(
            QuaternionStamped,
            '/filter/quaternion',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to quaternion topic.')
        self.quaternion = None

    def joint_state_callback(self, msg):
        self.quaternion = msg.quaternion

    def get_Imu_data(self):
        return self.quaternion

class DDSHandler:
    def __init__(self, ether_name: str=""):
        # self.jointStateSub = JointStateSub()

        # self.pidGainPub = PIDGainPub()
        # self.jointStatePub = JointStatePub() 

        self.pub = None
        self.low_state = None
        self.pub = ChannelPublisher("rt/lowstate", LowState_)
        self.pub.Init()  

        self.sub = None
        self.low_cmd = None
        self.sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.sub.Init(self.LowCmdMessageHandler, 10) 


        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'flh', 'frh', 'rlh', 'rrh',   # Hips
            'flu', 'fru', 'rlu', 'rru',  # Upper legs
            'fld', 'frd', 'rld', 'rrd'   # Lower legs
        ]
        self.pid_gain_msg = Float32MultiArray()
        self.pid_gain_msg.data = [float(0), float(0)]


    def LowCmdMessageHandler(self, msg: LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            # print(self.low_cmd.motor_cmd)
            self.joint_state_msg.position = []
            self.joint_state_msg.velocity = []
            self.joint_state_msg.effort = []
            i = 0
            for motor_cmd in self.low_cmd.motor_cmd:
                if i < 12:
                    self.joint_state_msg.position.append(motor_cmd.q)
                    self.joint_state_msg.velocity.append(motor_cmd.dq)
                    self.joint_state_msg.effort.append(motor_cmd.tau)
                    self.pid_gain_msg.data[0] = motor_cmd.kp
                    self.pid_gain_msg.data[1] = motor_cmd.kd
                    i += 1
                else:
                    break
        
def stand_up():
    runing_time = 0
    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        n = [i for i in range(9, 9+3)]

        if (runing_time < 3.0):
            # Stand up in first 3 second
            
            # Total time for standing up or standing down is about 1.2s
            phase = np.tanh(runing_time / 1.2)
            for i in n:
                # cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
                #     1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].q = 0
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 5
        else:
            # Then stand down
            phase = np.tanh((runing_time - 3.0) / 1.2)
            for i in n:
                cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (
                    1 - phase) * stand_up_joint_pos[i]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

class Controller:
    def __init__(self):
        if len(sys.argv) <2:
            ChannelFactoryInitialize(1, "lo")
            self.dds_handler = DDSHandler()
        else:
            ChannelFactoryInitialize(0, sys.argv[1])
            self.dds_handler = DDSHandler(sys.argv[1])

        rclpy.init()
        self.jointStateSub = JointStateSub()
        self.linearASub = LinarASub()
        self.angularVSub = AngularVSub()
        self.quaternionSub = QuaternionSub()
        self.jointStatePub = JointStatePub()
        self.pidGainPub = PIDGainPub()
        

        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.jointStateSub)
        self.executor.add_node(self.angularVSub)
        self.executor.add_node(self.linearASub)
        self.executor.add_node(self.quaternionSub)
        self.executor.add_node(self.jointStatePub)
        self.executor.add_node(self.pidGainPub)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        time.sleep(1)

    def low_cmd2topic(self):
        joint_state_msg = self.dds_handler.joint_state_msg
        pid_gain_msg = self.dds_handler.pid_gain_msg

        self.jointStatePub.publisher.publish(joint_state_msg)
        self.pidGainPub.publisher.publish(pid_gain_msg)

    def topic2low_state(self):
        joint_state = self.jointStateSub.get_jointAngle_data()

        orientation = self.quaternionSub.get_Imu_data()
        linear_acceleration = self.linearASub.get_Imu_data()
        angular_velocity = self.angularVSub.get_Imu_data()

        #TODO: Add the logic to process the data and send it to the robot


if __name__ == '__main__':
    controller = Controller()

    cmd_dict = {
        "read": 0
    }

    start_time = time.perf_counter()
    now_time = time.perf_counter()
    while (now_time - start_time) < 20:
        now_time = time.perf_counter()
        # print(controller.quaternionSub.get_Imu_data())
        controller.low_cmd2topic()
        # print("Quaternion: ", quaternionSub.get_Imu_data())
        # print("Linear Acceleration: ", linearASub.get_Imu_data())
        # print("Angular Velocity: ", angularVSub.get_Imu_data())
        # print(jointStateSub.get_jointAngle_data())
        # print(dds_handler.pid_gain_msg)




# /low_level_info/joint_states
# ---
# header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: ''
# name:
# - frd
# - fld
# - rrd
# - rld
# - fru
# - flu
# - rru
# - rlu
# - frh
# - flh
# - rrh
# - rlh
# position:
# - 1.7023346424102783
# - -1.5722514390945435
# - -1.6283283233642578
# - 1.7023346424102783
# - -0.692568838596344
# - 0.7360570430755615
# - 0.6017776727676392
# - -0.8802548050880432
# - -0.01888304017484188
# - 0.054741740226745605
# - 0.1356145590543747
# - -0.13904783129692078
# velocity:
# - -0.021978022530674934
# - 0.007326007355004549
# - -0.021978022530674934
# - -0.007326007355004549
# - -0.021978022530674934
# - -0.021978022530674934
# - -0.021978022530674934
# - -0.007326007355004549
# - -0.021978022530674934
# - 0.007326007355004549
# - -0.007326007355004549
# - -0.007326007355004549
# effort: []
# ---

# /pid_gain
# ---
# layout:
#   dim: []
#   data_offset: 0
# data:
# - 13.0
# - 0.4000000059604645
# ---

# /joint_states
# ---
# header:
#   stamp:
#     sec: 0
#     nanosec: 0
#   frame_id: ''
# name:
# - flh
# - frh
# - rlh
# - rrh
# - flu
# - fru
# - rlu
# - rru
# - fld
# - frd
# - rld
# - rrd
# position:
# - 0.024110764265060425
# - -0.03116896003484726
# - -0.16530171036720276
# - 0.13215360045433044
# - 0.7514330744743347
# - -0.6867855191230774
# - -0.8250313401222229
# - 0.6604069471359253
# - -1.4955912828445435
# - 1.5727512836456299
# - 1.613569974899292
# - -1.5072613954544067
# velocity: []
# effort: []

# sensor_msgs.msg.JointState(
#     header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), 
#     name=['flh', 'frh', 'rlh', 'rrh', 'flu', 'fru', 'rlu', 'rru', 'fld', 'frd', 'rld', 'rrd'], 
#     position=[0.04729447141289711, 1.221118450164795, -2.442246913909912, 
#               -0.04729447141289711, 1.221118450164795, -2.442246913909912, 
#               0.04729447141289711, 1.221118450164795, -2.442246913909912, 
#               -0.04729447141289711, 1.221118450164795, -2.442246913909912], 
#               velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
#               effort=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


