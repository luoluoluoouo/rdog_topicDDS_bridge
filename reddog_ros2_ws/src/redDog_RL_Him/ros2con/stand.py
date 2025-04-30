import time
import sys
import numpy as np

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

default_pose = [ 3, -1.6, 0,
                -3 , 1.6, 0,
                -3 , 1.6, 0,
                 3 ,-1.6, 0]

test_pose = [- 0.0414070188999176, 0.061594702303409576, -0.14110097289085388
            , 0.18578624725341797, 0.7276803851127625, -0.6791616678237915
            , -0.7462558150291443, 0.7220953106880188, -1.4941880702972412
            , 1.5796664953231812, 1.5809358358383179, -1.3872215747833252]


dt = 0.002
runing_time = 0.0
crc = CRC()

input("Press enter to start")

class Go2Channel:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.sub = None
        self.sub0 = None
        self.low_cmd = None

        self.low_state = None
        self.quaternion = [0.0, 0.0, 0.0, 0.0]
        self.gyroscope = [0.0, 0.0, 0.0]
        self.accelerometer = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.imu_temperature = 0.0

        self.q = []  # List of motor positions
        self.dq = []  # List of motor velocities
        self.ddq = []  # List of motor accelerations
        self.tau_est = []  # List of estimated torques
        self.motor_temperature = []  # List of motor temperatures
        self.motor_lost = []  # List of lost motor states
        self.motor_reserve = []  # List of motor reserves

        self.bms_version = (0, 0)
        self.bms_status = 0
        self.bms_soc = 0
        self.bms_current = 0
        self.bms_cycle = 0
        self.bms_cell_vol = [0] * 16

        self.foot_force = [0.0, 0.0, 0.0, 0.0]
        self.foot_force_est = [0.0, 0.0, 0.0, 0.0]

        self.tick = 0
        self.wireless_remote = b'\x00' * 32
        self.bit_flag = 0
        self.adc_reel = 0.0
        self.temperature_ntc1 = 0
        self.temperature_ntc2 = 0
        self.power_v = 0.0
        self.power_a = 0.0
        self.fan_frequency = [0] * 4
        self.reserve = 0
        self.crc = 0

        self.q = []
        self.dq = []
        self.tau = []
        self.kp = []
        self.kd = []
        self.motor_reserve = []

        self.version = None

        # if len(ether_name)>1:
        #     ChannelFactoryInitialize(0, ether_name)
        # else:
        #     ChannelFactoryInitialize(0)

        # # Create a publisher to publish the data defined in UserData class
        # self.pub = ChannelPublisher("rt/lowcmd", LowCmd_)
        # self.pub.Init()

        # Create a subscriber to receive the latest robot state every certain seconds 
        # self.low_state = None
        # self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        # self.sub.Init(self.LowStateMessageHandler, 10)  

        # self.sub0 = ChannelSubscriber("rt/lowcmd", LowCmd_)
        # self.sub0.Init(self.LowCmdMessageHandler, 10)  

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        if self.low_state is not None:
            # Parse IMU state and store the quaternion, gyroscope, and accelerometer data
            self.quaternion = self.low_state.imu_state.quaternion
            self.gyroscope = self.low_state.imu_state.gyroscope
            self.accelerometer = self.low_state.imu_state.accelerometer
            self.rpy = self.low_state.imu_state.rpy
            self.imu_temperature = self.low_state.imu_state.temperature

            # Initialize the motor state lists
            self.q.clear()
            self.dq.clear()
            self.ddq.clear()
            self.tau_est.clear()
            self.motor_temperature.clear()
            self.motor_lost.clear()
            self.motor_reserve.clear()

            # Parse motor states and store relevant information for each motor
            for motor_state in self.low_state.motor_state:
                self.q.append(motor_state.q)
                self.dq.append(motor_state.dq)
                self.ddq.append(motor_state.ddq)
                self.tau_est.append(motor_state.tau_est)
                self.motor_temperature.append(motor_state.temperature)
                self.motor_lost.append(motor_state.lost)
                self.motor_reserve.append(motor_state.reserve)

            # Parse BMS state and store its values
            self.bms_version = (self.low_state.bms_state.version_high, self.low_state.bms_state.version_low)
            self.bms_status = self.low_state.bms_state.status
            self.bms_soc = self.low_state.bms_state.soc
            self.bms_current = self.low_state.bms_state.current
            self.bms_cycle = self.low_state.bms_state.cycle
            self.bms_cell_vol = self.low_state.bms_state.cell_vol

            # Parse foot force data
            self.foot_force = self.low_state.foot_force
            self.foot_force_est = self.low_state.foot_force_est

            # Parse tick, wireless remote, bit_flag, and other related fields
            self.tick = self.low_state.tick
            self.wireless_remote = self.low_state.wireless_remote
            self.bit_flag = self.low_state.bit_flag
            self.adc_reel = self.low_state.adc_reel
            self.temperature_ntc1 = self.low_state.temperature_ntc1
            self.temperature_ntc2 = self.low_state.temperature_ntc2
            self.power_v = self.low_state.power_v
            self.power_a = self.low_state.power_a
            self.fan_frequency = self.low_state.fan_frequency
            self.reserve = self.low_state.reserve
            self.crc = self.low_state.crc

    def LowCmdMessageHandler(self, msg: LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            # Initialize motor state lists
            self.q = []
            self.dq = []
            self.tau = []
            self.kp = []
            self.kd = []
            self.motor_reserve = []

            # Parse motor commands and store relevant information for each motor
            for motor_cmd in self.low_cmd.motor_cmd:
                self.q.append(motor_cmd.q)
                self.dq.append(motor_cmd.dq)
                self.tau.append(motor_cmd.tau)
                self.kp.append(motor_cmd.kp)
                self.kd.append(motor_cmd.kd)
                self.motor_reserve.append(motor_cmd.reserve)

            # Parse other fields
            self.head = self.low_cmd.head
            self.level_flag = self.low_cmd.level_flag
            self.frame_reserve = self.low_cmd.frame_reserve
            self.sn = self.low_cmd.sn
            self.version = self.low_cmd.version
            self.bandwidth = self.low_cmd.bandwidth
            self.bms_cmd_off = self.low_cmd.bms_cmd.off
            self.bms_cmd_reserve = self.low_cmd.bms_cmd.reserve
            self.wireless_remote = self.low_cmd.wireless_remote
            self.led = self.low_cmd.led
            self.fan = self.low_cmd.fan
            self.gpio = self.low_cmd.gpio
            self.reserve = self.low_cmd.reserve
            self.crc = self.low_cmd.crc

            # Parse BMS command (if applicable)
            self.bms_cmd = {
                "off": self.bms_cmd_off,
                "reserve": self.bms_cmd_reserve
            }

            # Parse wireless remote data
            self.wireless_remote_data = self.wireless_remote.hex()

            # Parse LED data (might be used for visual feedback)
            self.led_data = self.led.hex()

            # Parse fan control data
            self.fan_data = self.fan.hex()
        

if __name__ == '__main__':

    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
        ch = Go2Channel()
    else:
        ChannelFactoryInitialize(0, sys.argv[1])
        ch = Go2Channel(sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    #TODO
    # ch = Go2Channel("hi")
    

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        # cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        # cmd.motor_cmd[i].tau = 0.0

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        if (runing_time < 5.0):
            # Stand up in first 3 second
            
            # Total time for standing up or standing down is about 1.2s
        #     phase = np.tanh(runing_time / 1.2)
        #     for i in range(12):
        #         cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
        #             1 - phase) * stand_down_joint_pos[i]
        #         # cmd.motor_cmd[i].q = 
        #         cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
        #         # cmd.motor_cmd[i].dq = 0.0
        #         cmd.motor_cmd[i].kd = 3.5
        #         # cmd.motor_cmd[i].tau = 0
        # else:
        #     # Then stand down
        #     phase = np.tanh((runing_time - 3.0) / 1.2)
        #     for i in range(12):
        #         cmd.motor_cmd[i].q = phase * stand_down_joint_pos[i] + (
        #             1 - phase) * stand_up_joint_pos[i]
        #         cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
        #         # cmd.motor_cmd[i].dq = 0.0
        #         cmd.motor_cmd[i].kd = 3.5
        #         # cmd.motor_cmd[i].tau = 0

            phase = np.tanh(runing_time / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * test_pose[i] + (
                    1 - phase) * default_pose[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                # cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                # cmd.motor_cmd[i].tau = 0
            else:
                # Then stand down
                phase = np.tanh((runing_time - 3.0) / 1.2)
                for i in range(12):
                    cmd.motor_cmd[i].q = phase * default_pose[i] + (
                        1 - phase) * test_pose[i]
                    cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                    # cmd.motor_cmd[i].dq = 0.0
                    cmd.motor_cmd[i].kd = 3.5
                    # cmd.motor_cmd[i].tau = 0


        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


# LowCmd_(head=b'\xfe\xef', level_flag=255, frame_reserve=0, sn=[0, 0], version=[0, 0], bandwidth=0,
#         motor_cmd=[
#         MotorCmd_(mode=1, q=0.04734550043940544, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=1.221869945526123, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=-2.4437499046325684, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=-0.04734550043940544, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=1.221869945526123, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=-2.4437499046325684, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.04734550043940544, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=1.221869945526123, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=-2.4437499046325684, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=-0.04734550043940544, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]),
#         MotorCmd_(mode=1, q=1.221869945526123, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=-2.4437499046325684, dq=0.0, tau=0.0, kp=50.0, kd=3.5, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0]), 
#         MotorCmd_(mode=1, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0, reserve=[0, 0, 0])], 
#         bms_cmd=BmsCmd_(off=0, reserve=b'\x00\x00\x00'), 
#         wireless_remote=b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 
#         led=b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', fan=b'\x00\x00', gpio=0, reserve=0, crc=543961840)

# LowState_(head=b'\x00\x00', level_flag=0, frame_reserve=0, sn=[0, 0], version=[0, 0], bandwidth=0, 
#           imu_state=IMUState_(quaternion=[-0.0003142616478726268, -0.12561771273612976, -0.9920760989189148, -0.0022499149199575186], 
#                               gyroscope=[0.0005036171642132103, -5.186332418816164e-05, -2.1727576040575514e-06], 
#                               accelerometer=[-0.0005544007872231305, 0.04462854564189911, -9.809723854064941], 
#                               rpy=[3.137049436569214, 5.8284607803216204e-05, 2.88969087600708], 
#                               temperature=0), 
#           motor_state=[
#             MotorState_(mode=0, q=0.09449685364961624, dq=-0.013804769143462181, ddq=0.0, tau_est=0.7607715725898743, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.11421481519937515, dq=-0.016516679897904396, ddq=0.0, tau_est=-0.12427928298711777, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=-0.7744351029396057, dq=0.00691996468231082, ddq=0.0, tau_est=40.107322692871094, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.12157013267278671, dq=-0.001587467035278678, ddq=0.0, tau_est=-0.5095061659812927, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.11421962827444077, dq=-0.0165124349296093, ddq=0.0, tau_est=-0.12450623512268066, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=-0.7744340896606445, dq=0.006921189371496439, ddq=0.0, tau_est=40.10727310180664, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.09449652582406998, dq=-0.013812578283250332, ddq=0.0, tau_est=0.7608120441436768, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.11421452462673187, dq=-0.016517378389835358, ddq=0.0, tau_est=-0.12426362931728363, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=-0.7744819521903992, dq=0.006915241479873657, ddq=0.0, tau_est=40.10946273803711, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.12183079868555069, dq=-0.000727802631445229, ddq=0.0, tau_est=-0.5242145657539368, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.11421950161457062, dq=-0.0165121927857399, ddq=0.0, tau_est=-0.12450221180915833, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=-0.774480938911438, dq=0.0069165960885584354, ddq=0.0, tau_est=40.109413146972656, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0]), 
#             MotorState_(mode=0, q=0.0, dq=0.0, ddq=0.0, tau_est=0.0, q_raw=0.0, dq_raw=0.0, ddq_raw=0.0, temperature=0, lost=0, reserve=[0, 0])], 
#             bms_state=BmsState_(version_high=0, version_low=0, status=0, soc=0, current=0, cycle=0, bq_ntc=b'\x00\x00', mcu_ntc=b'\x00\x00', cell_vol=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]), 
#             foot_force=[0, 0, 0, 0], foot_force_est=[0, 0, 0, 0], tick=0, 
#             wireless_remote=b'\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00', 
#             bit_flag=0, adc_reel=0.0, temperature_ntc1=0, temperature_ntc2=0, power_v=0.0, power_a=0.0, fan_frequency=[0, 0, 0, 0], reserve=0, crc=0)

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
# - 0.0414070188999176
# - 0.061594702303409576
# - -0.14110097289085388
# - 0.18578624725341797
# - 0.7276803851127625
# - -0.6791616678237915
# - -0.7462558150291443
# - 0.7220953106880188
# - -1.4941880702972412
# - 1.5796664953231812
# - 1.5809358358383179
# - -1.3872215747833252
# velocity: []
# effort: []
# ---

