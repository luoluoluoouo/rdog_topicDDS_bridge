import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC

class DDSHandler:
    def __init__(self, ether_name: str=""):
        self.sub = ChannelSubscriber("rt/lowstate", LowState_)
        self.sub.Init(self.LowStateMessageHandler, 10) 

        self.low_state = None

    def LowStateMessageHandler(self, msg: LowState_):
        if msg is not None:
            self.low_state = msg  
        
if __name__ == "__main__":
    if len(sys.argv) <2:
        ChannelFactoryInitialize(1, "lo")
        dds_handler = DDSHandler()
    else:
        ChannelFactoryInitialize(0, sys.argv[1])
        dds_handler = DDSHandler(sys.argv[1])

    while True:
        print(dds_handler.low_state)



