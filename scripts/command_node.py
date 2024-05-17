import rospy
import os
import logging
import json
from icecream import ic  # 假设这是一个用于调试的函数，实际项目中可能需要替换为标准日志记录方法
from ros_ht_msg.msg import ht_control
from ros_modbus_msg.msg import operation
from ros_modbus_msg.msg import spindle_argument


# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

# 建议使用的基类，用于消息发布
class BaseCommand:
    def __init__(self, pub_topic, msg_type):
        self.pub_topic = pub_topic
        self.pub = rospy.Publisher(self.pub_topic, msg_type, queue_size=10)
        self.param = ReadParam()

    def publish_command(self, command_func, *args, **kwargs):
        try:
            ic(*args, **kwargs)
            msg = command_func(*args, **kwargs)
            self.pub.publish(msg)
            rospy.loginfo(f"指令发布成功: {str(msg)}")
        except Exception as e:
            rospy.logerr(f"{str(self.pub_topic)}发布指令失败: {str(e)}")

    def sleep(self, seconds):
        rospy.sleep(seconds)

class BasePlateCommand(BaseCommand):
    def __init__(self):
        super().__init__("HT_Control", ht_control)
        self.move()

    def move(self):
        self.command_setting(self.param.position_param,self.param.position_param.keys())

    def command_setting(self, paramdict, keylist):
        for key in keylist:
            moveparam = paramdict[key][1:]
            for t in range(paramdict[key][0]):
                ic(paramdict[key][0])
                self.publish_command(self._base_plate_command, moveparam)
                self.sleep(1)

    def _base_plate_command(self, moveparam):
        control = ht_control()
        control.mode = moveparam[0]
        control.x = moveparam[1]
        control.y = moveparam[2]
        control.z = moveparam[3]
        return control

class SlidingTableCommand(BaseCommand):
    def __init__(self):
        super().__init__("sub_param", operation)
        self.move()

    def move(self):
        self.command_setting(self.param.splindle_param,self.param.splindle_param.keys())

    def command_setting(self, paramdict, keylist):
        for key in keylist:
            moveparam = paramdict[key]
            ic(moveparam)
            self.publish_command(self._sliding_table_command, moveparam)
            # self.sleep(15)

    def _sliding_table_command(self, moveparam):
        oper = operation()
        oper.operation = moveparam[0]
        oper.A_x = moveparam[1]
        oper.A_y = moveparam[3]
        oper.A_z = moveparam[5]
        oper.x_speed = moveparam[2]
        oper.y_speed = moveparam[4]
        oper.z_speed = moveparam[6]
        return oper

class SpindleMotorCommand(BaseCommand):
    def __init__(self):
        super().__init__("spindle_motor", spindle_argument)    
        self.spindlemotor_list = self._spindle_argument(self.param.motor_param.keys())   
        self.dopub(self._command_setting(self.spindlemotor_list))
    def dopub(self,command_list):
        try:
            self.pub.publish(command_list[0])
            ic(1)
            rospy.sleep(command_list[2])
            ic(f"wait{command_list[2]}")
            self.pub.publish(command_list[1])
            ic(2)

        except Exception as e:
            rospy.logerr(f"发布指令失败: {str(e)}") 

    def _command_setting(self, spindlemotorlist):
        start_command = self._spindle_motor_command([spindlemotorlist[1], spindlemotorlist[2]])
        end_command = self._spindle_motor_command([spindlemotorlist[1], spindlemotorlist[3]])
        time = self.param.motor_param["time"]
        return [start_command, end_command, time]

    def _spindle_motor_command(self, moveparam):
        spindle = spindle_argument()
        spindle.oper = 1
        spindle.address = moveparam[0]
        spindle.value = moveparam[1]
        spindle.num_reg = 1
        return spindle

    def _spindle_argument(self, keys):
        param_list = []
        for key in keys:
            param_list.append(self.param.motor_param[key])
        return param_list

class ReadParam:
    def __init__(self):
        self.configname = "config/DemoParameters.json"
        try:
            self.path = self._getDirectoryPath()
        except Exception as e:
            logging.error(f"初始化路径配置失败: {e}")
        
        self._initialize_properties()

    def _getDirectoryPath(self):
        try:
            current_script_directory = os.path.dirname(os.path.realpath(__file__))
            parent_directory = os.path.abspath(os.path.join(current_script_directory, os.pardir))
            ic(parent_directory)
            path = os.path.join(parent_directory,self.configname)
            ic(path)
            return path
        except Exception as e:
            logging.error(f"获取路径时出错: {e}")
            raise

    def _initialize_properties(self):
        """初始化类的属性"""

        try:
            demo_param = self._read_file(self.path)
        except Exception as e:
            # 日志记录或其他错误处理机制
            logging.error(f"初始化失败: {e}")
            raise

        self.position_param = demo_param["position_param"]
        self.splindle_param = demo_param["splindle_param"]
        self.motor_param = demo_param["motor_param"]
 

    def _read_file(self,file_path)-> object:
        """读取并返回文件内容"""
        with open(file_path, "r") as j:
            data = json.load(j)
        return data

if __name__ == "__main__":
    rospy.init_node("command_node")
    # 实例化并运行各个命令发布类
    BasePlateCommand()
    SlidingTableCommand()
    SpindleMotorCommand()