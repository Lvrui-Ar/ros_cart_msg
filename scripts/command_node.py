import rospy
import os
import logging
import json
from icecream import ic
from ros_ht_msg.msg import ht_control
from ros_modbus_msg.msg import operation
from ros_modbus_msg.msg import spindle_argument

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


# 移动指令发布：
class BasePlateCommand:
    def __init__(self):
        self.pub_BasePlate = rospy.Publisher("HT_Control", ht_control, queue_size=10)
        self.param = ReadParam()
        self.move()
    
    def move(self):
        self.Commandsetting(self.param.position_param,self.param.position_param.keys())

    def Commandsetting(self, paramdict,keylist:list):
        for key in keylist:
            moveparam = paramdict[key][1:]
            ic(moveparam)
            ic(paramdict[key][0])
            for t in range(paramdict[key][0]):
                self._dopub(self._BasePlateCommand(moveparam))

    def _BasePlateCommand(self,moveparam:list):
        control = ht_control()
        control.mode = moveparam[0]
        control.x = moveparam[1]
        control.y = moveparam[2]
        control.z = moveparam[3]
        return control

    def _dopub(self,control):
        try:
            self.pub_BasePlate.publish(control)
        except Exception as e:
            rospy.logerr(f"发布指令失败: {str(e)}")
        rospy.sleep(1)
        ic("sleep")

# 滑台指令发布：
class SlidingTableCommand:
    def __init__(self):
        self.pub_SlidingTable = rospy.Publisher("sub_param", operation, queue_size=10)
        self.param = ReadParam()
        self.move()
    
    def move(self):
        self.CommandSetting(self.param.splindle_param,self.param.splindle_param.keys())

    def CommandSetting(self,paramdict,keylist:list):
        for key in keylist:
            moveparam = paramdict[key]
            ic(moveparam)
            self._dopub(self._SlidingTableCommand(moveparam))

    def _SlidingTableCommand(self,moveparam:list):
        oper = operation()
        oper.operation = moveparam[0]
        oper.A_x = moveparam[1]
        oper.A_y = moveparam[3]
        oper.A_z = moveparam[5]
        oper.x_speed = moveparam[2]
        oper.y_speed = moveparam[4]
        oper.z_speed = moveparam[6]

        ic(oper)
        return oper
    
    def _dopub(self,oper):
        try:
            self.pub_SlidingTable.publish(oper)
        except Exception as e:
            rospy.logerr(f"发布指令失败: {str(e)}")
        rospy.sleep(15)

# 主轴电机启动：
class SpindleMotorCommand:
    def __init__(self):
        self.pub_SpindleMotor = rospy.Publisher("spindle_motor", spindle_argument, queue_size=10)
        self.param = ReadParam()        

        self.spindlemotor_list = self._spindleArgument(self.param.motor_param.keys())   

        self.dopub(self._CommandSetting(self.spindlemotor_list))

    def dopub(self,command_list):
        try:
            self.pub_SpindleMotor.publish(command_list[0])
            ic(1)
            rospy.sleep(command_list[2])
            ic("wait")
            self.pub_SpindleMotor.publish(command_list[1])
            ic(2)

        except Exception as e:
            rospy.logerr(f"发布指令失败: {str(e)}")
        
    def _CommandSetting(self,spindlemotorlist):
        start_command = self._SpindleMotorCommand([spindlemotorlist[1],spindlemotorlist[2]])
        end_command = self._SpindleMotorCommand([spindlemotorlist[1],spindlemotorlist[3]])
        time  = self.param.motor_param["time"]
        return [start_command, end_command, time]

    def _SpindleMotorCommand(self,moveparam:list):
        spindle = spindle_argument()
        spindle.oper = 1
        spindle.address = moveparam[0]
        spindle.value = moveparam[1]
        spindle.num_reg = 1
        return spindle
        
    def _spindleArgument(self,keys):
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
    # BasePlateCommand()
    # SlidingTableCommand()
    SpindleMotorCommand()

    



    