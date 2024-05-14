import os
import logging
import json
from icecream import ic

# 配置日志
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

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




