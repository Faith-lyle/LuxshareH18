import datetime
import logging
import os
from os import path

now = datetime.datetime.now()  # 获取当前时间
otherStyleTime = now.strftime("%Y-%m-%d")  # "%Y-%m-%d-%H-%M-%S"
user_path = f"{path.expanduser('~')}/logs"  # 获取用户路径
os.makedirs(user_path, exist_ok=True)  # 获取用户 logs 文件夹  如果不存在则创建文件夹
log_path = f"{user_path}/{otherStyleTime}.log"  # 以当前日期创建.log日志


class ConsolePanelHandler(logging.Handler):

    def __init__(self, parent):
        logging.Handler.__init__(self)
        self.parent = parent

    def emit(self, record):
        """输出格式可以按照自己的意思定义HTML格式"""
        record_dict = record.__dict__
        asctime = record_dict['asctime'] + " >> "
        line = record_dict['filename'] + " -> line:" + str(record_dict['lineno']) + " | "
        pid = f"(pid: {+record_dict['process']},tid: {record_dict['thread']})"
        levelname = record_dict['levelname']
        message = record_dict['message'].replace('\n','<br>')
        if levelname == 'ERROR':
            color = "#FF0000"
        elif levelname == 'WARNING':
            color = "#FFD700"
        else:
            color = "#008000"
        html = f'''
        <div >
            <span>{asctime}</span>
            <span style="color:#4e4848;">{line}</span>
            <span style="color: {color};">{levelname}</span>
            <span style="color:	#696969;">: {message}</span>
        </div>
        '''
        self.parent.write(html)  # 将日志信息传给父类 write 函数 需要在父类定义一个函数


class Log:
    def __init__(self, ):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(level=20)
        file_log = logging.FileHandler(log_path, encoding='utf-8')
        formatter = logging.Formatter(
            '%(asctime)s >> (pid: %(process)s,tid: %(thread)s (%(filename)s[line:%(lineno)d]) | %(levelname)s: %('
            'message)s - ')
        file_log.setFormatter(formatter)
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(file_log)
        self.logger.addHandler(stream_handler)

    def get_log(self):
        return self.logger


logger = Log().get_log()  # 全局能访问


def set_file_log_path(file_path):
    for hand in logger.handlers:
        if type(hand) == logging.FileHandler:
            logger.removeHandler(hand)
    file_log = logging.FileHandler(file_path, encoding='utf-8')
    formatter = logging.Formatter(
        '%(asctime)s >> (pid: %(process)s,tid: %(thread)s <%(filename)s [line:%(lineno)d]> | %(levelname)s: %('
        'message)s ')
    file_log.setFormatter(formatter)
    logger.addHandler(file_log)
