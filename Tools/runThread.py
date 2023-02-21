import time
from PyQt5.QtCore import QThread, pyqtSignal
from Tools.functions import TestFunctions, write_csv
from Tools.logHelper import LogHelper


class RunThread(QThread):
    test_value_signal = pyqtSignal(int, bool, str)
    test_result_signal = pyqtSignal(str)
    content_signal = pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)
        self.func = None
        self.log = None
        self.config_json = None

    def set_args(self, config=None, log=None):
        self.log = LogHelper(log)
        self.config_json = config
        self.func = TestFunctions(self.log)
        # print(self.log)

    def write(self,msg):
        self.content_signal.emit(msg)

    def run(self):
        self.log.show_infomation("START TEST, PARSE PARAMETERS SUCCESSFULLY!")
        self.log.show_infomation("Load Test Plan Successfully! Test Plan as below:")
        for item in self.config_json['testPlan']:
            self.log.show_infomation(str(item))
        self.main()

    def main(self):
        result_dict = {'StartTime': time.strftime("%Y-%m-%d %H:%M:%S"), 'Site': "ITJX",
                       "Product": "B698", 'UnitNumber': 'None',
                       "SerialNumber": "None", 'SlotNumber': 1, 'WorkStationNumber': 1,
                       'Station ID': self.config_json["setting"]['station_id'],
                       'Version': self.config_json["setting"]['sw_version']}
        fail_list = {}
        IsSkip = False
        # 测试部分
        for i, item in enumerate(self.config_json['testPlan']):
            try:
                if item['Mode'] == 'Special':
                    if IsSkip:
                        self.test_value_signal.emit(i, False, "--FAIL--")
                        # time.sleep(0.1)
                        break
                    result, value = self.func.choose_function(item['testItem'], *item['Input'])
                    self.test_value_signal.emit(i, result, value)
                    if result:
                        result_dict[item['testItem']] = value
                        result_dict['Test Pass/Fail Status'] = 'PASS'
                    else:
                        IsSkip = True
                        result_dict['Test Pass/Fail Status'] = 'FAIL'
                        result_dict[item['testItem']] = value
                        fail_list[item['testItem']] = 'Upper:NA,Lower:NA,Value:{}'.format(value)
                elif item['Mode'] == 'Init':
                    result, value = self.func.choose_function(item['testItem'], *item['Input'])
                    self.test_value_signal.emit(i, result, value)
                    if result:
                        self.test_value_signal.emit(i, "--PASS--")
                        result_dict[item['testItem']] = "PASS"
                    else:
                        result_dict[item['testItem']] = "FAIL"
                        fail_list[item['testItem']] = 'Upper:NA,Lower:NA,Value:FAIL'
                time.sleep(0.1)
                i += 1
            except Exception as msg:
                self.log.show_error("Test Item:" + item['testItem'] + "\nError Message:" + str(msg))
                result_dict[item['testItem']] = "FAIL"
                result_dict['Test Pass/Fail Status'] = 'FAIL'
                self.test_value_signal.emit(i, False, "--FAIL--")
                continue
        # 测试结束，发送退出信号
        for _ in range(3):
            try:
                write_csv('{}/{}.csv'.format(self.config_json['setting']['LogPath'], time.strftime("%Y-%m-%d")),
                          result_dict, self.config_json['testPlan'])
                break
            except Exception as e:
                self.log.show_error("Write CSV Error:" + "\nError Message:{}".format(e))
                time.sleep(0.05)
        if result_dict['Test Pass/Fail Status'] == 'FAIL':
            self.test_result_signal.emit('FAIL')
        else:
            self.test_result_signal.emit('PASS')
