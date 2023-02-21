import datetime
import logging


class LogHelper:

    def __init__(self, log):
        self._log = log
        self.index = 1
        self.startTime = None

    def set_file_log_path(self, file_path):
        for hand in self._log.handlers:
            if type(hand) == logging.FileHandler:
                self._log.removeHandler(hand)
        file_log = logging.FileHandler(file_path, encoding='utf-8')
        formatter = logging.Formatter(
            '%(asctime)s >> (pid: %(process)s,tid: %(thread)s <%(filename)s [line:%(lineno)d]> | %(levelname)s: %('
            'message)s ')
        file_log.setFormatter(formatter)
        self._log.addHandler(file_log)

    def mes_log(self, func_name, url, data, response):
        self._log.info("{}\nFunction:{}\nRequest URL:{}\nRequest Method: POST\nRequest Date:{}\n"
                       "Response Status Code:{}\nResponse Text:{}\n".format('-' * 20, func_name, url, data,
                                                                            response.status_code, response.text))

    def mes_error_log(self, func_name, url, data, error):
        self._log.error(
            "{}\nFunction:{}\nRequest URL:{}\nRequest Method: POST\nRequest Date:{}\nError information:{}\n".format(
                '-' * 20, func_name, url, data, error))

    def send_log(self, msg):
        self._log.info("Send Cmd: " + msg)

    def send_error(self, msg):
        self._log.info("Send Error: " + msg)

    def receive_log(self, msg):
        self._log.info("Receive Content:\n " + msg)

    def set_item_result(self, value, result):
        self._log.info("Get Value: {}".format(value))
        self._log.info('Test Result: {}'.format(result))

    def set_item_error(self,msg):
        self._log.error("Error Message: {}".format(msg))
        self._log.error('Test Result: FAIL')

    def item_end(self, item):
        EndTime = datetime.datetime.now().__sub__(self.startTime)
        msg = "Elapsed Seconds:{}".format(EndTime.microseconds / 1000000 + EndTime.seconds)
        self._log.info(msg)
        self._log.info('Step{}  "{}"  End   <------------------------------\n'.format(self.index, item))
        self.index += 1

    def item_start(self, item):
        self.startTime = datetime.datetime.now()
        msg = 'Step{}  "{}" Start   ------------------------------>'.format(self.index, item)
        self._log.info(msg)

    def system_reset_start(self):
        self.startTime = datetime.datetime.now()
        msg = 'Step{}  "System Reset" Start   ------------------------------>'.format(self.index)
        self._log.info(msg)

    def system_reset_end(self):
        EndTime = datetime.datetime.now().__sub__(self.startTime)
        msg = "Elapsed milliSeconds:{}".format(EndTime.microseconds / 1000000 + EndTime.seconds)
        self._log.info(msg)
        self._log.info('Step{}  "System Reset"  End   <------------------------------\n'.format(self.index))
        self.index += 1

    def show_infomation(self,msg):
        self._log.info(msg)

    def show_error(self,msg):
        self._log.error(msg)
