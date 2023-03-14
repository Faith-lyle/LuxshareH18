#!/usr/bin/python3
# -- coding: utf-8 --
# @Author : Long.Hou
# @Email : Long2.Hou@luxshare-ict.com
# @File : functions.py
# @Project : SIP-Tester3_1
# @Time : 2022/6/16 14:16
# -------------------------------
import csv
import glob
import inspect
import os.path
import re
import subprocess
import time
from threading import Timer
from Tools import serialPort

BASE_PATH = os.path.dirname(__file__)


def write_csv(file_name, result_dict, test_plan):
    exist = True
    if not os.path.exists(file_name):
        exist = False
    herder = ['Site', 'Product', 'SerialNumber', 'UnitNumber', 'WorkStationNumber', 'SlotNumber', 'Station ID',
              'Test Pass/Fail Status',
              'StartTime', 'EndTime',
              'Version', 'List of Failing Tests']
    if not exist:
        herder0 = ['']
        herder1 = ['Upper Limit -->', '', '', '', '', '', '', '', '', '', '', '']
        herder2 = ['Lower Limit -->', '', '', '', '', '', '', '', '', '', '', '']
        herder3 = ['Measurement Unit -->', '', '', '', '', '', '', '', '', '', '', '']
    for item in test_plan:
        herder.append(item['testItem'])
        if not exist:
            herder0.append(' ')
            herder1.append(item['Upper'])
            herder2.append(item['Lower'])
            herder3.append(item['Unit'])
    if not exist:
        with open(file_name, 'w') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(herder)
            writer.writerow(herder0)
            writer.writerow(herder1)
            writer.writerow(herder2)
            writer.writerow(herder3)
    data = []
    for test_item in herder:
        if test_item in result_dict.keys():
            data.append(result_dict[test_item])
        else:
            data.append('')
    with open(file_name, 'a') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(data)


def RunShellWithTimeout(cmd, timeout=3):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    timer = Timer(timeout, lambda process: process.kill(), [p])  # 设置定时器计算超时时间，超时杀死此线程
    try:  # 尝试运行，出错不停止
        timer.start()
        stdout, stderr = p.communicate()  # 读取终端返回的内容，stdout为返回正常内容。stderr为出错内容
        return_code = p.returncode  # 获取终端返回值，正常返回值为0，异常为其它
        return return_code, stdout, stderr
    finally:
        timer.cancel()


class TestFunctions():
    def __init__(self, log):
        self.log = log
        self.term = None

    def open_serial_port(self, *args):
        self.log.item_start("Open Serial Port")
        result = False, "--FAIL--"
        try:
            p = glob.glob("/dev/cu.kis-*-ch-2")
            if len(args) == 1:
                self.term = serialPort.SerialPort(args[0], log_driver=self.log)
            else:
                self.term = serialPort.SerialPort(p[0], log_driver=self.log)
            time.sleep(0.5)
            self.term.flush_out()
            result = True, "--PASS--"
            self.log.set_item_result("Port:{} Baudrate:921600".format(args[0] if len(args) ==1 else p[0]), "PASS")
        except Exception as e:
            self.log.set_item_error(str(e))
        self.log.item_end("Open Serial ort")
        return result

    def close_serial_port(self, *args):
        self.log.item_start("Close Serial Port")
        result = False, '--FAIL--'
        output = ''
        if self.term:
            try:
                self.term.flush_out()
                time.sleep(0.5)
                self.term.close_port()
                output += "Close Successful\n"
                result = True, '--PASS--'
                self.log.set_item_result(output, "PASS")
            except Exception as e:
                output += str(e)
                self.log.set_item_error(output)
        self.log.item_end("Close Serial Port")
        return result

    def create_grassninja_port(self, *args):
        self.log.item_start("Create Port")
        result = False, "--FAIL--"
        output = ''
        # 调用终端执行cmd并获取输出内容
        try:
            if len(args) == 2:
                cmd = "/usr/local/bin/python3 {}/GN/grassninja.py -p {} -s on".format(BASE_PATH, args[0])
            else:
                p = glob.glob("/dev/cu.usbserial*0")
                cmd = "/usr/local/bin/python3 {}/GN/grassninja.py -p {} -s on".format(BASE_PATH, p[0])
            output += "Run Cmd : {}\n".format(cmd)
            for _ in range(2):
                return_code, stdout, stderr = RunShellWithTimeout(cmd, 10)
                output += "return_code:{}\n".format(return_code)
                output += "stdout:{}\n".format(stdout.decode())
                if return_code != 0:
                    # output += "stdout:{}\n".format(stdout.decode())
                    output += "stderr:{}\n".format(stderr.decode())
                    continue
                output += "Serial Port:\n"
                stime = time.time()
                while 1:
                    if time.time() - stime > 5:
                        result = False, "--FAIL--"
                        break
                    port_list = glob.glob("/dev/cu.kis-*-ch-2")
                    if len(args) == 2:
                        if args[1] in port_list:
                            result = True, "--PASS--"
                            break
                    else:
                        if len(port_list) == 1:
                            result = True, "--PASS--"
                            break
                    time.sleep(0.5)
                for i in glob.glob("/dev/cu.kis-*-ch-2"):
                    output += i + "\n"
                output += "Target Serial Port:{}\n".format(args[1] if len(args) == 2 else "Auto Choose")
                if result[0]:
                    break
        except Exception as e:
            self.log.set_item_error(str(e))
        self.log.set_item_result(output, "--PASS--" if result[0] else "--FAIL--")
        self.log.item_end('Create Serial Port')
        return result

    def close_grassninja(self, *args):
        self.log.item_start("Close GrassNinja")
        time.sleep(0.5)
        result = False, "FAIL"
        output = ''
        try:
            if len(args) == 1:
                cmd = "/usr/local/bin/python3 {}/GN/gn_powerCycle.py -p {}".format(BASE_PATH, args[0])
            else:
                p = glob.glob("/dev/cu.usbserial*0")[0]
                cmd = "/usr/local/bin/python3 {}/GN/gn_powerCycle.py -p {}".format(BASE_PATH, p)
            for i in range(3):
                output += "cmd:{}\n".format(cmd)
                return_code, stdout, stderr = RunShellWithTimeout(cmd, 30)
                output += "return_code:{}\n".format(return_code)
                output += "stdout:{}\n".format(stdout.decode())
                output += "stderr:{}\n".format(stderr.decode())
                if 'ValueError' not in stderr.decode():
                    break
                else:
                    time.sleep(0.5)
            self.log.set_item_result(output, "PASS")
            result = True, "PASS"
        except Exception as e:
            output += str(e)
            self.log.set_item_error(output)
        self.log.item_end("Close GrassNinja")
        return result

    def read_product_mlb(self, *args):
        self.log.item_start("Read Product MLB")
        result = False, "FAIL"
        output = ''
        try:
            for i in range(3):
                output += self.term.send_and_read_until("syscfg print MLB#\n", timeout=1)
                if '> syscfg:ok' in output:
                    flag = re.findall(r'syscfg:ok "(.*)?"', output)
                    if len(flag) == 1:
                        result = True, flag[0]
                        self.log.set_item_result(flag[0], "PASS")
                        break
                else:
                    self.term.flush_out()
                    self.log.set_item_result("Read MLB# Failed", "FAIL")
        except Exception as e:
            output += str(e)
            self.log.set_item_error(output)
        self.log.item_end("Read Product MLB")
        return result

    # 读取产品FW
    def read_product_fw(self, *args):
        self.log.item_start("Read Product FW")
        result = False, "FAIL"
        output = ''
        try:
            for i in range(3):
                output += self.term.send_and_read_until("ft version\n", timeout=2)
                if '> ft:ok' in output:
                    flag = re.findall(r'ft:ok (.*)?\n', output)
                    if len(flag) == 1:
                        result = True, flag[0]
                        self.log.set_item_result(flag[0], "PASS")
                        break
                else:
                    self.term.flush_out()
                    self.log.set_item_result("None", "FAIL")
        except Exception as e:
            output += str(e)
            self.log.set_item_error(output)
        self.log.item_end("Read Product FW")
        return result

    def mic1_init(self, *args):
        self.log.item_start("MIC1 Init")
        result = False, "FAIL"
        fail_flag = False
        output = ''
        mic1_cmd = [
            'allen configure zor\n',
            'audio config mic1 memory record 16kHz 768kHz 10\n',
            'audio start 0\n',
            'audio stop\n']
        try:
            for cmd in mic1_cmd:
                text = self.term.send_and_read_until(cmd, timeout=2)
                output += text
                if 'ok' in text:
                    result = True, "PASS"
                    self.log.set_item_result("ok", "PASS")
                else:
                    fail_flag = True
                    self.log.set_item_result("None", "FAIL")
                self.term.flush_out()
        except Exception as e:
            fail_flag = True
            output += str(e)
            self.log.set_item_error(output)
        if fail_flag:
            result = False, "FAIL"
        self.log.item_end("MIC1 Init")
        return result

    def mic2_init(self, *args):
        self.log.item_start("MIC2 Init")
        result = False, "FAIL"
        fail_flag = False
        output = ''
        mic2_cmd = [
            'allen configure zor\n',
            'audio config mic2 memory record 16kHz 768kHz 10\n',
            'audio start 0\n',
            'audio stop\n']
        try:
            for cmd in mic2_cmd:
                text = self.term.send_and_read_until(cmd, timeout=2)
                output += text
                if 'ok' in text:
                    result = True, "PASS"
                    self.log.set_item_result("ok", "PASS")
                else:
                    fail_flag = True
                    self.log.set_item_result("None", "FAIL")
                self.term.flush_out()
        except Exception as e:
            fail_flag = True
            output += str(e)
            self.log.set_item_error(output)
        if fail_flag:
            result = False, "FAIL"
        self.log.item_end("MIC2 Init")
        return result

    def mic1_test(self, *args):
        self.log.item_start("MIC1 Test")
        result = False, "FAIL"
        output = ''
        value = 0
        try:
            output += self.term.send_and_read_until("audio dump\n", timeout=5)
            if 'audio:ok' in output:
                results = output.split(":")
                for res in results[-41:-1]:
                    for a in range(3):
                        ok1 = res[a * 16: (a + 1) * 16]
                        ok2 = res[(a + 1) * 16:(a + 2) * 16]
                        if ok1 == ok2:
                            value += 1
                if value < 80:
                    result = True, "PASS"
                    self.log.set_item_result(str(value), "PASS")
                else:
                    self.log.set_item_result(str(value), "FAIL")
            else:
                self.log.set_item_result("None", "FAIL")
        except Exception as e:
            output += str(e)
            self.log.set_item_error(output)
        self.log.item_end("MIC1 Test")
        return result

    def mic2_test(self, *args):
        self.log.item_start("MIC2 Test")
        result = False, "FAIL"
        output = ''
        value = 0
        try:
            output += self.term.send_and_read_until("audio dump\n", timeout=5)
            if 'audio:ok' in output:
                results = output.split(":")
                for res in results[-41:-1]:
                    for a in range(3):
                        ok1 = res[a * 16: (a + 1) * 16]
                        ok2 = res[(a + 1) * 16:(a + 2) * 16]
                        if ok1 == ok2:
                            value += 1
                if value < 80:
                    result = True, "PASS"
                    self.log.set_item_result(str(value), "PASS")
                else:
                    self.log.set_item_result(str(value), "FAIL")
            else:
                self.log.set_item_result("None", "FAIL")
        except Exception as e:
            output += str(e)
            self.log.set_item_error(output)
        self.log.item_end("MIC2 Test")
        return result

    def dut_info(self, *args):
        self.log.item_start("DUT_info")
        result = False, "FAIL"
        output = ''
        try:
            for i in range(3):
                output += self.term.send_and_read_until("ft uvp disable\n", timeout=1)
                output += self.term.send_and_read_until("ft mw32 0x42348008 0x00\n", timeout=1)
                if 'ft:ok' in output:
                    result = True, "PASS"
                    self.log.set_item_result("PASS", "PASS")
                    break
                else:
                    self.term.flush_out()
                    self.log.set_item_result("DUT_info", "FAIL")
        except Exception as e:
            result = False, "FAIL"
            self.log.set_item_error(output)
        self.log.item_end("DUT_info")
        return result

    def dut_shutdown(self, *args):
        self.log.item_start("DUT_shutdown")
        result = True, "PASS"
        output = ''
        try:
            for i in range(1):
                output += self.term.send_and_read("sysmgr shutdown core\n", timeout=0.1)
                if "reset: ok" in output:
                    self.log.set_item_result(output, "PASS")
                    break
                else:
                    self.term.flush_out()
                    self.log.set_item_result(output, "PASS")
        except Exception as e:
            result = False, "FAIL"
            self.log.set_item_error(output)
        self.log.item_end("DUT_shutdown")
        return result

    def choose_function(self, func_name, *args, **kwargs):
        func_list = [i[1] for i in inspect.getmembers(TestFunctions, inspect.isfunction)]
        func_name_list = [i[1].__name__ for i in inspect.getmembers(TestFunctions, inspect.isfunction)]
        if func_name in func_name_list:
            self.log.show_infomation(str(*args))
            return func_list[func_name_list.index(func_name)](self, *args, **kwargs)

