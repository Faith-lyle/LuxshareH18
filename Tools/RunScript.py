#!/usr/bin/python3
# -- coding: utf-8 --
# @Author : Long.Hou
# @Email : Long2.Hou@luxshare-ict.com
# @File : RunScript.py
# @Project : TestPlatform
# @Time : 2022/12/2 04:03
# -------------------------------
import argparse
import csv
import json
import os
import sys, socket
import time

BASE_PATH = os.path.dirname(os.path.abspath(sys.argv[0]))
sys.path.append(BASE_PATH + '/lib')
import logDriver, functions


def read_csv(file):
    with open(file, 'r') as f:
        datas = []
        reader = csv.reader(f)
        reader = list(reader)
        header = reader[0]
        for row in reader[1:]:
            data = {}
            for i in range(len(header)):
                data[header[i]] = row[i]
            datas.append(data)
        return datas


def read_json(file):
    with open(file, 'r') as f:
        data = json.load(f)
        return data


def get_argument():
    Parser = argparse.ArgumentParser()
    Parser.add_argument("-p", "--serialPort", help="serialPort")
    Parser.add_argument("-f", "--logFile", help="logFile")
    Parser.add_argument("-cs", "--CSVFile", help="CSVFile")
    arg = Parser.parse_args()
    return arg


def create_socket():
    client = None
    # 创建socket对象
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # 创建socket对象
        client.connect(('localhost', 9011))
        # log.show_information("Connect Socket Success, IP: 127.0.0.1, Port: 9011")
    except Exception as msg:
        log.show_error("Connect Socket Failed. Error Message: " + str(msg))
        exit()
    return client


def main():
    client = create_socket()
    # text = func.dispose_data(BasePath + '/content', slot)
    # log.show_information("get test content successfully!")
    # log.show_information("text content as below:\n{}".format(text))
    text = {}
    result_dict = {'StartTime': time.strftime("%Y-%m-%d %H:%M:%S"), 'Site': "ITJX",
                   "Product": "B698", 'UnitNumber': 'None',
                   "SerialNumber": "None", 'SlotNumber': 1, 'WorkStationNumber': 1,
                   'Station ID': config_json["Setting"]['station_id'],
                   'Version': config_json["Setting"]['sw_version']}
    fail_list = {}
    IsSkip = False
    # 测试部分
    for i, item in enumerate(test_plan):
        try:
            if item['Mode'] == 'Special':
                if IsSkip:
                    client.send("<INFO>::{}::{}\n".format(i + 1, "--FAIL--").encode())
                    time.sleep(0.1)
                    continue
                result, value = func.choose_function(item['Item'], *item['Input'])
                if result:
                    client.send("<INFO>::{}::{}\n".format(i + 1, value).encode())
                    result_dict[item['Item']] = "PASS"
                    result_dict['Test Pass/Fail Status'] = 'PASS'
                else:
                    IsSkip = True
                    client.send("<INFO>::{}::{}\n".format(i + 1, "--FAIL--").encode())
                    result_dict['Test Pass/Fail Status'] = 'FAIL'
                    fail_list[item['Item']] = 'Upper:NA,Lower:NA,Value:FAIL'
            elif item['Mode'] == 'Init':
                result, value = func.choose_function(item['Item'],*item['Input'])
                if result:
                    client.send("<INFO>::{}::{}\n".format(i + 1, value).encode())
                    result_dict[item['Item']] = "PASS"
                else:
                    client.send("<INFO>::{}::{}\n".format(i + 1, "--FAIL--").encode())
                    result_dict[item['Item']] = "FAIL"
                    fail_list[item['Item']] = 'Upper:NA,Lower:NA,Value:FAIL'
            time.sleep(0.2)
            i += 1
        except Exception as msg:
            log.show_error("Test Item:" + item['Item'] + "\nError Message:" + str(msg))
            result_dict['Test Pass/Fail Status'] = 'FAIL'
            client.send("<INFO>::{}::{}\n".format(i + 1, "--FAIL--").encode())
            continue
    # 测试结束，发送退出信号
    for _ in range(3):
        try:
            functions.write_csv('{}/{}.csv'.format(log_dir, time.strftime("%Y-%m-%d")), result_dict, test_plan)
            break
        except Exception as e:
            log.show_error("Write CSV Error:" + "\nError Message:{}".format(e))
            time.sleep(0.05)
    if result_dict['Test Pass/Fail Status'] == 'FAIL':
        client.send("<EXIT>::0::FAIL\n".encode())
    else:
        client.send("<EXIT>::0::PASS\n".encode())


if __name__ == '__main__':
    config_file = os.path.join(os.path.dirname(BASE_PATH), 'config', 'init.json')
    config_json = read_json(config_file)
    log_dir = "{}/{}".format(config_json['Setting']['LogPath'], time.strftime("%Y-%m-%d"))
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    log = logDriver.LogDriver("{}/{}.log".format(log_dir, time.strftime("%H-%M-%S")),mode='w')
    log.show_information("Run Script START, INPUT PARAMETERS SUCCESSFULLY!")
    log.show_information("logDriver init successfully!")
    test_plan = config_json['TestItems']
    for items in test_plan:
        log.show_information(items)
    log.show_information("Read config file successfully!")
    func = functions.TestFunctions(log)
    try:
        main()
        # os.remove(BasePath + '/content')
    except Exception as e:
        log.show_error("Main Error:" + str(e))
        log.show_error("Run Script END!")
        exit()
