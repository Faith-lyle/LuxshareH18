#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@Author  :   long.hou
@Email   :   long2.hou@luxshare-ict.com
@Ide     :   vscode & conda
@File    :   main.py
@Time    :   2023/02/16 16:21:54
"""
import os.path
import sys
import time

from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from GUI.mainPanel import MainPanel
from Tools.logger import logger, ConsolePanelHandler, set_file_log_path
from Tools.runThread import RunThread
import json


# read json config file
def read_config(file_path):
    data = None
    if os.path.exists(file_path):
        with open(file_path) as f:
            data = json.load(f)
    else:
        QMessageBox.information(None, "Error", "Failed to open file, Please check if file is exist or not.")
        sys.exit()
    return data


def response_main_start_signal_slot(sn):
    global run_thread
    main.tabel_content_clear()
    main.show_result("TEST")
    main.textEdit.clear()
    if not sn:
        sn = time.strftime("%H-%M-%S")
    dir_path = os.path.join(config_data['setting']['LogPath'], time.strftime("%Y-%m-%d"))
    if not os.path.exists(dir_path):
        os.mkdir(dir_path)
    # run_thread = RunThread(parent=main)
    set_file_log_path('{}/{}.log'.format(dir_path, sn))
    run_thread.set_args(config=config_data, log=logger)
    run_thread.start()


def response_thread_test_value_signal_slot(index, result, value):
    color = 'lime' if result else 'red'
    main.add_value(index, 4, value, color)


def response_thread_test_result_signal_slot(value):
    main.show_result(value)
    if value == 'PASS':
        main.add_pass_qty(1)
    else:
        main.add_fail_qty(1)


def response_main_close_signal_slot(data):
    with open(os.path.join(os.path.dirname(__file__), "Config/config.json"),'w') as f:
        json.dump(data, f, ensure_ascii=False,indent=4)


def signal_connect_slot():
    main.start_signal.connect(response_main_start_signal_slot)
    run_thread.test_value_signal.connect(response_thread_test_value_signal_slot)
    run_thread.test_result_signal.connect(response_thread_test_result_signal_slot)
    run_thread.content_signal.connect(main.write)
    main.close_signal.connect(response_main_close_signal_slot)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    config_data = read_config(os.path.join(os.path.dirname(__file__), "Config/config.json"))
    main = MainPanel(config_data)
    run_thread = RunThread(parent=main)
    handler = ConsolePanelHandler(run_thread)
    logger.addHandler(handler)
    signal_connect_slot()
    main.show()
    sys.exit(app.exec_())
