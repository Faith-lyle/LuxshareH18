#!/usr/bin/env python
# -*- encoding: utf-8 -*-


"""
@Author  :   long.hou
@Email   :   long2.hou@luxshare-ict.com
@Ide     :   vscode & conda
@File    :   mian.py
@Time    :   2023/02/16 16:21:54
"""
import os.path
import sys
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from GUI.mainPanel import MainPanel
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    config_data = read_config(os.path.join(os.path.dirname(__file__), "Config/config.json"))
    main = MainPanel(config_data)
    main.show()
    sys.exit(app.exec_())
