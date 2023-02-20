#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@Author  :   long.hou
@Email   :   long2.hou@luxshare-ict.com
@Ide     :   vscode & conda
@File    :   mainPanel.py
@Time    :   2023/02/18 09:07:28
"""

import sys, os
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QMainWindow, QFormLayout, QLabel
from PyQt5.QtCore import Qt
from GUI.UI.Ui_mainPanel import Ui_MainWindow


class MainPanel(QMainWindow, Ui_MainWindow):

    def __init__(self, config):
        super(MainPanel, self).__init__()
        self.config = config
        self.lb_result = QLabel("text")
        self.init_ui()

    def init_ui(self):
        """
        初始化界面UI
        :return:
        """
        self.setupUi(self)
        self.resize(900, 600)
        self.lb_result.setObjectName('lb_result')
        self.lb_result.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        self.lb_ct.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
        self.label_3.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight)
        self.formLayout_2.setWidget(2, QFormLayout.SpanningRole, self.lb_result)
        with open(os.path.join(os.path.dirname(__file__), "style.css"), 'r') as f:
            style = f.read()
        self.setStyleSheet(style)
        self.setWindowTitle(self.config['setting']['Title'])
        self.label.setText(self.config['setting']['Title'])
        self.lb_ct.setText('0.0')
        self.lb_result.setText('Wait')
        self.lb_pass_qty.setText(f"{self.config['setting']['passQty']}")
        self.lb_fail_qty.setText(f"{self.config['setting']['failQty']}")
        if self.config['setting']['passQty'] + self.config['setting']['failQty'] == 0:
            self.lb_yield.setText("100%")
        else:
            self.lb_yield.setText("{:.2f}%".format(self.config['setting']['failQty'] / (
                        self.config['setting']['passQty'] + self.config['setting']['failQty']) * 100))

    def add_pass_qty(self, qty):
        self.lb_pass_qty.setText()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = MainPanel()
    main.show()
    sys.exit(app.exec_())
