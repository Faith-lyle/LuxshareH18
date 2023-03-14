#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
@Author  :   long.hou
@Email   :   long2.hou@luxshare-ict.com
@Ide     :   vscode & conda
@File    :   mainPanel.py
@Time    :   2023/02/18 09:07:28
"""

import os
import sys

from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal, QTimer
from PyQt5.QtGui import QColor, QPalette
from PyQt5.QtWidgets import QApplication, QMainWindow, QFormLayout, QLabel, QAbstractItemView, QHeaderView, \
    QTableWidgetItem, QPushButton, QFrame,QMessageBox
from GUI.UI.Ui_mainPanel import Ui_MainWindow


class MainPanel(QMainWindow, Ui_MainWindow):
    start_signal = pyqtSignal(str)
    close_signal = pyqtSignal(dict)

    def __init__(self, config):
        super(MainPanel, self).__init__()
        self.action_time = None
        self.config = config
        self.lb_result = QLabel("text")
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_timeout_func)
        self.action_timer = QTimer(self)
        self.action_timer.timeout.connect(self.action_timer_timeout_func)
        self.bt_clear = QPushButton("Clear")
        self.init_ui()
        self.signal_connect_slot()
        self.tabWidget.setCurrentIndex(0)
        self.frame.setVisible(False)
        self.verticalLayout_2.setStretch(1, 2)

    def init_ui(self):
        """
        初始化界面UI
        :return:
        """
        self.setupUi(self)
        self.resize(900, 600)
        self.lb_result.setObjectName('lb_result')
        self.bt_clear.setObjectName('bt_clear')
        self.lb_result.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
        self.lb_ct.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft)
        self.label_3.setAlignment(Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight)
        self.formLayout_2.setWidget(2, QFormLayout.SpanningRole, self.lb_result)
        self.formLayout.setWidget(4, QFormLayout.SpanningRole, self.bt_clear)
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
            self.lb_yield.setText("{:.2f}%".format(self.config['setting']['passQty'] / (
                    self.config['setting']['passQty'] + self.config['setting']['failQty']) * 100))

        self.table_view_init()
        self.frame.setStyleSheet("""
                *{
                font-size: 15pt;
                }
                .QPushButton{
                    min-width:80px;
                    min-height:25px;
                    border: 2px solid rgb(78,138,138);/*边框颜色值*/
                    border-radius: 9px;
                    margin-left: 10px;
                    margin-right: 10px;
                    background-color:rgb(78,138,138);
                    font-size: 15pt;
                }
                .QPushButton:hover{
                    border: 2px solid lightsalmon;/*边框颜色值*/
                    background-color:lightsalmon;
                }
                .QPushButton:pressed{
                    border: 2px solid darkcyan;/*边框颜色值*/
                    background-color:darkcyan;
                }
                """)

    def signal_connect_slot(self):
        self.bt_clear.clicked.connect(self.clear_count_qty)

    def timer_timeout_func(self):
        ct = float(self.lb_ct.text()) + 0.1
        self.lb_ct.setText('{:.1f}'.format(ct))

    def table_view_init(self):
        herder = ["NO", "testItem", "Upper", "Lower", "Value"]
        self.tableWidget.setColumnCount(5)
        self.tableWidget.setRowCount(len(self.config['testPlan']))
        # 设置行头不可见
        self.tableWidget.verticalHeader().setVisible(False)
        # 设置不可编辑
        self.tableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        # 设置只能选中行
        self.tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)
        # 设置列头
        self.tableWidget.setHorizontalHeaderLabels(herder)
        # 设置列宽自动调整
        # self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # 设置列宽根据文本内容调整
        # self.tableWidget.resizeColumnsToContents()
        # 设置每行的列宽
        for i, width in enumerate([60, 270, 100, 100, 100]):
            self.tableWidget.setColumnWidth(i, width)
        for x, items in enumerate(self.config['testPlan']):
            for y, text in enumerate(herder):
                try:
                    item = QTableWidgetItem(self.config['testPlan'][x][text])
                except KeyError:
                    item = QTableWidgetItem('')
                item.setTextAlignment(Qt.AlignCenter)
                self.tableWidget.setItem(x, y, item)
                # 设置单元格交叉颜色
                if x % 2 == 0:
                    item.setBackground(QColor('#E4E4E4'))
            # 设置行高为30
            self.tableWidget.setRowHeight(x, 30)

    def add_value(self, row, column, value, color):
        """
        添加数据到表格中
        :param row: 行号
        :param column: 列号
        :param value: 显示值
        :param color:  字体颜色
        :return:
        """
        Qitem = self.tableWidget.item(row, column)
        Qitem.setText(str(value))
        Qitem.setForeground(QColor(color))

    def tabel_select_row(self, row):
        """
        选择表格的某一行
        :param row: 行数
        :return: None
        """
        self.tableWidget.selectRow(row)

    def tabel_content_clear(self):
        """
        清除表格的测试数据，保留Item和upper、lower
        :return: None
        """
        for index in range(self.tableWidget.rowCount()):
            item = self.tableWidget.item(index, 4)
            item.setText('')

    def show_result(self, result):
        if result == 'PASS':
            self.timer.stop()
            self.lb_result.setText("PASS")
            self.lb_result.setStyleSheet('background-color:lime')
            self.pbt_start.setEnabled(True)
        elif result == "FAIL":
            self.timer.stop()
            self.lb_result.setStyleSheet("background-color: red;")
            self.lb_result.setText("FAIL")
            self.pbt_start.setEnabled(True)
        elif result == "TEST":
            self.lb_result.setStyleSheet("background-color: yellow;")
            self.lb_result.setText("TEST")
            self.pbt_start.setEnabled(False)

    def add_pass_qty(self, qty):
        self.config['setting']['passQty'] += qty
        self.lb_pass_qty.setText(str(self.config['setting']['passQty']))
        total = self.config['setting']['passQty'] + self.config['setting']['failQty']
        self.lb_yield.setText("{:.2f}".format(self.config['setting']['passQty'] / total * 100))

    def add_fail_qty(self, qty):
        self.config['setting']['failQty'] += qty
        self.lb_fail_qty.setText(str(self.config['setting']['failQty']))
        total = self.config['setting']['passQty'] + self.config['setting']['failQty']
        self.lb_yield.setText("{:.2f}%".format(self.config['setting']['passQty'] / total * 100))

    def clear_count_qty(self):
        self.config['setting']['failQty'] = 0
        self.config['setting']['passQty'] = 0
        self.lb_pass_qty.setText("0")
        self.lb_fail_qty.setText("0")
        self.lb_yield.setText("100%")

    def write(self, msg):
        self.textEdit.append(msg)

    @pyqtSlot()
    def on_pbt_start_clicked(self):
        self.start_signal.emit(self.ed_sn.text().strip())
        self.lb_ct.setText('0.0')
        self.timer.start(100)

    def setting_panel_init(self):

        self.verticalLayout_2.setStretch(2, 100)
        self.label.setVisible(False)
        self.label_2.setVisible(False)
        self.label_4.setVisible(False)
        self.frame.setVisible(True)
        self.verticalLayout_2.setStretch(1, 0)

    def setting_panel_close(self):
        self.horizontalLayout_2.setParent(self.verticalLayout_2)
        self.label.setVisible(True)
        self.label_2.setVisible(True)
        self.label_4.setVisible(True)
        self.frame.setVisible(False)

        self.verticalLayout_2.setStretch(0, 0)
        self.verticalLayout_2.setStretch(1, 20)
        self.verticalLayout_2.setStretch(2, 100)

    def action_timer_timeout_func(self):
        self.action_time += 1
        self.verticalLayout_2.setStretch(0, self.action_time)
        if self.action_time > 20:
            self.action_timer.stop()

    @pyqtSlot()
    def on_action_triggered(self):
        self.setting_panel_init()
        self.action_time = 0
        self.action_timer.start(10)

    @pyqtSlot()
    def on_pushButton_clicked(self):
        if self.lineEdit.text() == 'admin':
            self.setting_panel_close()
        else:
            QMessageBox.critical(None,'error','密码错误！')

    @pyqtSlot()
    def on_pushButton_3_clicked(self):
        self.setting_panel_close()

    def closeEvent(self, a0):
        self.close_signal.emit(self.config)
        super().closeEvent(a0)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = MainPanel()
    main.show()
    sys.exit(app.exec_())
