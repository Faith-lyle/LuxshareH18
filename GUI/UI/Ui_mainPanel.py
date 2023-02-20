# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainPanel.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(818, 604)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setContentsMargins(-1, -1, 20, -1)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setStyleSheet("font: 75 24pt \"Helvetica\";\n"
"color: rgb(255, 16, 0);")
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.verticalLayout.addWidget(self.label)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setStyleSheet("font: 75 13\n"
"pt \"Helvetica\";")
        self.label_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_2.setObjectName("label_2")
        self.verticalLayout.addWidget(self.label_2)
        self.verticalLayout_2.addLayout(self.verticalLayout)
        self.horizontalFrame = QtWidgets.QFrame(self.centralwidget)
        self.horizontalFrame.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.horizontalFrame.setObjectName("horizontalFrame")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalFrame)
        self.horizontalLayout.setContentsMargins(0, 0, -1, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalFrame_2 = QtWidgets.QFrame(self.horizontalFrame)
        self.verticalFrame_2.setStyleSheet("background-color: rgb(220, 220, 220);")
        self.verticalFrame_2.setFrameShape(QtWidgets.QFrame.Box)
        self.verticalFrame_2.setObjectName("verticalFrame_2")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.verticalFrame_2)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.tabWidget = QtWidgets.QTabWidget(self.verticalFrame_2)
        self.tabWidget.setStyleSheet("QTabWidget::pane{\n"
"    background:rgb(246, 246, 246);\n"
"    border-top-color:transparent;\n"
"margin:0px;\n"
"}\n"
"QTabWidget::tab-bar{\n"
"    background:rgb(0, 0, 0);\n"
"    subcontrol-position:center;\n"
"}\n"
"QTabBar::tab{\n"
"    width:173px;/*宽度根据实际需要进行调整*/\n"
"    height:20px;\n"
"    background:rgb(210, 210, 210);\n"
"    border: 1px solid rgb(210, 210, 210);\n"
"    border-top-left-radius: 8px;\n"
"    border-top-right-radius: 8px;\n"
"}\n"
"QTabBar::tab:selected{    \n"
"    background:rgb(255, 255, 255);\n"
"    border-bottom-color:rgb(246, 246, 246);\n"
"}\n"
"QTabBar::tab:!selected{\n"
"    background:qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:0; stop:0 rgba(240, 240, 240, 255), stop:0.5 rgba(210, 210, 210, 255), stop:1 rgba(225, 225, 225, 255));\n"
"}\n"
" ")
        self.tabWidget.setObjectName("tabWidget")
        self.tab = QtWidgets.QWidget()
        self.tab.setObjectName("tab")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.tab)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_7.setSpacing(0)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.tableView = QtWidgets.QTableView(self.tab)
        self.tableView.setStyleSheet("")
        self.tableView.setObjectName("tableView")
        self.verticalLayout_7.addWidget(self.tableView)
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QtWidgets.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.verticalLayout_8 = QtWidgets.QVBoxLayout(self.tab_2)
        self.verticalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_8.setSpacing(0)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.textEdit = QtWidgets.QTextEdit(self.tab_2)
        self.textEdit.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.textEdit.setReadOnly(True)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout_8.addWidget(self.textEdit)
        self.tabWidget.addTab(self.tab_2, "")
        self.verticalLayout_5.addWidget(self.tabWidget)
        self.horizontalLayout.addWidget(self.verticalFrame_2)
        self.verticalWidget_2 = QtWidgets.QWidget(self.horizontalFrame)
        self.verticalWidget_2.setObjectName("verticalWidget_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalWidget_2)
        self.verticalLayout_3.setContentsMargins(-1, -1, -1, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setSpacing(20)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.ed_sn = QtWidgets.QLineEdit(self.verticalWidget_2)
        self.ed_sn.setMinimumSize(QtCore.QSize(0, 30))
        self.ed_sn.setObjectName("ed_sn")
        self.verticalLayout_4.addWidget(self.ed_sn)
        self.pbt_start = QtWidgets.QPushButton(self.verticalWidget_2)
        self.pbt_start.setMinimumSize(QtCore.QSize(0, 40))
        self.pbt_start.setObjectName("pbt_start")
        self.verticalLayout_4.addWidget(self.pbt_start)
        self.verticalLayout_3.addLayout(self.verticalLayout_4)
        self.formLayout_2 = QtWidgets.QFormLayout()
        self.formLayout_2.setObjectName("formLayout_2")
        self.label_3 = QtWidgets.QLabel(self.verticalWidget_2)
        self.label_3.setObjectName("label_3")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_3)
        self.lb_ct = QtWidgets.QLabel(self.verticalWidget_2)
        self.lb_ct.setObjectName("lb_ct")
        self.formLayout_2.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lb_ct)
        self.verticalLayout_3.addLayout(self.formLayout_2)
        self.formFrame = QtWidgets.QFrame(self.verticalWidget_2)
        self.formFrame.setStyleSheet(".QLabel{\n"
"    font: 25 16pt \"Helvetica\";\n"
"}\n"
"*{\n"
"background-color: rgb(255, 255, 255);\n"
"}\n"
"")
        self.formFrame.setFrameShape(QtWidgets.QFrame.Panel)
        self.formFrame.setObjectName("formFrame")
        self.formLayout = QtWidgets.QFormLayout(self.formFrame)
        self.formLayout.setContentsMargins(-1, 20, -1, 20)
        self.formLayout.setVerticalSpacing(20)
        self.formLayout.setObjectName("formLayout")
        self.label_5 = QtWidgets.QLabel(self.formFrame)
        self.label_5.setMinimumSize(QtCore.QSize(70, 0))
        self.label_5.setObjectName("label_5")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label_5)
        self.lb_pass_qty = QtWidgets.QLabel(self.formFrame)
        self.lb_pass_qty.setObjectName("lb_pass_qty")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.lb_pass_qty)
        self.label_7 = QtWidgets.QLabel(self.formFrame)
        self.label_7.setMinimumSize(QtCore.QSize(70, 0))
        self.label_7.setObjectName("label_7")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_7)
        self.lb_fail_qty = QtWidgets.QLabel(self.formFrame)
        self.lb_fail_qty.setObjectName("lb_fail_qty")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.lb_fail_qty)
        self.label_9 = QtWidgets.QLabel(self.formFrame)
        self.label_9.setMinimumSize(QtCore.QSize(70, 0))
        self.label_9.setObjectName("label_9")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.LabelRole, self.label_9)
        self.lb_yield = QtWidgets.QLabel(self.formFrame)
        self.lb_yield.setObjectName("lb_yield")
        self.formLayout.setWidget(2, QtWidgets.QFormLayout.FieldRole, self.lb_yield)
        self.verticalLayout_3.addWidget(self.formFrame)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.verticalLayout_3.setStretch(2, 3)
        self.horizontalLayout.addWidget(self.verticalWidget_2)
        self.horizontalLayout.setStretch(0, 7)
        self.horizontalLayout.setStretch(1, 2)
        self.verticalLayout_2.addWidget(self.horizontalFrame)
        self.verticalLayout_2.setStretch(0, 2)
        self.verticalLayout_2.setStretch(1, 10)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 818, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label.setText(_translate("MainWindow", "H28 MIC Sorting"))
        self.label_2.setText(_translate("MainWindow", "Versoin:1.0.0.1\n"
"BY SMT TE"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("MainWindow", "TestItem"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("MainWindow", "TestLog"))
        self.pbt_start.setText(_translate("MainWindow", "START"))
        self.label_3.setText(_translate("MainWindow", "CT:"))
        self.lb_ct.setText(_translate("MainWindow", "TextLabel"))
        self.label_5.setText(_translate("MainWindow", "PASS Qty:"))
        self.lb_pass_qty.setText(_translate("MainWindow", "0"))
        self.label_7.setText(_translate("MainWindow", "FAIL Qty:"))
        self.lb_fail_qty.setText(_translate("MainWindow", "0"))
        self.label_9.setText(_translate("MainWindow", "Yield:"))
        self.lb_yield.setText(_translate("MainWindow", "100%"))
