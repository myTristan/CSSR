#
#   Name:      CSSR: Car Simulation System by ROS
#   Version:   1.0
#   Developer: Zhmy
#   Date:      2025-02-07
#
#   Running Code
#   roscore
#   source ./devel/setup.bash
#   rosrun cssr simtest.py
#   rosrun cssr simmain.py
#   see README.md
#
import sys
import rospy
from cssr.msg import Control #Control Message
from cssr.msg import Obstacle #Obstacle Message
from cssr.msg import ObstacleVec #Obstacle Vector Message
from cssr.msg import VehicleState #Vehicle State Message
import numpy as np

import math
# import time
# import csv
import utm
# UTM Zone: 50N
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QGraphicsScene, QGraphicsView, QGraphicsPathItem, QGraphicsLineItem, QGraphicsEllipseItem
from PyQt5.QtWidgets import QGraphicsPixmapItem, QDialog, QVBoxLayout, QLineEdit, QPushButton, QLabel, QFileDialog,QWidget, QHBoxLayout
from PyQt5.QtGui import QPainterPath, QPen, QBrush,QPixmap, QPainter, QColor
from PyQt5.QtCore import Qt, QTimer, QCoreApplication

# Parameters
# Square boundary (1 corresponds to 0.1 m)
boundary = 600
# Time interval
T = 40 # 40 ms 25 Hz

# The number of obstacles
obs_num = 1

# RTK position of the start point
RTK_longitude = 116.340768 # longitude
RTK_latitude = 40.013847   # latitude
RTK_altitude = 59.0        # altitude

# RTK parameter (RTK value per meter)
RTK_longitude_per_meter = 1.173e-5
RTK_latitude_per_meter = 8.983e-6

class Ui_CSSR(object):
    def __init__(self):
        # Run parameter
        self.run = 0
        self.random = 0 # 0: not generate random obstacles, 1: have generated random obstacles
        self.fixed = 0 # 0: not generate fixed obstacles, 1: have generated fixed obstacles
        self.drawn_items = []
        self.flush_count = 0 # flush count
        # self.drawn_obs = []
        # start_time = rospy.get_time()
        pass

    def setupUi(self, CSSR):
        CSSR.setObjectName("CSSR")
        CSSR.resize(900, 700)
        self.centralwidget = QtWidgets.QWidget(CSSR)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(40, 40, 160, 91))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")

        self.radioButton = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radioButton.setFont(font)
        self.radioButton.setChecked(True)
        self.radioButton.setObjectName("radioButton")
        self.verticalLayout.addWidget(self.radioButton)
        self.radioButton.clicked.connect(self.radioButton_value_changed)

        self.radioButton_2 = QtWidgets.QRadioButton(self.verticalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.radioButton_2.setFont(font)
        self.radioButton_2.setObjectName("radioButton_2")
        self.verticalLayout.addWidget(self.radioButton_2)
        self.radioButton_2.clicked.connect(self.radioButton_2_value_changed)

        self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(240, 40, boundary+20, boundary+20))
        self.graphicsView.setObjectName("graphicsView")
        # Create a scene and set it to the graphics view
        self.scene = QGraphicsScene()
        self.graphicsView.setScene(self.scene)
        # Increase the resolution of the graph
        self.graphicsView.setRenderHint(QPainter.Antialiasing)
        self.graphicsView.setRenderHint(QPainter.SmoothPixmapTransform)
        self.graphicsView.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)

        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(40, 160, 160, 331))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")

        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")

        self.spinBox = QtWidgets.QSpinBox(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.spinBox.setFont(font)
        self.spinBox.setProperty("value", 1)
        self.spinBox.setObjectName("spinBox")
        self.verticalLayout_2.addWidget(self.spinBox)
        self.spinBox.valueChanged.connect(self.spinbox_value_changed)

        self.pushButton = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setStyleSheet("QPushButton{color:  green;}")
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout_2.addWidget(self.pushButton)
        self.pushButton.clicked.connect(self.run_clicked)

        self.pushButton_2 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setStyleSheet("QPushButton{color:  red;}")
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout_2.addWidget(self.pushButton_2)
        self.pushButton_2.clicked.connect(self.stop_clicked)

        self.pushButton_3 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setStyleSheet("QPushButton{color:  orange;}")
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout_2.addWidget(self.pushButton_3)
        self.pushButton_3.clicked.connect(self.save_clicked)

        self.pushButton_4 = QtWidgets.QPushButton(self.verticalLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        font.setStyleStrategy(QtGui.QFont.NoAntialias)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setStyleSheet("QPushButton{color:  dodgerblue;}")
        self.pushButton_4.setObjectName("pushButton_4")
        self.verticalLayout_2.addWidget(self.pushButton_4)
        self.pushButton_4.clicked.connect(self.clear_clicked)

        self.verticalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_3.setGeometry(QtCore.QRect(42, 510, 191, 136))
        self.verticalLayoutWidget_3.setObjectName("verticalLayoutWidget_3")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_3)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label_2 = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_3.addWidget(self.label_2)

        self.label_3 = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_3.addWidget(self.label_3)

        self.label_4 = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_3.addWidget(self.label_4)

        self.label_5 = QtWidgets.QLabel(self.verticalLayoutWidget_3)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")
        self.verticalLayout_3.addWidget(self.label_5)

        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(40, 140, 158, 41))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.label.setFont(font)
        self.label.setStyleSheet("")
        self.label.setObjectName("label")

        CSSR.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(CSSR)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 900, 23))
        self.menubar.setObjectName("menubar")
        CSSR.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(CSSR)
        self.statusbar.setObjectName("statusbar")
        CSSR.setStatusBar(self.statusbar)

        self.retranslateUi(CSSR)
        QtCore.QMetaObject.connectSlotsByName(CSSR)

        # Add background image
        # Change Figure
        self.add_background_image("background.jpg")

        # Draw boundary
        self.draw_rect_obs(0, 0, boundary, boundary)

        # Intialize ROS
        rospy.init_node('pub_obstacle', anonymous=True)
        global pub_obs
        pub_obs = rospy.Publisher('/Obstacle',ObstacleVec, queue_size=10)
        global obs_msg
        obs_msg = self.generate_obstaclevec_random()

        global pub_vec
        pub_vec = rospy.Publisher('/VehicleState',VehicleState, queue_size=10)
        global vec_msg
        vec_msg = self.init_VehicleState()
        global vec_msg_old
        vec_msg_old = self.init_VehicleState()

        global sub
        sub = rospy.Subscriber('/Control', Control, self.con_callback,queue_size=10)
        global con_msg
        con_msg = Control()
        # Set up a QTimer to call the update function periodically
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_function)
        self.timer.start(T)  # Call update_function every 1000 milliseconds (1 second)


    def retranslateUi(self, CSSR):
        _translate = QtCore.QCoreApplication.translate
        CSSR.setWindowTitle(_translate("CSSR", "CSSR: Car Simulation System by ROS"))
        self.radioButton.setText(_translate("CSSR", "Random obs"))
        self.radioButton_2.setText(_translate("CSSR", "Fixed obs"))
        self.pushButton.setText(_translate("CSSR", "Run"))
        self.pushButton_2.setText(_translate("CSSR", "Stop"))
        self.pushButton_3.setText(_translate("CSSR", "Save"))
        self.pushButton_4.setText(_translate("CSSR", "Clear"))
        self.label_2.setText(_translate("CSSR", "x"))
        self.label_3.setText(_translate("CSSR", "y"))
        self.label_4.setText(_translate("CSSR", "heading"))
        self.label_5.setText(_translate("CSSR", "speed"))
        self.label.setText(_translate("CSSR", "Random obs num"))
    
    def radioButton_value_changed(self):
        if self.run == 0:
            self.random = 1
            self.fixed = 0
            # print("Random obs button clicked!")
            self.clear_figure()
            self.generate_random_obs(obs_num)
        else:
            self.random = 1
            self.fixed = 0
            # print("Random obs button clicked!")

    def radioButton_2_value_changed(self):
        if self.run == 0:
            self.random = 0
            self.fixed = 1
            # print("Fixed obs button clicked!")
            self.clear_figure()
            self.generate_fixed_obs(obs_num)
        else:
            self.random = 0
            self.fixed = 1
            # print("Fixed obs button clicked!")

    def add_background_image(self, image_path):
        width = boundary
        height = boundary
        opacity = 0.5
        pixmap = QPixmap(image_path)
        
        # Resize the pixmap if width and height are provided
        if width and height:
            pixmap = pixmap.scaled(width, height, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # Create a transparent pixmap with the same size
        transparent_pixmap = QPixmap(pixmap.size())
        transparent_pixmap.fill(QColor(0, 0, 0, 0))
        
        # Create a painter to draw on the transparent pixmap
        painter = QPainter(transparent_pixmap)
        painter.setOpacity(opacity)
        painter.drawPixmap(0, 0, pixmap)
        painter.end()
        
        # Create a QGraphicsPixmapItem with the transparent pixmap
        background_item = QGraphicsPixmapItem(transparent_pixmap)
        self.scene.addItem(background_item)
        background_item.setZValue(-1) 

    def generate_obstaclevec_random(self):
        obs_msg = ObstacleVec()
        obs_msg.obstacleVec = []
        obs_msg.timestamp = rospy.get_time()
        for i in range(obs_num):
            obs = Obstacle()
        obs_msg.obstacleVec.append(Obstacle())
        return obs_msg

    def run_clicked(self):
        if self.run == 0:
            self.run = 1
            # print("Run button clicked!")
            # self.scene.clear()# clear the scene
            # Draw boundary
            self.draw_rect_obs(0, 0, boundary, boundary)

            global vec_msg
            vec_msg = self.init_VehicleState()

            if self.fixed == 1:
                self.generate_fixed_obs(obs_num)

            if self.fixed == 0 and self.random == 0:
                self.generate_random_obs(obs_num)
            # else:
            # print("Run button clicked!")

    def stop_clicked(self):
        if self.run == 1:
            self.run = 0
            self.random = 0
            # print("Stop button clicked!")
   
    def save_clicked(self):
        # print("Save button clicked!")
        
        # Create a QFileDialog
        dialog = QFileDialog()
        dialog.setWindowTitle("Save Image")
        dialog.setAcceptMode(QFileDialog.AcceptSave)  # Set mode to save
        dialog.setNameFilter("PNG Files (*.png);;All Files (*)")
        
        # Create a QWidget
        custom_widget = QWidget()
        layout = QHBoxLayout()
        
        # Add a label to the layout
        label = QLabel("Please select a save location \nand enter the file name")
        layout.addWidget(label)
        
        # Add a QLineEdit to the layout
        custom_widget.setLayout(layout)
        dialog.setOption(QFileDialog.DontUseNativeDialog, True) # Use the custom dialog
        dialog.layout().addWidget(custom_widget)
        
        # Show the dialog and get the file name
        if dialog.exec_() == QFileDialog.Accepted:
            file_name = dialog.selectedFiles()[0]
            
            # Ensure the file name ends with .png
            if not file_name.endswith('.png'):
                file_name += '.png'      
            # Create a QPixmap with the same size as the graphics view
            pixmap = QPixmap(self.graphicsView.size())
            # pixmap.fill(QColor(255, 255, 255, 0))  # Transparent background
            pixmap.fill(Qt.white)  # White background
            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setRenderHint(QPainter.SmoothPixmapTransform)
            self.graphicsView.render(painter)
            painter.end()
            
            # Save the pixmap to a file
            pixmap.save(file_name)
            print(f"Image saved as {file_name}")

    def clear_figure(self):
        if len(self.drawn_items) > 0:
            for item in self.drawn_items:
                self.scene.removeItem(item)
            self.drawn_items.clear()
        self.scene.clear()# clear the scene
        self.add_background_image("background.jpg")
        # draw boundary
        self.draw_rect_obs(0, 0, boundary, boundary)
        # print("Clear button clicked!")

    def clear_clicked(self):
        if self.run == 1:
            self.run = 0
        # test
        self.random = 0
        self.clear_figure()
        self.label_2.setText(f"x:")
        self.label_3.setText(f"y:")
        self.label_4.setText(f"heading:")
        self.label_5.setText(f"speed:")
        # print("Clear button clicked!")

    def spinbox_value_changed(self):
        if self.run == 0 and self.fixed == 0:
            value = self.spinBox.value()
            global obs_num
            obs_num = value
            self.clear_figure()
            self.generate_random_obs(obs_num)
            # print(f"SpinBox value: {obs_num}")
            self.fixed = 0
    
    def init_VehicleState(self):
        vec_msg = VehicleState()
        vec_msg.timestamp = rospy.get_time()
        vec_msg.x = 300
        vec_msg.y = 300 
        vec_msg.speed_x = 0
        vec_msg.speed_y = 5
        # vec_msg.acc_x = 0
        # vec_msg.acc_y = 0
        vec_msg.heading = math.pi/2
        vec_msg.steer_state_front_wheel = 0
        # RTK position与每个格子距离有关
        vec_msg.RTK_gps_longitude = RTK_longitude + RTK_longitude_per_meter * vec_msg.x / 10
        vec_msg.RTK_gps_latitude  = RTK_latitude  - RTK_latitude_per_meter  * vec_msg.y / 10
        vec_msg.RTK_gps_altitude  = RTK_altitude
        # utm position
        vec_msg.RTK_gps_UTM_x, vec_msg.RTK_gps_UTM_y, zone_number, zone_letter = utm.from_latlon(vec_msg.RTK_gps_latitude, vec_msg.RTK_gps_longitude)
        vec_msg.RTK_gps_UTM_z = vec_msg.RTK_gps_altitude
        return vec_msg

    def draw_line(self,x1,y1,x2,y2):
        # draw path line
        line = QGraphicsLineItem(x1, y1, x2, y2)
        pen = QPen(Qt.red, 2)
        line.setPen(pen)
        self.scene.addItem(line)

    def draw_circle(self,x,y,r):
        # draw circle
        global circle
        circle = QGraphicsEllipseItem(x-r, y-r, 2*r, 2*r)
        pen = QPen(Qt.blue, 2)
        circle.setPen(pen)
        self.scene.addItem(circle)
        self.drawn_items.append(circle)

    def draw_vec(self,vec_msg_new,vec_msg_old):
        # Clear old circle and directional line
        if 'circle' in globals():
            global circle
            self.scene.removeItem(circle)

        if 'directional_line' in globals():
            global directional_line
            self.scene.removeItem(directional_line)

        x1 = vec_msg_new.x
        y1 = vec_msg_new.y
        x2 = vec_msg_old.x
        y2 = vec_msg_old.y

        # draw circle
        self.draw_circle(x1, y1, 5)
        if not (x1 == x2 and y1 == y2):
            # draw path line
            self.draw_line(x1, y1, x2, y2)
            # draw destination
            steer = vec_msg_new.steer_state_front_wheel
            self.draw_directional_line(x1, y1, x2, y2, steer, 20)
        
    def draw_rect_obs(self,x,y,length1,length2):
        path = QPainterPath()
        path.moveTo(0, 0)  # Start point
        # Adjust the view to start from the top-left corner
        self.graphicsView.setTransform(QtGui.QTransform().translate(0, 0))
        path.addRect(x, y, length1, length2)

        # Create a QGraphicsPathItem and add it to the scene
        # global path_item
        path_item = QGraphicsPathItem(path)
        # pen = QPen(Qt.blue, 2)
        pen = QPen(QColor(153, 50, 204), 2)
        path_item.setPen(pen)
        self.scene.addItem(path_item)  

    def generate_random_obs(self,num):
        self.random = 1
        global obs_msg
        obs_msg = ObstacleVec()
        global obs_num 
        obs_num = self.spinBox.value()
        obs_msg.len = obs_num
        for i in range(obs_num):
            obs = Obstacle()
            obs.id = i
            size_x = np.random.uniform(20, 100)
            size_y = np.random.uniform(20, 100)
            x_upper = np.random.uniform(0, boundary - 100)
            y_upper = np.random.uniform(0, boundary - 100)
            self.draw_rect_obs(x_upper, y_upper, size_x, size_y)
            obs.size_x = size_x
            obs.size_y = size_y
            obs.x_global = x_upper + size_x/2
            obs.y_global = y_upper + size_y/2
            obs.h_global = 0
            obs_msg.obstacleVec.append(obs)

    def generate_fixed_obs(self,num):
        global obs_msg
        obs_msg = ObstacleVec()
        global obs_num
        obs_num = 4
        obs_msg.len = obs_num
        obs = Obstacle()
        obs.id = 0
        size_x = 160
        size_y = 160
        x_upper = 70
        y_upper = 70
        self.draw_rect_obs(x_upper, y_upper, size_x, size_y)
        obs.size_x = size_x
        obs.size_y = size_y
        obs.x_global = x_upper + size_x/2
        obs.y_global = y_upper + size_y/2
        obs.h_global = 0
        obs_msg.obstacleVec.append(obs)
        obs = Obstacle()
        obs.id = 1
        # size_x = 200
        # size_y = 200
        x_upper = 70
        y_upper = 370
        self.draw_rect_obs(x_upper, y_upper, size_x, size_y)
        obs.size_x = size_x
        obs.size_y = size_y
        obs.x_global = x_upper + size_x/2
        obs.y_global = y_upper + size_y/2
        obs.h_global = 0
        obs_msg.obstacleVec.append(obs)
        obs = Obstacle()
        obs.id = 2
        # size_x = 200
        # size_y = 200
        x_upper = 370
        y_upper = 70
        self.draw_rect_obs(x_upper, y_upper, size_x, size_y)
        obs.size_x = size_x
        obs.size_y = size_y
        obs.x_global = x_upper + size_x/2
        obs.y_global = y_upper + size_y/2
        obs.h_global = 0
        obs_msg.obstacleVec.append(obs)
        obs = Obstacle()
        obs.id = 3
        # size_x = 200
        # size_y = 200
        x_upper = 370
        y_upper = 370
        self.draw_rect_obs(x_upper, y_upper, size_x, size_y)
        obs.size_x = size_x
        obs.size_y = size_y
        obs.x_global = x_upper + size_x/2
        obs.y_global = y_upper + size_y/2
        obs.h_global = 0
        obs_msg.obstacleVec.append(obs)

    def con_callback(self, msg):
        global con_msg
        # self.con_msg = msg
        con_msg.timestamp = msg.timestamp
        con_msg.count = msg.count
        con_msg.steering_cmd_front_wheel = msg.steering_cmd_front_wheel
        con_msg.gear_cmd = msg.gear_cmd
        con_msg.turn_signal_cmd = msg.turn_signal_cmd
        con_msg.speed_cmd = msg.speed_cmd
        con_msg.acceleration_cmd = msg.acceleration_cmd

    def draw_directional_line(self, x1, y1, x2, y2, steer, length):
        
        angle_radians = math.atan2(y1-y2, x1-x2)

        angle = angle_radians + steer

        # Calculate the end point using the angle and length
        end_x = x1 + length * math.cos(angle)
        end_y = y1 + length * math.sin(angle)

        # Draw the line
        global directional_line
        directional_line = QGraphicsLineItem(x1, y1, end_x, end_y)
        pen = QPen(Qt.blue, 2)
        directional_line.setPen(pen)
        self.scene.addItem(directional_line)

    def renew_and_draw_vec_msg(self, vec_msg_old):
        # Receive control message
        global con_msg, vec_msg
        # Renew VehicleState message
        vec_msg = VehicleState()
        vec_msg.timestamp = rospy.get_time()
        # 转向系数
        k = 1
        heading = (vec_msg_old.heading + k * vec_msg_old.steer_state_front_wheel)% (2 * math.pi)
        vec_msg.x = vec_msg_old.x + vec_msg_old.speed_x * T/1000
        vec_msg.y = vec_msg_old.y + vec_msg_old.speed_y * T/1000

        vec_msg.speed_x = con_msg.speed_cmd * math.cos(heading)
        vec_msg.speed_y = con_msg.speed_cmd * math.sin(heading)

        # vec_msg.acc_x = con_msg.acc_x
        # vec_msg.acc_y = con_msg.acc_y
        vec_msg.heading = heading
        vec_msg.steer_state_front_wheel = con_msg.steering_cmd_front_wheel
        
        # RTK position与每个格子距离有关
        vec_msg.RTK_gps_longitude = RTK_longitude + RTK_longitude_per_meter * vec_msg.x / 10
        vec_msg.RTK_gps_latitude  = RTK_latitude  - RTK_latitude_per_meter  * vec_msg.y / 10
        vec_msg.RTK_gps_altitude  = RTK_altitude
        
        # utm position
        vec_msg.RTK_gps_UTM_x, vec_msg.RTK_gps_UTM_y, zone_number, zone_letter = utm.from_latlon(vec_msg.RTK_gps_latitude, vec_msg.RTK_gps_longitude)
        vec_msg.RTK_gps_UTM_z = vec_msg.RTK_gps_altitude

        # Remove
        if len(self.drawn_items) > 0:
            for item in self.drawn_items:
                self.scene.removeItem(item)
            self.drawn_items.clear()

        x1 = vec_msg.x
        y1 = vec_msg.y
        x2 = vec_msg_old.x
        y2 = vec_msg_old.y

        # draw circle
        self.draw_circle(x1, y1, 10)
        if not (x1 == x2 and y1 == y2):
            # draw path line
            self.draw_line(x1, y1, x2, y2)
            k = 3000/(math.fabs(con_msg.speed_cmd) + 0.5)
            end_x = (k + 1)*x1 - k*x2
            end_y = (k + 1)*y1 - k*y2

            # Draw the line
            # global directional_line
            directional_line = QGraphicsLineItem(x1, y1, end_x, end_y)
            pen = QPen(Qt.blue, 2)
            directional_line.setPen(pen)
            self.scene.addItem(directional_line)
            self.drawn_items.append(directional_line)
        return vec_msg

    def update_function(self):
        if not rospy.is_shutdown():
            # Subscribe the message from simtest.py
            global sub
            sub = rospy.Subscriber('/Control', Control, self.con_callback,queue_size=10)
            print(f"count: {con_msg.count} speed: {con_msg.speed_cmd}")

            # Publish VehicleState
            global vec_msg, vec_msg_old
            vec_msg_old = vec_msg
            if self.run == 1:
                vec_msg = self.renew_and_draw_vec_msg(vec_msg_old)
            pub_vec.publish(vec_msg)
            
            rospy.loginfo("x: %f, y: %f, vx: %f, vy %f, heading: %f, steer: %f, longitude: %f, latitude: %f, UTM_x: %f, UTM_y %f",\
                          vec_msg.x,vec_msg.y,vec_msg.speed_x,vec_msg.speed_y,vec_msg.heading*180/math.pi,vec_msg.steer_state_front_wheel,\
                          vec_msg.RTK_gps_longitude,vec_msg.RTK_gps_latitude,vec_msg.RTK_gps_UTM_x,vec_msg.RTK_gps_UTM_y)

            # Renew info in the UI
            if self.flush_count >= 12 and self.run == 1:
                self.flush_count = 0
                self.label_2.setText(f"x:                {vec_msg.x/10:.2f}")
                self.label_3.setText(f"y:                {vec_msg.y/10:.2f}")
                self.label_4.setText(f"heading:  {vec_msg.heading*180/math.pi:.2f}")
                self.label_5.setText(f"speed:      {con_msg.speed_cmd:.2f}")
            else:
                self.flush_count += 1

            # Publish Obstacles
            print(f"obs_num: {obs_num}")

            global obs_msg
            timestamp = rospy.get_time()
            obs_msg.timestamp = timestamp
            for i in range(obs_num):
                obs_msg.obstacleVec[i].timestamp = timestamp
            pub_obs.publish(obs_msg)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    CarSimulation = QtWidgets.QMainWindow()
    ui = Ui_CSSR()
    ui.setupUi(CarSimulation)
    CarSimulation.show()

    sys.exit(app.exec_())
