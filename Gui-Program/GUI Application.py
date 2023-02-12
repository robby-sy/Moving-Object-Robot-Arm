import sys
import platform
from PyQt5.QtWidgets import QApplication, QDialog
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QObject, QTimer,QPropertyAnimation,Qt
from PyQt5.QtWidgets import QMainWindow
from cv2 import data 
import numpy as np
import copy
import math
import cv2
from numpy.core.numeric import rollaxis
import serial
from main_gui import *
from error import *

class Warning(QDialog):
    def __init__(self):
        QDialog.__init__(self)
        self.ui = Ui_Error()
        self.ui.setupUi(self)
        self.setWindowFlag(Qt.FramelessWindowHint)

class MainWindow(QMainWindow):
    def __init__(self):
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
#===================================================================Button initial========================================================================
        self.ui.toogle_button.clicked.connect(self.toogle)
        self.ui.stackedWidget.setCurrentWidget(self.ui.image_processing_page)
        self.ui.image_menu_button.clicked.connect(self.camerapage)
        self.ui.serial_menu_button.clicked.connect(self.serialpage)
        self.ui.manual_menu_button.clicked.connect(self.manualpage)
        self.ui.calibration_menu_button.clicked.connect(self.calibrationpage)
        self.ui.camera_connect_button.clicked.connect(self.getcapture)
        self.ui.pushButton_serial_connect.clicked.connect(self.connect_serial)
        self.ui.pushButton_send_joint1.clicked.connect(lambda:self.CR_foward_kinematics(1))
        self.ui.pushButton_send_joint2.clicked.connect(lambda:self.CR_foward_kinematics(2))
        self.ui.pushButton_send_joint3.clicked.connect(lambda:self.CR_foward_kinematics(3))
        self.ui.pushButton_send_gripper.clicked.connect(lambda:self.CR_foward_kinematics(4))
        self.ui.pushButton_send_jaw.clicked.connect(lambda:self.CR_foward_kinematics(5))
        self.ui.pushButton_INV_send.clicked.connect(self.CR_inverse_kinematics)
        self.ui.pushButton_Joint1_apply.clicked.connect(lambda:self.calibration("0","0","0","0"))
        self.ui.pushButton_Joint2_apply.clicked.connect(lambda:self.calibration("0","0","0","0"))
        self.ui.pushButton_Joint3_apply.clicked.connect(lambda:self.calibration("0","0","0","0"))
        self.ui.pushButton_Joint4_apply.clicked.connect(lambda:self.calibration("0","0","0","0"))
        self.ui.pushButton_J1_auto.clicked.connect(lambda:self.calibration("1","0","0","0"))
        self.ui.pushButton_J2_auto.clicked.connect(lambda:self.calibration("0","1","0","0"))
        self.ui.pushButton_J3_auto.clicked.connect(lambda:self.calibration("0","0","1","0"))
        self.ui.pushButton_J1_manual.clicked.connect(lambda:self.calibration("2","0","0","0"))
        self.ui.pushButton_J2_manual.clicked.connect(lambda:self.calibration("0","2","0","0"))
        self.ui.pushButton_J3_manual.clicked.connect(lambda:self.calibration("0","0","2","0"))
        self.ui.pushButton_J4_manual.clicked.connect(lambda:self.calibration("0","0","0","2"))
        self.ui.pushButton_single.clicked.connect(self.single_pick)
        self.ui.pushButton_Home.clicked.connect(self.home)
        self.ui.pushButton_Zero.clicked.connect(lambda:self.write("2;0;0;0;0;22"))
        self.ui.pushButton_export.clicked.connect(self.export)
        self.ui.pushButton_continous.clicked.connect(self.cont_timer)
#===================================================================First Variable initial================================================================
        self.videocontroller = 0
        self.cap = cv2.VideoCapture('video/idle camera.mp4')
        self.image = self.read(self.cap)
        self.serial_act=0
        self.joint2_Position = -90
        self.joint3_Position = 45
        self.joint4_Position = 30
#===================================================================Timer Declaration=====================================================================
        self.timervideoalpha = QTimer()
        self.timerfindctr = QTimer()
        self.timercont = QTimer()
        self.timerfindctr.start(10)
        self.timervideoalpha.start(10)
        self.timercont.timeout.connect(self.continous)
        self.timervideoalpha.timeout.connect(self.video_update)
        self.timervideoalpha.timeout.connect(self.SerialUpdate)
        self.timerfindctr.timeout.connect(self.retrn_cntr)
    def cont_timer(self):
        if not self.timercont.isActive():
            self.timercont.start(1000)
            self.ui.pushButton_continous.setText("Stop")
        else:
            self.timercont.stop()
            self.ui.pushButton_continous.setText("Begin")
#===================================================================Animation and transition Fucntion=====================================================   
    def toogle(self):
        width = self.ui.left_menu_frame.width()
        if width == 60:
            widthExtended = 250
        else:
            widthExtended = 60          
        self.animation = QPropertyAnimation(self.ui.left_menu_frame, b"minimumWidth")
        self.animation.setDuration(400)
        self.animation.setStartValue(width)
        self.animation.setEndValue(widthExtended)
        self.animation.setEasingCurve(QtCore.QEasingCurve.InOutQuart)
        self.animation.start()
    def camerapage(self):
        self.ui.stackedWidget.setCurrentWidget(self.ui.image_processing_page)
    def serialpage(self):
        self.ui.stackedWidget.setCurrentWidget(self.ui.Serial_page)
    def manualpage(self):
        self.ui.stackedWidget.setCurrentWidget(self.ui.Manual_control)
    def calibrationpage(self):
        self.ui.stackedWidget.setCurrentWidget(self.ui.Calibration)
#===================================================================capture camera and video==============================================================
    def getcapture(self):
        if self.videocontroller == 0:
            self.videocontroller = 1
            self.videoCH = self.ui.camera_channel.currentIndex()
            self.cap = cv2.VideoCapture(self.videoCH)
            self.ui.camera_connect_button.setText("Disconnect")
        elif self.videocontroller == 1:
            self.videocontroller = 0
            self.cap = cv2.VideoCapture('video/idle camera2.mp4')
            self.ui.camera_connect_button.setText("Connect")
#===================================================================Read captured image===================================================================
    def read(self,cap):   
        ret,image = cap.read()
        #=============================================== Reread Video ==============================================================
        if ret == False:
            cap = cv2.VideoCapture('video/idle camera2.mp4')
            ret, image = cap.read()
        return image
#===================================================================Preprocessing image===================================================================
    def preprocessing(self,image,workspace,RGB,HSV,grayscale,clahe,blur,canny,dilate):
        if workspace:
            xr = self.ui.horizontalSlider_x.value()
            yr = self.ui.horizontalSlider_y.value()
            wr = self.ui.horizontalSlider_w.value()
            hr = self.ui.horizontalSlider_h.value() 
            image = image[yr:yr+hr,xr:xr+wr]
        else:
            image = image
        #=============color space===================
        if RGB:
            image = image
        elif HSV:
            image = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #============Grayscale======================
        if grayscale and RGB:
            image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        elif grayscale and HSV:
            _,_,image = cv2.split(image)
        #============Clahe==========================
        if clahe:
            if grayscale:
                gridsize = self.ui.horizontalSlider_grid_size.value()
                clipingLimit = self.ui.horizontalSlider_clipping_limit.value()
                klahe = cv2.createCLAHE(clipLimit=clipingLimit, tileGridSize=(gridsize,gridsize))
                image = klahe.apply(image)
            else:
                self.error("The image must be in grayscale color space")
                self.ui.checkBox_clahe.setChecked(False)
        #============Blur===========================
        if blur:
            if self.ui.radioButton.isChecked():
                self.GBsize = 3
            elif self.ui.radioButton_2.isChecked():
                self.GBsize = 5
            elif self.ui.radioButton_3.isChecked():
                self.GBsize = 7
            image = cv2.GaussianBlur(image,(self.GBsize,self.GBsize),0)
        #============canny==========================
        if canny:
            if grayscale:
                high = self.ui.horizontalSlider_high_th.value()
                low = self.ui.horizontalSlider_low_th.value()
                image = cv2.Canny(image,low,high)
            else:
                self.error("The image must be in grayscale color space")
                self.ui.checkBox_canny.setChecked(False)
        #===========dilate==========================
        if dilate:
            dilate_size = self.ui.horizontalSlider_dilate.value()
            dilate_kernel = np.ones((dilate_size,dilate_size))
            image = cv2.dilate(image,dilate_kernel,iterations=1)
        return image
#===================================================================find & selection contour=========================================================================
    def find_contour(self,image):
        if self.ui.checkBox_canny.isChecked():
            post_image = self.preprocessing(image,self.ui.checkBox_workspace.isChecked(),self.ui.radioButton_RGB.isChecked(),self.ui.radioButton_HSV.isChecked(),self.ui.checkBox_grayscale.isChecked(),self.ui.checkBox_clahe.isChecked(),self.ui.checkBox_gaussian_blur.isChecked(),self.ui.checkBox_canny.isChecked(),self.ui.checkBox_dilate.isChecked())
            contour,hierarchy = cv2.findContours(post_image,self.ui.comboBox_approx.currentIndex(),cv2.CHAIN_APPROX_NONE)
            convex = []
            for cx in contour:
                epsilon = 0.1*cv2.arcLength(cx,True)
                cvx = cv2.convexHull(cx,epsilon,True)
                convex.append(cvx)
            contour = copy.deepcopy(convex)
            contour_count = str(len(contour))
            self.ui.label_19.setText("Controur Retrived : " + contour_count)

            if self.ui.checkBox_active_selection.isChecked():
                if not hierarchy is None:
                    if self.ui.comboBox_hierarchy.currentIndex()==0:
                        Ncontour=contour
                    elif self.ui.comboBox_hierarchy.currentIndex()==1:
                        Ncontour = []
                        for h in range(len(hierarchy[0])):
                            child = hierarchy[0][h][3]
                            if child != -1:
                                Ncontour.append(contour[h])
                    elif self.ui.comboBox_hierarchy.currentIndex()==2:
                        Ncontour = []
                        for h in range(len(hierarchy[0])):
                            child = hierarchy[0][h][3]
                            if child == -1:
                                Ncontour.append(contour[h])
                    else:
                        Ncontour = contour
                else:
                    Ncontour = contour
            
                if self.ui.checkBox_contour_area.isChecked():
                    Acontour=[]
                    for c in Ncontour:
                        area = cv2.contourArea(c)
                        if self.ui.lineEdit_minimum_area.text() == "":
                            min_area = 0
                        else:
                            min_area = int(self.ui.lineEdit_minimum_area.text())
                        if self.ui.lineEdit_maximum_area.text() == "":
                            max_area = 0
                        else:
                            max_area = int(self.ui.lineEdit_maximum_area.text())
                        if area >= min_area and area<= max_area:
                            Acontour.append(c)
                else:
                    Acontour=Ncontour
            else:
                Acontour=contour
        else:
            self.error("Image must be applied by canny processing first")
            self.ui.checkBox_find_contour.setChecked(False)
            Acontour = []
        if self.ui.checkBox_workspace.isChecked():
            shiftedcontour = copy.deepcopy(Acontour)
            for sa in range(len(shiftedcontour)):
                for sb in range(len(shiftedcontour[sa])):
                    shiftedcontour[sa][sb][0][0]=shiftedcontour[sa][sb][0][0]+self.ui.horizontalSlider_x.value()
                    shiftedcontour[sa][sb][0][1]=shiftedcontour[sa][sb][0][1]+self.ui.horizontalSlider_y.value()
        else:
            shiftedcontour = Acontour
        shiftedcontour = sorted(shiftedcontour,key=cv2.contourArea,reverse=False)
        Acontour = sorted(Acontour,key=cv2.contourArea,reverse=False)
        return shiftedcontour,Acontour
#===================================================================find center coordinates===========================================================================
    def find_center(self,contour,uscontour,index):
        center = []
        rcenter = []
        angle = []
        area = []
        for cnt in range(len(uscontour)):
            Mo = cv2.moments(uscontour[cnt])
            ar = int(Mo['m00'])
            x = int(Mo['m10']/Mo['m00'])
            y = int(Mo['m01']/Mo['m00'])
            rx = round((x/self.ui.horizontalSlider_w.value())*465)
            ry = round((y/self.ui.horizontalSlider_h.value())*260)
            rect = cv2.minAreaRect(uscontour[cnt])
            width = int(rect[1][0])
            height = int(rect[1][1])
            shape = abs(width-height)
            sudut = int(rect[2])
            if shape>20:
                if width>height:
                    x = x-round((12*self.cos(sudut))*self.ui.horizontalSlider_w.value()/465)
                    y = y-round((12*self.sin(sudut))*self.ui.horizontalSlider_h.value()/260)
                    rx = rx - round(12*self.cos(sudut))
                    ry = ry - round(12*self.sin(sudut))
                    sudut = 90-sudut
                else:
                    x = x+round((12*self.cos(90-sudut))*self.ui.horizontalSlider_w.value()/465)
                    y = y-round((12*self.sin(90-sudut))*self.ui.horizontalSlider_h.value()/270)
                    rx = rx + round(12*self.cos(90-sudut))
                    ry = ry - round(12*self.sin(90-sudut))     
                    sudut = -sudut
            elif shape <=20:
                if sudut>45:
                    sudut = 90-sudut
                else:
                    sudut = -sudut
            HTM = np.array([[0,1,0,110],[1,0,0,-250],[0,0,1,0],[0,0,0,1]])
            A = np.array([[rx],[ry],[0],[1]])
            B = HTM.dot(A)
            X = B[0][0]
            Y = B[1][0]
            center.append([X,Y])
            angle.append(sudut)
            area.append(ar)
            rcenter.append([x,y])

        if index == -1:
            return center,rcenter,angle,area
        else:
            return center[index],rcenter[index],angle[index],area[index]
#===================================================================View Update======================================================================
    def video_update(self):
        image=self.image
        post_image = self.preprocessing(image,False,self.ui.radioButton_RGB.isChecked(),self.ui.radioButton_HSV.isChecked(),self.ui.checkBox_grayscale.isChecked(),self.ui.checkBox_clahe.isChecked(),self.ui.checkBox_gaussian_blur.isChecked(),self.ui.checkBox_canny.isChecked(),self.ui.checkBox_dilate.isChecked())      
    #======================================================Draw Area of Interest================================================
        if self.ui.radioButton_single.isChecked():
            if self.ui.checkBox_find_contour.isChecked():
                cntr,uscntr,angle,area = self.find_center(self.contour,self.uscontour,-1)
                max = len(cntr)
                self.ui.comboBox_single.setMaxVisibleItems(max)
                for m in range(max):
                    self.ui.comboBox_single.setItemText(m,"Object : "+str(m)+", "+"["+str(cntr[m][0])+","+str(cntr[m][1])+"]")
            else:
                pass
                
        if self.ui.checkBox_sh_workspace.isChecked():
            xr = self.ui.horizontalSlider_x.value()
            yr = self.ui.horizontalSlider_y.value()
            wr = self.ui.horizontalSlider_w.value()
            hr = self.ui.horizontalSlider_h.value() 
            cv2.rectangle(image,(xr,yr),(xr+wr,yr+hr),(148,53,95),3)
    #=======================================================draw contour=========================================================    
        if self.ui.checkBox_show_contours.isChecked():
            if self.ui.checkBox_find_contour.isChecked():
                cv2.drawContours(image,self.contour,-1,(0,255,0),3)
            else:
                contour = []
                cv2.drawContours(image,contour,-1,(0,255,0),3)
    #========================================================draw coordinate and angle============================================
        if self.ui.checkBox_show_coordinate.isChecked():
            if self.ui.checkBox_find_contour.isChecked():
                center,rcenter,angle,area = self.find_center(self.contour,self.uscontour,-1)
                for x in range(len(center)):
                    cv2.putText(image,str(center[x][0])+","+str(center[x][1]),(rcenter[x][0]+self.ui.horizontalSlider_x.value(),rcenter[x][1]+self.ui.horizontalSlider_y.value()),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0))
            else:
                self.error("Find Contours First")
                self.ui.checkBox_show_coordinate.setChecked(False)
        if self.ui.checkBox_show_angle.isChecked():
            if self.ui.checkBox_find_contour.isChecked():
                center,rcenter,angle,area = self.find_center(self.contour,self.uscontour,-1)
                for x in range(len(angle)):
                    cv2.putText(image,str(angle[x]),(rcenter[x][0]+self.ui.horizontalSlider_x.value(),rcenter[x][1]+20+self.ui.horizontalSlider_y.value()),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0))
        if self.ui.checkBox_show_area.isChecked():
            if self.ui.checkBox_find_contour.isChecked():
                center,rcenter,angle,area = self.find_center(self.contour,self.uscontour,-1)
                for x in range(len(area)):
                    cv2.putText(image,str(area[x]),(rcenter[x][0],rcenter[x][1]+40),cv2.FONT_HERSHEY_COMPLEX,0.5,(255,0,0))

    #========================================================show video on ui=====================================================
        if self.ui.radioButton_after_proces.isChecked():
            if self.ui.checkBox_grayscale.isChecked():
                height, width = post_image.shape
                step = width
                qImg = QImage(post_image.data, width, height, step, QImage.Format_Grayscale8)
                self.ui.camera.setPixmap(QPixmap.fromImage(qImg))
                self.Import = copy.deepcopy(post_image)
            else: 
                height, width, channel = post_image.shape
                step = channel * width
                qImg = QImage(post_image.data, width, height, step, QImage.Format_BGR888)
                self.ui.camera.setPixmap(QPixmap.fromImage(qImg))
                self.Import = copy.deepcopy(post_image)
        elif self.ui.radioButton_real_image.isChecked():
            height, width, channel = image.shape
            step = channel * width
            qImg = QImage(image.data, width, height, step, QImage.Format_BGR888)
            self.ui.camera.setPixmap(QPixmap.fromImage(qImg))
            self.Import = copy.deepcopy(post_image)
#===================================================================Kinematics=======================================================================
    def cos(self,sudut):
        hasil = np.cos(sudut/180*np.pi)
        return hasil
    def sin(self,sudut):
        hasil = np.sin(sudut/180*np.pi)
        return hasil
    def rad(self,sudut):
        return sudut/180*np.pi
    def foward(self,teta):
        teta1 = teta[0,0]
        teta2 = teta[1,0]
        teta3 = teta[2,0]
        H = np.matrix([[self.cos(teta1+teta2+teta3),-1*self.sin(teta1+teta2+teta3),0,50*self.cos(teta1+teta2+teta3)+130*self.cos(teta1+teta2)+195*self.cos(teta1)+104],[self.sin(teta1+teta2+teta3),self.cos(teta1+teta2+teta3),0,50*self.sin(teta1+teta2+teta3)+130*self.sin(teta1+teta2)+195*self.sin(teta1)],[0,0,1,0],[0,0,0,1]])
        h = np.matrix([[H[0,3]],[H[1,3]],[H[0,0]]])
        return h
    def jacobian(self,teta):
        teta1 = teta[0,0]
        teta2 = teta[1,0]
        teta3 = teta[2,0]
        jacobian = np.matrix([[-50*self.sin(teta1+teta2+teta3)-130*self.sin(teta1+teta2)-195*self.sin(teta1),-50*self.sin(teta1+teta2+teta3)-130*self.sin(teta1+teta2),-50*self.sin(teta1+teta2+teta3)],[50*self.cos(teta1+teta2+teta3)+130*self.cos(teta1+teta2)+195*self.cos(teta1),50*self.cos(teta1+teta2+teta3)+130*self.cos(teta1+teta2),50*self.cos(teta1+teta2+teta3)],[-1*self.sin(teta1+teta2+teta3),-1*self.sin(teta1+teta2+teta3),-1*self.sin(teta1+teta2+teta3)]])
        return jacobian
    def pseudoinverse(self,teta):
        A = self.jacobian(teta)
        U,S,Vt = np.linalg.svd(A)
        [m,n]=A.shape
        n_ = np.minimum(m,n)
        S_ = np.zeros((m,n))
        S_[:n_,:n_]=np.diag(1/S)
        S_ = np.transpose(S_)
        U_ = np.transpose(U)
        V = np.transpose(Vt)
        A_ = V*S_*U_
        return A_

    def New_teta(self,Goal,Current_teta):
        jac = self.pseudoinverse(Current_teta)
        DeltaP = Goal-self.foward(Current_teta)
        Dteta = jac*DeltaP
        teta = Current_teta/180*np.pi+Dteta
        return teta/np.pi*180

    def CR_foward_kinematics(self,joint):
        if joint==1:
            part = "1"
            joint = str(self.ui.horizontalSlider_joint1_newvalue.value())
        elif joint==2:
            part = "2"
            joint = str(self.ui.horizontalSlider_joint2_newvalue.value())
        elif joint==3:
            part = "3"
            joint = str(self.ui.horizontalSlider_joint3_newvalue.value())
        elif joint==4:
            part = "4"
            joint =str(self.ui.horizontalSlider_gripper_newvalue.value())
        elif joint==5:
            part = "5"
            joint = str(self.ui.horizontalSlider_jaw_newvalue.value()) 
        data_kirim = "5"+";"+part+";"+joint+"\n"
        self.write(data_kirim)
    def CR_inverse_kinematics(self):
        x = int(self.ui.lineEdit_X_new.text())
        y = int(self.ui.lineEdit_Y_new.text())
        Tta = self.cos(int(self.ui.lineEdit_Grip_New.text()))
        if self.ui.lineEdit_X_correction.text()=="" or self.ui.lineEdit_X_correction.text()=="-":
            CorX = 0
        else:
            CorX = int(self.ui.lineEdit_X_correction.text())
        if self.ui.lineEdit_Y_correction.text()=="" or self.ui.lineEdit_Y_correction.text()=="-":
            CorY = 0
        else:
            CorY = int(self.ui.lineEdit_Y_correction.text())
        if self.ui.radioButton_Numerical.isChecked():
            Xn = str(int(self.ui.lineEdit_X_new.text())+CorX)
            Yn = str(int(self.ui.lineEdit_Y_new.text())+CorY)
            data_kirim = "4"+";"+Xn+";"+Yn+";"+str(Tta)
            self.write(data_kirim)
        elif self.ui.radioButton_Analisys.isChecked():
            Xn = str(int(self.ui.lineEdit_X_new.text())+CorX)
            Yn = str(int(self.ui.lineEdit_Y_new.text())+CorY)
            data_kirim = "7"+";"+Xn+";"+Yn+";"+self.ui.lineEdit_Grip_New.text()
            self.write(data_kirim)
    def calibration(self,auto1,auto2,auto3,auto4):
        joint1_speed = self.ui.lineEdit_J1_maxSpeed.text()
        joint1_accel = self.ui.lineEdit_J1_Accel.text()
        joint2_speed = self.ui.lineEdit_J2_maxSpeed.text()
        joint2_accel = self.ui.lineEdit_J2_Accel.text()
        joint3_speed = self.ui.lineEdit_J3_maxSpeed.text()
        joint3_accel = self.ui.lineEdit_J3_Accel.text()
        joint4_speed = self.ui.lineEdit_J4_maxSpeed.text()
        joint4_accel = self.ui.lineEdit_J4_Accel.text()
        joint1_manual = self.ui.lineEdit_J1_manual.text()
        joint2_manual = self.ui.lineEdit_J2_manual.text()
        joint3_manual = self.ui.lineEdit_J3_manual.text()
        joint4_manual = self.ui.lineEdit_J4_manual.text()
        data_kirim = "1"+";"+joint1_speed+";"+joint1_accel+";"+auto1+";"+joint1_manual+";"+joint2_speed+";"+joint2_accel+";"+auto2+";"+joint2_manual+";"+joint3_speed+";"+joint3_accel+";"+auto3+";"+joint3_manual+";"+joint4_speed+";"+joint4_accel+";"+auto4+";"+joint4_manual       
        self.write(data_kirim)

    def single_pick(self):
        if len(self.contour)<=0:
            pass
        else:
            if self.ui.radioButton_Numerical.isChecked():
                index = self.ui.comboBox_single.currentIndex()
                if self.ui.checkBox_find_contour.isChecked():
                    if self.ui.lineEdit_X_correction.text()=="" or self.ui.lineEdit_X_correction.text()=="-":
                        CorX = 0
                    else:
                        CorX = int(self.ui.lineEdit_X_correction.text())
                    if self.ui.lineEdit_Y_correction.text()=="" or self.ui.lineEdit_Y_correction.text()=="-":
                        CorY = 0
                    else:
                        CorY = int(self.ui.lineEdit_Y_correction.text())
                    index = self.ui.comboBox_single.currentIndex()
                    cntr,uscntr,angle,area = self.find_center(self.contour,self.uscontour,index)
                    X = cntr[0]+CorX
                    Y = cntr[1]+CorY
                    ag = np.cos(angle/180*3.14)
                    data_kirim = "6"+";"+str(X)+";"+str(Y)+";"+str(ag)+";"+"25"
                    self.write(data_kirim)
            elif self.ui.radioButton_Analisys.isChecked():
                index = self.ui.comboBox_single.currentIndex()
                if self.ui.checkBox_find_contour.isChecked():
                    if self.ui.lineEdit_X_correction.text()=="" or self.ui.lineEdit_X_correction.text()=="-":
                        CorX = 0
                    else:
                        CorX = int(self.ui.lineEdit_X_correction.text())
                    if self.ui.lineEdit_Y_correction.text()=="" or self.ui.lineEdit_Y_correction.text()=="-":
                        CorY = 0
                    else:
                        CorY = int(self.ui.lineEdit_Y_correction.text())
                    index = self.ui.comboBox_single.currentIndex()
                    cntr,uscntr,angle,area = self.find_center(self.contour,self.uscontour,index)
                    X = cntr[0]+CorX
                    Y = cntr[1]+CorY
                    ag = angle
                    data_kirim = "8"+";"+str(X)+";"+str(Y)+";"+str(ag)+";"+"25"
                    self.write(data_kirim)

    def continous(self):
        if len(self.contour)<=0:
            pass
        else:
            if self.ui.radioButton_continous.isChecked():
                if self.cont == 1:
                    index = self.ui.comboBox_single.currentIndex()
                    if self.ui.checkBox_find_contour.isChecked():
                        if self.ui.lineEdit_X_correction.text()=="" or self.ui.lineEdit_X_correction.text()=="-":
                            CorX = 0
                        else:
                            CorX = int(self.ui.lineEdit_X_correction.text())
                        if self.ui.lineEdit_Y_correction.text()=="" or self.ui.lineEdit_Y_correction.text()=="-":
                            CorY = 0
                        else:
                            CorY = int(self.ui.lineEdit_Y_correction.text())
                        index = self.ui.comboBox_single.currentIndex()
                        cntr,uscntr,angle,area = self.find_center(self.contour,self.uscontour,index)
                        X = cntr[0]-CorX
                        Y = cntr[1]-CorY
                        ag = np.cos(angle/180*3.14)
                        data_kirim = "6"+";"+str(X)+";"+str(Y)+";"+str(ag)+";"+"30"
                        self.write()
                else:
                    pass
            

    def home(self):
        data = "2"+";"+"30"+";"+"90"+";"+"-90"+";"+"-45"+";"+"22"+"\n"
        self.write(data)        
#===================================================================Serial Connection================================================================
    def connect_serial(self):
        port = self.ui.lineEdit_serial_port.text()
        Baud = self.ui.comboBox_baudrate.currentText()
        if self.serial_act == 0:
            self.ser = serial.Serial(port,baudrate=int(Baud),timeout=0.02)
            self.ui.label_serial_indicator.setText("Connected to Port : "+port)
            self.ui.pushButton_serial_connect.setText("Disconnect")
            self.serial_act=1
        elif self.serial_act == 1:
            self.ser.close()
            self.serial_act=0
            self.ui.label_serial_indicator.setText("Not Connected to Any Port")
            self.ui.pushButton_serial_connect.setText("Connect")
    def write(self,data):
        self.ser.write(data.encode())
    def SerialUpdate(self):
        if self.serial_act == 1:
            data_masuk = str(self.ser.readline()).split(';')
            if len(data_masuk) == 11:
                self.mode = data_masuk[0][-1:]
                joint1_maxSpeed = int(data_masuk[1])
                joint1_Position = int(data_masuk[2])
                joint2_maxSpeed = data_masuk[3]
                self.joint2_Position = int(data_masuk[4])
                joint3_maxSpeed = data_masuk[5]
                self.joint3_Position = int(data_masuk[6])  
                joint4_maxSpeed = data_masuk[7]
                self.joint4_Position = int(data_masuk[8])   
                self.grip = int(data_masuk[9])
                self.cont = int(data_masuk[10][:-5])            
                self.ui.label_Joint1_speed.setText("Current Max Speed :"+str(joint1_maxSpeed))
                self.ui.label_Joint2_speed.setText("Current Max Speed :"+str(joint2_maxSpeed))
                self.ui.label_Joint3_speed.setText("Current Max Speed :"+str(joint3_maxSpeed))
                self.ui.label_Joint4_speed.setText("Current Max Speed :"+str(joint4_maxSpeed))
                self.ui.label_joint1_currentvalue.setText(str(joint1_Position))
                self.ui.label_joint2_currentvalue.setText(str(self.joint2_Position))
                self.ui.label_joint3_currentvalue.setText(str(self.joint3_Position))
                self.ui.label_gripper_currentvalue.setText(str(self.joint4_Position))
                self.ui.label_jaw_currentvalue.setText(str(self.grip))

#==================================================================================================================================================
    def error(self,error):
        false.ui.label_error.setText(error)
        false.show()
    def retrn_cntr(self):
        if self.ui.radioButton_Custom.isChecked():
            pass
        elif self.ui.radioButton_Default.isChecked():
            self.ui.checkBox_workspace.setChecked(True)
            self.ui.horizontalSlider_x.setValue(40)
            self.ui.horizontalSlider_y.setValue(17)
            self.ui.horizontalSlider_w.setValue(1210)
            self.ui.horizontalSlider_h.setValue(690)
            self.ui.checkBox_gaussian_blur.setChecked(True)
            self.ui.radioButton_HSV.setChecked(True)
            self.ui.checkBox_grayscale.setChecked(True)
            self.ui.horizontalSlider_low_th.setValue(100)
            self.ui.horizontalSlider_high_th.setValue(180)
            self.ui.checkBox_canny.setChecked(True)
            self.ui.horizontalSlider_dilate.setValue(5)
            self.ui.checkBox_dilate.setChecked(True)
            self.ui.checkBox_find_contour.setChecked(True)
            self.ui.comboBox_approx.setCurrentIndex(3)
            self.ui.checkBox_show_contours.setChecked(True)
            self.ui.checkBox_active_selection.setChecked(True)
            self.ui.comboBox_hierarchy.setCurrentIndex(1)
            self.ui.checkBox_contour_area.setChecked(True)
            self.ui.lineEdit_minimum_area.setText("6000")
            self.ui.lineEdit_maximum_area.setText("100000")
            self.ui.checkBox_show_coordinate.setChecked(True)
        else:
            self.ui.checkBox_workspace.setChecked(False)
            self.ui.radioButton_HSV.setChecked(False)
            self.ui.checkBox_grayscale.setChecked(False)
            self.ui.horizontalSlider_low_th.setValue(10)
            self.ui.horizontalSlider_high_th.setValue(10)
            self.ui.checkBox_canny.setChecked(False)
            self.ui.horizontalSlider_dilate.setValue(1)
            self.ui.checkBox_dilate.setChecked(False)
            self.ui.checkBox_find_contour.setChecked(False)
            self.ui.comboBox_approx.setCurrentIndex(0)
            self.ui.checkBox_show_contours.setChecked(False)
            self.ui.checkBox_active_selection.setChecked(False)
            self.ui.comboBox_hierarchy.setCurrentIndex(0)
            self.ui.checkBox_contour_area.setChecked(False)
            self.ui.lineEdit_minimum_area.setText("100")
            self.ui.lineEdit_maximum_area.setText("1000")
            self.ui.checkBox_show_coordinate.setChecked(False)
        self.image = cv2.imread("obj.jpg")
        if self.ui.checkBox_find_contour.isChecked():
            self.contour,self.uscontour = self.find_contour(self.image)
        else:
            pass
    def export(self):
        cv2.imwrite(self.ui.lineEdit_pic.text(),self.Import)
if __name__ == '__main__':
    def exit_app():
        false.close()
    app = QApplication(sys.argv)
    window = MainWindow()
    false = Warning()
    window.show()
    false.ui.pushButton.clicked.connect(exit_app)
    sys.exit(app.exec_())
