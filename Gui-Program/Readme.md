GUI application is used to control the robot via computer. the application is made using the python programming language and the graphics are made with QT5

used library
1. QT5 (pyqt5)
2. openCV
3. Serial Comunication
4. numpy

"GUI Aplication.py" >> This is the main file where all the code for calculations, button functions and image processing is placed. 
                        change this file if you want to change the function of the application

"main_gui.ui & main_gui.py" >> Both of these files contain code for application views such as buttons, boxes, images and other controls. 
                                the ui extension can be edited with QT5 software, this file is code in QML format. 
                                the py extension is ported from ui extension using pyqt5 library. 
                                the py extension  will be imported into the GUI Application file so that the application can display a GUI that has been edited using QT5.

"rsc.qrc & rsc_rc.py" >> file that contain the logo icon for main application
