
#launches ui template

#from pyqtgraph.Qt import QtGui, QtCore
from PyQt5 import QtWidgets, uic
import sys, subprocess

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi('layout_test.ui', self) # Load the .ui file
        self.Camera.setText("changed the text!") #example of accessing custom named widgets from .ui file. If doesn't work for some widgets, try finding them with findChildren
        self.RunButton.clicked.connect(self.RunButton_pressed)
        self.show() # Show the GUI
        self.i = 0
    
    def RunButton_pressed(self):
        # need to shutdown flexbe BEFORE closing main window
        if self.i == 0:
            # launches flexbe fine. If need to manage inputs/outputs: use the communicate() method
            # for apparatus launch: need script to automate connection and pass in infrastructure_raspi launch command with apparatus name
            self.flexbe_launch = subprocess.Popen(["roslaunch", "infrastructure_flexbe_behaviors", "start_test.launch", "collect_data:=true", "video:=true"])
        self.Camera.setText("Run button was pressed! {}".format(self.i))
        self.i += 1

    # override default closeEvent of MainWindow object. Performance issues? - python crashed once
    def closeEvent(self, event):
        if self.i > 0:
            if self.flexbe_launch.poll() == None:
                self.Camera.setText("You must close down flexbe before closing this window")
                event.ignore()
            else:
                event.accept()
        else:
            event.accept()


if __name__ == "__main__":
    #app = QtGui.QApplication([])
    #label = QtGui.QLabel('Hello World!')
    #label.show()
    #QtGui.QApplication.instance().exec_()
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()