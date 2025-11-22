from uitest import Ui_Form
import sys
from PyQt5 import QtWidgets

class mywindowsshow(QtWidgets.QWidget):
    def __init__(self):
        super(mywindowsshow, self).__init__()
        self.new = Ui_Form()
        self.new.setupUi(self)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    myshow = mywindowsshow()
    myshow.show()
    sys.exit(app.exec_())
