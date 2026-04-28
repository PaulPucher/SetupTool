import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt6.QtCore import Qt

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SetupTool")
        self.resize(900, 600)

        label = QLabel("SetupTool - Phase 1 OK", self)
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setCentralWidget(label)
if __name__ == "__main__":
    qpp = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(qpp.exec())