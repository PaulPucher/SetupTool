import sys
from PyQt6.QtWidgets import QApplication
from ui.main_window import MainWindow
from ui.style import STYLESHEET
from models.db import init_db
init_db()
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet(STYLESHEET)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

