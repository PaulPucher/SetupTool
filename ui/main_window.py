# Main window: topbar, sidebar, content area. Layout and navigation only.

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QLabel, QListWidget, QStackedWidget,QListWidgetItem  )
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import Qt, QSize
from ui.views.weekends import WeekendsView
from ui.views.drivers import DriversView
from ui.views.settings_view import SettingsView
from modules.storage import repo_path

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SetupTool")
        self.resize(1100, 680)

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        root_layout.addWidget(self._build_topbar())

        body = QWidget()
        body_layout = QHBoxLayout(body)
        body_layout.setContentsMargins(0, 0, 0, 0)
        body_layout.setSpacing(0)

        self.nav = self._build_sidebar()
        self.stack = QStackedWidget()

        body_layout.addWidget(self.nav)
        body_layout.addWidget(self.stack)
        root_layout.addWidget(body)

        self._build_pages()
        self.nav.setCurrentRow(0)
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)

    def _build_topbar(self):
        bar = QWidget()
        bar.setFixedHeight(52)
        bar.setStyleSheet("background-color: #1a1a1a; border-bottom: 1px solid #2a2a2a;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(16, 0, 16, 0)

        title = QLabel("SetupTool")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title.setStyleSheet("color: #666; font-size: 16px; letter-spacing: 3px;")

        layout.addWidget(title)

        return bar
   
    def _build_sidebar(self):
        nav = QListWidget()
        nav.setFixedWidth(52)
        nav.setIconSize(QSize(22,22))
        nav.setSpacing(4)
        nav.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        icon_paths = [repo_path("ui", "icons", name)
                      for name in ("weekends.svg", "drivers.svg", "settings.svg")]

        for icon_path in icon_paths:
            item = QListWidgetItem()
            item.setIcon(QIcon(icon_path))
            nav.addItem(item)

        return nav
    

    def _build_pages(self):
        self.stack.addWidget(WeekendsView())
        self.stack.addWidget(DriversView())
        self.stack.addWidget(SettingsView())
        