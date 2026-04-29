# Global application stylesheet.
# All colors and styles are defined here and applied once at startup.

STYLESHEET = """
    QMainWindow, QWidget {
        background-color: #141414;
        color: #e0e0e0;
        font-family: system-ui;
        font-size: 13px;
    }

    QLabel {
        color: #e0e0e0;
    }

    QPushButton {
        background-color: #C0A060;
        color: #1a1200;
        border: none;
        border-radius: 6px;
        padding: 6px 14px;
        font-weight: 600;
        font-size: 12px;
    }

    QPushButton:hover {
        background-color: #d4b472;
    }

    QPushButton:pressed {
        background-color: #a88c50;
    }

    QListWidget {
        background-color: #1a1a1a;
        border: none;
        border-right: 1px solid #2a2a2a;
        outline: 0;
        selection-background-color: #252525;
        selection-color: #C0A060;
    }

    QListWidget::item {
        padding: 10px 8px;
        border-bottom: 1px solid #1e1e1e;
        color: #888;
    }

    QListWidget::item:selected {
        background-color: #252525;
        border-left: 2px solid #C0A060;
    }

    QListWidget::item:hover {
        background-color: #1e1e1e;
        color: #888;
    }

    QTableWidget {
        background-color: #141414;
        border: none;
        gridline-color: #1e1e1e;
    }

    QTableWidget::item {
        padding: 8px;
        border-bottom: 1px solid #1e1e1e;
        color: #d0d0d0;
    }

    QTableWidget::item:selected {
        background-color: #1e1e1e;
        color: #e0e0e0;
    }

    QHeaderView::section {
        background-color: #1a1a1a;
        color: #555;
        font-size: 11px;
        padding: 8px;
        border: none;
        border-bottom: 1px solid #222;
        text-transform: uppercase;
    }

    QScrollBar:vertical {
        background: #141414;
        width: 6px;
    }

    QScrollBar::handle:vertical {
        background: #2a2a2a;
        border-radius: 3px;
    }
"""