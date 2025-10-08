import sys
from PyQt6.QtWidgets import QApplication
from gui import MainWindow
import cv2

def main():
    try:
        app = QApplication(sys.argv)
        window = MainWindow()
        window.show()
        sys.exit(app.exec())    
    except Exception as e:
        sys.exit(1)

if __name__ == '__main__':
    main()