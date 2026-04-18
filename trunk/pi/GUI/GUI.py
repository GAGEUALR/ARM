from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class StatusLED(QWidget):
    def __init__(self, labelText, buttonText="Details", color="#808080"):
        super().__init__()

        self.led = QLabel()
        self.led.setFixedSize(20, 20)
        self.led.setStyleSheet(
            f"""
            QLabel {{
                background-color: {color};
                border: 2px solid #202020;
                border-radius: 10px;
            }}
            """
        )

        self.label = QLabel(labelText)
        self.label.setAlignment(Qt.AlignCenter)

        self.button = QPushButton(buttonText)
        self.button.setMinimumHeight(32)

        layout = QVBoxLayout()
        layout.setSpacing(8)
        layout.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.led, alignment=Qt.AlignCenter)
        layout.addWidget(self.label)
        layout.addWidget(self.button)
        self.setLayout(layout)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.centralWidget = QWidget()
        self.setCentralWidget(self.centralWidget)

        self.mainLayout = QVBoxLayout()
        self.centralWidget.setLayout(self.mainLayout)

        self.title = QLabel("Home")
        self.title.setObjectName("pageTitle")

        self.statusPanel = QFrame()
        self.statusPanel.setObjectName("panel")
        self.statusLayout = QHBoxLayout()
        self.statusPanel.setLayout(self.statusLayout)

        self.systemLed = StatusLED("System", "Details", "#32CD32")
        self.espLed = StatusLED("ESP Comm", "Configure", "#32CD32")
        self.servoLed = StatusLED("Servo Current", "Details", "#FFD700")
        self.cameraLed = StatusLED("Camera", "Configure", "#32CD32")
        self.tofLed = StatusLED("ToF", "Configure", "#32CD32")

        self.contentLayout = QGridLayout()

        self.cameraPanel = QFrame()
        self.cameraPanel.setObjectName("panel")
        self.cameraLayout = QVBoxLayout()
        self.cameraPanel.setLayout(self.cameraLayout)

        self.cameraTitle = QLabel("Camera Output")
        self.cameraTitle.setObjectName("sectionTitle")

        self.cameraView = QLabel("Placeholder Camera Feed")
        self.cameraView.setAlignment(Qt.AlignCenter)
        self.cameraView.setMinimumHeight(360)
        self.cameraView.setObjectName("cameraView")

        self.infoPanel = QFrame()
        self.infoPanel.setObjectName("panel")
        self.infoLayout = QVBoxLayout()
        self.infoPanel.setLayout(self.infoLayout)

        self.infoTitle = QLabel("System Summary")
        self.infoTitle.setObjectName("sectionTitle")

        self.armState = QLabel("Arm State: Idle")
        self.espState = QLabel("ESP: Connected")
        self.tofState = QLabel("ToF: Valid")
        self.cameraState = QLabel("Camera: Ready")

        self.modePanel = QFrame()
        self.modePanel.setObjectName("panel")
        self.modeLayout = QVBoxLayout()
        self.modePanel.setLayout(self.modeLayout)

        self.modeTitle = QLabel("Modes")
        self.modeTitle.setObjectName("sectionTitle")

        self.controllerButton = QPushButton("Controller Mode")
        self.controllerButton.setMinimumHeight(56)

        self.chessButton = QPushButton("Chess Mode")
        self.chessButton.setMinimumHeight(56)

        self.settingsButton = QPushButton("Settings")
        self.settingsButton.setMinimumHeight(56)

        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background-color: #1e1f22;
                color: #f0f0f0;
                font-size: 15px;
            }

            QLabel#pageTitle {
                font-size: 28px;
                font-weight: bold;
                padding: 4px 0px;
            }

            QLabel#sectionTitle {
                font-size: 18px;
                font-weight: bold;
            }

            QFrame#panel {
                background-color: #2b2d31;
                border: 1px solid #3a3d42;
                border-radius: 12px;
            }

            QLabel#cameraView {
                background-color: #151618;
                border: 1px solid #3a3d42;
                border-radius: 8px;
                font-size: 18px;
            }

            QPushButton {
                background-color: #3b82f6;
                color: white;
                border: none;
                border-radius: 10px;
                padding: 12px;
                font-size: 16px;
                font-weight: bold;
            }

            QPushButton:hover {
                background-color: #2563eb;
            }

            QPushButton:pressed {
                background-color: #1d4ed8;
            }
            """
        )

        self.load_main_gui()

    def load_main_gui(self):
        self.resize(1280, 720)
        self.setWindowTitle("ARM Control GUI")

        self.mainLayout.setContentsMargins(20, 20, 20, 20)
        self.mainLayout.setSpacing(16)

        self.mainLayout.addWidget(self.title)
        self.mainLayout.addWidget(self.statusPanel)
        self.mainLayout.addLayout(self.contentLayout)

        self.statusLayout.setContentsMargins(20, 20, 20, 20)
        self.statusLayout.setSpacing(28)

        self.statusLayout.addWidget(self.systemLed)
        self.statusLayout.addWidget(self.espLed)
        self.statusLayout.addWidget(self.servoLed)
        self.statusLayout.addWidget(self.cameraLed)
        self.statusLayout.addWidget(self.tofLed)

        self.contentLayout.setHorizontalSpacing(16)
        self.contentLayout.setVerticalSpacing(16)

        self.cameraLayout.setContentsMargins(16, 16, 16, 16)
        self.cameraLayout.setSpacing(12)
        self.cameraLayout.addWidget(self.cameraTitle)
        self.cameraLayout.addWidget(self.cameraView)

        self.infoLayout.setContentsMargins(16, 16, 16, 16)
        self.infoLayout.setSpacing(12)
        self.infoLayout.addWidget(self.infoTitle)
        self.infoLayout.addWidget(self.armState)
        self.infoLayout.addWidget(self.espState)
        self.infoLayout.addWidget(self.tofState)
        self.infoLayout.addWidget(self.cameraState)
        self.infoLayout.addStretch()

        self.modeLayout.setContentsMargins(16, 16, 16, 16)
        self.modeLayout.setSpacing(12)
        self.modeLayout.addWidget(self.modeTitle)
        self.modeLayout.addWidget(self.controllerButton)
        self.modeLayout.addWidget(self.chessButton)
        self.modeLayout.addWidget(self.settingsButton)
        self.modeLayout.addStretch()

        self.contentLayout.addWidget(self.cameraPanel, 0, 0, 2, 2)
        self.contentLayout.addWidget(self.infoPanel, 0, 2)
        self.contentLayout.addWidget(self.modePanel, 1, 2)


if __name__ == "__main__":
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()