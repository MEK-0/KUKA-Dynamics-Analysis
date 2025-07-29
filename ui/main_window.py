from PyQt5.QtWidgets import (QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, 
                             QHBoxLayout, QGroupBox, QGridLayout, QFrame, QSplitter,
                             QComboBox, QTabWidget, QTextEdit, QMessageBox, QFileDialog,
                             QProgressBar, QSlider, QSpinBox, QDoubleSpinBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QPalette, QColor, QIcon
from logic.newton_euler import calculate_newton_euler_torque
from logic.lagrange import calculate_lagrange, calculate_lagrange_numerical
from graphics.plotter import PlotWidget
from graphics.robot_visualizer import RobotVisualizer
from robots.kuka_robots import get_available_robots, get_robot_by_name
from robots.kuka_dynamics import (calculate_kuka_newton_euler, calculate_kuka_lagrange,
                                 calculate_kuka_kinetic_energy, calculate_kuka_potential_energy,
                                 get_kuka_robot_info, calculate_kuka_workspace_torques)
from utils.export_utils import RobotResultsExporter
import numpy as np

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("KUKA Dynamics Studio")
        self.setGeometry(100, 100, 1200, 800)
        self.setup_ui()
        self.setup_styles()
        
    def setup_ui(self):
        # Ana layout
        main_layout = QHBoxLayout()
        
        # Sol panel - Kontroller
        left_panel = self.create_control_panel()
        
        # Sağ panel - Grafik ve Sonuçlar
        right_panel = self.create_right_panel()
        
        # Splitter ile bölünmüş layout
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setSizes([450, 750])  # Sol panel 450px, sağ panel 750px
        
        main_layout.addWidget(splitter)
        self.setLayout(main_layout)
        
    def create_control_panel(self):
        panel = QFrame()
        panel.setFrameStyle(QFrame.Box)
        panel.setMaximumWidth(450)
        
        layout = QVBoxLayout()
        
        # Başlık
        title = QLabel("KUKA Dynamics Studio")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #2c3e50; margin: 10px;")
        layout.addWidget(title)
        
        # KUKA Robot Seçimi
        robot_group = self.create_robot_selection_group()
        layout.addWidget(robot_group)
        
        # Tab Widget
        tab_widget = QTabWidget()
        tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 2px solid #bdc3c7;
                border-radius: 5px;
            }
            QTabBar::tab {
                background-color: #ecf0f1;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
            QTabBar::tab:selected {
                background-color: #3498db;
                color: white;
            }
        """)
        
        # Genel Hesaplamalar Tab
        general_tab = self.create_general_tab()
        tab_widget.addTab(general_tab, "General")
        
        # KUKA Robot Tab
        kuka_tab = self.create_kuka_tab()
        tab_widget.addTab(kuka_tab, "KUKA Robots")
        
        layout.addWidget(tab_widget)
        
        # Stretch ekle
        layout.addStretch()
        
        panel.setLayout(layout)
        return panel
        
    def create_newton_euler_group(self):
        group = QGroupBox("Newton-Euler Method")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QGridLayout()
        
        # Input alanları
        self.mass_input = QLineEdit()
        self.mass_input.setPlaceholderText("Enter mass (kg)")
        self.mass_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
        
        self.length_input = QLineEdit()
        self.length_input.setPlaceholderText("Enter length (m)")
        self.length_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
        
        self.acc_input = QLineEdit()
        self.acc_input.setPlaceholderText("Enter angular acceleration (rad/s²)")
        self.acc_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")

        # Buton
        self.calc_button = QPushButton("Calculate Newton-Euler Torque")
        self.calc_button.clicked.connect(self.calculate_newton_euler)
        self.calc_button.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                padding: 10px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #21618c;
            }
        """)
        
        # Layout'a ekle
        layout.addWidget(QLabel("Mass (kg):"), 0, 0)
        layout.addWidget(self.mass_input, 0, 1)
        layout.addWidget(QLabel("Length (m):"), 1, 0)
        layout.addWidget(self.length_input, 1, 1)
        layout.addWidget(QLabel("Angular Acc. (rad/s²):"), 2, 0)
        layout.addWidget(self.acc_input, 2, 1)
        layout.addWidget(self.calc_button, 3, 0, 1, 2)
        
        group.setLayout(layout)
        return group
        
    def create_lagrange_group(self):
        group = QGroupBox("Lagrange Method")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QVBoxLayout()
        
        # Lagrange parametreleri
        param_layout = QGridLayout()
        
        self.m1_input = QLineEdit()
        self.m1_input.setPlaceholderText("Link 1 mass (kg)")
        self.m1_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
        
        self.m2_input = QLineEdit()
        self.m2_input.setPlaceholderText("Link 2 mass (kg)")
        self.m2_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
        
        self.l1_input = QLineEdit()
        self.l1_input.setPlaceholderText("Link 1 length (m)")
        self.l1_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
        
        self.l2_input = QLineEdit()
        self.l2_input.setPlaceholderText("Link 2 length (m)")
        self.l2_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
        
        param_layout.addWidget(QLabel("m₁ (kg):"), 0, 0)
        param_layout.addWidget(self.m1_input, 0, 1)
        param_layout.addWidget(QLabel("m₂ (kg):"), 1, 0)
        param_layout.addWidget(self.m2_input, 1, 1)
        param_layout.addWidget(QLabel("l₁ (m):"), 2, 0)
        param_layout.addWidget(self.l1_input, 2, 1)
        param_layout.addWidget(QLabel("l₂ (m):"), 3, 0)
        param_layout.addWidget(self.l2_input, 3, 1)
        
        layout.addLayout(param_layout)
        
        # Lagrange butonu
        self.lagrange_button = QPushButton("Calculate Lagrange Equations")
        self.lagrange_button.clicked.connect(self.calculate_lagrange)
        self.lagrange_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                padding: 10px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
            QPushButton:pressed {
                background-color: #a93226;
            }
        """)
        layout.addWidget(self.lagrange_button)
        
        group.setLayout(layout)
        return group
        
    def create_result_group(self):
        group = QGroupBox("Results")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QVBoxLayout()
        
        self.result_label = QLabel("Results will be displayed here.")
        self.result_label.setWordWrap(True)
        self.result_label.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                min-height: 60px;
            }
        """)
        
        layout.addWidget(self.result_label)
        group.setLayout(layout)
        return group
        
    def create_robot_selection_group(self):
        group = QGroupBox("KUKA Robot Selection")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QVBoxLayout()
        
        # Robot seçim combobox
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(get_available_robots())
        self.robot_combo.setStyleSheet("""
            QComboBox {
                padding: 8px;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                background-color: white;
            }
            QComboBox::drop-down {
                border: none;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #2c3e50;
            }
        """)
        self.robot_combo.currentTextChanged.connect(self.on_robot_selection_changed)
        
        # Robot bilgi butonu
        self.robot_info_button = QPushButton("Show Robot Info")
        self.robot_info_button.clicked.connect(self.show_robot_info)
        self.robot_info_button.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                padding: 8px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        
        layout.addWidget(QLabel("Select KUKA Robot:"))
        layout.addWidget(self.robot_combo)
        layout.addWidget(self.robot_info_button)
        
        group.setLayout(layout)
        return group
    
    def create_general_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Newton-Euler Grubu
        newton_group = self.create_newton_euler_group()
        layout.addWidget(newton_group)
        
        # Lagrange Grubu
        lagrange_group = self.create_lagrange_group()
        layout.addWidget(lagrange_group)
        
        # Sonuç Grubu
        result_group = self.create_result_group()
        layout.addWidget(result_group)
        
        tab.setLayout(layout)
        return tab
    
    def create_kuka_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # KUKA Newton-Euler Grubu (sadece input alanları)
        kuka_newton_group = self.create_kuka_newton_euler_group()
        layout.addWidget(kuka_newton_group)
        
        # Robot Max Değerleri Grubu
        robot_max_group = self.create_robot_max_values_group()
        layout.addWidget(robot_max_group)
        
        # Stretch ekle
        layout.addStretch()
        
        # En alt kısım: Butonlar
        buttons_group = self.create_kuka_buttons_group()
        layout.addWidget(buttons_group)
        
        tab.setLayout(layout)
        return tab
    
    def create_kuka_newton_euler_group(self):
        group = QGroupBox("KUKA Newton-Euler Analysis")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QGridLayout()
        
        # Başlık satırı ekle
        layout.addWidget(QLabel("Axis"), 0, 0)
        layout.addWidget(QLabel("Angle (rad)"), 0, 1)
        layout.addWidget(QLabel("Velocity (rad/s)"), 0, 2)
        layout.addWidget(QLabel("Acceleration (rad/s²)"), 0, 3)
        
        # Her eksen için ayrı input alanları
        self.joint_angles_inputs = []
        self.joint_velocities_inputs = []
        self.joint_accelerations_inputs = []
        
        for i in range(6):
            # Joint angle input
            angle_input = QLineEdit()
            angle_input.setPlaceholderText(f"Axis {i+1} angle value (0-360°)")
            angle_input.setText("0")
            angle_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
            self.joint_angles_inputs.append(angle_input)
            
            # Joint velocity input
            velocity_input = QLineEdit()
            velocity_input.setPlaceholderText(f"Axis {i+1} angular velocity value")
            velocity_input.setText("0")
            velocity_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
            self.joint_velocities_inputs.append(velocity_input)
            
            # Joint acceleration input
            acceleration_input = QLineEdit()
            acceleration_input.setPlaceholderText(f"Axis {i+1} angular acceleration value")
            acceleration_input.setText("0")
            acceleration_input.setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; }")
            self.joint_accelerations_inputs.append(acceleration_input)
            
            # Layout'a ekle
            layout.addWidget(QLabel(f"Axis {i+1}:"), i+1, 0)
            layout.addWidget(angle_input, i+1, 1)
            layout.addWidget(velocity_input, i+1, 2)
            layout.addWidget(acceleration_input, i+1, 3)
        
        # Açıklama metni ekle
        info_text = QLabel("Angle: Joint angle (in radians)\nVelocity: Joint angular velocity (rad/s)\nAcceleration: Joint angular acceleration (rad/s²)")
        info_text.setStyleSheet("color: #7f8c8d; font-size: 10px; margin-top: 10px;")
        layout.addWidget(info_text, 7, 0, 1, 4)
        
        group.setLayout(layout)
        return group
    
    def create_robot_max_values_group(self):
        group = QGroupBox("Selected Robot Max Values")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QVBoxLayout()
        
        # Robot bilgilerini gösterecek label
        self.robot_max_info_label = QLabel("No robot selected")
        self.robot_max_info_label.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                color: #7f8c8d;
            }
        """)
        self.robot_max_info_label.setWordWrap(True)
        
        layout.addWidget(self.robot_max_info_label)
        
        group.setLayout(layout)
        return group
    
    def create_kuka_buttons_group(self):
        group = QGroupBox("Analysis Buttons")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QHBoxLayout()
        
        # Newton-Euler butonu
        self.kuka_newton_button = QPushButton("Calculate KUKA Newton-Euler")
        self.kuka_newton_button.clicked.connect(self.calculate_kuka_newton_euler)
        self.kuka_newton_button.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                padding: 10px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        
        # Lagrange butonu
        self.kuka_lagrange_button = QPushButton("Calculate KUKA Lagrange")
        self.kuka_lagrange_button.clicked.connect(self.calculate_kuka_lagrange)
        self.kuka_lagrange_button.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c;
                color: white;
                padding: 10px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #c0392b;
            }
        """)
        
        # Workspace butonu
        self.workspace_button = QPushButton("Analyze Workspace Dynamics")
        self.workspace_button.clicked.connect(self.analyze_workspace)
        self.workspace_button.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                padding: 10px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #8e44ad;
            }
        """)
        
        layout.addWidget(self.kuka_newton_button)
        layout.addWidget(self.kuka_lagrange_button)
        layout.addWidget(self.workspace_button)
        
        group.setLayout(layout)
        return group
    
    def create_workspace_analysis_group(self):
        group = QGroupBox("Workspace Analysis")
        group.setFont(QFont("Arial", 12, QFont.Bold))
        
        layout = QVBoxLayout()
        
        # Workspace analysis button
        self.workspace_button = QPushButton("Analyze Workspace Dynamics")
        self.workspace_button.clicked.connect(self.analyze_workspace)
        self.workspace_button.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                padding: 10px;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #8e44ad;
            }
        """)
        
        layout.addWidget(self.workspace_button)
        
        group.setLayout(layout)
        return group
    
    def create_right_panel(self):
        panel = QFrame()
        panel.setFrameStyle(QFrame.Box)
        
        layout = QVBoxLayout()
        
        # Tab widget for results
        results_tab = QTabWidget()
        results_tab.setStyleSheet("""
            QTabWidget::pane {
                border: 2px solid #bdc3c7;
                border-radius: 5px;
            }
            QTabBar::tab {
                background-color: #ecf0f1;
                padding: 8px 16px;
                margin-right: 2px;
                border-top-left-radius: 5px;
                border-top-right-radius: 5px;
            }
            QTabBar::tab:selected {
                background-color: #3498db;
                color: white;
            }
        """)
        
        # Robot Visualization tab
        robot_viz_tab = self.create_robot_visualization_tab()
        results_tab.addTab(robot_viz_tab, "Robot View")
        
        # Grafik tab
        plot_tab = self.create_plot_tab()
        results_tab.addTab(plot_tab, "Graphs")
        
        # Sonuçlar tab
        results_text_tab = self.create_results_text_tab()
        results_tab.addTab(results_text_tab, "Results")
        
        # Export tab
        export_tab = self.create_export_tab()
        results_tab.addTab(export_tab, "Export")
        
        layout.addWidget(results_tab)
        
        panel.setLayout(layout)
        return panel
    
    def create_plot_tab(self):
        tab = QWidget()
        main_layout = QVBoxLayout()

        # Başlık
        plot_title = QLabel("Dynamic Analysis Results")
        plot_title.setFont(QFont("Arial", 14, QFont.Bold))
        plot_title.setAlignment(Qt.AlignCenter)
        plot_title.setStyleSheet("color: #2c3e50; margin: 10px;")
        main_layout.addWidget(plot_title)

        # Üst kısım: Tek grafik alanı
        self.main_plot = PlotWidget()
        main_layout.addWidget(self.main_plot, stretch=2)

        # Alt kısım: Robot özellikleri ve metotlar
        lower_widget = QWidget()
        lower_layout = QHBoxLayout()

        # Sol: Robot özellikleri grafiği
        self.robot_specs_plot = PlotWidget()
        lower_layout.addWidget(self.robot_specs_plot, stretch=2)

        # Sağ: Hesaplama metotları ve açıklamalar
        self.methods_text = QTextEdit()
        self.methods_text.setReadOnly(True)
        self.methods_text.setStyleSheet("background-color: #f8f9fa; border: 1px solid #bdc3c7; border-radius: 5px; padding: 10px;")
        self.methods_text.setMinimumWidth(250)
        self.methods_text.setMaximumWidth(350)
        self.methods_text.setPlaceholderText("Calculation methods and explanations used here will be shown.")
        lower_layout.addWidget(self.methods_text, stretch=1)

        lower_widget.setLayout(lower_layout)
        main_layout.addWidget(lower_widget, stretch=1)

        tab.setLayout(main_layout)
        return tab

    def create_results_text_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Sonuç metni
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setStyleSheet("""
            QTextEdit {
                background-color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                padding: 10px;
                font-family: 'Courier New';
                font-size: 11px;
            }
        """)
        self.results_text.setPlaceholderText("Results will be displayed here...")
        
        layout.addWidget(self.results_text)
        
        tab.setLayout(layout)
        return tab
        
    def setup_styles(self):
        # Siemens SIMATIC HMI Panel Design
        self.setStyleSheet("""
            QWidget {
                background-color: #d3d3d3;
                font-family: 'Arial', sans-serif;
                color: #000000;
            }
            
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #808080, stop:1 #696969);
            }
            
            QGroupBox {
                font-weight: bold;
                font-size: 12px;
                border: 2px solid #696969;
                border-radius: 4px;
                margin-top: 12px;
                padding-top: 12px;
                background-color: #f0f0f0;
                color: #000000;
            }
            
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px 0 6px;
                background-color: #f0f0f0;
                color: #000000;
                font-weight: bold;
            }
            
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #696969, stop:1 #808080);
                color: white;
                border: 2px solid #696969;
                border-radius: 4px;
                padding: 10px 16px;
                font-weight: bold;
                font-size: 11px;
                min-height: 30px;
            }
            
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #808080, stop:1 #696969);
                border-color: #000000;
            }
            
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #404040, stop:1 #696969);
                border-color: #000000;
            }
            
            QLineEdit {
                border: 2px solid #696969;
                border-radius: 4px;
                padding: 8px 12px;
                background-color: #ffffff;
                color: #000000;
                font-size: 11px;
                selection-background-color: #0080ff;
            }
            
            QLineEdit:focus {
                border-color: #0080ff;
                background-color: #ffffff;
            }
            
            QComboBox {
                border: 2px solid #696969;
                border-radius: 4px;
                padding: 8px 12px;
                background-color: #ffffff;
                color: #000000;
                font-size: 11px;
                min-height: 30px;
            }
            
            QComboBox:focus {
                border-color: #0080ff;
            }
            
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 5px solid #000000;
                margin-right: 5px;
            }
            
            QTabWidget::pane {
                border: 2px solid #696969;
                border-radius: 4px;
                background-color: #f0f0f0;
            }
            
            QTabBar::tab {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #d3d3d3, stop:1 #c0c0c0);
                color: #000000;
                padding: 10px 20px;
                margin-right: 2px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-weight: bold;
                font-size: 11px;
                border: 2px solid #696969;
                border-bottom: none;
            }
            
            QTabBar::tab:selected {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #ffffff, stop:1 #f0f0f0);
                color: #000000;
                border-color: #696969;
            }
            
            QTabBar::tab:hover:!selected {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #e0e0e0, stop:1 #d3d3d3);
                color: #000000;
            }
            
            QTextEdit {
                border: 2px solid #696969;
                border-radius: 4px;
                padding: 10px;
                background-color: #ffffff;
                color: #000000;
                font-family: 'Consolas', 'Courier New', monospace;
                font-size: 10px;
                selection-background-color: #0080ff;
            }
            
            QLabel {
                color: #000000;
                font-size: 11px;
            }
            
            QSlider::groove:horizontal {
                border: 2px solid #696969;
                height: 10px;
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #d3d3d3, stop:1 #c0c0c0);
                border-radius: 5px;
            }
            
            QSlider::handle:horizontal {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #696969, stop:1 #808080);
                border: 2px solid #696969;
                width: 20px;
                margin: -2px 0;
                border-radius: 10px;
            }
            
            QSlider::handle:horizontal:hover {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #808080, stop:1 #696969);
                border-color: #000000;
            }
            
            QFrame {
                border: 2px solid #696969;
                border-radius: 4px;
                background-color: #f0f0f0;
            }
            
            /* Siemens HMI status indicators */
            QLabel[class="status-ok"] {
                color: #008000;
                font-weight: bold;
                font-size: 12px;
            }
            
            QLabel[class="status-warning"] {
                color: #ff8000;
                font-weight: bold;
                font-size: 12px;
            }
            
            QLabel[class="status-error"] {
                color: #ff0000;
                font-weight: bold;
                font-size: 12px;
            }
            
            /* Toggle switch styling */
            QPushButton[class="toggle-on"] {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #008000, stop:1 #006000);
                border-color: #008000;
            }
            
            QPushButton[class="toggle-off"] {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #808080, stop:1 #696969);
                border-color: #696969;
            }
        """)

    def calculate_newton_euler(self):
        try:
            mass = float(self.mass_input.text())
            length = float(self.length_input.text())
            acc = float(self.acc_input.text())

            # Basit bir zaman aralığında sabit tork hesaplama (örnek)
            time = np.linspace(0, 10, 100)  # 0-10 saniye
            torque = np.array([calculate_newton_euler_torque(mass, length, acc) for _ in time])

            # Sonuç mesajını güncelle
            result_text = f"""
            <b>Newton-Euler Method Results:</b><br>
            Mass: {mass} kg<br>
            Length: {length} m<br>
            Angular Acceleration: {acc} rad/s²<br>
            <b>Calculated Torque: {torque[0]:.4f} Nm</b>
            """
            self.result_label.setText(result_text)

            # Grafik çizimi (YENİ)
            self.main_plot.plot_torque(time, torque)
            # Store Newton-Euler data for combined plotting
            self.newton_euler_data = (time, torque)

        except ValueError:
            self.result_label.setText("<b style='color: red;'>Error:</b> Please enter valid numbers for all fields.")
        except Exception as e:
            self.result_label.setText(f"<b style='color: red;'>Error:</b> {str(e)}")

    def calculate_lagrange(self):
        try:
            # Get input values from Lagrange fields
            try:
                m1 = float(self.m1_input.text()) if self.m1_input.text() else 1.0
                m2 = float(self.m2_input.text()) if self.m2_input.text() else 1.0
                l1 = float(self.l1_input.text()) if self.l1_input.text() else 1.0
                l2 = float(self.l2_input.text()) if self.l2_input.text() else 1.0
            except ValueError:
                m1, m2, l1, l2 = 1.0, 1.0, 1.0, 1.0

            # Calculate numerical values for plotting
            time, tau1, tau2 = calculate_lagrange_numerical(m1, m2, l1, l2)

            # Sonuç mesajını güncelle
            result_text = f"""
            <b>Lagrange Method Results:</b><br>
            Link 1: m₁={m1} kg, l₁={l1} m<br>
            Link 2: m₂={m2} kg, l₂={l2} m<br>
            <b>Joint 1 Torque: {tau1[0]:.4f} Nm</b><br>
            <b>Joint 2 Torque: {tau2[0]:.4f} Nm</b>
            """
            self.result_label.setText(result_text)

            # Grafik çizimi (YENİ)
            self.main_plot.plot_lagrange_torques(time, tau1, tau2)
            # Store Lagrange data for combined plotting
            self.lagrange_data = (time, tau1, tau2)

        except Exception as e:
            self.result_label.setText(f"<b style='color: red;'>Error:</b> {str(e)}")
    
    def on_robot_selection_changed(self):
        """Handle robot selection change"""
        selected_robot = self.robot_combo.currentText()
        if selected_robot:
            self.update_robot_info_display(selected_robot)
            # --- YENİ: Robotun eksen başına tork değerlerini grafikle çiz ---
            from robots.kuka_dynamics import get_kuka_robot_info
            import numpy as np
            info = get_kuka_robot_info(selected_robot)
            static_torques = info['static_torques']
            joints = np.arange(1, len(static_torques)+1)
            self.robot_specs_plot.figure.clear()
            ax = self.robot_specs_plot.figure.add_subplot(1, 1, 1)
            ax.bar(joints, static_torques, color='#e74c3c', label='Static Torque (Nm)')
            ax.set_xlabel('Joint No')
            ax.set_ylabel('Torque (Nm)')
            ax.set_title('Joint Base Torque Values (Newton-Euler)')
            ax.legend()
            ax.grid(True, alpha=0.3)
            self.robot_specs_plot.figure.tight_layout()
            self.robot_specs_plot.canvas.draw()
            
            # --- YENİ: Robotun max değerlerini göster ---
            max_info_text = f"""
<b>Robot: {selected_robot}</b><br><br>
<b>Max Values:</b><br>
• Max Payload Capacity: {info['max_payload']} kg<br>
• Max Reach: {info['reach']} m<br>
• Max Speed: {info['max_speed']} rad/s<br>
• Repeatability: {info['repeatability']} mm<br>
• Total Mass: {info['total_mass']:.2f} kg<br>
• Total Inertia: {info['total_inertia']:.4f} kg·m²<br><br>
<b>Max Torque Values (Nm):</b><br>
"""
            for i, torque in enumerate(static_torques):
                max_info_text += f"• Joint {i+1}: {torque:.2f} Nm<br>"
            
            self.robot_max_info_label.setText(max_info_text)
    
    def show_robot_info(self):
        """Show detailed robot information"""
        selected_robot = self.robot_combo.currentText()
        if selected_robot:
            try:
                robot_info = get_kuka_robot_info(selected_robot)
                info_text = f"""
KUKA Robot Information:
======================

Name: {robot_info['name']}
Model: {robot_info['model']}
Degrees of Freedom: {robot_info['dof']}
Max Payload: {robot_info['max_payload']} kg
Reach: {robot_info['reach']} m
Repeatability: {robot_info['repeatability']} mm
Max Speed: {robot_info['max_speed']} rad/s
Total Mass: {robot_info['total_mass']:.2f} kg
Total Inertia: {robot_info['total_inertia']:.4f} kg·m²

Static Torques (Nm):
"""
                for i, torque in enumerate(robot_info['static_torques']):
                    info_text += f"  Joint {i+1}: {torque:.4f}\n"
                
                info_text += "\nLink Specifications:\n"
                for i, link in enumerate(robot_info['links']):
                    info_text += f"""
Link {i+1}:
  Mass: {link['mass']} kg
  Length: {link['length']} m
  Inertia: {link['inertia']} kg·m²
  Center of Mass: {link['center_of_mass']} m
  Joint Type: {link['joint_type']}
"""
                
                self.results_text.setText(info_text)
            except Exception as e:
                self.results_text.setText(f"Error loading robot info: {str(e)}")
    
    def update_robot_info_display(self, robot_name):
        """Update robot information display"""
        try:
            robot = get_robot_by_name(robot_name)
            if robot:
                info_text = f"Selected Robot: {robot.name}\n"
                info_text += f"Max Payload: {robot.max_payload} kg\n"
                info_text += f"Reach: {robot.reach} m\n"
                info_text += f"Total Mass: {sum(link.mass for link in robot.links):.2f} kg"
                self.result_label.setText(info_text)
        except Exception as e:
            self.result_label.setText(f"Error: {str(e)}")
    
    def calculate_kuka_newton_euler(self):
        """Calculate KUKA robot torques using Newton-Euler method"""
        try:
            selected_robot = self.robot_combo.currentText()
            if not selected_robot:
                self.results_text.setText("Please select a KUKA robot first.")
                self.methods_text.setText("Please select a KUKA robot first.")
                return
            
            # Parse input values from new input fields
            angles = [float(self.joint_angles_inputs[i].text()) for i in range(6)]
            velocities = [float(self.joint_velocities_inputs[i].text()) for i in range(6)]
            accelerations = [float(self.joint_accelerations_inputs[i].text()) for i in range(6)]
            
            # Calculate torques
            torques = calculate_kuka_newton_euler(selected_robot, angles, velocities, accelerations)
            
            # Display results
            result_text = f"KUKA Newton-Euler Analysis Results:\n"
            result_text += f"Robot: {selected_robot}\n"
            result_text += f"Joint Angles: {angles}\n"
            result_text += f"Joint Velocities: {velocities}\n"
            result_text += f"Joint Accelerations: {accelerations}\n\n"
            result_text += f"Calculated Torques (Nm):\n"
            
            for i, torque in enumerate(torques):
                result_text += f"  Joint {i+1}: {torque:.4f} Nm\n"
            
            self.results_text.setText(result_text)
            self.methods_text.setText(result_text)
            
            # Check safety limits
            violations = self.check_safety_limits(selected_robot, torques)
            
            # Update result label
            if violations:
                self.result_label.setText(f"⚠️ KUKA Newton-Euler: Max Torque = {max(torques):.4f} Nm (SAFETY WARNING)")
                QMessageBox.warning(self, "Safety Warning", 
                                  f"Torque limits exceeded for {selected_robot}:\n" + "\n".join(violations[:3]))
            else:
                self.result_label.setText(f"✅ KUKA Newton-Euler: Max Torque = {max(torques):.4f} Nm")
            
            # Store analysis data for export
            self.current_analysis_data = {
                'robot_specs': get_kuka_robot_info(selected_robot),
                'torque_analysis': {
                    'newton_euler': torques,
                    'lagrange': []
                },
                'warnings': violations
            }
            
        except Exception as e:
            self.results_text.setText(f"Error in KUKA Newton-Euler calculation: {str(e)}")
            self.methods_text.setText(f"Error in KUKA Newton-Euler calculation: {str(e)}")
    
    def calculate_kuka_lagrange(self):
        """Calculate KUKA robot torques using Lagrange method"""
        try:
            selected_robot = self.robot_combo.currentText()
            if not selected_robot:
                self.results_text.setText("Please select a KUKA robot first.")
                self.methods_text.setText("Please select a KUKA robot first.")
                return
            
            # Parse input values from new input fields
            angles = [float(self.joint_angles_inputs[i].text()) for i in range(6)]
            velocities = [float(self.joint_velocities_inputs[i].text()) for i in range(6)]
            accelerations = [float(self.joint_accelerations_inputs[i].text()) for i in range(6)]
            
            # Calculate torques
            torques = calculate_kuka_lagrange(selected_robot, angles, velocities, accelerations)
            
            # Calculate energies
            kinetic_energy = calculate_kuka_kinetic_energy(selected_robot, velocities)
            potential_energy = calculate_kuka_potential_energy(selected_robot, angles)
            
            # Display results
            result_text = f"KUKA Lagrange Analysis Results:\n"
            result_text += f"Robot: {selected_robot}\n"
            result_text += f"Joint Angles: {angles}\n"
            result_text += f"Joint Velocities: {velocities}\n"
            result_text += f"Joint Accelerations: {accelerations}\n\n"
            result_text += f"Calculated Torques (Nm):\n"
            
            for i, torque in enumerate(torques):
                result_text += f"  Joint {i+1}: {torque:.4f} Nm\n"
            
            result_text += f"\nEnergy Analysis:\n"
            result_text += f"  Kinetic Energy: {kinetic_energy:.4f} J\n"
            result_text += f"  Potential Energy: {potential_energy:.4f} J\n"
            result_text += f"  Total Energy: {kinetic_energy + potential_energy:.4f} J\n"
            
            self.results_text.setText(result_text)
            self.methods_text.setText(result_text)
            
            # Check safety limits
            violations = self.check_safety_limits(selected_robot, torques)
            
            # Update result label
            if violations:
                self.result_label.setText(f"⚠️ KUKA Lagrange: Max Torque = {max(torques):.4f} Nm (SAFETY WARNING)")
                QMessageBox.warning(self, "Safety Warning", 
                                  f"Torque limits exceeded for {selected_robot}:\n" + "\n".join(violations[:3]))
            else:
                self.result_label.setText(f"✅ KUKA Lagrange: Max Torque = {max(torques):.4f} Nm")
            
            # Store analysis data for export
            self.current_analysis_data = {
                'robot_specs': get_kuka_robot_info(selected_robot),
                'torque_analysis': {
                    'newton_euler': [],
                    'lagrange': torques
                },
                'energy_analysis': {
                    'kinetic_energy': kinetic_energy,
                    'potential_energy': potential_energy,
                    'total_energy': kinetic_energy + potential_energy
                },
                'warnings': violations
            }
            
        except Exception as e:
            self.results_text.setText(f"Error in KUKA Lagrange calculation: {str(e)}")
            self.methods_text.setText(f"Error in KUKA Lagrange calculation: {str(e)}")
    
    def analyze_workspace(self):
        """Analyze KUKA robot workspace dynamics"""
        try:
            selected_robot = self.robot_combo.currentText()
            if not selected_robot:
                self.results_text.setText("Please select a KUKA robot first.")
                return
            
            # Calculate workspace torques
            time, torque_arrays = calculate_kuka_workspace_torques(selected_robot)
            newton_euler_torques = torque_arrays[0]
            lagrange_torques = torque_arrays[1]
            
            # Display results
            result_text = f"KUKA Workspace Analysis Results:\n"
            result_text += f"Robot: {selected_robot}\n"
            result_text += f"Simulation Time: 10 seconds\n"
            result_text += f"Time Points: {len(time)}\n\n"
            
            # Calculate statistics
            ne_max_torques = np.max(newton_euler_torques, axis=0)
            lag_max_torques = np.max(lagrange_torques, axis=0)
            
            result_text += f"Maximum Torques (Newton-Euler):\n"
            for i, torque in enumerate(ne_max_torques):
                result_text += f"  Joint {i+1}: {torque:.4f} Nm\n"
            
            result_text += f"\nMaximum Torques (Lagrange):\n"
            for i, torque in enumerate(lag_max_torques):
                result_text += f"  Joint {i+1}: {torque:.4f} Nm\n"
            
            self.results_text.setText(result_text)
            
            # Update result label
            max_ne_torque = np.max(ne_max_torques)
            max_lag_torque = np.max(lag_max_torques)
            self.result_label.setText(f"Workspace Analysis: NE Max = {max_ne_torque:.4f} Nm, Lag Max = {max_lag_torque:.4f} Nm")
            
        except Exception as e:
            self.results_text.setText(f"Error in workspace analysis: {str(e)}")
    
    def create_robot_visualization_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Robot visualization widget
        self.robot_viz_widget = PlotWidget()
        self.robot_visualizer = RobotVisualizer(self.robot_viz_widget.figure, self.robot_viz_widget.canvas)
        
        # Joint angle controls
        joint_control_group = QGroupBox("Joint Angle Controls")
        joint_control_group.setFont(QFont("Arial", 10, QFont.Bold))
        
        joint_layout = QGridLayout()
        
        self.joint_sliders = []
        self.joint_labels = []
        
        for i in range(6):
            # Joint label
            label = QLabel(f"Joint {i+1}: 0°")
            self.joint_labels.append(label)
            
            # Joint slider
            slider = QSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setStyleSheet("""
                QSlider::groove:horizontal {
                    border: 1px solid #bdc3c7;
                    height: 8px;
                    background: #ecf0f1;
                    border-radius: 4px;
                }
                QSlider::handle:horizontal {
                    background: #3498db;
                    border: 1px solid #2980b9;
                    width: 18px;
                    margin: -2px 0;
                    border-radius: 9px;
                }
                QSlider::handle:horizontal:hover {
                    background: #2980b9;
                }
            """)
            
            slider.valueChanged.connect(lambda value, idx=i: self.update_joint_angle(idx, value))
            self.joint_sliders.append(slider)
            
            joint_layout.addWidget(label, i, 0)
            joint_layout.addWidget(slider, i, 1)
        
        joint_control_group.setLayout(joint_layout)
        layout.addWidget(joint_control_group)
        layout.addWidget(self.robot_viz_widget)
        
        tab.setLayout(layout)
        return tab
    
    def create_export_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Export options group
        export_group = QGroupBox("Export Options")
        export_group.setFont(QFont("Arial", 12, QFont.Bold))
        
        export_layout = QVBoxLayout()
        
        # Export buttons
        self.export_json_btn = QPushButton("Export to JSON")
        self.export_json_btn.clicked.connect(lambda: self.export_results('json'))
        self.export_json_btn.setStyleSheet("""
            QPushButton {
                background-color: #27ae60;
                color: white;
                padding: 12px;
                border: none;
                border-radius: 6px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #229954;
            }
        """)
        
        self.export_csv_btn = QPushButton("Export to CSV")
        self.export_csv_btn.clicked.connect(lambda: self.export_results('csv'))
        self.export_csv_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                padding: 12px;
                border: none;
                border-radius: 6px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        
        self.export_txt_btn = QPushButton("Export to TXT")
        self.export_txt_btn.clicked.connect(lambda: self.export_results('txt'))
        self.export_txt_btn.setStyleSheet("""
            QPushButton {
                background-color: #9b59b6;
                color: white;
                padding: 12px;
                border: none;
                border-radius: 6px;
                font-weight: bold;
                font-size: 11px;
            }
            QPushButton:hover {
                background-color: #8e44ad;
            }
        """)
        
        export_layout.addWidget(self.export_json_btn)
        export_layout.addWidget(self.export_csv_btn)
        export_layout.addWidget(self.export_txt_btn)
        
        export_group.setLayout(export_layout)
        layout.addWidget(export_group)
        
        # Export status
        self.export_status = QLabel("No analysis results to export yet.")
        self.export_status.setStyleSheet("""
            QLabel {
                padding: 10px;
                background-color: #ecf0f1;
                border: 2px solid #bdc3c7;
                border-radius: 5px;
                color: #7f8c8d;
            }
        """)
        layout.addWidget(self.export_status)
        
        layout.addStretch()
        tab.setLayout(layout)
        return tab
    
    def update_joint_angle(self, joint_idx, value):
        """Update joint angle and redraw robot"""
        # Update label
        self.joint_labels[joint_idx].setText(f"Joint {joint_idx+1}: {value}°")
        
        # Convert to radians
        joint_angles = [np.radians(self.joint_sliders[i].value()) for i in range(6)]
        
        # Get selected robot
        selected_robot = self.robot_combo.currentText()
        if selected_robot:
            # Draw robot
            self.robot_visualizer.draw_kuka_robot(selected_robot, joint_angles)
    
    def check_safety_limits(self, robot_name, torques):
        """Check if torques exceed safety limits"""
        # KUKA robot torque limits (approximate)
        limits = {
            'KR3 R540': [50, 50, 20, 20, 10, 10],
            'KR6 R900': [100, 100, 50, 50, 20, 20],
            'KR10 R1100': [150, 150, 80, 80, 30, 30],
            'KR16 R1610': [200, 200, 120, 120, 50, 50]
        }
        
        robot_limits = limits.get(robot_name, [100] * 6)
        violations = []
        
        for i, (torque, limit) in enumerate(zip(torques, robot_limits)):
            if abs(torque) > limit:
                violations.append(f"Joint {i+1}: {torque:.1f} Nm > {limit} Nm")
        
        return violations
    
    def export_results(self, format_type):
        """Export analysis results"""
        try:
            if not hasattr(self, 'current_analysis_data'):
                QMessageBox.warning(self, "Export Error", "No analysis results to export. Please run an analysis first.")
                return
            
            exporter = RobotResultsExporter()
            robot_name = self.robot_combo.currentText()
            
            if format_type == 'json':
                filename = exporter.export_to_json(robot_name, self.current_analysis_data)
            elif format_type == 'csv':
                filename = exporter.export_to_csv(robot_name, self.current_analysis_data)
            elif format_type == 'txt':
                filename = exporter.export_to_txt(robot_name, self.current_analysis_data)
            else:
                QMessageBox.warning(self, "Export Error", "Unsupported export format.")
                return
            
            # Update status
            self.export_status.setText(f"Exported successfully to: {filename}")
            self.export_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #d5f4e6;
                    border: 2px solid #27ae60;
                    border-radius: 5px;
                    color: #27ae60;
                }
            """)
            
            QMessageBox.information(self, "Export Success", f"Results exported successfully to:\n{filename}")
            
        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export results: {str(e)}")
            self.export_status.setText(f"Export failed: {str(e)}")
            self.export_status.setStyleSheet("""
                QLabel {
                    padding: 10px;
                    background-color: #fadbd8;
                    border: 2px solid #e74c3c;
                    border-radius: 5px;
                    color: #e74c3c;
                }
            """)
