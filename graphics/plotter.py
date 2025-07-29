from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

class PlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        self.setLayout(layout)

    def plot_torque(self, time, torque):
        self.figure.clear()
        
        # Set figure size
        self.figure.set_size_inches(10, 8)
        
        # Create subplots: Newton-Euler on top, robot specs on bottom
        ax1 = self.figure.add_subplot(2, 1, 1)
        ax2 = self.figure.add_subplot(2, 1, 2)
        
        # Top plot: Newton-Euler
        ax1.plot(time, torque, label='Torque (Nm)', color='#ff0000', linewidth=2)
        ax1.set_xlabel('Time (s)', color='#000000')
        ax1.set_ylabel('Torque (Nm)', color='#000000')
        ax1.set_title('Newton-Euler Torques vs Time', fontsize=12, fontweight='bold', color='#000000')
        ax1.legend(facecolor='#f0f0f0', edgecolor='#696969', labelcolor='#000000')
        ax1.grid(True, alpha=0.3, color='#696969')
        ax1.set_facecolor('#ffffff')
        ax1.tick_params(colors='#000000')
        
        # Bottom plot: Robot specifications comparison
        self.plot_robot_specs_comparison(ax2)
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def plot_lagrange_torques(self, time, tau1, tau2):
        self.figure.clear()
        
        # Set figure size
        self.figure.set_size_inches(10, 8)
        
        # Create subplots: Lagrange on top, robot specs on bottom
        ax1 = self.figure.add_subplot(2, 1, 1)
        ax2 = self.figure.add_subplot(2, 1, 2)
        
        # Top plot: Lagrange
        ax1.plot(time, tau1, label='τ₁ (Joint 1 Torque)', color='#ff0000', linewidth=2)
        ax1.plot(time, tau2, label='τ₂ (Joint 2 Torque)', color='#0080ff', linewidth=2)
        ax1.set_xlabel('Time (s)', color='#000000')
        ax1.set_ylabel('Torque (Nm)', color='#000000')
        ax1.set_title('Lagrange Torques vs Time', fontsize=12, fontweight='bold', color='#000000')
        ax1.legend(facecolor='#f0f0f0', edgecolor='#696969', labelcolor='#000000')
        ax1.grid(True, alpha=0.3, color='#696969')
        ax1.set_facecolor('#ffffff')
        ax1.tick_params(colors='#000000')
        
        # Bottom plot: Robot specifications comparison
        self.plot_robot_specs_comparison(ax2)
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def plot_workspace_analysis(self, time, newton_euler_torques, lagrange_torques):
        """Plot workspace analysis results for KUKA robots"""
        self.figure.clear()
        
        # Set figure size
        self.figure.set_size_inches(10, 8)
        
        # Create subplots: Torques on top, robot specs on bottom
        ax1 = self.figure.add_subplot(2, 1, 1)
        ax2 = self.figure.add_subplot(2, 1, 2)
        
        # Plot combined torques (top)
        colors = ['blue', 'red', 'green', 'orange', 'purple', 'brown']
        
        # Plot Newton-Euler torques
        for i in range(newton_euler_torques.shape[1]):
            ax1.plot(time, newton_euler_torques[:, i], 
                    label=f'NE Joint {i+1}', 
                    color=colors[i % len(colors)], linewidth=2, linestyle='-')
        
        # Plot Lagrange torques with dashed lines
        for i in range(lagrange_torques.shape[1]):
            ax1.plot(time, lagrange_torques[:, i], 
                    label=f'LG Joint {i+1}', 
                    color=colors[i % len(colors)], linewidth=2, linestyle='--')
        
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Torque (Nm)')
        ax1.set_title('KUKA Robot - Combined Torque Analysis', fontsize=12, fontweight='bold')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)
        ax1.set_facecolor('#f8f9fa')
        
        # Bottom plot: Robot specifications comparison
        self.plot_robot_specs_comparison(ax2)
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def plot_combined_analysis(self, time, newton_euler_torque, lagrange_tau1, lagrange_tau2):
        """Plot both Newton-Euler and Lagrange results together"""
        self.figure.clear()
        
        # Set figure size
        self.figure.set_size_inches(10, 8)
        
        # Create subplots: Newton-Euler on top, Lagrange on bottom
        ax1 = self.figure.add_subplot(2, 1, 1)
        ax2 = self.figure.add_subplot(2, 1, 2)
        
        # Top plot: Newton-Euler
        ax1.plot(time, newton_euler_torque, label='Newton-Euler Torque', color='#ff0000', linewidth=2)
        ax1.set_xlabel('Time (s)', color='#000000')
        ax1.set_ylabel('Torque (Nm)', color='#000000')
        ax1.set_title('Newton-Euler Torques vs Time', fontsize=12, fontweight='bold', color='#000000')
        ax1.legend(facecolor='#f0f0f0', edgecolor='#696969', labelcolor='#000000')
        ax1.grid(True, alpha=0.3, color='#696969')
        ax1.set_facecolor('#ffffff')
        ax1.tick_params(colors='#000000')
        
        # Bottom plot: Lagrange
        ax2.plot(time, lagrange_tau1, label='τ₁ (Joint 1 Torque)', color='#ff0000', linewidth=2)
        ax2.plot(time, lagrange_tau2, label='τ₂ (Joint 2 Torque)', color='#0080ff', linewidth=2)
        ax2.set_xlabel('Time (s)', color='#000000')
        ax2.set_ylabel('Torque (Nm)', color='#000000')
        ax2.set_title('Lagrange Torques vs Time', fontsize=12, fontweight='bold', color='#000000')
        ax2.legend(facecolor='#f0f0f0', edgecolor='#696969', labelcolor='#000000')
        ax2.grid(True, alpha=0.3, color='#696969')
        ax2.set_facecolor('#ffffff')
        ax2.tick_params(colors='#000000')
        
        self.figure.tight_layout()
        self.canvas.draw()

    def plot_robot_specs_comparison(self, ax):
        """Plot robot specifications comparison chart"""
        try:
            from robots.kuka_robots import KUKA_ROBOTS
            
            # Get robot data
            robot_names = list(KUKA_ROBOTS.keys())
            max_payloads = [robot.max_payload for robot in KUKA_ROBOTS.values()]
            reaches = [robot.reach for robot in KUKA_ROBOTS.values()]
            max_speeds = [robot.max_speed for robot in KUKA_ROBOTS.values()]
            
            # Create bar chart
            x = range(len(robot_names))
            width = 0.25
            
            # Plot payload comparison
            bars1 = ax.bar([i - width for i in x], max_payloads, width, 
                          label='Max Payload (kg)', color='#ff0000', alpha=0.8)
            
            # Plot reach comparison
            bars2 = ax.bar(x, reaches, width, 
                          label='Reach (m)', color='#0080ff', alpha=0.8)
            
            # Plot speed comparison (scaled down for better visualization)
            scaled_speeds = [speed/10 for speed in max_speeds]  # Scale down by 10
            bars3 = ax.bar([i + width for i in x], scaled_speeds, width, 
                          label='Max Speed (rad/s ÷ 10)', color='#000000', alpha=0.8)
            
            # Customize the plot
            ax.set_xlabel('KUKA Robot Models', color='#000000')
            ax.set_ylabel('Values', color='#000000')
            ax.set_title('Robot Specifications Comparison', fontsize=12, fontweight='bold', color='#000000')
            ax.set_xticks(x)
            ax.set_xticklabels([name.replace(' ', '\n') for name in robot_names], rotation=0, color='#000000')
            ax.legend(facecolor='#f0f0f0', edgecolor='#696969', labelcolor='#000000')
            ax.grid(True, alpha=0.3, color='#696969')
            ax.set_facecolor('#ffffff')
            ax.tick_params(colors='#000000')
            
            # Add value labels on bars
            for bars in [bars1, bars2, bars3]:
                for bar in bars:
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{height:.1f}', ha='center', va='bottom', fontsize=9, color='#000000', weight='bold')
            
        except ImportError:
            # Fallback if robot data is not available
            ax.text(0.5, 0.5, 'Robot data not available', 
                   ha='center', va='center', transform=ax.transAxes,
                   fontsize=12, color='#000000')
            ax.set_title('Robot Specifications', fontsize=12, fontweight='bold', color='#000000')
            ax.set_facecolor('#ffffff')
