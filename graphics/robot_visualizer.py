# graphics/robot_visualizer.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon
import matplotlib.patches as mpatches

class RobotVisualizer:
    def __init__(self, figure, canvas):
        self.figure = figure
        self.canvas = canvas
        
    def draw_kuka_robot(self, robot_name, joint_angles, show_limits=True):
        """Draw KUKA robot with given joint angles"""
        self.figure.clear()
        
        # Set figure size and style
        self.figure.set_size_inches(8, 6)
        ax = self.figure.add_subplot(111, aspect='equal')
        
        # Robot dimensions (simplified KUKA robot)
        link_lengths = [0.25, 0.56, 0.035, 0.0, 0.0, 0.0]  # Base to end-effector
        joint_radii = [0.05, 0.04, 0.03, 0.02, 0.02, 0.02]
        
        # Calculate positions
        positions = self.calculate_robot_positions(joint_angles, link_lengths)
        
        # Draw robot
        self.draw_robot_links(ax, positions, link_lengths, joint_radii)
        
        # Draw workspace limits if requested
        if show_limits:
            self.draw_workspace_limits(ax, robot_name)
        
        # Set plot properties
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_title(f'KUKA {robot_name} - Robot Configuration', fontsize=14, fontweight='bold', color='#000000')
        ax.set_xlabel('X (m)', color='#000000')
        ax.set_ylabel('Y (m)', color='#000000')
        ax.grid(True, alpha=0.3, color='#696969')
        ax.set_facecolor('#ffffff')
        ax.tick_params(colors='#000000')
        
        self.figure.tight_layout()
        self.canvas.draw()
    
    def calculate_robot_positions(self, joint_angles, link_lengths):
        """Calculate positions of robot joints"""
        positions = [(0, 0)]  # Base position
        
        x, y = 0, 0
        angle_sum = 0
        
        for i, (angle, length) in enumerate(zip(joint_angles, link_lengths)):
            angle_sum += angle
            x += length * np.cos(angle_sum)
            y += length * np.sin(angle_sum)
            positions.append((x, y))
        
        return positions
    
    def draw_robot_links(self, ax, positions, link_lengths, joint_radii):
        """Draw robot links and joints"""
        colors = ['#2c3e50', '#34495e', '#7f8c8d', '#95a5a6', '#bdc3c7', '#ecf0f1']
        
        # Draw links
        for i in range(len(positions) - 1):
            x1, y1 = positions[i]
            x2, y2 = positions[i + 1]
            
            # Draw link
            ax.plot([x1, x2], [y1, y2], color=colors[i % len(colors)], 
                   linewidth=8, solid_capstyle='round', alpha=0.8)
            
            # Draw joint
            joint = Circle((x1, y1), joint_radii[i], facecolor='#e74c3c', 
                          edgecolor='#c0392b', linewidth=2)
            ax.add_patch(joint)
        
        # Draw end-effector
        if positions:
            x, y = positions[-1]
            end_effector = Circle((x, y), 0.03, facecolor='#27ae60', 
                                edgecolor='#229954', linewidth=2)
            ax.add_patch(end_effector)
            
            # Add end-effector label
            ax.text(x + 0.05, y + 0.05, 'EE', fontsize=10, fontweight='bold', 
                   color='#27ae60')
    
    def draw_workspace_limits(self, ax, robot_name):
        """Draw workspace limits for different KUKA robots"""
        limits = {
            'KR3 R540': 0.541,
            'KR6 R900': 0.901,
            'KR10 R1100': 1.101,
            'KR16 R1610': 1.61
        }
        
        reach = limits.get(robot_name, 1.0)
        
        # Draw workspace circle
        workspace = Circle((0, 0), reach, fill=False, color='#3498db', 
                          linestyle='--', linewidth=2, alpha=0.7)
        ax.add_patch(workspace)
        
        # Add workspace label
        ax.text(0, reach + 0.1, f'Workspace\n(Reach: {reach}m)', 
               ha='center', va='bottom', fontsize=10, 
               bbox=dict(boxstyle="round,pad=0.3", facecolor='#3498db', alpha=0.3))
    
    def draw_safety_warning(self, ax, message):
        """Draw safety warning on the plot"""
        # Add warning box
        warning_box = FancyBboxPatch((0.1, 0.8), 0.8, 0.15, 
                                   boxstyle="round,pad=0.02",
                                   facecolor='#e74c3c', alpha=0.9,
                                   edgecolor='#c0392b', linewidth=2)
        ax.add_patch(warning_box)
        
        # Add warning text
        ax.text(0.5, 0.875, '⚠️ SAFETY WARNING ⚠️', 
               ha='center', va='center', fontsize=12, fontweight='bold', 
               color='white')
        ax.text(0.5, 0.825, message, 
               ha='center', va='center', fontsize=10, 
               color='white', wrap=True)
    
    def draw_torque_limits(self, ax, robot_name, calculated_torques):
        """Draw torque limits and warnings"""
        # KUKA robot torque limits (approximate)
        limits = {
            'KR3 R540': [50, 50, 20, 20, 10, 10],
            'KR6 R900': [100, 100, 50, 50, 20, 20],
            'KR10 R1100': [150, 150, 80, 80, 30, 30],
            'KR16 R1610': [200, 200, 120, 120, 50, 50]
        }
        
        robot_limits = limits.get(robot_name, [100] * 6)
        
        # Check for limit violations
        violations = []
        for i, (torque, limit) in enumerate(zip(calculated_torques, robot_limits)):
            if abs(torque) > limit:
                violations.append(f"Joint {i+1}: {torque:.1f} Nm > {limit} Nm")
        
        if violations:
            warning_msg = "Torque limits exceeded:\n" + "\n".join(violations[:3])
            if len(violations) > 3:
                warning_msg += f"\n... and {len(violations) - 3} more"
            
            self.draw_safety_warning(ax, warning_msg)
            
        return violations 