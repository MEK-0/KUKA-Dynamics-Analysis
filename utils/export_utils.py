# utils/export_utils.py

import json
import csv
from datetime import datetime
from typing import Dict, List, Any

class RobotResultsExporter:
    def __init__(self):
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def export_to_json(self, robot_name: str, analysis_data: Dict[str, Any], filename: str = None) -> str:
        """Export robot analysis results to JSON format"""
        if filename is None:
            filename = f"robot_analysis_{robot_name.replace(' ', '_')}_{self.timestamp}.json"
        
        export_data = {
            "robot_name": robot_name,
            "analysis_timestamp": datetime.now().isoformat(),
            "analysis_data": analysis_data
        }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(export_data, f, indent=2, ensure_ascii=False)
        
        return filename
    
    def export_to_csv(self, robot_name: str, analysis_data: Dict[str, Any], filename: str = None) -> str:
        """Export robot analysis results to CSV format"""
        if filename is None:
            filename = f"robot_analysis_{robot_name.replace(' ', '_')}_{self.timestamp}.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # Write header
            writer.writerow(["Robot Analysis Report"])
            writer.writerow([f"Robot: {robot_name}"])
            writer.writerow([f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"])
            writer.writerow([])
            
            # Write robot specifications
            if 'robot_specs' in analysis_data:
                writer.writerow(["Robot Specifications"])
                specs = analysis_data['robot_specs']
                for key, value in specs.items():
                    writer.writerow([key, value])
                writer.writerow([])
            
            # Write torque analysis
            if 'torque_analysis' in analysis_data:
                writer.writerow(["Torque Analysis"])
                torques = analysis_data['torque_analysis']
                writer.writerow(["Joint", "Newton-Euler (Nm)", "Lagrange (Nm)", "Status"])
                
                for i, (ne_torque, lag_torque) in enumerate(zip(torques.get('newton_euler', []), 
                                                              torques.get('lagrange', []))):
                    status = "OK" if abs(ne_torque) < 100 and abs(lag_torque) < 100 else "WARNING"
                    writer.writerow([f"Joint {i+1}", f"{ne_torque:.4f}", f"{lag_torque:.4f}", status])
                writer.writerow([])
            
            # Write energy analysis
            if 'energy_analysis' in analysis_data:
                writer.writerow(["Energy Analysis"])
                energy = analysis_data['energy_analysis']
                for key, value in energy.items():
                    writer.writerow([key, f"{value:.4f}"])
                writer.writerow([])
            
            # Write warnings
            if 'warnings' in analysis_data and analysis_data['warnings']:
                writer.writerow(["Warnings"])
                for warning in analysis_data['warnings']:
                    writer.writerow([warning])
        
        return filename
    
    def export_to_txt(self, robot_name: str, analysis_data: Dict[str, Any], filename: str = None) -> str:
        """Export robot analysis results to human-readable text format"""
        if filename is None:
            filename = f"robot_analysis_{robot_name.replace(' ', '_')}_{self.timestamp}.txt"
        
        with open(filename, 'w', encoding='utf-8') as f:
            f.write("=" * 60 + "\n")
            f.write(f"KUKA ROBOT ANALYSIS REPORT\n")
            f.write("=" * 60 + "\n")
            f.write(f"Robot Model: {robot_name}\n")
            f.write(f"Analysis Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 60 + "\n\n")
            
            # Robot specifications
            if 'robot_specs' in analysis_data:
                f.write("ROBOT SPECIFICATIONS:\n")
                f.write("-" * 30 + "\n")
                specs = analysis_data['robot_specs']
                for key, value in specs.items():
                    f.write(f"{key}: {value}\n")
                f.write("\n")
            
            # Torque analysis
            if 'torque_analysis' in analysis_data:
                f.write("TORQUE ANALYSIS:\n")
                f.write("-" * 30 + "\n")
                torques = analysis_data['torque_analysis']
                
                if 'newton_euler' in torques:
                    f.write("Newton-Euler Method:\n")
                    for i, torque in enumerate(torques['newton_euler']):
                        status = "✓" if abs(torque) < 100 else "⚠"
                        f.write(f"  Joint {i+1}: {torque:.4f} Nm {status}\n")
                    f.write("\n")
                
                if 'lagrange' in torques:
                    f.write("Lagrange Method:\n")
                    for i, torque in enumerate(torques['lagrange']):
                        status = "✓" if abs(torque) < 100 else "⚠"
                        f.write(f"  Joint {i+1}: {torque:.4f} Nm {status}\n")
                    f.write("\n")
            
            # Energy analysis
            if 'energy_analysis' in analysis_data:
                f.write("ENERGY ANALYSIS:\n")
                f.write("-" * 30 + "\n")
                energy = analysis_data['energy_analysis']
                for key, value in energy.items():
                    f.write(f"{key}: {value:.4f} J\n")
                f.write("\n")
            
            # Warnings
            if 'warnings' in analysis_data and analysis_data['warnings']:
                f.write("WARNINGS:\n")
                f.write("-" * 30 + "\n")
                for warning in analysis_data['warnings']:
                    f.write(f"⚠ {warning}\n")
                f.write("\n")
            
            # Summary
            f.write("SUMMARY:\n")
            f.write("-" * 30 + "\n")
            if 'torque_analysis' in analysis_data:
                ne_torques = torques.get('newton_euler', [])
                lag_torques = torques.get('lagrange', [])
                if ne_torques:
                    max_ne = max(abs(t) for t in ne_torques)
                    f.write(f"Max Newton-Euler Torque: {max_ne:.4f} Nm\n")
                if lag_torques:
                    max_lag = max(abs(t) for t in lag_torques)
                    f.write(f"Max Lagrange Torque: {max_lag:.4f} Nm\n")
            
            f.write("=" * 60 + "\n")
            f.write("Analysis completed successfully.\n")
            f.write("=" * 60 + "\n")
        
        return filename
    
    def create_analysis_report(self, robot_name: str, newton_euler_torques: List[float], 
                              lagrange_torques: List[float], energy_data: Dict[str, float] = None,
                              warnings: List[str] = None) -> Dict[str, Any]:
        """Create a comprehensive analysis report"""
        from robots.kuka_robots import get_robot_by_name
        
        robot = get_robot_by_name(robot_name)
        robot_specs = {}
        
        if robot:
            robot_specs = {
                "Model": robot.model,
                "Degrees of Freedom": robot.dof,
                "Max Payload": f"{robot.max_payload} kg",
                "Reach": f"{robot.reach} m",
                "Repeatability": f"{robot.repeatability} mm",
                "Max Speed": f"{robot.max_speed} rad/s",
                "Total Mass": f"{sum(link.mass for link in robot.links):.2f} kg"
            }
        
        analysis_data = {
            "robot_specs": robot_specs,
            "torque_analysis": {
                "newton_euler": newton_euler_torques,
                "lagrange": lagrange_torques
            },
            "energy_analysis": energy_data or {},
            "warnings": warnings or []
        }
        
        return analysis_data 