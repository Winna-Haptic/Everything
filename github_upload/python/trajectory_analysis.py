#!/usr/bin/env python3
"""
Haptic Glove Trajectory Analysis & Visualization
Real-time and offline analysis of throwing biomechanics

This notebook provides:
- Trajectory visualization from simulation data
- Biomechanical analysis tools
- Golden zone threshold tuning
- Performance metrics calculation
- Calibration data processing
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import signal
from scipy.spatial.transform import Rotation as R
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import warnings
warnings.filterwarnings('ignore')

# Set style
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

class TrajectoryAnalyzer:
    """
    Comprehensive analysis of throwing biomechanics from haptic glove data
    """
    
    def __init__(self, sensor_data_file='simulation_sensor_data.csv', 
                 haptic_log_file='simulation_haptic_log.csv'):
        """
        Initialize analyzer with data files
        
        Args:
            sensor_data_file: CSV file with IMU sensor data
            haptic_log_file: CSV file with haptic feedback log
        """
        self.sensor_data = None
        self.haptic_data = None
        self.sensor_locations = {
            0: 'THUMB_TIP', 1: 'INDEX_TIP', 2: 'MIDDLE_TIP', 
            3: 'RING_TIP', 4: 'PINKY_TIP', 5: 'BACK_OF_HAND',
            6: 'WRIST', 7: 'ELBOW'
        }
        self.haptic_zones = {
            0: 'THUMB', 1: 'INDEX', 2: 'MIDDLE', 3: 'RING',
            4: 'PINKY', 5: 'WRIST_FLEX', 6: 'WRIST_ROT', 7: 'ELBOW'
        }
        
        self.load_data(sensor_data_file, haptic_log_file)
    
    def load_data(self, sensor_file, haptic_file):
        """Load and preprocess data files"""
        try:
            # Load sensor data
            self.sensor_data = pd.read_csv(sensor_file)
            self.sensor_data['timestamp'] = pd.to_datetime(self.sensor_data['timestamp'], unit='us')
            self.sensor_data['sensor_name'] = self.sensor_data['sensor'].map(self.sensor_locations)
            
            # Load haptic data
            self.haptic_data = pd.read_csv(haptic_file)
            self.haptic_data['timestamp'] = pd.to_datetime(self.haptic_data['timestamp'], unit='us')
            self.haptic_data['zone_name'] = self.haptic_data['zone'].map(self.haptic_zones)
            
            print(f"✅ Loaded {len(self.sensor_data)} sensor readings")
            print(f"✅ Loaded {len(self.haptic_data)} haptic events")
            
        except FileNotFoundError as e:
            print(f"❌ Data file not found: {e}")
            print("💡 Run the simulation first to generate data files")
    
    def calculate_joint_angles(self):
        """
        Calculate joint angles from IMU data using sensor fusion
        """
        joint_angles = []
        
        for sensor_id in range(8):
            sensor_subset = self.sensor_data[self.sensor_data['sensor'] == sensor_id].copy()
            
            if len(sensor_subset) > 0:
                # Calculate magnitude of acceleration and angular velocity
                sensor_subset['accel_mag'] = np.sqrt(
                    sensor_subset['accel_x']**2 + 
                    sensor_subset['accel_y']**2 + 
                    sensor_subset['accel_z']**2
                )
                sensor_subset['gyro_mag'] = np.sqrt(
                    sensor_subset['gyro_x']**2 + 
                    sensor_subset['gyro_y']**2 + 
                    sensor_subset['gyro_z']**2
                )
                
                # Estimate joint angles (simplified - in real system use quaternions)
                sensor_subset['joint_angle'] = np.arctan2(
                    sensor_subset['accel_y'], 
                    sensor_subset['accel_x']
                ) * 180 / np.pi
                
                joint_angles.append(sensor_subset)
        
        return pd.concat(joint_angles, ignore_index=True) if joint_angles else pd.DataFrame()
    
    def detect_throw_phases(self, window_size=10):
        """
        Detect throw phases based on overall motion intensity
        """
        # Calculate overall motion intensity
        motion_data = self.sensor_data.groupby('timestamp').agg({
            'accel_x': 'mean', 'accel_y': 'mean', 'accel_z': 'mean',
            'gyro_x': 'mean', 'gyro_y': 'mean', 'gyro_z': 'mean'
        }).reset_index()
        
        motion_data['motion_intensity'] = np.sqrt(
            motion_data['accel_x']**2 + motion_data['accel_y']**2 + 
            motion_data['gyro_x']**2 + motion_data['gyro_y']**2
        )
        
        # Smooth the signal
        motion_data['smooth_intensity'] = signal.savgol_filter(
            motion_data['motion_intensity'], window_size, 3
        )
        
        # Define phase thresholds
        intensity_threshold_low = motion_data['smooth_intensity'].quantile(0.3)
        intensity_threshold_high = motion_data['smooth_intensity'].quantile(0.7)
        
        # Classify phases
        phases = []
        for intensity in motion_data['smooth_intensity']:
            if intensity < intensity_threshold_low:
                phases.append('PREPARATION/RECOVERY')
            elif intensity < intensity_threshold_high:
                phases.append('WIND_UP/FOLLOW_THROUGH')
            else:
                phases.append('ACCELERATION/RELEASE')
        
        motion_data['phase'] = phases
        return motion_data
    
    def analyze_biomechanical_consistency(self):
        """
        Analyze consistency of biomechanical patterns
        """
        joint_data = self.calculate_joint_angles()
        
        consistency_metrics = {}
        
        for sensor_name in self.sensor_locations.values():
            sensor_subset = joint_data[joint_data['sensor_name'] == sensor_name]
            
            if len(sensor_subset) > 10:  # Need enough data points
                # Calculate consistency metrics
                angle_std = sensor_subset['joint_angle'].std()
                velocity_std = sensor_subset['gyro_mag'].std()
                
                consistency_metrics[sensor_name] = {
                    'angle_consistency': 1.0 / (1.0 + angle_std),  # Higher = more consistent
                    'velocity_consistency': 1.0 / (1.0 + velocity_std),
                    'data_points': len(sensor_subset)
                }
        
        return consistency_metrics
    
    def calculate_haptic_effectiveness(self):
        """
        Analyze effectiveness of haptic feedback
        """
        if self.haptic_data.empty:
            return {}
        
        effectiveness = {}
        
        for zone_name in self.haptic_zones.values():
            zone_data = self.haptic_data[self.haptic_data['zone_name'] == zone_name]
            
            if len(zone_data) > 0:
                effectiveness[zone_name] = {
                    'activation_count': len(zone_data),
                    'avg_intensity': zone_data['intensity'].mean(),
                    'avg_duration': zone_data['duration'].mean(),
                    'total_feedback_time': zone_data['duration'].sum()
                }
        
        return effectiveness
    
    def plot_sensor_trajectories(self, figsize=(15, 10)):
        """
        Plot 3D trajectories for all sensors
        """
        fig = make_subplots(
            rows=2, cols=4,
            subplot_titles=list(self.sensor_locations.values()),
            specs=[[{'type': 'scatter3d'} for _ in range(4)] for _ in range(2)]
        )
        
        colors = px.colors.qualitative.Set3
        
        for i, (sensor_id, sensor_name) in enumerate(self.sensor_locations.items()):
            sensor_subset = self.sensor_data[self.sensor_data['sensor'] == sensor_id]
            
            if len(sensor_subset) > 0:
                row = i // 4 + 1
                col = i % 4 + 1
                
                fig.add_trace(
                    go.Scatter3d(
                        x=sensor_subset['accel_x'],
                        y=sensor_subset['accel_y'],
                        z=sensor_subset['accel_z'],
                        mode='lines+markers',
                        name=sensor_name,
                        line=dict(color=colors[i % len(colors)], width=3),
                        marker=dict(size=3)
                    ),
                    row=row, col=col
                )
        
        fig.update_layout(
            title="3D Sensor Trajectories (Acceleration Space)",
            height=800,
            showlegend=False
        )
        
        return fig
    
    def plot_throw_phases(self, figsize=(12, 8)):
        """
        Visualize throw phases over time
        """
        motion_data = self.detect_throw_phases()
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=figsize, sharex=True)
        
        # Plot motion intensity
        ax1.plot(motion_data.index, motion_data['motion_intensity'], 
                alpha=0.3, label='Raw Intensity', color='gray')
        ax1.plot(motion_data.index, motion_data['smooth_intensity'], 
                linewidth=2, label='Smoothed Intensity', color='blue')
        ax1.set_ylabel('Motion Intensity')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_title('Motion Intensity and Detected Phases')
        
        # Plot phases as colored background
        phase_colors = {'PREPARATION/RECOVERY': 'lightblue', 
                       'WIND_UP/FOLLOW_THROUGH': 'lightgreen',
                       'ACCELERATION/RELEASE': 'lightcoral'}
        
        current_phase = motion_data['phase'].iloc[0]
        start_idx = 0
        
        for i, phase in enumerate(motion_data['phase']):
            if phase != current_phase or i == len(motion_data) - 1:
                ax2.axvspan(start_idx, i, alpha=0.3, 
                           color=phase_colors.get(current_phase, 'lightgray'),
                           label=current_phase if current_phase not in ax2.get_legend_handles_labels()[1] else "")
                current_phase = phase
                start_idx = i
        
        ax2.set_xlabel('Time Steps')
        ax2.set_ylabel('Throw Phase')
        ax2.legend()
        ax2.set_ylim(-0.5, 0.5)
        ax2.set_yticks([])
        
        plt.tight_layout()
        return fig
    
    def plot_haptic_feedback_timeline(self, figsize=(14, 8)):
        """
        Visualize haptic feedback events over time
        """
        if self.haptic_data.empty:
            print("No haptic data available")
            return None
        
        fig, ax = plt.subplots(figsize=figsize)
        
        # Create timeline plot
        zones = sorted(self.haptic_data['zone_name'].unique())
        zone_positions = {zone: i for i, zone in enumerate(zones)}
        
        for _, event in self.haptic_data.iterrows():
            zone_pos = zone_positions[event['zone_name']]
            intensity = event['intensity']
            duration = event['duration']
            
            # Plot as horizontal bar with height representing intensity
            ax.barh(zone_pos, duration, left=event.name, height=intensity*0.8, 
                   alpha=0.7, color=plt.cm.viridis(intensity))
        
        ax.set_yticks(range(len(zones)))
        ax.set_yticklabels(zones)
        ax.set_xlabel('Time')
        ax.set_ylabel('Haptic Zones')
        ax.set_title('Haptic Feedback Timeline\n(Bar height = intensity, width = duration)')
        ax.grid(True, alpha=0.3)
        
        # Add colorbar
        sm = plt.cm.ScalarMappable(cmap=plt.cm.viridis, norm=plt.Normalize(0, 1))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_label('Intensity')
        
        plt.tight_layout()
        return fig
    
    def generate_calibration_recommendations(self):
        """
        Generate recommendations for golden zone calibration
        """
        consistency = self.analyze_biomechanical_consistency()
        haptic_effectiveness = self.calculate_haptic_effectiveness()
        
        recommendations = {
            'biomechanical_consistency': consistency,
            'haptic_effectiveness': haptic_effectiveness,
            'recommendations': []
        }
        
        # Analyze consistency and make recommendations
        for sensor_name, metrics in consistency.items():
            if metrics['angle_consistency'] < 0.5:
                recommendations['recommendations'].append(
                    f"⚠️  {sensor_name}: Low angle consistency ({metrics['angle_consistency']:.2f}). "
                    f"Consider widening golden zone tolerance."
                )
            elif metrics['angle_consistency'] > 0.8:
                recommendations['recommendations'].append(
                    f"✅ {sensor_name}: High consistency ({metrics['angle_consistency']:.2f}). "
                    f"Golden zone can be tightened for more precise feedback."
                )
        
        # Analyze haptic effectiveness
        for zone_name, metrics in haptic_effectiveness.items():
            if metrics['activation_count'] < 5:
                recommendations['recommendations'].append(
                    f"⚠️  {zone_name}: Low activation count ({metrics['activation_count']}). "
                    f"Golden zone might be too restrictive."
                )
            elif metrics['avg_intensity'] < 0.3:
                recommendations['recommendations'].append(
                    f"💡 {zone_name}: Low average intensity ({metrics['avg_intensity']:.2f}). "
                    f"Consider increasing feedback intensity."
                )
        
        return recommendations
    
    def export_analysis_report(self, filename='haptic_glove_analysis_report.html'):
        """
        Generate comprehensive HTML analysis report
        """
        from plotly.offline import plot
        
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Haptic Glove Analysis Report</title>
            <style>
                body {{ font-family: Arial, sans-serif; margin: 40px; }}
                h1, h2 {{ color: #2c3e50; }}
                .metric {{ background: #ecf0f1; padding: 15px; margin: 10px 0; border-radius: 5px; }}
                .recommendation {{ background: #e8f5e8; padding: 10px; margin: 5px 0; border-left: 4px solid #27ae60; }}
                .warning {{ background: #fdf2e9; border-left: 4px solid #e67e22; }}
            </style>
        </head>
        <body>
            <h1>🎯 Haptic Glove Performance Analysis</h1>
            
            <h2>📊 System Overview</h2>
            <div class="metric">
                <strong>Sensor Data Points:</strong> {len(self.sensor_data)}<br>
                <strong>Haptic Events:</strong> {len(self.haptic_data)}<br>
                <strong>Analysis Duration:</strong> {(self.sensor_data['timestamp'].max() - self.sensor_data['timestamp'].min()).total_seconds():.1f} seconds
            </div>
        """
        
        # Add recommendations
        recommendations = self.generate_calibration_recommendations()
        html_content += "<h2>🔧 Calibration Recommendations</h2>"
        
        for rec in recommendations['recommendations']:
            css_class = "warning" if "⚠️" in rec else "recommendation"
            html_content += f'<div class="recommendation {css_class}">{rec}</div>'
        
        html_content += """
            </body>
        </html>
        """
        
        with open(filename, 'w') as f:
            f.write(html_content)
        
        print(f"✅ Analysis report exported to {filename}")

def main():
    """
    Main analysis function - run this to analyze simulation data
    """
    print("🎯 Haptic Glove Trajectory Analysis")
    print("==================================")
    
    # Initialize analyzer
    analyzer = TrajectoryAnalyzer()
    
    if analyzer.sensor_data is None or analyzer.haptic_data is None:
        print("❌ No data available. Run the simulation first!")
        return
    
    # Generate visualizations
    print("📈 Generating trajectory visualization...")
    trajectory_fig = analyzer.plot_sensor_trajectories()
    trajectory_fig.write_html("sensor_trajectories.html")
    
    print("📊 Analyzing throw phases...")
    phase_fig = analyzer.plot_throw_phases()
    phase_fig.savefig("throw_phases.png", dpi=300, bbox_inches='tight')
    
    print("🎯 Visualizing haptic feedback...")
    haptic_fig = analyzer.plot_haptic_feedback_timeline()
    if haptic_fig:
        haptic_fig.savefig("haptic_timeline.png", dpi=300, bbox_inches='tight')
    
    # Generate analysis report
    print("📋 Generating analysis report...")
    analyzer.export_analysis_report()
    
    # Print summary
    consistency = analyzer.analyze_biomechanical_consistency()
    effectiveness = analyzer.calculate_haptic_effectiveness()
    
    print("\n✅ Analysis Complete!")
    print(f"📊 Biomechanical consistency analyzed for {len(consistency)} sensors")
    print(f"🎯 Haptic effectiveness analyzed for {len(effectiveness)} zones")
    print("📁 Generated files:")
    print("   • sensor_trajectories.html")
    print("   • throw_phases.png") 
    print("   • haptic_timeline.png")
    print("   • haptic_glove_analysis_report.html")

if __name__ == "__main__":
    main()
