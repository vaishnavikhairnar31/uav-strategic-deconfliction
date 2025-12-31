"""
Visualization module for UAV deconfliction system
Creates 2D, 3D, and 4D (3D + time) visualizations
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D
from typing import List, Tuple
import matplotlib.patches as patches

from uav_deconfliction_main import Mission, Waypoint, DeconflictionSystem, Conflict


class DeconflictionVisualizer:
    """Handles all visualization for the deconfliction system"""
    
    def __init__(self, system: DeconflictionSystem):
        self.system = system
        self.colors = ['red', 'blue', 'green', 'orange', 'purple', 'cyan']
    
    def plot_2d_scenario(self, primary: Mission, conflicts: List[Conflict] = None,
                        filename: str = None, show: bool = True):
        """Create 2D top-down view of flight paths"""
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # Plot primary mission
        self._plot_mission_2d(ax, primary, 'red', 'Primary', linewidth=3)
        
        # Plot simulated flights
        for i, sim in enumerate(self.system.simulated_flights):
            color = self.colors[i % len(self.colors)]
            self._plot_mission_2d(ax, sim, color, sim.drone_id, linewidth=2, alpha=0.7)
        
        # Mark conflicts
        if conflicts:
            conflict_x = [c.location.x for c in conflicts]
            conflict_y = [c.location.y for c in conflicts]
            ax.scatter(conflict_x, conflict_y, c='red', s=200, marker='X',
                      edgecolors='black', linewidths=2, label='Conflicts', zorder=10)
            
            # Draw safety circles at conflict points
            for conflict in conflicts[::max(1, len(conflicts)//5)]:  # Show subset
                circle = patches.Circle(
                    (conflict.location.x, conflict.location.y),
                    self.system.safety_buffer,
                    color='red', fill=False, linestyle='--', alpha=0.3
                )
                ax.add_patch(circle)
        
        ax.set_xlabel('X Position (m)', fontsize=12)
        ax.set_ylabel('Y Position (m)', fontsize=12)
        ax.set_title('UAV Flight Paths - Top View', fontsize=14, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        plt.tight_layout()
        
        if filename:
            plt.savefig(filename, dpi=300, bbox_inches='tight')
        if show:
            plt.show()
        
        return fig, ax
    
    def plot_3d_scenario(self, primary: Mission, conflicts: List[Conflict] = None,
                        filename: str = None, show: bool = True):
        """Create 3D visualization of flight paths"""
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        # Plot primary mission
        self._plot_mission_3d(ax, primary, 'red', 'Primary', linewidth=3)
        
        # Plot simulated flights
        for i, sim in enumerate(self.system.simulated_flights):
            color = self.colors[i % len(self.colors)]
            self._plot_mission_3d(ax, sim, color, sim.drone_id, linewidth=2, alpha=0.7)
        
        # Mark conflicts
        if conflicts:
            conflict_x = [c.location.x for c in conflicts]
            conflict_y = [c.location.y for c in conflicts]
            conflict_z = [c.location.z for c in conflicts]
            ax.scatter(conflict_x, conflict_y, conflict_z, c='red', s=200, marker='X',
                      edgecolors='black', linewidths=2, label='Conflicts', zorder=10)
        
        ax.set_xlabel('X Position (m)', fontsize=11)
        ax.set_ylabel('Y Position (m)', fontsize=11)
        ax.set_zlabel('Altitude (m)', fontsize=11)
        ax.set_title('UAV Flight Paths - 3D View', fontsize=14, fontweight='bold')
        ax.legend(loc='best')
        ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if filename:
            plt.savefig(filename, dpi=300, bbox_inches='tight')
        if show:
            plt.show()
        
        return fig, ax
    
    def create_4d_animation(self, primary: Mission, conflicts: List[Conflict] = None,
                           filename: str = 'deconfliction_4d.gif', fps: int = 20):
        """Create 4D animation (3D space + time)"""
        
        # Determine time range
        all_missions = [primary] + self.system.simulated_flights
        min_time = min(m.start_time for m in all_missions)
        max_time = max(m.end_time for m in all_missions)
        
        # Create time points
        time_points = np.linspace(min_time, max_time, int((max_time - min_time) * fps / 10))
        
        fig = plt.figure(figsize=(16, 12))
        
        # Create subplots
        ax1 = fig.add_subplot(221, projection='3d')  # 3D view
        ax2 = fig.add_subplot(222)  # Top view (XY)
        ax3 = fig.add_subplot(223)  # Side view (XZ)
        ax4 = fig.add_subplot(224)  # Front view (YZ)
        
        # Initialize plot limits
        self._set_plot_limits([ax1, ax2, ax3, ax4], all_missions)
        
        def update(frame):
            t = time_points[frame]
            
            # Clear all axes
            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()
            
            # Plot trajectories and current positions
            self._plot_4d_frame(ax1, ax2, ax3, ax4, primary, all_missions, t, conflicts)
            
            # Reset limits
            self._set_plot_limits([ax1, ax2, ax3, ax4], all_missions)
            
            fig.suptitle(f'UAV Deconfliction - Time: {t:.1f}s', 
                        fontsize=16, fontweight='bold')
        
        anim = FuncAnimation(fig, update, frames=len(time_points), 
                           interval=1000/fps, repeat=True)
        
        # Save animation
        writer = PillowWriter(fps=fps)
        anim.save(filename, writer=writer)
        print(f"Animation saved to {filename}")
        
        return anim
    
    def _plot_mission_2d(self, ax, mission: Mission, color: str, label: str,
                        linewidth: int = 2, alpha: float = 1.0):
        """Helper to plot a mission in 2D"""
        x = [wp.x for wp in mission.waypoints]
        y = [wp.y for wp in mission.waypoints]
        
        ax.plot(x, y, color=color, linewidth=linewidth, alpha=alpha, 
               label=f'{label} ({mission.start_time:.0f}s-{mission.end_time:.0f}s)',
               marker='o', markersize=6)
        
        # Mark start and end
        ax.scatter(x[0], y[0], c=color, s=150, marker='s', 
                  edgecolors='black', linewidths=2, zorder=5)
        ax.scatter(x[-1], y[-1], c=color, s=150, marker='^', 
                  edgecolors='black', linewidths=2, zorder=5)
    
    def _plot_mission_3d(self, ax, mission: Mission, color: str, label: str,
                        linewidth: int = 2, alpha: float = 1.0):
        """Helper to plot a mission in 3D"""
        x = [wp.x for wp in mission.waypoints]
        y = [wp.y for wp in mission.waypoints]
        z = [wp.z for wp in mission.waypoints]
        
        ax.plot(x, y, z, color=color, linewidth=linewidth, alpha=alpha,
               label=f'{label} ({mission.start_time:.0f}s-{mission.end_time:.0f}s)',
               marker='o', markersize=5)
        
        # Mark start and end
        ax.scatter(x[0], y[0], z[0], c=color, s=150, marker='s',
                  edgecolors='black', linewidths=2, zorder=5)
        ax.scatter(x[-1], y[-1], z[-1], c=color, s=150, marker='^',
                  edgecolors='black', linewidths=2, zorder=5)
    
    def _plot_4d_frame(self, ax1, ax2, ax3, ax4, primary, all_missions, t, conflicts):
        """Plot a single frame of the 4D animation"""
        
        # Plot all trajectory paths (faded)
        for mission in all_missions:
            color = 'red' if mission == primary else 'gray'
            alpha = 0.3
            
            x = [wp.x for wp in mission.waypoints]
            y = [wp.y for wp in mission.waypoints]
            z = [wp.z for wp in mission.waypoints]
            
            # 3D view
            ax1.plot(x, y, z, color=color, alpha=alpha, linewidth=1)
            
            # 2D views
            ax2.plot(x, y, color=color, alpha=alpha, linewidth=1)
            ax3.plot(x, z, color=color, alpha=alpha, linewidth=1)
            ax4.plot(y, z, color=color, alpha=alpha, linewidth=1)
        
        # Plot current positions
        for i, mission in enumerate(all_missions):
            pos = self.system.interpolate_position(mission, t)
            if pos:
                color = 'red' if mission == primary else self.colors[i % len(self.colors)]
                marker = 'o' if mission == primary else 's'
                size = 200 if mission == primary else 100
                
                # 3D view
                ax1.scatter([pos.x], [pos.y], [pos.z], c=color, s=size, 
                           marker=marker, edgecolors='black', linewidths=2)
                
                # 2D views
                ax2.scatter([pos.x], [pos.y], c=color, s=size, 
                           marker=marker, edgecolors='black', linewidths=2)
                ax3.scatter([pos.x], [pos.z], c=color, s=size, 
                           marker=marker, edgecolors='black', linewidths=2)
                ax4.scatter([pos.y], [pos.z], c=color, s=size, 
                           marker=marker, edgecolors='black', linewidths=2)
        
        # Mark conflicts at current time
        if conflicts:
            current_conflicts = [c for c in conflicts if abs(c.time - t) < 1.0]
            if current_conflicts:
                for c in current_conflicts:
                    # 3D view
                    ax1.scatter([c.location.x], [c.location.y], [c.location.z],
                               c='red', s=300, marker='X', edgecolors='yellow', 
                               linewidths=3, zorder=100)
                    
                    # 2D views
                    ax2.scatter([c.location.x], [c.location.y], c='red', s=300,
                               marker='X', edgecolors='yellow', linewidths=3, zorder=100)
        
        # Set labels
        ax1.set_title('3D View')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        
        ax2.set_title('Top View (XY)')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.grid(True, alpha=0.3)
        
        ax3.set_title('Side View (XZ)')
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Z (m)')
        ax3.grid(True, alpha=0.3)
        
        ax4.set_title('Front View (YZ)')
        ax4.set_xlabel('Y (m)')
        ax4.set_ylabel('Z (m)')
        ax4.grid(True, alpha=0.3)
    
    def _set_plot_limits(self, axes, missions):
        """Set consistent plot limits across all axes"""
        all_x = []
        all_y = []
        all_z = []
        
        for mission in missions:
            all_x.extend([wp.x for wp in mission.waypoints])
            all_y.extend([wp.y for wp in mission.waypoints])
            all_z.extend([wp.z for wp in mission.waypoints])
        
        margin = 50
        x_range = (min(all_x) - margin, max(all_x) + margin)
        y_range = (min(all_y) - margin, max(all_y) + margin)
        z_range = (min(all_z) - margin, max(all_z) + margin)
        
        for ax in axes:
            if hasattr(ax, 'set_zlim'):  # 3D axis
                ax.set_xlim(x_range)
                ax.set_ylim(y_range)
                ax.set_zlim(z_range)
            else:  # 2D axis
                if 'Top' in ax.get_title() or not ax.get_title():
                    ax.set_xlim(x_range)
                    ax.set_ylim(y_range)
                elif 'Side' in ax.get_title():
                    ax.set_xlim(x_range)
                    ax.set_ylim(z_range)
                elif 'Front' in ax.get_title():
                    ax.set_xlim(y_range)
                    ax.set_ylim(z_range)


if __name__ == "__main__":
    from uav_deconfliction_main import create_sample_scenario
    
    print("Generating visualizations...")
    
    primary, system = create_sample_scenario()
    is_safe, conflicts = system.verify_mission(primary)
    
    viz = DeconflictionVisualizer(system)
    
    # Create 2D visualization
    viz.plot_2d_scenario(primary, conflicts, filename='deconfliction_2d.png', show=False)
    print("✓ 2D visualization saved")
    
    # Create 3D visualization
    viz.plot_3d_scenario(primary, conflicts, filename='deconfliction_3d.png', show=False)
    print("✓ 3D visualization saved")
    
    # Create 4D animation
    viz.create_4d_animation(primary, conflicts, filename='deconfliction_4d.gif', fps=20)
    print("✓ 4D animation saved")
