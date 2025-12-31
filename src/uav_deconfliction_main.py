"""
UAV Strategic Deconfliction System
Main module for drone flight path conflict detection
"""

import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
from datetime import datetime, timedelta
import json

@dataclass
class Waypoint:
    """Represents a waypoint in 3D space"""
    x: float
    y: float
    z: float = 0.0  # Altitude (0 for 2D mode)
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate Euclidean distance to another waypoint"""
        return np.linalg.norm(self.to_array() - other.to_array())

@dataclass
class Mission:
    """Represents a drone mission with waypoints and time window"""
    drone_id: str
    waypoints: List[Waypoint]
    start_time: float  # Unix timestamp or relative time
    end_time: float
    speed: float = 10.0  # meters per second
    
    def duration(self) -> float:
        return self.end_time - self.start_time
    
    def total_distance(self) -> float:
        """Calculate total mission distance"""
        distance = 0.0
        for i in range(len(self.waypoints) - 1):
            distance += self.waypoints[i].distance_to(self.waypoints[i + 1])
        return distance

@dataclass
class Conflict:
    """Represents a detected conflict between drones"""
    primary_drone: str
    conflicting_drone: str
    location: Waypoint
    time: float
    distance: float
    description: str

class DeconflictionSystem:
    """
    Strategic deconfliction system for UAV flight path verification
    """
    
    def __init__(self, safety_buffer: float = 50.0, time_resolution: float = 1.0):
        """
        Initialize deconfliction system
        
        Args:
            safety_buffer: Minimum safe distance between drones (meters)
            time_resolution: Time step for trajectory sampling (seconds)
        """
        self.safety_buffer = safety_buffer
        self.time_resolution = time_resolution
        self.simulated_flights: List[Mission] = []
    
    def add_simulated_flight(self, mission: Mission):
        """Add a simulated flight to the airspace"""
        self.simulated_flights.append(mission)
    
    def interpolate_position(self, mission: Mission, time: float) -> Optional[Waypoint]:
        """
        Interpolate drone position at a specific time
        
        Args:
            mission: Drone mission
            time: Query time
            
        Returns:
            Interpolated position or None if drone not flying at that time
        """
        if time < mission.start_time or time > mission.end_time:
            return None
        
        # Calculate progress through mission
        elapsed = time - mission.start_time
        total_duration = mission.duration()
        
        if total_duration == 0:
            return mission.waypoints[0] if mission.waypoints else None
        
        # Calculate total distance and distance traveled
        total_distance = mission.total_distance()
        distance_traveled = (elapsed / total_duration) * total_distance
        
        # Find which segment the drone is on
        cumulative_distance = 0.0
        for i in range(len(mission.waypoints) - 1):
            segment_distance = mission.waypoints[i].distance_to(mission.waypoints[i + 1])
            
            if cumulative_distance + segment_distance >= distance_traveled:
                # Drone is on this segment
                segment_progress = (distance_traveled - cumulative_distance) / segment_distance
                
                # Linear interpolation
                p1 = mission.waypoints[i].to_array()
                p2 = mission.waypoints[i + 1].to_array()
                pos = p1 + segment_progress * (p2 - p1)
                
                return Waypoint(pos[0], pos[1], pos[2])
            
            cumulative_distance += segment_distance
        
        # Return last waypoint if we've completed the mission
        return mission.waypoints[-1] if mission.waypoints else None
    
    def check_spatial_conflict(self, pos1: Waypoint, pos2: Waypoint) -> bool:
        """Check if two positions violate safety buffer"""
        return pos1.distance_to(pos2) < self.safety_buffer
    
    def verify_mission(self, primary_mission: Mission) -> Tuple[bool, List[Conflict]]:
        """
        Verify if a mission is safe to execute
        
        Args:
            primary_mission: The mission to verify
            
        Returns:
            Tuple of (is_safe, list_of_conflicts)
        """
        conflicts = []
        
        # Sample time points throughout the mission
        num_samples = int(primary_mission.duration() / self.time_resolution) + 1
        time_points = np.linspace(
            primary_mission.start_time,
            primary_mission.end_time,
            num_samples
        )
        
        # Check against each simulated flight
        for sim_flight in self.simulated_flights:
            for t in time_points:
                # Get positions at time t
                primary_pos = self.interpolate_position(primary_mission, t)
                sim_pos = self.interpolate_position(sim_flight, t)
                
                if primary_pos is None or sim_pos is None:
                    continue
                
                # Check for spatial conflict
                distance = primary_pos.distance_to(sim_pos)
                if distance < self.safety_buffer:
                    conflict = Conflict(
                        primary_drone=primary_mission.drone_id,
                        conflicting_drone=sim_flight.drone_id,
                        location=primary_pos,
                        time=t,
                        distance=distance,
                        description=f"Conflict at t={t:.1f}s: distance={distance:.2f}m (min={self.safety_buffer}m)"
                    )
                    conflicts.append(conflict)
        
        return len(conflicts) == 0, conflicts
    
    def get_conflict_summary(self, conflicts: List[Conflict]) -> str:
        """Generate human-readable conflict summary"""
        if not conflicts:
            return "✓ Mission APPROVED: No conflicts detected"
        
        summary = f"✗ Mission DENIED: {len(conflicts)} conflict(s) detected\n\n"
        
        # Group conflicts by drone
        conflicts_by_drone = {}
        for conflict in conflicts:
            if conflict.conflicting_drone not in conflicts_by_drone:
                conflicts_by_drone[conflict.conflicting_drone] = []
            conflicts_by_drone[conflict.conflicting_drone].append(conflict)
        
        for drone_id, drone_conflicts in conflicts_by_drone.items():
            summary += f"Conflicts with {drone_id}:\n"
            for i, c in enumerate(drone_conflicts[:3], 1):  # Show first 3
                summary += f"  {i}. Time: {c.time:.1f}s, Location: ({c.location.x:.1f}, {c.location.y:.1f}, {c.location.z:.1f}), Distance: {c.distance:.2f}m\n"
            if len(drone_conflicts) > 3:
                summary += f"  ... and {len(drone_conflicts) - 3} more conflicts\n"
            summary += "\n"
        
        return summary.strip()


def create_sample_scenario() -> Tuple[Mission, DeconflictionSystem]:
    """Create a sample scenario for testing"""
    
    # Create deconfliction system
    system = DeconflictionSystem(safety_buffer=50.0, time_resolution=1.0)
    
    # Primary mission: Straight line from (0,0) to (500,0)
    primary = Mission(
        drone_id="PRIMARY-001",
        waypoints=[
            Waypoint(0, 0, 100),
            Waypoint(250, 0, 100),
            Waypoint(500, 0, 100)
        ],
        start_time=0,
        end_time=100,
        speed=10.0
    )
    
    # Simulated flight 1: Crossing path (conflict)
    sim1 = Mission(
        drone_id="SIM-001",
        waypoints=[
            Waypoint(250, -100, 100),
            Waypoint(250, 100, 100)
        ],
        start_time=40,
        end_time=60,
        speed=10.0
    )
    
    # Simulated flight 2: Parallel path (no conflict)
    sim2 = Mission(
        drone_id="SIM-002",
        waypoints=[
            Waypoint(0, 100, 150),
            Waypoint(500, 100, 150)
        ],
        start_time=0,
        end_time=100,
        speed=10.0
    )
    
    # Simulated flight 3: Same path, different time (no conflict)
    sim3 = Mission(
        drone_id="SIM-003",
        waypoints=[
            Waypoint(0, 0, 100),
            Waypoint(500, 0, 100)
        ],
        start_time=120,
        end_time=220,
        speed=10.0
    )
    
    system.add_simulated_flight(sim1)
    system.add_simulated_flight(sim2)
    system.add_simulated_flight(sim3)
    
    return primary, system


if __name__ == "__main__":
    # Test the system
    print("UAV Strategic Deconfliction System")
    print("=" * 50)
    print()
    
    primary, system = create_sample_scenario()
    
    print(f"Verifying mission for {primary.drone_id}")
    print(f"Time window: {primary.start_time}s to {primary.end_time}s")
    print(f"Safety buffer: {system.safety_buffer}m")
    print()
    
    is_safe, conflicts = system.verify_mission(primary)
    
    print(system.get_conflict_summary(conflicts))
