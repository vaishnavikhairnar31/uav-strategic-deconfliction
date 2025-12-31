"""
Demo script showcasing various deconfliction scenarios
Generates visualizations and detailed reports
"""

import numpy as np
from uav_deconfliction_main import (
    Waypoint, Mission, DeconflictionSystem
)
from uav_visualization import DeconflictionVisualizer


class ScenarioGenerator:
    """Generate various test scenarios"""
    
    @staticmethod
    def scenario_1_conflict_free():
        """Scenario 1: Multiple drones with no conflicts"""
        print("\n" + "="*70)
        print("SCENARIO 1: Conflict-Free Operations")
        print("="*70)
        
        system = DeconflictionSystem(safety_buffer=50.0, time_resolution=0.5)
        
        # Primary mission: Diagonal flight
        primary = Mission(
            drone_id="PRIMARY-001",
            waypoints=[
                Waypoint(0, 0, 100),
                Waypoint(200, 200, 150),
                Waypoint(400, 400, 100)
            ],
            start_time=0,
            end_time=60
        )
        
        # Simulated flight 1: Parallel path with safe distance
        sim1 = Mission(
            drone_id="DELIVERY-101",
            waypoints=[
                Waypoint(0, 150, 200),
                Waypoint(400, 150, 200)
            ],
            start_time=0,
            end_time=60
        )
        
        # Simulated flight 2: Different altitude, overlapping XY
        sim2 = Mission(
            drone_id="SURVEY-201",
            waypoints=[
                Waypoint(100, 100, 250),
                Waypoint(300, 300, 250)
            ],
            start_time=10,
            end_time=50
        )
        
        # Simulated flight 3: Same path, different time
        sim3 = Mission(
            drone_id="PATROL-301",
            waypoints=[
                Waypoint(0, 0, 100),
                Waypoint(400, 400, 100)
            ],
            start_time=80,
            end_time=140
        )
        
        system.add_simulated_flight(sim1)
        system.add_simulated_flight(sim2)
        system.add_simulated_flight(sim3)
        
        return primary, system
    
    @staticmethod
    def scenario_2_crossing_conflict():
        """Scenario 2: Direct path crossing - conflict"""
        print("\n" + "="*70)
        print("SCENARIO 2: Path Crossing Conflict")
        print("="*70)
        
        system = DeconflictionSystem(safety_buffer=50.0, time_resolution=0.5)
        
        # Primary mission: West to East
        primary = Mission(
            drone_id="PRIMARY-002",
            waypoints=[
                Waypoint(0, 200, 120),
                Waypoint(400, 200, 120)
            ],
            start_time=0,
            end_time=40
        )
        
        # Simulated flight 1: South to North (crossing primary's path)
        sim1 = Mission(
            drone_id="DELIVERY-102",
            waypoints=[
                Waypoint(200, 0, 120),
                Waypoint(200, 400, 120)
            ],
            start_time=0,
            end_time=40
        )
        
        # Simulated flight 2: Safe parallel path
        sim2 = Mission(
            drone_id="SURVEY-202",
            waypoints=[
                Waypoint(0, 100, 180),
                Waypoint(400, 100, 180)
            ],
            start_time=0,
            end_time=40
        )
        
        system.add_simulated_flight(sim1)
        system.add_simulated_flight(sim2)
        
        return primary, system
    
    @staticmethod
    def scenario_3_multiple_conflicts():
        """Scenario 3: Multiple conflict points"""
        print("\n" + "="*70)
        print("SCENARIO 3: Multiple Conflict Points")
        print("="*70)
        
        system = DeconflictionSystem(safety_buffer=50.0, time_resolution=0.5)
        
        # Primary mission: Complex path
        primary = Mission(
            drone_id="PRIMARY-003",
            waypoints=[
                Waypoint(0, 200, 100),
                Waypoint(100, 200, 100),
                Waypoint(200, 300, 100),
                Waypoint(300, 200, 100),
                Waypoint(400, 200, 100)
            ],
            start_time=0,
            end_time=80
        )
        
        # Simulated flight 1: Conflicts at two points
        sim1 = Mission(
            drone_id="EMERGENCY-401",
            waypoints=[
                Waypoint(50, 150, 100),
                Waypoint(50, 250, 100),
                Waypoint(350, 250, 100),
                Waypoint(350, 150, 100)
            ],
            start_time=0,
            end_time=80
        )
        
        # Simulated flight 2: Hovering drone at conflict point
        sim2 = Mission(
            drone_id="INSPECTION-501",
            waypoints=[
                Waypoint(200, 300, 110)
            ],
            start_time=30,
            end_time=50
        )
        
        system.add_simulated_flight(sim1)
        system.add_simulated_flight(sim2)
        
        return primary, system
    
    @staticmethod
    def scenario_4_near_miss():
        """Scenario 4: Near miss - just within safety buffer"""
        print("\n" + "="*70)
        print("SCENARIO 4: Near Miss Detection")
        print("="*70)
        
        system = DeconflictionSystem(safety_buffer=50.0, time_resolution=0.5)
        
        # Primary mission
        primary = Mission(
            drone_id="PRIMARY-004",
            waypoints=[
                Waypoint(0, 0, 100),
                Waypoint(300, 0, 100)
            ],
            start_time=0,
            end_time=30
        )
        
        # Simulated flight: Passes 40m away (within 50m buffer)
        sim1 = Mission(
            drone_id="TRANSPORT-601",
            waypoints=[
                Waypoint(150, -40, 100),
                Waypoint(150, 40, 100)
            ],
            start_time=10,
            end_time=25
        )
        
        system.add_simulated_flight(sim1)
        
        return primary, system
    
    @staticmethod
    def scenario_5_complex_3d():
        """Scenario 5: Complex 3D airspace"""
        print("\n" + "="*70)
        print("SCENARIO 5: Complex 3D Airspace")
        print("="*70)
        
        system = DeconflictionSystem(safety_buffer=50.0, time_resolution=0.5)
        
        # Primary mission: Ascending spiral
        primary = Mission(
            drone_id="PRIMARY-005",
            waypoints=[
                Waypoint(200, 200, 50),
                Waypoint(250, 200, 100),
                Waypoint(250, 250, 150),
                Waypoint(200, 250, 200),
                Waypoint(150, 250, 250),
                Waypoint(150, 200, 300)
            ],
            start_time=0,
            end_time=90
        )
        
        # Multiple drones at different altitudes
        sim1 = Mission(
            drone_id="LAYER-1",
            waypoints=[
                Waypoint(150, 150, 100),
                Waypoint(250, 250, 100)
            ],
            start_time=0,
            end_time=50
        )
        
        sim2 = Mission(
            drone_id="LAYER-2",
            waypoints=[
                Waypoint(250, 150, 200),
                Waypoint(150, 250, 200)
            ],
            start_time=20,
            end_time=70
        )
        
        sim3 = Mission(
            drone_id="LAYER-3",
            waypoints=[
                Waypoint(200, 200, 150),
                Waypoint(200, 200, 250)
            ],
            start_time=30,
            end_time=60
        )
        
        system.add_simulated_flight(sim1)
        system.add_simulated_flight(sim2)
        system.add_simulated_flight(sim3)
        
        return primary, system


def run_scenario(scenario_func, scenario_num: int, generate_viz: bool = True):
    """Run a scenario and generate output"""
    
    # Generate scenario
    primary, system = scenario_func()
    
    # Verify mission
    print(f"\nVerifying {primary.drone_id}...")
    print(f"Mission: {len(primary.waypoints)} waypoints")
    print(f"Time window: {primary.start_time:.1f}s to {primary.end_time:.1f}s")
    print(f"Total distance: {primary.total_distance():.1f}m")
    print(f"Active simulated drones: {len(system.simulated_flights)}")
    
    is_safe, conflicts = system.verify_mission(primary)
    
    # Print results
    print("\n" + "-"*70)
    print(system.get_conflict_summary(conflicts))
    print("-"*70)
    
    # Generate visualizations
    if generate_viz:
        viz = DeconflictionVisualizer(system)
        
        print(f"\nGenerating visualizations for Scenario {scenario_num}...")
        
        # 2D plot
        viz.plot_2d_scenario(
            primary, conflicts,
            filename=f'scenario_{scenario_num}_2d.png',
            show=False
        )
        print(f"  ✓ 2D visualization saved")
        
        # 3D plot
        viz.plot_3d_scenario(
            primary, conflicts,
            filename=f'scenario_{scenario_num}_3d.png',
            show=False
        )
        print(f"  ✓ 3D visualization saved")
        
        # 4D animation (only for scenarios with conflicts or interesting dynamics)
        if scenario_num in [2, 3, 5]:
            viz.create_4d_animation(
                primary, conflicts,
                filename=f'scenario_{scenario_num}_4d.gif',
                fps=15
            )
            print(f"  ✓ 4D animation saved")
    
    return is_safe, conflicts


def main():
    """Run all demo scenarios"""
    
    print("="*70)
    print("UAV STRATEGIC DECONFLICTION SYSTEM - DEMONSTRATION")
    print("="*70)
    print("\nThis demo showcases the deconfliction system with various scenarios")
    print("including conflict-free operations, path crossings, and complex 3D airspace.")
    
    scenarios = [
        (ScenarioGenerator.scenario_1_conflict_free, 1),
        (ScenarioGenerator.scenario_2_crossing_conflict, 2),
        (ScenarioGenerator.scenario_3_multiple_conflicts, 3),
        (ScenarioGenerator.scenario_4_near_miss, 4),
        (ScenarioGenerator.scenario_5_complex_3d, 5),
    ]
    
    results = []
    
    for scenario_func, num in scenarios:
        is_safe, conflicts = run_scenario(scenario_func, num, generate_viz=True)
        results.append({
            'scenario': num,
            'safe': is_safe,
            'conflicts': len(conflicts)
        })
        print("\n")
    
    # Final summary
    print("\n" + "="*70)
    print("DEMONSTRATION COMPLETE - SUMMARY")
    print("="*70)
    
    for r in results:
        status = "✓ APPROVED" if r['safe'] else "✗ DENIED"
        print(f"Scenario {r['scenario']}: {status} ({r['conflicts']} conflicts)")
    
    print("\nAll visualizations have been generated.")
    print("Check the current directory for PNG images and GIF animations.")
    print("="*70)


if __name__ == "__main__":
    main()
