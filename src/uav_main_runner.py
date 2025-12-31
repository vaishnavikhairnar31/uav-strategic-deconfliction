#!/usr/bin/env python3
"""
Main runner script for UAV Deconfliction System
Provides interactive menu for running different parts of the system
"""

import sys
import os

def print_banner():
    banner = """
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║         UAV STRATEGIC DECONFLICTION SYSTEM                     ║
║         FlytBase Robotics Assignment 2025                      ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
    """
    print(banner)

def print_menu():
    menu = """
Please select an option:

1. Run All Demo Scenarios (Recommended)
   - Executes 5 different conflict scenarios
   - Generates 2D/3D visualizations and 4D animations
   - Shows detailed conflict reports

2. Run Test Suite
   - Executes 30+ unit tests
   - Validates all system components
   - Shows test coverage report

3. Run Simple Example
   - Quick demonstration of basic functionality
   - Shows conflict detection in action
   - Minimal output

4. Generate Custom Scenario
   - Interactive scenario builder
   - Custom waypoints and timing
   - Immediate visualization

5. View Documentation
   - Opens README in browser/text viewer
   - Shows system architecture
   - Usage examples

6. Exit

Enter your choice (1-6): """
    return input(menu)

def run_demo_scenarios():
    print("\n" + "="*70)
    print("RUNNING ALL DEMO SCENARIOS")
    print("="*70)
    print("\nThis will generate multiple visualizations and may take 2-3 minutes...")
    print("Please wait...\n")
    
    try:
        import uav_demo_scenarios
        uav_demo_scenarios.main()
    except Exception as e:
        print(f"\n❌ Error running demos: {e}")
        print("Make sure all dependencies are installed: pip install -r requirements.txt")

def run_tests():
    print("\n" + "="*70)
    print("RUNNING TEST SUITE")
    print("="*70 + "\n")
    
    try:
        import uav_test_suite
        result = uav_test_suite.run_all_tests()
        
        if result.wasSuccessful():
            print("\n✅ All tests passed!")
        else:
            print("\n❌ Some tests failed. Please review the output above.")
    except Exception as e:
        print(f"\n❌ Error running tests: {e}")
        print("Make sure all dependencies are installed: pip install -r requirements.txt")

def run_simple_example():
    print("\n" + "="*70)
    print("SIMPLE EXAMPLE")
    print("="*70 + "\n")
    
    try:
        from uav_deconfliction_main import (
            Waypoint, Mission, DeconflictionSystem
        )
        
        print("Creating deconfliction system...")
        system = DeconflictionSystem(safety_buffer=50.0, time_resolution=1.0)
        
        print("Setting up primary mission...")
        primary = Mission(
            drone_id="PRIMARY-001",
            waypoints=[
                Waypoint(0, 0, 100),
                Waypoint(500, 0, 100)
            ],
            start_time=0,
            end_time=50
        )
        
        print("Adding simulated flight (crossing path)...")
        simulated = Mission(
            drone_id="DELIVERY-101",
            waypoints=[
                Waypoint(250, -100, 100),
                Waypoint(250, 100, 100)
            ],
            start_time=20,
            end_time=30
        )
        system.add_simulated_flight(simulated)
        
        print("\nVerifying mission safety...\n")
        is_safe, conflicts = system.verify_mission(primary)
        
        print(system.get_conflict_summary(conflicts))
        
        if conflicts:
            print("\nGenerating visualization...")
            from uav_visualization import DeconflictionVisualizer
            viz = DeconflictionVisualizer(system)
            viz.plot_2d_scenario(primary, conflicts, filename='simple_example.png', show=False)
            print("✅ Visualization saved as 'simple_example.png'")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

def create_custom_scenario():
    print("\n" + "="*70)
    print("CUSTOM SCENARIO BUILDER")
    print("="*70 + "\n")
    
    try:
        from uav_deconfliction_main import (
            Waypoint, Mission, DeconflictionSystem
        )
        from uav_visualization import DeconflictionVisualizer
        
        print("Let's create a custom scenario!")
        print("\nPrimary Drone Configuration:")
        
        # Get primary mission details
        num_waypoints = int(input("Number of waypoints (2-10): "))
        waypoints = []
        
        for i in range(num_waypoints):
            print(f"\nWaypoint {i+1}:")
            x = float(input("  X coordinate (meters): "))
            y = float(input("  Y coordinate (meters): "))
            z = float(input("  Z altitude (meters): "))
            waypoints.append(Waypoint(x, y, z))
        
        start_time = float(input("\nMission start time (seconds): "))
        end_time = float(input("Mission end time (seconds): "))
        
        primary = Mission(
            drone_id="CUSTOM-PRIMARY",
            waypoints=waypoints,
            start_time=start_time,
            end_time=end_time
        )
        
        # Set up system
        safety_buffer = float(input("\nSafety buffer (meters, default 50): ") or "50")
        system = DeconflictionSystem(safety_buffer=safety_buffer)
        
        # Add simulated flights
        num_sims = int(input("\nNumber of simulated drones (0-5): "))
        
        for i in range(num_sims):
            print(f"\n--- Simulated Drone {i+1} ---")
            x1 = float(input("Start X: "))
            y1 = float(input("Start Y: "))
            z1 = float(input("Start Z: "))
            x2 = float(input("End X: "))
            y2 = float(input("End Y: "))
            z2 = float(input("End Z: "))
            t1 = float(input("Start time: "))
            t2 = float(input("End time: "))
            
            sim = Mission(
                drone_id=f"SIM-{i+1:03d}",
                waypoints=[Waypoint(x1, y1, z1), Waypoint(x2, y2, z2)],
                start_time=t1,
                end_time=t2
            )
            system.add_simulated_flight(sim)
        
        # Verify and visualize
        print("\n" + "="*70)
        print("VERIFYING CUSTOM SCENARIO")
        print("="*70 + "\n")
        
        is_safe, conflicts = system.verify_mission(primary)
        print(system.get_conflict_summary(conflicts))
        
        print("\nGenerating visualizations...")
        viz = DeconflictionVisualizer(system)
        viz.plot_2d_scenario(primary, conflicts, filename='custom_2d.png', show=False)
        viz.plot_3d_scenario(primary, conflicts, filename='custom_3d.png', show=False)
        print("✅ Visualizations saved as 'custom_2d.png' and 'custom_3d.png'")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

def view_documentation():
    print("\n" + "="*70)
    print("DOCUMENTATION")
    print("="*70 + "\n")
    
    files = {
        '1': ('README.md', 'System Overview and Usage Guide'),
        '2': ('REFLECTION.md', 'Design Decisions and Scalability'),
        '3': ('uav_deconfliction_main.py', 'Core System Code'),
        '4': ('uav_visualization.py', 'Visualization Module Code'),
    }
    
    print("Available documentation:\n")
    for key, (filename, description) in files.items():
        print(f"{key}. {filename}")
        print(f"   {description}\n")
    
    choice = input("Select file to view (1-4, or Enter to return): ")
    
    if choice in files:
        filename = files[choice][0]
        if os.path.exists(filename):
            print(f"\n{'='*70}")
            print(f"VIEWING: {filename}")
            print('='*70 + "\n")
            with open(filename, 'r') as f:
                content = f.read()
                # Show first 50 lines
                lines = content.split('\n')
                for i, line in enumerate(lines[:50]):
                    print(line)
                if len(lines) > 50:
                    print(f"\n... ({len(lines) - 50} more lines)")
                    print(f"\nOpen {filename} in a text editor to view the complete file.")
        else:
            print(f"❌ File {filename} not found in current directory.")
    
    input("\nPress Enter to continue...")

def main():
    """Main program loop"""
    
    print_banner()
    
    # Check dependencies
    try:
        import numpy
        import matplotlib
    except ImportError:
        print("❌ Missing dependencies!")
        print("\nPlease install required packages:")
        print("  pip install -r requirements.txt\n")
        sys.exit(1)
    
    while True:
        try:
            choice = print_menu()
            
            if choice == '1':
                run_demo_scenarios()
            elif choice == '2':
                run_tests()
            elif choice == '3':
                run_simple_example()
            elif choice == '4':
                create_custom_scenario()
            elif choice == '5':
                view_documentation()
            elif choice == '6':
                print("\n👋 Thank you for using UAV Deconfliction System!")
                print("=" * 70)
                sys.exit(0)
            else:
                print("\n❌ Invalid choice. Please enter 1-6.")
            
            input("\n\nPress Enter to continue...")
            
        except KeyboardInterrupt:
            print("\n\n👋 Exiting...")
            sys.exit(0)
        except Exception as e:
            print(f"\n❌ Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            input("\nPress Enter to continue...")

if __name__ == "__main__":
    main()
