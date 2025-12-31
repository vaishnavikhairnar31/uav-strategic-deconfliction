"""
Comprehensive test suite for UAV deconfliction system
Tests various conflict scenarios and edge cases
"""

import unittest
import numpy as np
from typing import List

from uav_deconfliction_main import (
    Waypoint, Mission, DeconflictionSystem, Conflict
)


class TestWaypoint(unittest.TestCase):
    """Test Waypoint functionality"""
    
    def test_waypoint_creation(self):
        wp = Waypoint(10.0, 20.0, 30.0)
        self.assertEqual(wp.x, 10.0)
        self.assertEqual(wp.y, 20.0)
        self.assertEqual(wp.z, 30.0)
    
    def test_waypoint_2d(self):
        wp = Waypoint(10.0, 20.0)
        self.assertEqual(wp.z, 0.0)
    
    def test_distance_calculation(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(3, 4, 0)
        self.assertEqual(wp1.distance_to(wp2), 5.0)
    
    def test_distance_3d(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(1, 1, 1)
        self.assertAlmostEqual(wp1.distance_to(wp2), np.sqrt(3))


class TestMission(unittest.TestCase):
    """Test Mission functionality"""
    
    def test_mission_creation(self):
        mission = Mission(
            drone_id="TEST-001",
            waypoints=[Waypoint(0, 0), Waypoint(100, 0)],
            start_time=0,
            end_time=10
        )
        self.assertEqual(mission.drone_id, "TEST-001")
        self.assertEqual(len(mission.waypoints), 2)
        self.assertEqual(mission.duration(), 10)
    
    def test_total_distance(self):
        mission = Mission(
            drone_id="TEST-001",
            waypoints=[
                Waypoint(0, 0),
                Waypoint(3, 0),
                Waypoint(3, 4)
            ],
            start_time=0,
            end_time=10
        )
        self.assertEqual(mission.total_distance(), 7.0)


class TestDeconflictionSystem(unittest.TestCase):
    """Test core deconfliction functionality"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.system = DeconflictionSystem(safety_buffer=50.0, time_resolution=1.0)
    
    def test_system_initialization(self):
        self.assertEqual(self.system.safety_buffer, 50.0)
        self.assertEqual(self.system.time_resolution, 1.0)
        self.assertEqual(len(self.system.simulated_flights), 0)
    
    def test_add_simulated_flight(self):
        mission = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(0, 0), Waypoint(100, 0)],
            start_time=0,
            end_time=10
        )
        self.system.add_simulated_flight(mission)
        self.assertEqual(len(self.system.simulated_flights), 1)
    
    def test_interpolate_position_start(self):
        mission = Mission(
            drone_id="TEST-001",
            waypoints=[Waypoint(0, 0, 0), Waypoint(100, 0, 0)],
            start_time=0,
            end_time=10
        )
        pos = self.system.interpolate_position(mission, 0)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.x, 0)
    
    def test_interpolate_position_end(self):
        mission = Mission(
            drone_id="TEST-001",
            waypoints=[Waypoint(0, 0, 0), Waypoint(100, 0, 0)],
            start_time=0,
            end_time=10
        )
        pos = self.system.interpolate_position(mission, 10)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.x, 100)
    
    def test_interpolate_position_middle(self):
        mission = Mission(
            drone_id="TEST-001",
            waypoints=[Waypoint(0, 0, 0), Waypoint(100, 0, 0)],
            start_time=0,
            end_time=10
        )
        pos = self.system.interpolate_position(mission, 5)
        self.assertIsNotNone(pos)
        self.assertAlmostEqual(pos.x, 50, places=1)
    
    def test_interpolate_position_outside_time(self):
        mission = Mission(
            drone_id="TEST-001",
            waypoints=[Waypoint(0, 0), Waypoint(100, 0)],
            start_time=10,
            end_time=20
        )
        pos = self.system.interpolate_position(mission, 5)
        self.assertIsNone(pos)
    
    def test_spatial_conflict_detection(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(30, 0, 0)  # Within 50m buffer
        self.assertTrue(self.system.check_spatial_conflict(wp1, wp2))
    
    def test_no_spatial_conflict(self):
        wp1 = Waypoint(0, 0, 0)
        wp2 = Waypoint(60, 0, 0)  # Outside 50m buffer
        self.assertFalse(self.system.check_spatial_conflict(wp1, wp2))


class TestConflictScenarios(unittest.TestCase):
    """Test various conflict scenarios"""
    
    def setUp(self):
        self.system = DeconflictionSystem(safety_buffer=50.0, time_resolution=0.5)
    
    def test_no_conflict_parallel_paths(self):
        """Test parallel paths with sufficient separation"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100), Waypoint(100, 0, 100)],
            start_time=0,
            end_time=10
        )
        
        simulated = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(0, 100, 100), Waypoint(100, 100, 100)],
            start_time=0,
            end_time=10
        )
        
        self.system.add_simulated_flight(simulated)
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertTrue(is_safe)
        self.assertEqual(len(conflicts), 0)
    
    def test_conflict_crossing_paths(self):
        """Test crossing paths at same time"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100), Waypoint(100, 0, 100)],
            start_time=0,
            end_time=10
        )
        
        simulated = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(50, -50, 100), Waypoint(50, 50, 100)],
            start_time=0,
            end_time=10
        )
        
        self.system.add_simulated_flight(simulated)
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertFalse(is_safe)
        self.assertGreater(len(conflicts), 0)
    
    def test_no_conflict_different_times(self):
        """Test same path but different time windows"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100), Waypoint(100, 0, 100)],
            start_time=0,
            end_time=10
        )
        
        simulated = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(0, 0, 100), Waypoint(100, 0, 100)],
            start_time=20,
            end_time=30
        )
        
        self.system.add_simulated_flight(simulated)
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertTrue(is_safe)
        self.assertEqual(len(conflicts), 0)
    
    def test_no_conflict_different_altitudes(self):
        """Test same XY path but different altitudes"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100), Waypoint(100, 0, 100)],
            start_time=0,
            end_time=10
        )
        
        simulated = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(0, 0, 200), Waypoint(100, 0, 200)],
            start_time=0,
            end_time=10
        )
        
        self.system.add_simulated_flight(simulated)
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertTrue(is_safe)
        self.assertEqual(len(conflicts), 0)
    
    def test_conflict_vertical_separation_insufficient(self):
        """Test insufficient vertical separation"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100), Waypoint(100, 0, 100)],
            start_time=0,
            end_time=10
        )
        
        simulated = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(0, 0, 120), Waypoint(100, 0, 120)],  # Only 20m vertical sep
            start_time=0,
            end_time=10
        )
        
        self.system.add_simulated_flight(simulated)
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertFalse(is_safe)
        self.assertGreater(len(conflicts), 0)
    
    def test_multiple_conflicts(self):
        """Test detection of multiple conflicts"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100), Waypoint(200, 0, 100)],
            start_time=0,
            end_time=20
        )
        
        # Add multiple conflicting flights
        sim1 = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(50, -50, 100), Waypoint(50, 50, 100)],
            start_time=0,
            end_time=20
        )
        
        sim2 = Mission(
            drone_id="SIM-002",
            waypoints=[Waypoint(150, -50, 100), Waypoint(150, 50, 100)],
            start_time=0,
            end_time=20
        )
        
        self.system.add_simulated_flight(sim1)
        self.system.add_simulated_flight(sim2)
        
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertFalse(is_safe)
        # Should have conflicts with both drones
        conflicting_drones = set(c.conflicting_drone for c in conflicts)
        self.assertEqual(len(conflicting_drones), 2)
    
    def test_edge_case_single_waypoint(self):
        """Test mission with single waypoint (hovering)"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(50, 50, 100)],
            start_time=0,
            end_time=10
        )
        
        simulated = Mission(
            drone_id="SIM-001",
            waypoints=[Waypoint(60, 60, 100)],  # Close hover
            start_time=0,
            end_time=10
        )
        
        self.system.add_simulated_flight(simulated)
        is_safe, conflicts = self.system.verify_mission(primary)
        
        self.assertFalse(is_safe)
    
    def test_edge_case_zero_duration(self):
        """Test mission with zero duration"""
        primary = Mission(
            drone_id="PRIMARY",
            waypoints=[Waypoint(0, 0, 100)],
            start_time=5,
            end_time=5
        )
        
        # Should handle gracefully
        is_safe, conflicts = self.system.verify_mission(primary)
        self.assertTrue(is_safe)  # No conflicts possible with zero duration


class TestConflictReporting(unittest.TestCase):
    """Test conflict reporting functionality"""
    
    def test_conflict_summary_no_conflicts(self):
        system = DeconflictionSystem()
        summary = system.get_conflict_summary([])
        self.assertIn("APPROVED", summary)
        self.assertIn("No conflicts", summary)
    
    def test_conflict_summary_with_conflicts(self):
        system = DeconflictionSystem()
        conflicts = [
            Conflict(
                primary_drone="PRIMARY",
                conflicting_drone="SIM-001",
                location=Waypoint(50, 50, 100),
                time=5.0,
                distance=30.0,
                description="Test conflict"
            )
        ]
        
        summary = system.get_conflict_summary(conflicts)
        self.assertIn("DENIED", summary)
        self.assertIn("SIM-001", summary)
        self.assertIn("50", summary)


def run_all_tests():
    """Run all test suites"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add all test classes
    suite.addTests(loader.loadTestsFromTestCase(TestWaypoint))
    suite.addTests(loader.loadTestsFromTestCase(TestMission))
    suite.addTests(loader.loadTestsFromTestCase(TestDeconflictionSystem))
    suite.addTests(loader.loadTestsFromTestCase(TestConflictScenarios))
    suite.addTests(loader.loadTestsFromTestCase(TestConflictReporting))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    # Print summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Tests run: {result.testsRun}")
    print(f"Successes: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print("="*70)
    
    return result


if __name__ == "__main__":
    run_all_tests()
