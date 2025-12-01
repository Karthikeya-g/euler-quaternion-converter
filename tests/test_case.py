import unittest
import numpy as np
from src.rotation_converter import RotationConverter, EulerAngles

class TestGimbalLock(unittest.TestCase):
    def test_north_pole_singularity(self):
        """
        Test the specific edge case where Pitch = 90 degrees (North Pole).
        In this state, standard conversion often results in NaN or flipping.
        """
        # Input: Roll=0, Pitch=90, Yaw=0
        lock_input = EulerAngles(0.0, np.pi/2, 0.0)
        
        # Convert to Q
        q = RotationConverter.euler_to_quaternion(lock_input)
        
        # Convert back
        result = RotationConverter.quaternion_to_euler(q)
        
        # Assertion: Pitch should be exactly 90 degrees (pi/2)
        # We use a small epsilon for float comparison
        self.assertTrue(np.isclose(result.pitch, np.pi/2, atol=1e-5), 
                        f"Failed at North Pole! Got {result.pitch} instead of {np.pi/2}")
        
        print(f"\n[Success] North Pole Test Passed: Input Pitch 90° -> Output Pitch {np.degrees(result.pitch):.2f}°")

if __name__ == '__main__':
    unittest.main()
