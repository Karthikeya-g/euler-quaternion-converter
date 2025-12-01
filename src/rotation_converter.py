import numpy as np
import math
from dataclasses import dataclass
from typing import Tuple

@dataclass
class EulerAngles:
    """Data class to ensure clear Input/Output structure."""
    roll: float  # X-axis rotation (radians)
    pitch: float # Y-axis rotation (radians)
    yaw: float   # Z-axis rotation (radians)

    def to_degrees(self):
        return [np.degrees(self.roll), np.degrees(self.pitch), np.degrees(self.yaw)]

class RotationConverter:
    """
    Production-grade converter between Euler Angles (Intrinsic Z-Y-X) 
    and Quaternions [w, x, y, z].
    """
    
    # Tolerance for Gimbal Lock detection (North/South Pole)
    SINGULARITY_TOLERANCE = 1e-6

    @staticmethod
    def euler_to_quaternion(euler: EulerAngles) -> np.ndarray:
        """
        Converts Euler Angles to a normalized Quaternion [w, x, y, z].
        Optimization: Uses batch numpy operations for efficiency.
        """
        # 1. Compute half-angles (quaternions operate on theta/2)
        cr = np.cos(euler.roll * 0.5)
        sr = np.sin(euler.roll * 0.5)
        cp = np.cos(euler.pitch * 0.5)
        sp = np.sin(euler.pitch * 0.5)
        cy = np.cos(euler.yaw * 0.5)
        sy = np.sin(euler.yaw * 0.5)

        # 2. Compute components (Z-Y-X sequence)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        # 3. Normalize to prevent drift over time
        q = np.array([w, x, y, z])
        return q / np.linalg.norm(q)

    @staticmethod
    def quaternion_to_euler(q: np.ndarray) -> EulerAngles:
        """
        Converts Quaternion [w, x, y, z] to EulerAngles.
        Handled Edge Cases:
          - Normalization: Auto-normalizes input.
          - Gimbal Lock: Handles Pitch +/- 90 degrees explicitly.
        """
        # Ensure quaternion is normalized
        q = q / np.linalg.norm(q)
        w, x, y, z = q

        # 1. Calculate the 'sin(pitch)' term (t0)
        # This relates to the matrix element R[2,0] in the Rotation Matrix
        t0 = 2.0 * (w * y - z * x)

        # EDGE CASE A: Numerical Stability
        # Floating point errors can make t0 > 1.0, causing arcsin to crash.
        t0 = np.clip(t0, -1.0, 1.0)

        # EDGE CASE B: Gimbal Lock (Singularity Check)
        # If we are looking straight up/down, Gimbal Lock occurs.
        if np.abs(t0) > (1.0 - RotationConverter.SINGULARITY_TOLERANCE):
            # Pitch is +/- 90 degrees
            pitch = np.pi / 2.0 * np.sign(t0)
            
            # Singularity: Roll and Yaw axes align. 
            # Solution: Lock Roll to 0, place all rotation in Yaw.
            roll = 0.0
            yaw = 2.0 * np.arctan2(x, w)
            
            print(f"[WARN] Gimbal Lock detected! Pitch is {np.degrees(pitch):.1f}°")
        
        else:
            # Standard Case
            pitch = np.arcsin(t0)
            
            # Roll (X-axis)
            roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
            
            # Yaw (Z-axis)
            yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return EulerAngles(roll, pitch, yaw)

# --- EXECUTION BLOCK ---
if __name__ == "__main__":
    converter = RotationConverter()

    # Case 1: Complex Rotation
    print("--- Case 1: Standard Rotation ---")
    input_euler = EulerAngles(roll=0.5, pitch=0.5, yaw=0.5)
    q = converter.euler_to_quaternion(input_euler)
    output_euler = converter.quaternion_to_euler(q)
    
    print(f"Input (deg) : {np.round(input_euler.to_degrees(), 2)}")
    print(f"Quaternion  : {np.round(q, 4)}")
    print(f"Output (deg): {np.round(output_euler.to_degrees(), 2)}")
    
    # Case 2: The Edge Case (Pitch = 90 deg)
    print("\n--- Case 2: Gimbal Lock (Pitch = 90°) ---")
    lock_euler = EulerAngles(roll=0.0, pitch=np.pi/2, yaw=0.0)
    q_lock = converter.euler_to_quaternion(lock_euler)
    out_lock = converter.quaternion_to_euler(q_lock)
    
    print(f"Input (deg) : {np.round(lock_euler.to_degrees(), 2)}")
    print(f"Output (deg): {np.round(out_lock.to_degrees(), 2)}")
