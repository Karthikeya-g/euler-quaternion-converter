# Euler <-> Quaternion Converter (Robust Implementation)

This repository implements a bi-directional converter between **Euler Angles (Intrinsic Z-Y-X)** and **Quaternions**, specifically designed to handle the mathematical edge cases inherent in 3D rotation representations.

## 🧠 The Mathematical Concept
While Euler angles are intuitive for human understanding (Roll, Pitch, Yaw), they suffer from **Gimbal Lock**, a loss of one degree of freedom when two axes align. Quaternions ($q = w + xi + yj + zk$) avoid this singularity by representing rotation as a point on a 4D hypersphere.

### Core Algorithm
The conversion logic relies on the standard rotation sequence for aerospace/robotics (Intrinsic Z-Y-X).
- **Euler to Quaternion:** Calculated via the product of half-angle sine/cosine terms.
- **Quaternion to Euler:** Calculated by solving the rotation matrix elements.
  - *Roll:* $\arctan2(2(wx + yz), 1 - 2(x^2 + y^2))$
  - *Pitch:* $\arcsin(2(wy - zx))$
  - *Yaw:* $\arctan2(2(wz + xy), 1 - 2(y^2 + z^2))$

## ⚠️ Edge Case Handling (The "Why")
The primary challenge in this implementation is **Gimbal Lock** (North/South Pole singularity).

### The Problem
When the Pitch ($\theta$) approaches $\pm 90^\circ$ ($\pi/2$), the term $2(wy - zx)$ approaches $1.0$.
Mathematically, this corresponds to the "North Pole" where the Gimbal axes for Roll and Yaw become parallel. In this state, there are infinite solutions for Roll and Yaw that produce the same orientation.
- Standard libraries often fail here because the denominator in the `arctan2` formula approaches zero.
- Numerical precision errors can also cause the input to `arcsin` to slightly exceed 1.0, causing a runtime crash.

### My Solution (`rotation_converter.py`)
1.  **Numerical Stability:** I implemented `np.clip` to force the `arcsin` input into the range $[-1.0, 1.0]$.
2.  **Singularity Branch:** I explicitly check if the pitch is within `1e-6` of the pole.
    - If detected, I arbitrarily lock **Roll = 0**.
    - All rotation is then assigned to **Yaw** using the simplified linear mapping derived from the singularity state.

## 🛠️ Usage

```python
from src.rotation_converter import RotationConverter, EulerAngles
import numpy as np

# Create an input (using Dataclass for type safety)
angles = EulerAngles(roll=0.0, pitch=np.pi/2, yaw=0.0)

# Convert
q = RotationConverter.euler_to_quaternion(angles)
result = RotationConverter.quaternion_to_euler(q)
