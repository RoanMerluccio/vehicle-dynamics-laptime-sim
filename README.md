# Vehicle Dynamics & Lap-Time Simulator

A point-mass lap-time simulation tool written in MATLAB for circuit racing and Formula SAE vehicle analysis.

This project estimates the maximum speed and minimum lap time of a vehicle around a race track by balancing tire grip, aerodynamic forces, engine power, and braking limits.

---

## How to Run

In the MATLAB Command Window, run:

```matlab
RUN_SPA      % Simulates Circuit de Spa-Francorchamps (7.004 km) + 3D animation
RUN_ME       % Simulates synthetic test track with Formula SAE car
RUN_3D       % 1-click launcher for the 3D car animation
```

To run the automated tests:
```matlab
addpath(genpath('src'));
runtests('tests');
```

---

## Physics & Mathematical Formulation

The simulation uses a point-mass quasi-steady-state (QSS) model grounded in first-principles physics and calculus:

### 1. Track Geometry & Arc Length (Calc 2)
The track centerline is sampled as discrete $(x_i, y_i)$ waypoints.
* **Segment distance** between consecutive points (distance formula):
  $$\Delta s_i = \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$
* **Cumulative track distance** (Riemann sum approximation of arc length):
  $$s_i = \sum_{k=1}^{i-1} \Delta s_k \approx \int ds$$

### 2. Curvature & Centripetal Acceleration (Physics 1)
Curvature ($\kappa = 1/R$) is calculated by fitting a circle through each set of three consecutive track points:
$$\kappa = \frac{2 \cdot (\vec{A} \times \vec{B})}{|\vec{A}| \cdot |\vec{B}| \cdot |\vec{C}|}$$
For a vehicle travelling at speed $v$, the required lateral centripetal acceleration is:
$$a_{\text{lat}} = \frac{v^2}{R} = v^2 \cdot \kappa$$
And the lateral force required to stay on track is $F_{\text{lat}} = m \cdot a_{\text{lat}} = m v^2 \kappa$.

### 3. Vehicle Forces & Friction Circle
* **Aerodynamic Drag:** $F_{\text{drag}} = \frac{1}{2} \rho C_d A v^2$ (opposes motion)
* **Aerodynamic Downforce:** $F_{\text{down}} = \frac{1}{2} \rho C_l A v^2$ (pushes car down)
* **Total Normal Force:** $F_{\text{normal}} = m g + F_{\text{down}}$
* **Tire Grip Limit:** $F_{\text{max}} = \mu \cdot F_{\text{normal}}$
* **Friction Circle (Pythagorean Theorem):** Combined cornering and acceleration cannot exceed total tire grip:
  $$F_{\text{lon}}^2 + F_{\text{lat}}^2 \le F_{\text{max}}^2 \implies F_{\text{avail}} = \sqrt{F_{\text{max}}^2 - F_{\text{lat}}^2}$$

### 4. Maximum Cornering Speed
In a turn, lateral force cannot exceed available grip:
$$m v^2 \kappa = \mu \left(m g + \frac{1}{2} \rho C_l A v^2\right)$$
Rearranging terms directly yields a closed-form solution for maximum cornering speed:
$$v_{\text{corner}} = \sqrt{\frac{\mu g}{|\kappa| - K_{\text{aero}}}} \quad \text{where} \quad K_{\text{aero}} = \frac{\mu \cdot \frac{1}{2}\rho C_l A}{m}$$
On straightaways ($\kappa \approx 0$), $v_{\text{corner}} \to \infty$.

### 5. Speed Profile (Kinematics & Two-Pass Solver)
Using the standard kinematics equation from Physics 1 ($v_f^2 = v_i^2 + 2 a \Delta s \implies v_f = \sqrt{v_i^2 + 2 a \Delta s}$):
1. **Forward Pass (Acceleration):** Starts at corner exit and steps forward along the track, accelerating using available engine force:
   $$a_{\text{accel}} = \frac{\min(F_{\text{engine}}, F_{\text{avail}}) - F_{\text{drag}} - F_{\text{roll}}}{m}$$
   $$v_{i+1} = \min\left(\sqrt{v_i^2 + 2 a_{\text{accel}} \Delta s_i},\; v_{\text{corner}, i+1}\right)$$
2. **Backward Pass (Braking):** Starts from the corner entry speed and steps backward in distance using maximum braking force:
   $$a_{\text{brake}} = \frac{F_{\text{brake}} + F_{\text{drag}} + F_{\text{roll}}}{m}$$
   $$v_i = \min\left(\sqrt{v_{i+1}^2 + 2 a_{\text{brake}} \Delta s_i},\; v_{\text{corner}, i}\right)$$
3. The final speed at every track point is the minimum of the cornering limit, acceleration limit, and braking limit.

### 6. Lap Time (Calc 1 / Calc 2 Integration)
Because velocity is the rate of change of distance ($v = \frac{ds}{dt} \implies dt = \frac{ds}{v}$), total lap time is the definite integral of $dt$:
$$t_{\text{lap}} = \int_0^L \frac{1}{v(s)} \, ds \approx \sum_{i=1}^{N-1} \frac{\Delta s_i}{v_i}$$

---

## Project Structure

```
MATLAB_VD_LAPTIME/
├── RUN_SPA.m                     # Main script for Circuit de Spa-Francorchamps
├── RUN_ME.m                      # Synthetic test track script
├── RUN_3D.m                      # 3D animation launcher
├── README.md                     # Project documentation
│
├── src/
│   ├── physics/                  # Vehicle dynamics & speed profile
│   │   ├── calculateVehicleForces.m    # Drag, downforce, tire grip, engine power
│   │   ├── calculateCorneringSpeed.m   # Max cornering speed with downforce
│   │   ├── calculateSpeedProfile.m     # Forward/backward kinematics solver
│   │   ├── calculateLapTime.m          # Numerical integration (dt = ds / v)
│   │   ├── calculateSectorTimes.m      # Sector split calculations
│   │   └── runSimulation.m             # Simulation runner
│   ├── track/                    # Track geometry and coordinates
│   │   ├── calculateTrackGeometry.m    # Arc length & curvature
│   │   ├── generateSpaElevation.m      # 3D elevation profile
│   │   ├── loadTrack.m                 # Track CSV reader
│   │   └── importTrackGPS.m            # Flat-earth GPS projection
│   └── visualization/            # Plots & 3D animation
│       ├── simulateCar3D.m             # 3D car animation & telemetry HUD
│       ├── visualizeResults.m          # Engineering plots (speed, g-forces, G-G)
│       └── exportSimulationData.m      # Telemetry export
│
├── data/                         # Track CSV files
│   ├── spa_francorchamps.csv     # 1,401 waypoints (7.004 km)
│   └── example_track.csv         # Synthetic test track
│
└── tests/                        # Verification test suite
    ├── testSimulation3D.m
    ├── testSpeedProfile.m
    ├── testTrackGeometry.m
    └── testVehicleModel.m
```

---

## Assumptions & Limitations

1. **Point-Mass Model:** The vehicle is modeled as a lumped point mass. Body roll, pitch inertia, and individual 4-wheel suspension deflection are omitted.
2. **Fixed Trajectory:** The car is assumed to follow the track centerline. Optimal racing lines across track limits are not calculated.
3. **Linear Friction:** Tire grip uses a Coulomb friction model ($F = \mu F_N$) with a friction circle constraint, rather than a nonlinear slip-angle curve (e.g. Pacejka).
