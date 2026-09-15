# Vehicle Dynamics & Lap-Time Simulator – Core Equations

| # | Equation | Description | Units | Assumptions |
|---|----------|-------------|-------|-------------|
| 1 | $ds_i = \sqrt{(\Delta x_i)^2 + (\Delta y_i)^2}$ | Segment length between points | metres | Straight-line segment approximation |
| 2 | $s_i = \sum ds_k$ | Cumulative distance along track | metres | Monotonically increasing |
| 3 | $\kappa = \frac{2 \cdot (\vec{A} \times \vec{B})}{\|\vec{A}\| \cdot \|\vec{B}\| \cdot \|\vec{C}\|}$ | Curvature (circumscribed circle / Menger) | $1/\text{m}$ | Smoothed with moving average / Gaussian filter |
| 4 | $F_\text{drag} = \frac{1}{2} \rho C_d A v^2$ | Aerodynamic drag | $\text{N}$ | Standard quadratic drag model |
| 5 | $F_\text{down} = \frac{1}{2} \rho C_l A v^2$ | Aerodynamic downforce | $\text{N}$ | Constant downforce coefficient $C_l$ |
| 6 | $F_\text{roll} = C_{rr} m g$ | Rolling resistance | $\text{N}$ | Constant $C_{rr} \approx 0.015$ |
| 7 | $F_\text{trac,max} = \mu (m g + F_\text{down})$ | Tire grip limit | $\text{N}$ | Linear Coulomb friction model |
| 8 | $v_\text{corner} = \sqrt{\frac{\mu g}{\|\kappa\| - K_\text{aero}}}$ | Maximum cornering speed | $\text{m/s}$ | Closed-form balance of lateral g and downforce |
| 9 | $v_{i+1} = \sqrt{v_i^2 + 2 a ds_i}$ | Forward pass (acceleration) | $\text{m/s}$ | Constant segment acceleration kinematics |
| 10| $v_i = \sqrt{v_{i+1}^2 + 2 a_\text{brake} ds_i}$ | Backward pass (braking) | $\text{m/s}$ | Deceleration limited by tire grip + aero drag |
| 11| $dt_i = ds_i / v_i$ | Segment traversal time | $\text{s}$ | First-order Euler integration |
| 12| $t_\text{lap} = \sum dt_i$ | Total lap time | $\text{s}$ | Full closed circuit integration |
