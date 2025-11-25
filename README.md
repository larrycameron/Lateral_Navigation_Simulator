Lateral Navigation (LNAV) Simulator

A simplified flight-management LNAV lateral-path tracking simulator written in modern C++ (C++17).

This project models how an aircraft follows a lateral flight plan (series of waypoints) using an LNAV control law.
It simulates heading, bank angle, turn rate, cross-track error (XTK), winds, and basic great-circle navigation.

It also logs diagnostic data to a CSV file for analysis.

📌 Key Features
1. Complete LNAV Guidance Loop

The simulator implements a simplified LNAV autopilot model:

Computes desired course between two waypoints

Computes bearing error

Computes cross-track error (XTK)

Commands bank angle using a proportional controller

Updates aircraft heading from turn-rate physics

Moves aircraft forward in lat/long

Winds push the aircraft off-course, forcing LNAV to correct

2. Real Navigation Math

The simulation models:

Great-circle distance

Initial bearing

Turn-rate physics

𝜓
˙
=
𝑔
tan
⁡
(
𝜙
)
𝑉
ψ
˙
	​

=
V
gtan(ϕ)
	​


Cross-track error

XTK
=
𝑑
⋅
sin
⁡
(
Δ
bearing
)
XTK=d⋅sin(Δbearing)

Flat-earth local position update for small steps

3. Multi-Leg Flight Plan

A flight plan with these airports:

BWI → MCO → DFW → IKA → ATL → HKG → LAX → HND → PEK → ICN


You can easily extend or modify this route.

4. Wind Drift & Initial Offsets

To make LNAV work harder, the simulator includes:

Initial heading error (+12°)

Initial lateral displacement (~2 nm)

Crosswind drift (15 ft/s)

This ensures non-zero XTK and meaningful bank-angle corrections.

5. Diagnostic CSV Logging

Every 1 second, the simulator logs:

Leg index

Latitude / longitude

Desired vs actual track

Cross-track error (ft)

Commanded bank angle (deg)

Distance to next waypoint (mi)

The file is created at:

lnav_diag.csv


You can load this into Excel, MATLAB, Python, or plotting tools to visualize LNAV performance.

🧠 How LNAV Works (Simplified Explanation)

Desired course is computed from current leg (FROM → TO waypoint).

Aircraft heading may disagree due to drift or wind → this creates bearing error.

Bearing error × distance traveled sideways → cross-track error (XTK).

LNAV computes a bank angle using a proportional gain:

bank_cmd = gain * XTK


Bank produces turn rate, which adjusts heading.

Heading + speed move the aircraft toward the desired course.

Wind + initial offset continually push the aircraft off course → LNAV keeps correcting.

This cycle repeats at every time-step.

📁 Project Structure
Lateral_Navigation_Simulator.cpp
README.md
lnav_diag.csv      (auto-generated)

🔧 Build Instructions
Linux / WSL / macOS
g++ -std=c++17 -O2 Lateral_Navigation_Simulator.cpp -o lnav_sim

Windows (MinGW)
g++ -std=c++17 -O2 Lateral_Navigation_Simulator.cpp -o lnav_sim.exe

▶️ Run the Simulator
./lnav_sim


You will see console output showing:

Time step

Latitude / longitude

Heading

Cross-track error

Bank command

Distance to next waypoint

Example:

t = 100 s, Lat = 39.21, Lon = -76.63, Heading = 102.3,
Dist to MCO = 657.3 mi, XTK = 4321.4 ft, BankCmd = 12.5 deg

📊 Output: lnav_diag.csv

Sample columns:

step,leg_index,from_code,to_code,
lat_deg,lon_deg,
desired_course_deg,actual_track_deg,
xtk_ft,bank_cmd_deg,dist_to_to_mi


This file is ideal for analysis such as:

Plotting XTK vs. time

Plotting bank angle vs. time

Visualizing LNAV convergence behavior

Studying wind-drift compensation

📌 Known Limitations

This is an educational LNAV model. It does not yet include:

Full spherical earth kinematics

High-fidelity winds aloft

Roll-rate limits

Aircraft acceleration dynamics

FMS leg types (RF, CF, AF, etc.)

Dubins turn anticipations

Flight director command smoothing

These can be added later if you want to evolve the simulator.

✨ Future Enhancements

Here are improvements you can add next:

✔ Add aircraft roll-dynamics (bank rate)
✔ Add wind from any direction (not just fixed drift)
✔ Add a Kalman filter for smoothed XTK
✔ Implement hold entries (parallel, teardrop, direct)
✔ Add RF (radius-to-fix) legs
✔ Implement turn anticipation (fly-by turns)
✔ Add auto-tuning for LNAV gains
✔ Merge LNAV + VNAV to create a full FMS

🙌 Author

Larry Cameron — LNAV / VNAV / avionics simulation project
C++17 / Aeronautics Systems Engineering
