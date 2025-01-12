## Interfaces

* PX4/sensors
* Strapdown
* UKF

core logic:
ukf -> separate module
strap down -> separate module

interfaces:
PX4 -> exposed => no knowledge about px4
ukf <--> strap down both separate interfaces

strap down interface:
in: true/corrected accelerations (translational + rotation) + dt
=> dt per acceleration?
out: estimated position, attitude, velocity
interfaces: structs containing out + input data

ukf:
in: current position, attitude, velocity estimation, dt, measured accelerations
out: error state (d[x,v,theta],b_w,b_a)
interfaces: Setter per sensor, setter for current state, setter for corrected acceleration, struct per input => no
knowledge about PX4 sensor format!
=> current state = { X, a_B, w_B }
Getter for current error state

PX4: Subscribe to needed data, transform to expected data format, provide the data to components




## Coordinate systems

### Possibilities
* NED (North East Down)
    * Cartesian
    * Body fixed with reference point
    * Smaller values => Error/delta smaller
* ECEF (Earth Centered Earth Fixed) 
    * Cartesian
    * Absolute Coordinate System
* LLA (Longitude, Latitude, Altitude)
    * Curvilinear orthogonal  
    * Absolute coordinate system
