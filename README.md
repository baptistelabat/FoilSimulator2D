# FoilSimulator2D
A 2D foil simulator  
See demo at https://rawgit.com/baptistelabat/FoilSimulator2D/master/foil.html

To do

Interface with arduino leonardo to test different controls  
Add waves orbital velocity  
Add modification of free surface  
Add variations in speed  
Add stall angle in parameters and hysteresis parameters (http://www.aere.iastate.edu/~huhui/paper/2008/AIAA-2008-0315.pdf)  
Plug user polar function  
Add recording  
Ensure correct real time  

Explanation of user parameters
------------------------------
Pitch: pitch is the angle between the horizontal and the boat keel (positive bow up)  
Is dynamic?: if untick the pitch can be manually changed. Otherwise, it comes from equations of dynamics (2nd Newton's law)  

Height: height of the computation point (which is currently forced at center of gravity)  
Is dynamic?: if untick the height can be manually changed. Otherwise, it comes from equations of dynamics (2nd Newton's law)  
Is surface?: if the foils are out of the water (above zero), the lift and drag are zero  
Is surface effect?: if ticked, the lift is reduced when the foil is approaching the surface according to Hydrodynamics of High Speed Marine Vehicles equation 6.144 on page 200  
Is buoyancy?: if ticked, a buoyancy force is computed and avoid the boat to sink!  

Elevator Rake Angle: positive lift up
Foil Rake Angle: positive lift up. Might be changed with up and down arrow keys, or with wheelmouse
Foil Rake Delay: added delay between the foil rake command to simulate actuator delay.
Foil Rake Step: increment of rake on key or wheelmouse event

Flight speed: speed of flight

Mass: mass of the ship (no added masses).
Longitudinal coordinates: position of center of inertia from stern of ship in ship coordinates
Vertical coordintates: position of center of inertia from keel in ship coordinates

Pitch inertia: inertia in rotation

Elevator area: surface of the elevator
Foil area: surface of the foil

Elevator aspect ratio: ratio between span and chord of elevator
Foil aspect ratio: ratio between span and chord

Elevator longitudinal coordinates: position from hull reference point (positive forward)
Foil longitudinal coordinates: idem

Elevator vertical coordinates: idem
Foil longitudinal coordinates:

Computational reference point longitudinal coordinates: not ready yet!
Computational reference point vertical coordinates: idem

Water/Air: switch the density of the fluid below the surface, which enables to go for plane simulator!

Polar selection: not ready yet

