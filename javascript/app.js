console.log("This is a foil simulator");
var CL = [];
var CD = [];
var alpha_deg = [];
function liftCoefficient(alpha, AR){
  // This is a simplified formula for lift coefficient
  // Maximum lift at 45°
  // No lift at 90°
  // Negative lift from 90°
  //http://people.clarkson.edu/~pmarzocc/AE429/AE-429-4.pdf
  dCl = 2*Math.PI; //infinite elliptic wing for small angle
  
  //AR= span^2/kite_surface;
  e  = 1// Oswald efficiency factor
  //alphai= Cl/(Math.PI*e*AR);
  
  //dCL = dCl*AR/(AR+2.5)
  dCL = dCl/(1+dCl/(Math.PI*e*AR))
  Cl = dCL/2.*Math.sin (2*alpha);
  if (CL.length >0)
  {
    Cl = everpolate.linear(alpha*180/Math.PI, alpha_deg, CL);
  }

  return Cl;
 }
function dragCoefficient(alpha, AR){
  Cd0 = 0.1;
  e = 1;
  inducedDragCoefficient = Math.pow(2*Math.PI*Math.sin(alpha),2)/(Math.PI*AR*e);
  Cd = inducedDragCoefficient + Cd0;
  if (CD.length >0)
  {
    Cd = everpolate.linear(alpha*180/Math.PI, alpha_deg, CD);
  }
  return Cd;
}

// Environment parameters
airDensity         = 1.225;
waterDensity       = 1025;
earth_gravity      = 9.81;

g= earth_gravity;
density = waterDensity

// Ship parameters
mass                = 3000; 
pitchInertia        = 2e5 
foilArea            = 1;
elevatorArea        = 0.19*2;
foilAspectRatio     = 5;
elevatorAspectRatio = 5;
initialDraft = 0.25
Lpp=13

isFoilStall = false
isElevStall = false

// Simulation parameter
sampleTime      = 0.0005; // Sample time
isHeaveDynamic = true;
isPitchDyanmic = true;
isBuoyancy = true;
isSurfaceEffect=true;
isSurface = true;
foilRakeDelay=0;

// Plot parameter
meter2pix = 50;
Newton2meter = 0.00001

// Initial condition
var V = 10;
pqr0          = 0;    // Angular rate
pitch=2.2*Math.PI/180;
rakeMeanPower = 0

// NED (North, East, Down) convention is used
// x is positive forward
// y is positive to starboard
// z is positive down
// h is positive up

// FSD means Forward Starboard Down
// The convention is the following 
// -uvw is a speed
// -xyz is a position
// -XYZ is a force
// -KMN is a torque
// -pqr is a rotation velocity vector
// -rpy is a set of roll,pitch,yaw angle (Euler angles)
// uvw_fluid_ground_NED_kt means the velocity of the fluid relative to the ground projected in NED frame expressed in knots
// unit is omitted if SI units.

// body is the reference used for computation
// ref  is the point from which other points are defined
// fluid describe the fluid at infinity
// meas is the point for output

// Position coordinates are given relative to the hull reference point which is aft, at MWP (waterline in equilibrium condition)
uvw_fluid_grnd_NED  = new THREE.Vector3( 0, 0, 0 );
xyz_body_grnd_NED   = new THREE.Vector3( 0, 0, -1.2 );
xyz_body_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
uvw_body_grnd_NED   = new THREE.Vector3( V, 0, 0 );
xyz_CoG_ref_FSD     = new THREE.Vector3( 0, 0, 0 );
xyz_CoG_body_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_foil_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_foil_body_FSD   = new THREE.Vector3( 0, 0, 0 );
xyz_foil_grnd_NED   = new THREE.Vector3( 0, 0, 0 );
uvw_foil_grnd_NED   = new THREE.Vector3( 0, 0, 0 );
xyz_elev_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_elev_body_FSD   = new THREE.Vector3( 0, 0, 0 );
xyz_foil_grnd_FSD   = new THREE.Vector3( 0, 0, 0 );
uvw_elev_grnd_NED   = new THREE.Vector3( 0, 0, 0 );

// Hull extremity for archimedian thrust
xyz_hAB_ref_FSD = new THREE.Vector3( 0, 0, initialDraft);//Hull Aft Bottom
xyz_hFB_ref_FSD = new THREE.Vector3( Lpp, 0, initialDraft);//Hull fore Bottom
xyz_hAT_ref_FSD = new THREE.Vector3( 0, 0, -1);//Hull Aft Top
xyz_hFT_ref_FSD = new THREE.Vector3( Lpp, 0, -1);//Hull fore Top

xyz_buoyancy_ref_FSD = new THREE.Vector3( 0, 0, 0 );
xyz_buoyancy_body_FSD= new THREE.Vector3( 0, 0, 0 );
xyz_buoyancy_grnd_NED = new THREE.Vector3( 0, 0, 0 );

// Sensor
xyz_meas_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_meas_body_FSD   = new THREE.Vector3( 0, 0, 0 );

// Forces vectors
XYZ_foil_body_NED    = new THREE.Vector3( 0, 0, 0 );
XYZ_elev_body_NED    = new THREE.Vector3( 0, 0, 0 );
XYZ_wght_body_NED    = new THREE.Vector3( 0, 0, 0 );
XYZ_buoyancy_body_NED = new THREE.Vector3( 0, 0, 0 );
XYZ_all_body_NED     = new THREE.Vector3( 0, 0, 0 );
XYZ_foil_body_FSD    = new THREE.Vector3( 0, 0, 0 );
XYZ_elev_body_FSD    = new THREE.Vector3( 0, 0, 0 );
XYZ_wght_body_FSD    = new THREE.Vector3( 0, 0, 0 );
XYZ_buoyancy_body_FSD = new THREE.Vector3( 0, 0, 0 );
XYZ_all_body_FSD     = new THREE.Vector3( 0, 0, 0 );

// Torque vectors
KMN_foil_body_FSD    = new THREE.Vector3( 0, 0, 0 );
KMN_elev_body_FSD    = new THREE.Vector3( 0, 0, 0 );
KMN_wght_body_FSD    = new THREE.Vector3( 0, 0, 0 );
KMN_buoyancy_body_FSD = new THREE.Vector3( 0, 0, 0 );
KMN_all_body_FSD     = new THREE.Vector3( 0, 0, 0 );
pqr_body_grnd_FSD    = new THREE.Vector3( 0, 0, 0 );

pqr_body_grnd_FSD.x = pqr0;



// Control
elevatorRake  = 0;
foilRake      = 0;

document.getElementById("pitchRange")               .addEventListener("change", updatePitch);
document.getElementById("pitchDynamicCheck")        .addEventListener("change", updatePitchDynamic);
document.getElementById("heaveRange")               .addEventListener("change", updateHeave);
document.getElementById("heaveDynamicCheck")        .addEventListener("change", updateHeaveDynamic);
document.getElementById("surfaceCheck")             .addEventListener("change", updateSurface);
document.getElementById("surfaceEffectCheck")       .addEventListener("change", updateSurfaceEffect);
document.getElementById("buoyancyCheck")            .addEventListener("change", updateBuoyancy);
document.getElementById("elevatorRakeRange")        .addEventListener("change", updateElevatorRake);
document.getElementById("foilRakeRange")            .addEventListener("change", updateFoilRake);
document.getElementById("foilRakeDelayRange")       .addEventListener("change", updateFoilRakeDelay);
document.getElementById("flightSpeedRange")         .addEventListener("change", updateFlightSpeed);
document.getElementById("massRange")                .addEventListener("change", updateMass);
document.getElementById("CGLongiRange")             .addEventListener("change", updateLongitudinalCenterOfInertiaPosition);
document.getElementById("CGVertUpRange")            .addEventListener("change", updateVerticalUpCenterOfInertiaPosition);
document.getElementById("gravityCheck")             .addEventListener("change", updateGravity);
document.getElementById("pitchInertiaRange")        .addEventListener("change", updatePitchInertia);
document.getElementById("elevatorAreaRange")        .addEventListener("change", updateElevatorArea);
document.getElementById("foilAreaRange")            .addEventListener("change", updateFoilArea);
document.getElementById("elevatorAspectRatioRange") .addEventListener("change", updateElevatorAspectRatio);
document.getElementById("foilAspectRatioRange")     .addEventListener("change", updateFoilAspectRatio);
document.getElementById("elevatorLongiRange")       .addEventListener("change", updateElevatorLongitudinalPosition);
document.getElementById("foilLongiRange")           .addEventListener("change", updateFoilLongitudinalPosition);
document.getElementById("elevatorVertUpRange")      .addEventListener("change", updateElevatorVerticalUpPosition);
document.getElementById("foilVertUpRange")          .addEventListener("change", updateFoilVerticalUpPosition);
//document.getElementById("bodyLongiRange")           .addEventListener("change", updateBodyLongitudinalPosition);
//document.getElementById("bodyVertUpRange")          .addEventListener("change", updateBodyVerticalUpPosition);
document.getElementById("fluidSelect")              .addEventListener("change", updateFluid);

setInterval(updaten, 1);
setInterval(updatePlot,100);
setInterval(saveRake,100);
var d = new Date();
var t0 = d.getTime();
told = 0;
simulation_time = 0;

//create and fill a circular buffar to store foil rake a be able to apply pure delay
var rake=[]
i_rakeBuffer=0
function saveRake(){
	i_rakeBuffer = i_rakeBuffer+1
	if (i_rakeBuffer>20) {i_rakeBuffer = 0;}
	rake[i_rakeBuffer]= foilRake;
}
function getRakeDelayed(delay)
{
	i_delay = i_rakeBuffer-Math.round(delay/0.1)
	if (i_delay<0){i_delay = i_delay+20}
	return rake[i_delay]
}

function init(){
  // Do init to be sure values are the same as described in html page
  updatePitch();
  updatePitchDynamic();
  updateHeave();
  updateHeaveDynamic();
  updateSurface();
  updateSurfaceEffect();
  updateBuoyancy();
  updateElevatorRake();
  updateFoilRake();
  updateFoilRakeDelay();
  updateFlightSpeed();
  updateMass();
  updateLongitudinalCenterOfInertiaPosition();
  updateVerticalUpCenterOfInertiaPosition();
  updateGravity();
  updatePitchInertia();
  updateElevatorArea();
  updateFoilArea();
  updateElevatorAspectRatio();
  updateFoilAspectRatio();
  updateElevatorLongitudinalPosition();
  updateFoilLongitudinalPosition();
  updateElevatorVerticalUpPosition();
  updateFoilVerticalUpPosition();
  updateBodyLongitudinalPosition();
  updateBodyVerticalUpPosition();
  updateFluid();
}
init();
function plot(body_position, foil_position, elevator_position, foil_rake, elevator_rake){
  plotFoil();
  plotElevator();
  plotWeight();
  plotBuoyancy();
  plotShip();
  plotFfoil();
  plotFelevator();
  plotFall();
  translateBody(xyz_body_grnd_NED.x, xyz_body_grnd_NED.z);
  rotateBody(pitch);
  translateRef(-xyz_body_ref_FSD.x, -xyz_body_ref_FSD.z);
  translateBuoyancy(xyz_buoyancy_ref_FSD.x, xyz_buoyancy_ref_FSD.z)
  translateFoil(xyz_foil_ref_FSD.x, xyz_foil_ref_FSD.z);
  translateElevator(xyz_elev_ref_FSD.x, xyz_elev_ref_FSD.z);
  translateCenterOfInertia(xyz_CoG_ref_FSD.x, xyz_CoG_ref_FSD.z);
  rotateFoil(foil_rake);
  rotateElevator(elevator_rake); 
}
function updatePlot(){
  plot(xyz_body_grnd_NED,xyz_foil_body_FSD, xyz_elev_body_FSD, foilRake, elevatorRake, pitch);
  updateOutput();
}
function updaten()
{
  for (i=0;i<8;i++) // Dirty manual tuning to get close of real time on my computer
  {
    update();
  }
}

function computeForces(){
  computeBuoyancy()
  CTM = new THREE.Matrix4;
  invCTM = new THREE.Matrix4;
  CTMf = new THREE.Matrix4;
  var rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
  tmp    = new THREE.Vector3( 0, 0, 0 );
  
  XYZ_wght_body_NED.set(0, 0, mass*g);

  CTM.makeRotationFromEuler(rpy);
  invCTM.getInverse(CTM);
  XYZ_wght_body_FSD = XYZ_wght_body_NED.clone().applyMatrix4(CTM)
  KMN_wght_body_FSD = tmp.clone().crossVectors(xyz_CoG_body_FSD, XYZ_wght_body_FSD )
  
  uvw_fluid_grnd_NED.set(0, 0, 0);
  


  
  // Compute forces on foil
  uvw_foil_grnd_NED = uvw_body_grnd_NED.clone().add(tmp.crossVectors(pqr_body_grnd_FSD, xyz_foil_body_FSD).applyMatrix4(invCTM))
  uvw_fluid_foil_NED =  uvw_fluid_grnd_NED.clone().sub(uvw_foil_grnd_NED);
  angle_fluid_body = Math.atan2(-uvw_fluid_foil_NED.z, -uvw_fluid_foil_NED.x);
  q = 1/2*density *Math.pow(uvw_fluid_foil_NED.length(),2);
  AoA = pitch + getRakeDelayed(foilRakeDelay) + angle_fluid_body
    xyz_foil_grnd_NED = xyz_foil_body_FSD.clone().applyMatrix4(invCTM).add(xyz_body_grnd_NED)
  chord = Math.sqrt(foilArea/foilAspectRatio);
  inverseMirrorEffect = (1-Math.exp(Math.min(0,-xyz_foil_grnd_NED.z)/chord));
  if (xyz_foil_grnd_NED.z>0)
  {
    // formula according to Hydrodynamics of High Speed Marine Vehicles equation 6.144 on page 200
    inverseMirrorEffect= (1+16*Math.pow(Math.min(0,-xyz_foil_grnd_NED.z)/chord,2))/(2+16*Math.pow(Math.min(0,-xyz_foil_grnd_NED.z)/chord,2))
    if (!isSurfaceEffect) {inverseMirrorEffect=1}
  }
  else
  {
    inverseMirrorEffect = 0
  }
  if (!isSurface) {inverseMirrorEffect=1}
  if (Math.abs(AoA)>10*Math.PI/180)
  { isFoilStall = true}
  if (Math.abs(AoA)<5*Math.PI/180)
  { isFoilStall = false}
  if (isFoilStall)
  {   
    stallEffect = 0.3
  }
  else
  {
    stallEffect = 1
  }
  lift   = q*foilArea*liftCoefficient(AoA, foilAspectRatio)*inverseMirrorEffect*stallEffect;
  drag   = q*foilArea*dragCoefficient(AoA, foilAspectRatio);
  
  // Rotate to ground frame
  
  CTMf.makeRotationY(angle_fluid_body);
  XYZ_foil_body_NED.set(  -drag,0, -lift);
  XYZ_foil_body_NED.applyMatrix4(invCTM);
  
  // Compute torque
  XYZ_foil_body_FSD = XYZ_foil_body_NED.clone().applyMatrix4(CTM)
  KMN_foil_body_FSD = tmp.clone().crossVectors(xyz_foil_body_FSD, XYZ_foil_body_FSD )
  //console.log(KMN_foil_body_FSD)
  
  // Compute forces on elevator
  uvw_elev_grnd_NED = uvw_body_grnd_NED.clone().add(tmp.crossVectors(pqr_body_grnd_FSD, xyz_elev_body_FSD).applyMatrix4(invCTM))
  uvw_fluid_elev_NED =  uvw_fluid_grnd_NED.clone().sub(uvw_elev_grnd_NED);
  angle_fluid_body = Math.atan2(-uvw_fluid_elev_NED.z, -uvw_fluid_elev_NED.x);
  q = 1/2*density *Math.pow(uvw_fluid_elev_NED.length(),2);
  

  xyz_elev_grnd_NED = xyz_elev_body_FSD.clone().applyMatrix4(invCTM).add(xyz_body_grnd_NED)
  AoA = pitch + elevatorRake + angle_fluid_body
  chord = Math.sqrt(elevatorArea/elevatorAspectRatio);
  if (xyz_elev_grnd_NED.z>0)
  {
    inverseMirrorEffect= (1+16*Math.pow(Math.min(0,-xyz_elev_grnd_NED.z)/chord,2))/(2+16*Math.pow(Math.min(0,-xyz_elev_grnd_NED.z)/chord,2))
    if (!isSurfaceEffect) {inverseMirrorEffect=1}
  }
  else
  {inverseMirrorEffect = 0}
  if (!isSurface) {inverseMirrorEffect=1}
  if (Math.abs(AoA)>10*Math.PI/180)
  { isElevStall = true}
  if (Math.abs(AoA)<5*Math.PI/180)
  { isElevStall = false}
  if (isElevStall)
  {   
    stallEffect = 0.3
  }
  else
  {
    stallEffect = 1
  }
  lift   = q*foilArea*liftCoefficient(AoA, elevatorAspectRatio)*inverseMirrorEffect*stallEffect;
  drag   = q*foilArea*dragCoefficient(AoA, elevatorAspectRatio);
  
  // Rotate to ground frame
  
  CTMf.makeRotationY(angle_fluid_body);
  XYZ_elev_body_NED.set(  -drag,0, -lift);
  XYZ_elev_body_NED.applyMatrix4(invCTM);
  
  // Compute torque
  rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
  CTM.makeRotationFromEuler(rpy);
  XYZ_elev_body_FSD = XYZ_elev_body_NED.clone().applyMatrix4(CTM)
  KMN_elev_body_FSD = tmp.clone().crossVectors(xyz_elev_body_FSD, XYZ_elev_body_FSD)
  //console.log(KMN_elev_body_FSD)
  
  XYZ_all_body_NED = XYZ_elev_body_NED.clone().add(XYZ_foil_body_NED).add(XYZ_wght_body_NED).add(XYZ_buoyancy_body_NED);
  XYZ_all_body_FSD = XYZ_all_body_NED.clone().applyMatrix4(CTM)
  KMN_all_body_FSD = KMN_elev_body_FSD.clone().add(KMN_foil_body_FSD).add(KMN_wght_body_FSD).add(KMN_buoyancy_body_FSD);
  //console.log(XYZ_wght_body_FSD);
  //onsole.log(KMN_wght_body_FSD);
  
}
function update(){
  computeForces()
  dt = sampleTime// +0*dt;
  simulation_time = simulation_time + dt;
  inv_mass = 1./mass;
  uvw_body_grnd_NED.add(XYZ_all_body_NED.clone().multiplyScalar(inv_mass*dt))
  if (false==isHeaveDynamic)
  {
    uvw_body_grnd_NED.z = 0
  }
  uvw_body_grnd_NED.x = V
  //console.log(uvw_body_grnd_NED)
  x_old = xyz_body_grnd_NED.x
  xyz_body_grnd_NED.add(uvw_body_grnd_NED.clone().multiplyScalar(dt))
  xyz_body_grnd_NED.x = x_old
  
  pqr_body_grnd_FSD.add(KMN_all_body_FSD.clone().multiplyScalar(1/pitchInertia*dt));
  if (false==isPitchDynamic)
  {
    pqr_body_grnd_FSD.y = 0;
  }
  pitch = pitch + pqr_body_grnd_FSD.y*dt;
  
  //console.log(xyz_body_grnd_NED)
  updateHeave();
  updatePitch();
  
}
function rotateFoil(r){
    foilFrame = document.getElementById("foil");
    r_deg = r*180/Math.PI;
		foilFrame.setAttribute('transform', 'rotate(' +-r_deg +')');
}
function rotateElevator(r){
    elevatorFrame = document.getElementById("elevator");
    r_deg = r*180/Math.PI;
		elevatorFrame.setAttribute('transform', 'rotate(' +-r_deg +')');
}
function rotateBody(r){
    bodyFrame = document.getElementById("body__rotate_frame");
    r_deg = r*180/Math.PI;
		bodyFrame.setAttribute('transform', 'rotate(' +-r_deg +')');
}
function translateFoil(x, z){
  foil_frame = document.getElementById("foil_frame");
  foil_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
}
function translateElevator(x, z){
  elevator_frame = document.getElementById("elevator_frame");
  elevator_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
}
function translateCenterOfInertia(x, z){
  elevator_frame = document.getElementById("CoG_frame");
  elevator_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
}
function translateBuoyancy(x,z){
  B_frame = document.getElementById("B_frame");
  B_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
}
function translateBody(x, z){
  body_frame = document.getElementById("body_frame");
  body_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
}
function translateRef(x, z){
  ref_frame = document.getElementById("ref_frame");
  ref_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
}

function plotFoil(){
  chord = Math.sqrt(foilArea/foilAspectRatio);
  foil = document.getElementById("foil");
  foil.setAttribute('x1', -2/3.*chord*meter2pix);
  foil.setAttribute('x2', 1/3*chord*meter2pix);
}
function plotElevator(){
  chord = Math.sqrt(elevatorArea/elevatorAspectRatio);
  elevator = document.getElementById("elevator");
  elevator.setAttribute('x1', -2/3.*chord*meter2pix);
  elevator.setAttribute('x2', 1/3*chord*meter2pix);
}
function plotShip(){
  length = 13;
  elevator = document.getElementById("keel");
  elevator.setAttribute('x1', 0);
  elevator.setAttribute('x2', length*meter2pix);
}
function plotWeight(){
  arrow = document.getElementById("Fweight");
  arrow.setAttribute('d', "M"+ 0*meter2pix +" "+ 0*meter2pix+ " L"+ XYZ_wght_body_FSD.x*Newton2meter*meter2pix +" "+ XYZ_wght_body_FSD.z*Newton2meter*meter2pix);
}
function plotBuoyancy(){
  arrow = document.getElementById("Fbuoyancy");
  arrow.setAttribute('d', "M"+ 0*meter2pix +" "+ 0*meter2pix+ " L"+ XYZ_buoyancy_body_FSD.x*Newton2meter*meter2pix +" "+ XYZ_buoyancy_body_FSD.z*Newton2meter*meter2pix);
}
function plotFfoil(){
  arrow = document.getElementById("Ffoil");
  arrow.setAttribute('d', "M"+ 0*meter2pix +" "+ 0*meter2pix+ " L"+ XYZ_foil_body_FSD.x*Newton2meter*meter2pix +" "+ XYZ_foil_body_FSD.z*Newton2meter*meter2pix);
}
function plotFelevator(){
  arrow = document.getElementById("Felevator");
  arrow.setAttribute('d', "M"+ 0*meter2pix +" "+ 0*meter2pix+ " L"+ XYZ_elev_body_FSD.x*Newton2meter*meter2pix +" "+ XYZ_elev_body_FSD.z*Newton2meter*meter2pix);
}
function plotFall(){
  arrow = document.getElementById("Fall");
  arrow.setAttribute('d', "M"+ 0*meter2pix +" "+ 0*meter2pix+ " L"+ XYZ_all_body_FSD.x*Newton2meter*meter2pix +" "+ XYZ_all_body_FSD.z*Newton2meter*meter2pix);
}

function updatePitch(){
		//get elements
		var myRange = document.getElementById("pitchRange");
		var myOutput = document.getElementById("pitch");
    var myCheck = document.getElementById("pitchDynamicCheck");
    isPitchDynamic = myCheck.checked;
    if (isPitchDynamic==true)
    {
      myRange.value = pitch*180/Math.PI;
    }
    else
    {
      pitch = myRange.value*Math.PI/180;
    }
    //copy the value over
		myOutput.value = myRange.value;
}
function updatePitchDynamic(){
		//get elements
    var myCheck = document.getElementById("pitchDynamicCheck");
    isPitchDynamic = myCheck.checked;
}
function updateHeave(){
		//get elements
		var myRange = document.getElementById("heaveRange");
		var myOutput = document.getElementById("heave");
    var myCheck = document.getElementById("heaveDynamicCheck");
    isHeaveDynamic = myCheck.checked;
    if (isHeaveDynamic==true)
    {
      myRange.value = -xyz_body_grnd_NED.z;
    }
    else
    {
      xyz_body_grnd_NED.z = -myRange.value;
    }
    //copy the value over
		myOutput.value = myRange.value;
}
function updateHeaveDynamic(){
		//get elements
    var myCheck = document.getElementById("heaveDynamicCheck");
    isHeaveDynamic = myCheck.checked;
}
function updateSurface(){
		//get elements
    var myCheck = document.getElementById("surfaceCheck");
    isSurface = myCheck.checked;
}
function updateSurfaceEffect(){
		//get elements
    var myCheck = document.getElementById("surfaceEffectCheck");
    isSurfaceEffect = myCheck.checked;
}
function updateBuoyancy(){
		//get elements
    var myCheck = document.getElementById("buoyancyCheck");
    isBuoyancy = myCheck.checked;
}
function updateElevatorRake(){
		//get elements
		var myRange = document.getElementById("elevatorRakeRange");
		var myOutput = document.getElementById("elevatorRake");
		//copy the value over
		myOutput.value = myRange.value;
    elevatorRake  = myRange.value*Math.PI/180;
}
function updateFoilRake(){
		//get elements
		var myRange = document.getElementById("foilRakeRange");
		var myOutput = document.getElementById("foilRake");
		//copy the value over
		myOutput.value = myRange.value;
    foilRake  = myRange.value*Math.PI/180;
}
function updateFoilRakeDelay(){
		//get elements
		var myRange = document.getElementById("foilRakeDelayRange");
		var myOutput = document.getElementById("foilRakeDelay");
		//copy the value over
		myOutput.value = myRange.value;
		foilRakeDelay  = myRange.value*1;
}



function updateFlightSpeed(){
		//get elements
		var myRange = document.getElementById("flightSpeedRange");
		var myOutput = document.getElementById("flightSpeed");
		//copy the value over
		myOutput.value = myRange.value;
    V = 1*myOutput.value*1852/3600;
}
function updateMass(){
		//get elements
		var myRange = document.getElementById("massRange");
		var myOutput = document.getElementById("mass");
		//copy the value over
		myOutput.value = myRange.value;
    mass = myOutput.value;
}
function updateLongitudinalCenterOfInertiaPosition(){
		//get elements
		var myRange = document.getElementById("CGLongiRange");
		var myOutput = document.getElementById("CGLongi");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_CoG_ref_FSD.x = myOutput.value*1.;
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
	var myRange = document.getElementById("bodyLongiRange");
	myRange.value = myOutput.value
	updateBodyLongitudinalPosition
    
}
function updateVerticalUpCenterOfInertiaPosition(){
		//get elements
		var myRange = document.getElementById("CGVertUpRange");
		var myOutput = document.getElementById("CGVertUp");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_CoG_ref_FSD.z = -myOutput.value;
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
	
	var myRange = document.getElementById("bodyVertUpRange");
	myRange.value = myOutput.value
	updateBodyVerticalUpPosition
}
function updateGravity(){
		//get elements
    
		var myCheck = document.getElementById("gravityCheck");
    g = earth_gravity*myCheck.checked;
}
function updatePitchInertia(){
		//get elements
		var myRange = document.getElementById("pitchInertiaRange");
		var myOutput = document.getElementById("pitchInertia");
		//copy the value over
		myOutput.value = myRange.value;
    pitchInertia = myOutput.value;
}
function updateElevatorArea(){
		//get elements
		var myRange = document.getElementById("elevatorAreaRange");
		var myOutput = document.getElementById("elevatorArea");
		//copy the value over
		myOutput.value = myRange.value;
    elevatorArea = myOutput.value;
}
function updateFoilArea(){
		//get elements
		var myRange = document.getElementById("foilAreaRange");
		var myOutput = document.getElementById("foilArea");
		//copy the value over
		myOutput.value = myRange.value;
    foilArea = myOutput.value;
}
function updateElevatorAspectRatio(){
		//get elements
		var myRange = document.getElementById("elevatorAspectRatioRange");
		var myOutput = document.getElementById("elevatorAspectRatio");
		//copy the value over
		myOutput.value = myRange.value;
    elevatorAspectRatio = myOutput.value;
}
function updateFoilAspectRatio(){
		//get elements
		var myRange = document.getElementById("foilAspectRatioRange");
		var myOutput = document.getElementById("foilAspectRatio");
		//copy the value over
		myOutput.value = myRange.value;
    foilAspectRatio = myOutput.value;
}
function updateElevatorLongitudinalPosition(){
		//get elements
		var myRange = document.getElementById("elevatorLongiRange");
		var myOutput = document.getElementById("elevatorLongi");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_elev_ref_FSD.x = myOutput.value*1.;
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateFoilLongitudinalPosition(){
		//get elements
		var myRange = document.getElementById("foilLongiRange");
		var myOutput = document.getElementById("foilLongi");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_foil_ref_FSD.x = myOutput.value*1.;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateElevatorVerticalUpPosition(){
		//get elements
		var myRange = document.getElementById("elevatorVertUpRange");
		var myOutput = document.getElementById("elevatorVertUp");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_elev_ref_FSD.z = -myOutput.value*1.;
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateFoilVerticalUpPosition(){
		//get elements
		var myRange = document.getElementById("foilVertUpRange");
		var myOutput = document.getElementById("foilVertUp");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_foil_ref_FSD.z = -myOutput.value;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateBodyLongitudinalPosition(){
		//get elements
		var myRange = document.getElementById("bodyLongiRange");
		var myOutput = document.getElementById("bodyLongi");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_body_ref_FSD_old = new THREE.Vector3( 0, 0, 0 );
	xyz_body_ref_FSD_old=xyz_body_ref_FSD.clone()
    xyz_body_ref_FSD.x = myOutput.value*1.;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    CTM = new THREE.Matrix4;
    invCTM = new THREE.Matrix4;
    var rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
    xyz_body_grnd_NED.add((xyz_body_ref_FSD.clone().sub(xyz_body_ref_FSD_old)).applyMatrix4(invCTM))
	var myRange = document.getElementById("heaveRange");
	var myOutput = document.getElementById("heave");
	myRange.value = -xyz_body_grnd_NED.z
	myOutput.value = -xyz_body_grnd_NED.z
}
function updateBodyVerticalUpPosition(){
		//get elements
		var myRange = document.getElementById("bodyVertUpRange");
		var myOutput = document.getElementById("bodyVertUp");
		//copy the value over
		myOutput.value = myRange.value*1.;
    xyz_body_ref_FSD_old = new THREE.Vector3( 0, 0, 0 );
    xyz_body_ref_FSD_old=xyz_body_ref_FSD.clone()
    xyz_body_ref_FSD.z = -myOutput.value;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    
    CTM = new THREE.Matrix4;
    invCTM = new THREE.Matrix4;
    var rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
    xyz_body_grnd_NED.add((xyz_body_ref_FSD.clone().sub(xyz_body_ref_FSD_old)).applyMatrix4(invCTM))
	var myRange = document.getElementById("heaveRange");
	var myOutput = document.getElementById("heave");
	myRange.value = -xyz_body_grnd_NED.z
	myOutput.value = -xyz_body_grnd_NED.z
    
}
function updateOutput(){
    
    myOutput = document.getElementById("time");
    myOutput.value = Math.round(simulation_time*100)/100;

    myOutput = document.getElementById("rakeMeanPower");
    myOutput.value = Math.round(rakeMeanPower*10)/10;
}

function updateFluid(){
  var mySelect = document.getElementById("fluidSelect");
  if (mySelect.value == "Air")
  {
    density = 1.225;
  }
  else if (mySelect.value == "Water")
  {

    density = 1025;
  }
  
}

function computeBarycenter3(P1,P2,P3)
{
  P=P1.clone().add(P2).add(P3).multiplyScalar(1/3.)
  return P
}
function computeBuoyancy()
{  
  volume = 0
  CTM = new THREE.Matrix4;
  invCTM = new THREE.Matrix4;
  var rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
  tmp    = new THREE.Vector3( 0, 0, 0 );
  tmp1    = new THREE.Vector3( 0, 0, 0 );
  tmp2    = new THREE.Vector3( 0, 0, 0 );
  

  CTM.makeRotationFromEuler(rpy);
  invCTM.getInverse(CTM);
  xyz_ref_grnd_NED = xyz_body_ref_FSD.clone().multiplyScalar(-1).applyMatrix4(invCTM).add(xyz_body_grnd_NED)
  xyz_hAB_grnd_NED = xyz_hAB_ref_FSD.clone().applyMatrix4(invCTM).add(xyz_ref_grnd_NED)
  xyz_hFB_grnd_NED = xyz_hFB_ref_FSD.clone().applyMatrix4(invCTM).add(xyz_ref_grnd_NED)
  xyz_hAT_grnd_NED = xyz_hAT_ref_FSD.clone().applyMatrix4(invCTM).add(xyz_ref_grnd_NED)
  xyz_hFT_grnd_NED = xyz_hFT_ref_FSD.clone().applyMatrix4(invCTM).add(xyz_ref_grnd_NED)
  
  // deals only with most likely case
  if ((xyz_hAB_grnd_NED.z<=0)&(xyz_hFB_grnd_NED.z<=0))
  { // the hull is fully out of water
    volume = 0
    if (xyz_hAB_grnd_NED.z<xyz_hFB_grnd_NED.z)
    {
      // Take the most down point to ensure continuity
      xyz_buoyancy_grnd_NED = xyz_hFB_grnd_NED.clone()
    }
    else{
      if (xyz_hAB_grnd_NED.z==xyz_hFB_grnd_NED.z)
      {
        //Take the mean
        xyz_buoyancy_grnd_NED = xyz_hAB_grnd_NED.clone().add(xyz_hFB_grnd_NED).multiplyScalar(1/2.)
      }
      else{
        xyz_buoyancy_grnd_NED = xyz_hAB_grnd_NED.clone()
      }
    }
  }
  else
  {
    if ((xyz_hAB_grnd_NED.z>=0)&(xyz_hFB_grnd_NED.z<=0))
    { // Only bottom aft corner is in the water
      
      // use Thales to compute intersection points
      tmp1 = xyz_hAB_grnd_NED.clone().add(xyz_hAT_grnd_NED.clone().sub(xyz_hAB_grnd_NED).multiplyScalar(xyz_hAB_grnd_NED.z/(xyz_hAB_grnd_NED.z-xyz_hAT_grnd_NED.z)));
      tmp2 = xyz_hAB_grnd_NED.clone().add(xyz_hFB_grnd_NED.clone().sub(xyz_hAB_grnd_NED).multiplyScalar(xyz_hAB_grnd_NED.z/(xyz_hAB_grnd_NED.z-xyz_hFB_grnd_NED.z)));
      xyz_buoyancy_grnd_NED = computeBarycenter3(tmp1,tmp2,xyz_hAB_grnd_NED);
      volume = tmp.crossVectors(tmp1.sub(xyz_hAB_grnd_NED), tmp2.sub(xyz_hAB_grnd_NED)).length()/2;
      
    }
    if ((xyz_hFB_grnd_NED.z>=0)&(xyz_hAB_grnd_NED.z<=0))
    { // Only bottom fore corner is in the water
      
      // use Thales to compute intersection points
      tmp1 = xyz_hFB_grnd_NED.clone().add(xyz_hFT_grnd_NED.clone().sub(xyz_hFB_grnd_NED).multiplyScalar(xyz_hFB_grnd_NED.z/(xyz_hFB_grnd_NED.z-xyz_hFT_grnd_NED.z)));
      tmp2 = xyz_hFB_grnd_NED.clone().add(xyz_hAB_grnd_NED.clone().sub(xyz_hFB_grnd_NED).multiplyScalar(xyz_hFB_grnd_NED.z/(xyz_hFB_grnd_NED.z-xyz_hAB_grnd_NED.z)));
      xyz_buoyancy_grnd_NED = computeBarycenter3(tmp1,tmp2,xyz_hFB_grnd_NED);
      volume = tmp.crossVectors(tmp1.sub(xyz_hFB_grnd_NED), tmp2.sub(xyz_hFB_grnd_NED)).length()/2;
      
    }
    if ((xyz_hFB_grnd_NED.z>=0)&(xyz_hAB_grnd_NED.z>=0))
    { // Two bottom points in the water
      tmp1 = xyz_hAB_grnd_NED.clone().add(xyz_hAT_grnd_NED.clone().sub(xyz_hAB_grnd_NED).multiplyScalar(xyz_hAB_grnd_NED.z/(xyz_hAB_grnd_NED.z-xyz_hAT_grnd_NED.z)));
      tmp2 = xyz_hFB_grnd_NED.clone().add(xyz_hFT_grnd_NED.clone().sub(xyz_hFB_grnd_NED).multiplyScalar(xyz_hFB_grnd_NED.z/(xyz_hFB_grnd_NED.z-xyz_hFT_grnd_NED.z)));
      volume1 = tmp.crossVectors(tmp1.clone().sub(xyz_hAB_grnd_NED), xyz_hFB_grnd_NED.clone().sub(xyz_hAB_grnd_NED)).length()/2;
      volume2 = tmp.crossVectors(tmp1.clone().sub(xyz_hFB_grnd_NED), tmp2.clone().sub(xyz_hFB_grnd_NED)).length()/2;
      xyz_buoyancy_grnd_NED = computeBarycenter3(tmp1,xyz_hFB_grnd_NED,xyz_hAB_grnd_NED).multiplyScalar(volume1).add(computeBarycenter3(tmp1,tmp2,xyz_hFB_grnd_NED).multiplyScalar(volume2)).multiplyScalar(1/(volume1+volume2));
      volume = volume1+volume2;
    }
  }
  force = mass*g*volume/(Lpp*initialDraft)*isBuoyancy;
  XYZ_buoyancy_body_NED.z = -force;
  XYZ_buoyancy_body_FSD = XYZ_buoyancy_body_NED.clone().applyMatrix4(CTM);
  xyz_buoyancy_body_FSD = xyz_buoyancy_grnd_NED.clone().sub(xyz_body_grnd_NED).applyMatrix4(CTM);
  xyz_buoyancy_ref_FSD = xyz_buoyancy_body_FSD.clone().add(xyz_body_ref_FSD);
  KMN_buoyancy_body_FSD = tmp.clone().crossVectors(xyz_buoyancy_body_FSD, XYZ_buoyancy_body_FSD);
}

  
  
