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

// Simulation parameter
sampleTime      = 0.0005; // Sample time
isHeaveDynamic = true;
isPitchDyanmic = true;

// Plot parameter
meter2pix = 50;
Newton2meter = 0.00001

// Initial condition
var V = 10;
pqr0          = 0;    // Angular rate
pitch=0*Math.PI/180;
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
xyz_body_grnd_NED   = new THREE.Vector3( 0, 0, 0 );
xyz_body_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
uvw_body_grnd_NED   = new THREE.Vector3( V, 0, 0 );
xyz_CoG_ref_FSD     = new THREE.Vector3( 0, 0, 0 );
xyz_CoG_body_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_foil_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_foil_body_FSD   = new THREE.Vector3( 0, 0, 0 );
uvw_foil_grnd_NED   = new THREE.Vector3( 0, 0, 0 );
xyz_elev_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_elev_body_FSD   = new THREE.Vector3( 0, 0, 0 );
uvw_elev_grnd_NED   = new THREE.Vector3( 0, 0, 0 );

xyz_meas_ref_FSD    = new THREE.Vector3( 0, 0, 0 );
xyz_meas_body_FSD   = new THREE.Vector3( 0, 0, 0 );

XYZ_foil_body_NED    = new THREE.Vector3( 0, 0, 0 );
XYZ_elev_body_NED    = new THREE.Vector3( 0, 0, 0 );
XYZ_wght_body_NED    = new THREE.Vector3( 0, 0, 0 );
XYZ_all_body_NED     = new THREE.Vector3( 0, 0, 0 );
XYZ_foil_body_FSD    = new THREE.Vector3( 0, 0, 0 );
XYZ_elev_body_FSD    = new THREE.Vector3( 0, 0, 0 );
XYZ_wght_body_FSD    = new THREE.Vector3( 0, 0, 0 );
XYZ_all_body_FSD     = new THREE.Vector3( 0, 0, 0 );
KMN_foil_body_FSD    = new THREE.Vector3( 0, 0, 0 );
KMN_elev_body_FSD    = new THREE.Vector3( 0, 0, 0 );
KMN_wght_body_FSD    = new THREE.Vector3( 0, 0, 0 );
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
document.getElementById("elevatorRakeRange")        .addEventListener("change", updateElevatorRake);
document.getElementById("foilRakeRange")            .addEventListener("change", updateFoilRake);
document.getElementById("flightSpeedRange")         .addEventListener("change", updateFlightSpeed);
document.getElementById("massRange")                .addEventListener("change", updateMass);
document.getElementById("CGLongiRange")             .addEventListener("change", updateLongitudinalCenterOfInertiaPosition);
document.getElementById("CGVertUpRange")              .addEventListener("change", updateVerticalUpCenterOfInertiaPosition);
document.getElementById("gravityCheck")             .addEventListener("change", updateGravity);
document.getElementById("pitchInertiaRange")        .addEventListener("change", updatePitchInertia);
document.getElementById("elevatorAreaRange")        .addEventListener("change", updateElevatorArea);
document.getElementById("foilAreaRange")            .addEventListener("change", updateFoilArea);
document.getElementById("elevatorAspectRatioRange") .addEventListener("change", updateElevatorAspectRatio);
document.getElementById("foilAspectRatioRange")     .addEventListener("change", updateFoilAspectRatio);
document.getElementById("elevatorLongiRange")       .addEventListener("change", updateElevatorLongitudinalPosition);
document.getElementById("foilLongiRange")           .addEventListener("change", updateFoilLongitudinalPosition);
document.getElementById("elevatorVertUpRange")        .addEventListener("change", updateElevatorVerticalUpPosition);
document.getElementById("foilVertUpRange")            .addEventListener("change", updateFoilVerticalUpPosition);
document.getElementById("fluidSelect")              .addEventListener("change", updateFluid);

setInterval(updaten, 1);
setInterval(updatePlot,100);
var d = new Date();
var t0 = d.getTime();
told = 0;
simulation_time = 0;

function init(){
  // Do init to be sure values are the same as described in html page
  updatePitch();
  updatePitchDynamic();
  updateHeave();
  updateHeaveDynamic();
  updateElevatorRake();
  updateFoilRake();
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
  updateFluid();
}
init();
function plot(body_position, foil_position, elevator_position, foil_rake, elevator_rake){
  plotFoil();
  plotElevator();
  plotWeight();
  plotShip();
  plotFfoil();
  plotFelevator();
  plotFall();
  rotateFoil(foil_rake);
  rotateBody(pitch);
  rotateElevator(elevator_rake);
  translateFoil(xyz_foil_ref_FSD.x, xyz_foil_ref_FSD.z);
  translateElevator(xyz_elev_ref_FSD.x, xyz_elev_ref_FSD.z);
  translateCenterOfInertia(xyz_CoG_ref_FSD.x, xyz_CoG_ref_FSD.z);
  translateBody(xyz_body_grnd_NED.x, xyz_body_grnd_NED.z);
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
  angle_fluid_body = Math.atan2(uvw_fluid_foil_NED.z, uvw_fluid_foil_NED.x);
  q = 1/2*density *Math.pow(uvw_fluid_foil_NED.length(),2);
  AoA = pitch + foilRake + angle_fluid_body
  lift   = q*foilArea*liftCoefficient(AoA, foilAspectRatio);
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
  angle_fluid_body = Math.atan2(uvw_fluid_elev_NED.z, uvw_fluid_elev_NED.x);
  q = 1/2*density *Math.pow(uvw_fluid_elev_NED.length(),2);
  
  AoA = pitch + elevatorRake + angle_fluid_body
  lift   = q*foilArea*liftCoefficient(AoA, elevatorAspectRatio);
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
  
  XYZ_all_body_NED = XYZ_elev_body_NED.clone().add(XYZ_foil_body_NED).add(XYZ_wght_body_NED);
  XYZ_all_body_FSD = XYZ_all_body_NED.clone().applyMatrix4(CTM)
  KMN_all_body_FSD = KMN_elev_body_FSD.clone().add(KMN_foil_body_FSD).add(KMN_wght_body_FSD);
  //console.log(XYZ_wght_body_FSD);
  //console.log(KMN_wght_body_FSD);
  
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
  xyz_body_grnd_NED.add(uvw_body_grnd_NED.clone().multiplyScalar(dt))
  xyz_body_grnd_NED.x = 0
  
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
function translateBody(x, z){
  body_frame = document.getElementById("body_frame");
  body_frame.setAttribute('transform', 'translate(' +x*meter2pix +','+ z*meter2pix +')');
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
    xyz_CoG_ref_FSD.x = myOutput.value;
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    
}
function updateVerticalUpCenterOfInertiaPosition(){
		//get elements
		var myRange = document.getElementById("CGVertUpRange");
		var myOutput = document.getElementById("CGVertUp");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_CoG_ref_FSD.z = -myOutput.value;
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
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
    xyz_elev_ref_FSD.x = myOutput.value;
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateFoilLongitudinalPosition(){
		//get elements
		var myRange = document.getElementById("foilLongiRange");
		var myOutput = document.getElementById("foilLongi");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_foil_ref_FSD.x = myOutput.value;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateElevatorVerticalUpPosition(){
		//get elements
		var myRange = document.getElementById("elevatorVertUpRange");
		var myOutput = document.getElementById("elevatorVertUp");
		//copy the value over
		myOutput.value = myRange.value;
    xyz_elev_ref_FSD.z = -myOutput.value;
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
  
  
