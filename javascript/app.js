"use strict"
var clientX = 0;
var clientY = 0;
var ApRakeOld = 0;
var APRakeNFU = 0;
var integralTerm = 0;
document.addEventListener("mousemove", function(event) {
  // Save the last known mouse position
    clientX = event.clientX;
    clientY = event.clientY + document.body.scrollTop;
 })
console.log("This is a foil simulator");
var g_CLs = [],
    g_CDs = [],
    g_alphas_deg = [];

// LED param    
var strip, strip2; // need to be global
var container = document.getElementById("ledstrip");
var container2 = document.getElementById("ledstrip2");
var light_count = 17;
strip = LEDstrip(container, light_count);
strip2 = LEDstrip(container2, light_count);
    

// Environment parameters
var airDensity         = 1.225,
    waterDensity       = 1025,
    earth_gravity      = 9.81,
    g,
    density;

g = earth_gravity;
density = waterDensity;

// Ship parameters
var mass                = 3000, 
    pitchInertia        = 2e5,
    foilArea            = 1,
    elevatorArea        = 0.19 * 2,
    foilAspectRatio     = 5,
    elevatorAspectRatio = 5,
    stallAngle          = 0,
    stallRecoveryAngle  = 0,
    initialDraft        = 0.25,
    Lpp                 = 15,

    isFoilStall         = false,
    isElevStall         = false,
    
    heaveStiffness      = 0,
    pitchStiffness      = 0;

// Simulation parameter
var sampleTime          = 0.0005, // Sample time
    isHeaveDynamic      = false,
    isPitchDynamic      = false,
    isBuoyancy          = true,
    isSurfaceEffect     = true,
    isSurface           = true,
    foilRakeDelay       = 0,
    foilRakeStep        = 0.2 * Math.PI / 180.,
    elevatorRakeStep    = 0.1 * Math.PI / 180.,
    targetHeight        = -1;
   
var ma_ts = 0.01;
var ma = simple_moving_averager(3/ma_ts);
var movingAverage = 0;
var wasConsecutiveClicks = 0;

    // Plot parameter
var meter2pix           = 50,
    Newton2meter        = 0.00001;

// Initial condition
var V                    = 10,
    q0                   = 0,    // Angular rate
    pitch                = 2.2 * Math.PI / 180,
    rakeMeanPower        = 0,
    flightSpeedVariation = 0;
var    elevatorRakeFiltCorrection = 0;

// Control parameters
var x_virtual = 13;
var allowedElevatorRakeForControl = 0.5*Math.PI/180;
var allowedElevatorRakeForControlTimeConstant = 5;

var APEngage = 0;
var APtarget = 0;
var APRake   = 0;

var x_rotationAxis = 0;
var z_rotationAxis = 0;

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
// uvw_fluid_ground_NED_kt means the velocity of the fluid relative
// to the ground projected in NED frame expressed in knots
// unit is omitted if SI units.

// body is the reference used for computation
// ref  is the point from which other points are defined
// fluid describe the fluid at infinity
// meas is the point for output

// Position coordinates are given relative to the hull reference point
// which is aft, at MWP (waterline in equilibrium condition)
var uvw_fluid_grnd_NED  = new THREE.Vector3( 0, 0, 0 ),
    xyz_body_grnd_NED   = new THREE.Vector3( 0, 0, -1.2 ),
    xyz_body_ref_FSD    = new THREE.Vector3( 0, 0, 0 ),
    uvw_body_grnd_NED   = new THREE.Vector3( V, 0, 0 ),
    xyz_CoG_ref_FSD     = new THREE.Vector3( 0, 0, 0 ),
    xyz_CoG_body_FSD    = new THREE.Vector3( 0, 0, 0 ),
    xyz_foil_ref_FSD    = new THREE.Vector3( 0, 0, 0 ),
    xyz_foil_body_FSD   = new THREE.Vector3( 0, 0, 0 ),
    xyz_foil_grnd_NED   = new THREE.Vector3( 0, 0, 0 ),
    uvw_foil_grnd_NED   = new THREE.Vector3( 0, 0, 0 ),
    xyz_elev_ref_FSD    = new THREE.Vector3( 0, 0, 0 ),
    xyz_elev_body_FSD   = new THREE.Vector3( 0, 0, 0 ),
    xyz_foil_grnd_FSD   = new THREE.Vector3( 0, 0, 0 ),
    uvw_elev_grnd_NED   = new THREE.Vector3( 0, 0, 0 ),
    xyz_output_ref_FSD  = new THREE.Vector3( 0, 0, 0 ),
    xyz_output_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
	uvw_output_grnd_NED = new THREE.Vector3( 0, 0, 0 );

// Hull extremity for archimedian thrust
var xyz_hAB_ref_FSD = new THREE.Vector3( 0, 0, 0),   // Hull Aft Bottom
    xyz_hFB_ref_FSD = new THREE.Vector3( Lpp, 0, 0), // Hull fore Bottom
    xyz_hAT_ref_FSD = new THREE.Vector3( 0, 0, -1),             // Hull Aft Top
    xyz_hFT_ref_FSD = new THREE.Vector3( Lpp, 0, -1);           // Hull fore Top

var xyz_buoyancy_ref_FSD  = new THREE.Vector3( 0, 0, 0 ),
    xyz_buoyancy_body_FSD = new THREE.Vector3( 0, 0, 0 ),
    xyz_buoyancy_grnd_NED = new THREE.Vector3( 0, 0, 0 );

// Sensors
var xyz_meas_ref_FSD    = new THREE.Vector3( 0, 0, 0 ),
    xyz_meas_body_FSD   = new THREE.Vector3( 0, 0, 0 );

// Forces vectors
var XYZ_foil_body_NED     = new THREE.Vector3( 0, 0, 0 ),
    XYZ_elev_body_NED     = new THREE.Vector3( 0, 0, 0 ),
    XYZ_wght_body_NED     = new THREE.Vector3( 0, 0, 0 ),
    XYZ_buoyancy_body_NED = new THREE.Vector3( 0, 0, 0 ),
    XYZ_all_body_NED      = new THREE.Vector3( 0, 0, 0 ),
    XYZ_foil_body_FSD     = new THREE.Vector3( 0, 0, 0 ),
    XYZ_elev_body_FSD     = new THREE.Vector3( 0, 0, 0 ),
    XYZ_wght_body_FSD     = new THREE.Vector3( 0, 0, 0 ),
    XYZ_buoyancy_body_FSD = new THREE.Vector3( 0, 0, 0 ),
    XYZ_all_body_FSD      = new THREE.Vector3( 0, 0, 0 );

// Torque vectors
var KMN_foil_body_FSD     = new THREE.Vector3( 0, 0, 0 ),
    KMN_elev_body_FSD     = new THREE.Vector3( 0, 0, 0 ),
    KMN_wght_body_FSD     = new THREE.Vector3( 0, 0, 0 ),
    KMN_buoyancy_body_FSD = new THREE.Vector3( 0, 0, 0 ),
    KMN_all_body_FSD      = new THREE.Vector3( 0, 0, 0 ),
    pqr_body_grnd_FSD     = new THREE.Vector3( 0, 0, 0 );

// Control
var elevatorRake  = 0,
    elevatorRakeTotal =0,  
    foilRake      = 0;
    
var keyExpCount = 1;
var previousKeySign = 0;
var stepGain = 0;

pqr_body_grnd_FSD.y = q0;

document.getElementById("viewSelect")        .
    addEventListener("change", updateView);
document.getElementById("targetHeightRange")        .
    addEventListener("change", updateTargetHeight);
document.getElementById("virtualForceLongiRange")        .
    addEventListener("change", updateVirtualForceLongi);
document.getElementById("allowedElevatorRakeForControlRange")        .
    addEventListener("change", updateAllowedElevatorRakeForControl);
document.getElementById("allowedElevatorRakeForControlTimeConstantRange")        .
    addEventListener("change", updateAllowedElevatorRakeForControlTimeConstant);
document.getElementById("pitchRange")               .
    addEventListener("change", updatePitch);
document.getElementById("pitchDynamicCheck")        .
    addEventListener("change", updatePitchDynamic);
document.getElementById("pitchRateRange")               .
    addEventListener("change", updatePitchRate);
document.getElementById("pitchStiffnessRange")               .
    addEventListener("change", updatePitchStiffness);
document.getElementById("heaveRange")               .
    addEventListener("change", updateHeave);
document.getElementById("heaveDynamicCheck")        .
    addEventListener("change", updateHeaveDynamic);
document.getElementById("heaveRateRange")               .
    addEventListener("change", updateHeaveRate);
document.getElementById("heaveStiffnessRange")               .
    addEventListener("change", updateHeaveStiffness);
document.getElementById("surfaceCheck")             .
    addEventListener("change", updateSurface);
document.getElementById("surfaceEffectCheck")       .
    addEventListener("change", updateSurfaceEffect);
document.getElementById("buoyancyCheck")            .
    addEventListener("change", updateBuoyancy);
document.getElementById("elevatorRakeRange")        .
    addEventListener("change", updateElevatorRake);
document.getElementById("foilRakeRange")            .
    addEventListener("change", updateFoilRake);
document.getElementById("foilRakeDelayRange")       .
    addEventListener("change", updateFoilRakeDelay);
document.getElementById("foilRakeStepRange")        .
    addEventListener("change", updateFoilRakeStep);
document.getElementById("flightSpeedRange")         .
    addEventListener("change", updateFlightSpeed);
document.getElementById("flightSpeedVariationRange").
    addEventListener("change", updateFlightSpeedVariation);
document.getElementById("massRange")                .
    addEventListener("change", updateMass);
document.getElementById("CGLongiRange")             .
    addEventListener("change", updateLongitudinalCenterOfInertiaPosition);
document.getElementById("CGVertUpRange")            .
    addEventListener("change", updateVerticalUpCenterOfInertiaPosition);
document.getElementById("gravityCheck")             .
    addEventListener("change", updateGravity);
document.getElementById("pitchInertiaRange")        .
    addEventListener("change", updatePitchInertia);
document.getElementById("elevatorAreaRange")        .
    addEventListener("change", updateElevatorArea);
document.getElementById("foilAreaRange")            .
    addEventListener("change", updateFoilArea);
document.getElementById("elevatorAspectRatioRange") .
    addEventListener("change", updateElevatorAspectRatio);
document.getElementById("foilAspectRatioRange")     .
    addEventListener("change", updateFoilAspectRatio);
document.getElementById("stallAngleRange") .
    addEventListener("change", updateStallAngle);
document.getElementById("stallRecoveryAngleRange")     .
    addEventListener("change", updateStallRecoveryAngle);
document.getElementById("elevatorLongiRange")       .
    addEventListener("change", updateElevatorLongitudinalPosition);
document.getElementById("foilLongiRange")           .
    addEventListener("change", updateFoilLongitudinalPosition);
document.getElementById("elevatorVertUpRange")      .
    addEventListener("change", updateElevatorVerticalUpPosition);
document.getElementById("foilVertUpRange")          .
    addEventListener("change", updateFoilVerticalUpPosition);
document.getElementById("outputLongiRange")           . 
    addEventListener("change", updateOutputLongitudinalPosition);
document.getElementById("outputVertUpRange")          .
    addEventListener("change", updateOutputVerticalUpPosition);
document.getElementById("fluidSelect")              .
    addEventListener("change", updateFluid);
setInterval(updateMovingAverage, ma_ts*1000);
function updateMovingAverage() {
    movingAverage = ma(wasConsecutiveClicks);
    movingAverage = movingAverage*3/ma_ts;
    wasConsecutiveClicks = 0;
}    
document.addEventListener("wheel", function(ev) {
    foilRake = foilRake - ev.deltaY / Math.abs(ev.deltaY) * foilRakeStep
    var myRange  = document.getElementById("foilRakeRange");
    var myOutput = document.getElementById("foilRake");
    //copy the value over
    myOutput.value = Math.round(foilRake * 180 / Math.PI * 100) / 100;
    myRange.value  = Math.round(foilRake * 180 / Math.PI * 100) / 100;
});
//To prevent scolling the page
window.onwheel = function() { return false; }

document.addEventListener("keydown", function (event) {
    var keySign=0;
  var mySelect = document.getElementById("controlSelect");
    var x_rudder = xyz_elev_ref_FSD.x;
    var x_foil = xyz_foil_ref_FSD.x;
    var S_foil = foilArea;
    var S_rudder = elevatorArea;
    if (event.defaultPrevented) {
        return; // Should do nothing if the key event was already consumed.
    }
    switch (event.key) {
        case "i":
            APEngage = 1;
            break;
        case "o":
            APEngage = 0;
            break;
        case "ArrowUp":
            keySign = +1;
            switch (mySelect.value){
                case "FoilOnlySelect":
                    foilRake = foilRake + foilRakeStep*stepGain;
                    break;
                case "ElevatorOnlySelect":
                    elevatorRake = elevatorRake - elevatorRakeStep;
                    break;
                case "FoilAndElevatorSelect":
                    foilRake = foilRake + foilRakeStep*stepGain;
                    elevatorRake = elevatorRake - (x_virtual-x_foil)/(x_virtual-x_rudder)*S_foil/S_rudder*foilRakeStep*stepGain;
					break;
                case "FoilAndElevatorSatSelect":
                    foilRake = foilRake + foilRakeStep*stepGain;
                    elevatorRakeFiltCorrection = elevatorRakeFiltCorrection - (x_virtual-x_foil)/(x_virtual-x_rudder)*S_foil/S_rudder*foilRakeStep*stepGain
                    break;
                default:
                    //console.log(event.key)
                    return; // Quit when this doesn't handle the key event.
            }
            break
        case "ArrowDown":
            keySign = -1;
            switch (mySelect.value){
                case "FoilOnlySelect":
                    foilRake = foilRake - foilRakeStep*stepGain;
                    break;
                case "ElevatorOnlySelect":
                    elevatorRake = elevatorRake + elevatorRakeStep;
                    break;
                case "FoilAndElevatorSelect":
                    foilRake = foilRake - foilRakeStep*stepGain;
                    elevatorRake = elevatorRake + (x_virtual-x_foil)/(x_virtual-x_rudder)*S_foil/S_rudder*foilRakeStep*stepGain;
					break;
                case "FoilAndElevatorSatSelect":
                    foilRake = foilRake - foilRakeStep*stepGain;
                    elevatorRakeFiltCorrection = elevatorRakeFiltCorrection + (x_virtual-x_foil)/(x_virtual-x_rudder)*S_foil/S_rudder*foilRakeStep*stepGain
                    break;
                default:
                    //console.log(event.key)
                    return; // Quit when this doesn't handle the key event.
            }
            break;
        case "ArrowRight":
            elevatorRake = elevatorRake + elevatorRakeStep;
            break;
        case "ArrowLeft":
            elevatorRake = elevatorRake - elevatorRakeStep;
            break;
        default:
            //console.log(event.key)
            return; // Quit when this doesn't handle the key event.
    }
    elevatorRakeFiltCorrection = Math.min(Math.max(elevatorRakeFiltCorrection, -allowedElevatorRakeForControl), allowedElevatorRakeForControl);
                    
    if (keySign == previousKeySign) {
        keyExpCount = keyExpCount + 1;
        wasConsecutiveClicks = 1;
    }
    previousKeySign = keySign;
        
    var myRange = document.getElementById("foilRakeRange");
    var myOutput = document.getElementById("foilRake");
    //copy the value over
    myOutput.value = Math.round(foilRake * 180 / Math.PI * 100) / 100;
    myRange.value = myOutput.value;
    
    var myRange = document.getElementById("elevatorRakeRange");
    var myOutput = document.getElementById("elevatorRake");
    //copy the value over
    myOutput.value = Math.round(elevatorRakeTotal * 180 / Math.PI * 100) / 100;
    myRange.value=myOutput.value;

    // Consume the event for suppressing "double action".
    event.preventDefault();
}, true);

setInterval(updaten, 1);
setInterval(updatePlot, 100);
setInterval(saveRake, 100);
var d = new Date();
var t0 = d.getTime();
var simulation_time = 0;

//create and fill a circular buffar to store foil rake a be able to apply pure delay
var rake = [];
var i_rakeBuffer = 0;
init();
function saveRake() {
    i_rakeBuffer = i_rakeBuffer + 1
    if (i_rakeBuffer>20) {
        i_rakeBuffer = 0;
    }
    rake[i_rakeBuffer] = foilRake;
}
function getRakeDelayed(delay) {
    // Create a round robin circular list to store history of rake
    var i_delay = i_rakeBuffer - Math.round(delay / 0.1); // History is sampled at 10Hz
    if (i_delay<0) {
        i_delay = i_delay + 20;
    }
    return rake[i_delay];
}
function init() {
    // Do init to be sure values are the same as described in html page
    updateTargetHeight();
    updateVirtualForceLongi();
    updateAllowedElevatorRakeForControl();
    updateAllowedElevatorRakeForControlTimeConstant();
    updatePitch();
    updatePitchDynamic();
    updatePitchRate();
    updatePitchStiffness();
    updateHeave();
    updateHeaveDynamic();
    updateHeaveRate();
    updateHeaveStiffness();
    updateSurface();
    updateSurfaceEffect();
    updateBuoyancy();
    updateElevatorRake();
    updateFoilRake();
    updateFoilRakeDelay();
    updateFoilRakeStep();
    updateFlightSpeed();
    updateFlightSpeedVariation();
    updateMass();
    updateLongitudinalCenterOfInertiaPosition();
    updateVerticalUpCenterOfInertiaPosition();
    updateGravity();
    updatePitchInertia();
    updateElevatorArea();
    updateFoilArea();
    updateElevatorAspectRatio();
    updateFoilAspectRatio();
    updateStallAngle();
    updateStallRecoveryAngle();
    updateElevatorLongitudinalPosition();
    updateFoilLongitudinalPosition();
    updateElevatorVerticalUpPosition();
    updateFoilVerticalUpPosition();
    updateBodyLongitudinalPosition();
    updateBodyVerticalUpPosition();
    updateOutputLongitudinalPosition();
    updateOutputVerticalUpPosition();
    updateFluid();
}
function plot(body_position, foil_position, elevator_position, foil_rake, elevator_rake) {
    plotTargetHeight();
    plotFoil();
    plotElevator();
    plotWeight();
    plotBuoyancy();
    plotShip();
    plotFfoil();
    plotFelevator();
    plotFall();
    translateBody(xyz_body_grnd_NED.x * 0, xyz_body_grnd_NED.z);
    rotateBody(pitch);
    translateRef(-xyz_body_ref_FSD.x, -xyz_body_ref_FSD.z);
    translateBuoyancy(xyz_buoyancy_ref_FSD.x, xyz_buoyancy_ref_FSD.z)
    translateFoil(xyz_foil_ref_FSD.x, xyz_foil_ref_FSD.z);
    translateElevator(xyz_elev_ref_FSD.x, xyz_elev_ref_FSD.z);
    translateCenterOfInertia(xyz_CoG_ref_FSD.x, xyz_CoG_ref_FSD.z);
    translateVirtual(x_virtual)
    translateRotationAxis(x_rotationAxis, z_rotationAxis);
    rotateFoil(foil_rake);
    rotateElevator(elevator_rake); 

    // For 3D view in other tab
    localStore()
}
function updateLED() {
    var mu;
    var mySelect = document.getElementById("LEDSelect");
    switch (mySelect.value){
        case "HeightSelect":
            mu = (xyz_output_grnd_NED.z - pitch*7)/2
            break
        case "TrimSelect":
            mu = -pitch*180/Math.PI/5
            break;
        case "APSelect":
            mu = -(APRake-getRakeDelayed(foilRakeDelay))/3*180/Math.PI/3;
			
			// incremental control
			mu = -APRakeNFU/(3*Math.PI/180);
			
			// absolute control
			mu= -(APRake -3*Math.PI/180)/(2*Math.PI/180)
			
            break
        default:
            return
    }
        
    //mu = 0
    mu = Math.max(-1, Math.min(mu, 1))
    var sigma = 0.01;
    for (var i=0;i<light_count;i++) {
        var x = i/(light_count-1)*2 -1
        if ((mu==1) || (mu==-1)) {
            if (Math.abs(mu-x)<0.1) {
                strip.buffer[i] = [255, 0, 0]
            }
            else {
                strip.buffer[i] = [0, 0, 0]
            }
            
        }
        else {    
                strip.buffer[i] = [0, Math.floor(255*Math.exp(-Math.pow(x-mu, 2)/(2*sigma))), 0]
        }
    }
    strip.send();
}

function updateLED2() {
    var mu;
    var mySelect = document.getElementById("LEDSelect");
    switch ("TrimSelect"){
        case "HeightSelect":
            mu = (xyz_output_grnd_NED.z + pitch*7)/2
            break
        case "TrimSelect":
            mu = pitch*180/Math.PI/5
            break;
        case "APSelect":
            mu = (APRake-getRakeDelayed(foilRakeDelay))/foilRakeStep/10;
            break
        default:
            return
    }
        
    //mu = 0
    //mu = 0
    mu = Math.max(-1, Math.min(mu, 1))
    var sigma = 0.01;
    for (var i=0;i<light_count;i++) {
        var x = i/(light_count-1)*2 -1
        if ((mu==1) || (mu==-1)) {
            if (Math.abs(mu-x)<0.1) {
                strip2.buffer[i] = [255, 0, 0]
            }
            else {
                strip2.buffer[i] = [0, 0, 0]
            }
            
        }
        else {    
                strip2.buffer[i] = [0, Math.floor(255*Math.exp(-Math.pow(x-mu, 2)/(2*sigma))), 0]
        }
    }
    strip2.send();
}
function updatePlot() {
    plot(xyz_body_grnd_NED, xyz_foil_body_FSD, xyz_elev_body_FSD,
        foilRake, elevatorRakeTotal, pitch);
    updateOutput();
}
function updaten() {
    for (var i = 0;i<8;i++) { 
    // Dirty manual tuning to get close of real time on my computer
        update();
    }
    updateLED();
    updateLED2();
	//foilRake = foilRake - (clientY-242)/106*sampleTime*8*Math.PI/180*3;
	//foilRake = -(clientY-242)/106*Math.PI/180*2+3*Math.PI/180;
}
function computeForcesOnLiftingSurface(CTM, pitch, uvw_fluid_grnd_NED,
    uvw_surf_grnd_NED, xyz_surf_grnd_NED, xyz_surf_body_FSD, AoK, density, chord, z_waterSurface,
    isSurfaceEffect, isSurface, isSurfStall, surfArea, surfAspectRatio) {
        
    var angle_uvw_surf_fluid_NED, q, AoA, chord, lift, drag,
        inverseMirrorEffect=1,
        dragSurfaceEffect=1,
        stallEffect=1;
    var tmp    = new THREE.Vector3( 0, 0, 0 ),
        uvw_fluid_surf_NED = new THREE.Vector3( 0, 0, 0 );
    var CTMf   = new THREE.Matrix4;
    var XYZ_surf_body_FSD = new THREE.Vector3( 0, 0, 0 ),
        KMN_surf_body_FSD = new THREE.Vector3( 0, 0, 0 ),
        XYZ_surf_body_NED = new THREE.Vector3( 0, 0, 0 ),
        KMN_surf_body_NED = new THREE.Vector3( 0, 0, 0 );
    
    // Compute the velocity of lifting surface with respect to the fluid
    uvw_fluid_surf_NED =  uvw_fluid_grnd_NED.clone().sub(uvw_surf_grnd_NED);
    
    // Compute the angle of relative velocity of lifting surface
    // in fluid from ground reference frame
    angle_uvw_surf_fluid_NED = Math.atan2(
        -uvw_fluid_surf_NED.z, -uvw_fluid_surf_NED.x);
    
    // Compute dynamic pressure
    q = 1 / 2 * density * Math.pow(uvw_fluid_surf_NED.length(), 2);
    AoA = pitch + AoK + angle_uvw_surf_fluid_NED;
    
    
    // -------Compute the inverse mirror effect on lift and drag
    
    if (xyz_surf_grnd_NED.z>z_waterSurface) { // z is positive down
        // formula according to Hydrodynamics of High Speed 
        // Marine Vehicles equation 6.144 on page 200
        inverseMirrorEffect = 
            (1 + 16 * Math.pow(
                Math.min(0, z_waterSurface - xyz_surf_grnd_NED.z) / chord, 2))
            /
            (2 + 16 * Math.pow(
            Math.min(0, z_waterSurface - xyz_surf_grnd_NED.z) / chord, 2));
        if (!isSurfaceEffect) {
            inverseMirrorEffect = 1;
        }
        dragSurfaceEffect = 1;
    }
    else {
        inverseMirrorEffect = 0;
        dragSurfaceEffect   = 0;
    }
    if (!isSurface) {
        inverseMirrorEffect = 1;
        dragSurfaceEffect   = 1;
    }
    
    // Add stall effect
    if (Math.abs(AoA)>stallAngle) {
        isSurfStall = true;
    }
    if (Math.abs(AoA)<stallRecoveryAngle) { 
        isSurfStall = false;
    }
    if (isSurfStall) {   
        stallEffect = 0.3;
    }
    else {
        stallEffect = 1;
    }
    
    // Compute lift and drag
    lift   = q * surfArea*liftCoefficient(
        AoA, surfAspectRatio, g_CLs, g_alphas_deg)*
            inverseMirrorEffect*stallEffect;
    drag   = q * surfArea*dragCoefficient(
        AoA, surfAspectRatio, g_CDs, g_alphas_deg)*
            dragSurfaceEffect;
    
    // Rotate to ground frame
    CTMf.makeRotationY(angle_uvw_surf_fluid_NED);
    XYZ_surf_body_NED.set( -drag, 0, -lift);
    XYZ_surf_body_NED.applyMatrix4(CTMf);
    
    // Compute torque
    XYZ_surf_body_FSD = XYZ_surf_body_NED.clone().applyMatrix4(CTM);
    KMN_surf_body_FSD = tmp.clone().
        crossVectors(xyz_surf_body_FSD, XYZ_surf_body_FSD );
    return [XYZ_surf_body_FSD, KMN_surf_body_FSD, XYZ_surf_body_NED, KMN_surf_body_NED, isSurfStall];
}
function computeWeight(CTM, mass, g, xyz_CoG_body_FSD) {
  
    var tmp    = new THREE.Vector3(0, 0, 0 );
    XYZ_wght_body_NED.set(0, 0, mass * g); // z is vertical done
    XYZ_wght_body_FSD = XYZ_wght_body_NED.clone().applyMatrix4(CTM);
    KMN_wght_body_FSD = tmp.clone().
        crossVectors(xyz_CoG_body_FSD, XYZ_wght_body_FSD );
}
function computeForces() {
    
    var rpy    = new THREE.Euler( 0, -pitch, 0, 'XYZ' ),
        CTM    = new THREE.Matrix4,
        invCTM = new THREE.Matrix4,
        tmp    = new THREE.Vector3( 0, 0, 0 ),
        uvw_foil_grnd_NED_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        xyz_foil_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        uvw_elev_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        xyz_elev_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        KMN_foil_body_NED = new THREE.Vector3( 0, 0, 0 ),
        KMN_elev_body_NED = new THREE.Vector3( 0, 0, 0 );
      
    var z_waterSurface = 0, AoK, chord, res = [];
      
    // Compute the Coordinate Transform Matrix from ground to body frame
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
    
    // Compute weight
    computeWeight(CTM, mass, g, xyz_CoG_body_FSD);
    
    // Compute Buoyancy
    computeBuoyancy();
    
    // Compute hydrodynamic forces
    // Assume there is no current
    uvw_fluid_grnd_NED.set(0, 0, 0);
    // First compute the velocity of foil relative to ground frame
    uvw_foil_grnd_NED = uvw_body_grnd_NED.clone().add(
        tmp.crossVectors(pqr_body_grnd_FSD, xyz_foil_body_FSD).
            applyMatrix4(invCTM));
    
    xyz_foil_grnd_NED = xyz_foil_body_FSD.clone().
    	applyMatrix4(invCTM).add(xyz_body_grnd_NED);
    AoK = getRakeDelayed(foilRakeDelay) + heaveStiffness *(xyz_foil_grnd_NED.z-z_waterSurface) - pitchStiffness * pitch;
    
    chord = Math.sqrt(foilArea / foilAspectRatio);
    
    res = computeForcesOnLiftingSurface(CTM, pitch, uvw_fluid_grnd_NED,
        uvw_foil_grnd_NED, xyz_foil_grnd_NED, xyz_foil_body_FSD, AoK, density, chord,
            z_waterSurface, isSurfaceEffect, isSurface, isFoilStall, foilArea, foilAspectRatio) 
    XYZ_foil_body_FSD = res[0].clone();
    KMN_foil_body_FSD = res[1].clone();
    XYZ_foil_body_NED = res[2].clone();
    KMN_foil_body_NED = res[3].clone();
    isFoilStall = res[4];
    
    // Compute forces on elevator
    uvw_elev_grnd_NED = uvw_body_grnd_NED.clone().add(
        tmp.crossVectors(pqr_body_grnd_FSD, xyz_elev_body_FSD).applyMatrix4(invCTM))
    AoK = elevatorRake;
    xyz_elev_grnd_NED = xyz_elev_body_FSD.clone().applyMatrix4(invCTM).add(xyz_body_grnd_NED)
    chord = Math.sqrt(elevatorArea/elevatorAspectRatio);
        
    res = computeForcesOnLiftingSurface(CTM, pitch, uvw_fluid_grnd_NED,
        uvw_elev_grnd_NED, xyz_elev_grnd_NED, xyz_elev_body_FSD, AoK, density, chord,
            z_waterSurface, isSurfaceEffect, isSurface, isElevStall, elevatorArea, elevatorAspectRatio) 
    XYZ_elev_body_FSD = res[0].clone();
    KMN_elev_body_FSD = res[1].clone();
    XYZ_elev_body_NED = res[2].clone();
    KMN_elev_body_NED = res[3].clone();
    isElevStall = res[4];
    
    XYZ_all_body_NED = XYZ_elev_body_NED.clone().
        add(XYZ_foil_body_NED).add(XYZ_wght_body_NED).
            add(XYZ_buoyancy_body_NED);
    XYZ_all_body_FSD = XYZ_all_body_NED.clone().applyMatrix4(CTM)
    KMN_all_body_FSD = KMN_elev_body_FSD.clone().
        add(KMN_foil_body_FSD).add(KMN_wght_body_FSD).
        add(KMN_buoyancy_body_FSD);
    //console.log(XYZ_wght_body_FSD);
    //console.log(KMN_wght_body_FSD);
    
        xyz_output_grnd_NED = (xyz_output_ref_FSD.clone().sub(xyz_body_ref_FSD)).
    	applyMatrix4(invCTM).add(xyz_body_grnd_NED);
		tmp    = new THREE.Vector3( 0, 0, 0 ),
		uvw_output_grnd_NED = uvw_body_grnd_NED.clone().add(tmp.crossVectors(pqr_body_grnd_FSD, xyz_output_ref_FSD));
    
}

function computeFF() {
    return 3*Math.PI/180
}
function computeAP() {
    APtarget = computeFF() + (xyz_output_grnd_NED.z-targetHeight)*(4)*Math.PI/180 + (pitch - 0)*(-0.5) -0.5*pqr_body_grnd_FSD.y
    return APtarget;
}
function computeAP_NFU() {
			return 2*(uvw_output_grnd_NED.z)*Math.PI/180 - 0.5*pqr_body_grnd_FSD.y + (xyz_output_grnd_NED.z-targetHeight)*(2)*Math.PI/180/2;
}
function update() {
    computeForces();
    var dt = sampleTime;
    
    elevatorRakeFiltCorrection  =     elevatorRakeFiltCorrection*Math.exp(-dt/allowedElevatorRakeForControlTimeConstant);
    elevatorRakeTotal = elevatorRake + elevatorRakeFiltCorrection;
    
    keyExpCount = keyExpCount*Math.exp(-dt/5);
    //keyMovingAverageCount = 
    if (document.getElementById("autoRakeStepCheck").checked) {
        stepGain = Math.min(0.7511*Math.exp(0.1*movingAverage), 2.5);
        //stepGain = Math.min(0.7511*Math.exp(0.1*keyExpCount), 2.5);
    }
    else {
        stepGain = 1;
    }
    
    APRake = computeAP();
	APRakeNFU = computeAP_NFU();
    if (APEngage) {
        foilRake = APRake;
		//foilRake = foilRake + APRakeNFU*sampleTime;
    }      
    
        
        
    simulation_time = simulation_time + dt;
    var inv_mass = 1. / mass;
    if (true==isHeaveDynamic) {
        uvw_body_grnd_NED.add(XYZ_all_body_NED.clone().multiplyScalar(inv_mass*dt))
    }
    else {
        uvw_body_grnd_NED.z = 0
    }
    uvw_body_grnd_NED.x = V +
        flightSpeedVariation * Math.sin(2 * Math.PI / 30. * simulation_time)
    //console.log(uvw_body_grnd_NED)
    xyz_body_grnd_NED.add(uvw_body_grnd_NED.clone().multiplyScalar(dt))
    
    if (true==isPitchDynamic) {
        pqr_body_grnd_FSD.add(KMN_all_body_FSD.clone().
            multiplyScalar(1 / pitchInertia * dt));
    }
    else {
        pqr_body_grnd_FSD.y = pqr_body_grnd_FSD.y;
    }
    pitch = pitch + pqr_body_grnd_FSD.y * dt;
    
    //console.log(xyz_body_grnd_NED)
    updateHeave();
    updateHeaveRate();
    updatePitch();
    updatePitchRate();
    updateRotationAxis();
    
}
function updateRotationAxis() {
    x_rotationAxis = xyz_CoG_ref_FSD.x-pitchInertia/(mass*x_virtual-xyz_CoG_ref_FSD.x);
    z_rotationAxis = xyz_CoG_ref_FSD.z;
}
function rotateFoil(r) {
    var foilFrame = document.getElementById("foil");
    var r_deg = r * 180 / Math.PI;
    foilFrame.setAttribute('transform', 'rotate(' +-r_deg + ')');
}
function rotateElevator(r) {
    var elevatorFrame = document.getElementById("elevator");
    var r_deg = r * 180 / Math.PI;
    elevatorFrame.setAttribute('transform', 'rotate(' +-r_deg + ')');
}
function rotateBody(r) {
    var bodyFrame = document.getElementById("body__rotate_frame");
    var r_deg = r * 180 / Math.PI;
    bodyFrame.setAttribute('transform', 'rotate(' +-r_deg + ')');
}
function translateFoil(x, z) {
    var foil_frame = document.getElementById("foil_frame");
    foil_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateElevator(x, z) {
    var elevator_frame = document.getElementById("elevator_frame");
    elevator_frame.setAttribute('transform', 'translate(' + (x * meter2pix) + ',' + (z * meter2pix) + ')');
}
function translateCenterOfInertia(x, z) {
    var elevator_frame = document.getElementById("CoG_frame");
    elevator_frame.setAttribute('transform', 'translate(' + (x * meter2pix) + ',' + (z * meter2pix) + ')');
}
function translateBuoyancy(x, z) {
    var B_frame = document.getElementById("B_frame");
    B_frame.setAttribute('transform', 'translate(' + (x * meter2pix) + ',' + (z * meter2pix) + ')');
}
function translateBody(x, z) {
    var body_frame = document.getElementById("body_frame");
    body_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateRef(x, z) {
    var ref_frame = document.getElementById("ref_frame");
    ref_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateVirtual(x) {
    var ref_frame = document.getElementById("virtual_frame");
    ref_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + 0 * meter2pix + ')');
}
function translateRotationAxis(x, z) {
    var ref_frame = document.getElementById("inertialRotationAxis_frame");
    ref_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function plotFoil() {
    var chord = Math.sqrt(foilArea / foilAspectRatio);
    var foil = document.getElementById("foil");
    foil.setAttribute('x1', -2 / 3. * chord * meter2pix);
    foil.setAttribute('x2', 1 / 3 * chord * meter2pix);
}
function plotElevator() {
    var chord = Math.sqrt(elevatorArea / elevatorAspectRatio);
    var elevator = document.getElementById("elevator");
    elevator.setAttribute('x1', -2 / 3. * chord * meter2pix);
    elevator.setAttribute('x2', 1 / 3 * chord * meter2pix);
}
function plotShip() {
    var length = 15;
    var keel = document.getElementById("keel");
    keel.setAttribute('x1', 0 * meter2pix);
    keel.setAttribute('x2', length * meter2pix);
    var freeboard = document.getElementById("freeboard");
    freeboard.setAttribute('x1', 0 * meter2pix);
    freeboard.setAttribute('x2', length * meter2pix);
}
function plotTargetHeight() {
    var targetHeightLine = document.getElementById("target_height");
    targetHeightLine.setAttribute('y1', targetHeight*meter2pix);
    targetHeightLine.setAttribute('y2', targetHeight*meter2pix);
}
function plotWeight() {
    var arrow = document.getElementById("Fweight");
    arrow.setAttribute('d', "M" + 0 * meter2pix + " " + 0 * meter2pix+ " L"+ XYZ_wght_body_FSD.x * Newton2meter*meter2pix +" "+ XYZ_wght_body_FSD.z*Newton2meter*meter2pix);
}
function plotBuoyancy() {
    var arrow = document.getElementById("Fbuoyancy");
    arrow.setAttribute('d', "M" + 0 * meter2pix + " " + 0 * meter2pix+ " L"+ XYZ_buoyancy_body_FSD.x * Newton2meter*meter2pix +" "+ XYZ_buoyancy_body_FSD.z*Newton2meter*meter2pix);
}
function plotFfoil() {
    var arrow = document.getElementById("Ffoil");
    arrow.setAttribute('d', "M" + 0 * meter2pix + " " + 0 * meter2pix+ " L"+ XYZ_foil_body_FSD.x * Newton2meter*meter2pix +" "+ XYZ_foil_body_FSD.z*Newton2meter*meter2pix);
}
function plotFelevator() {
    var arrow = document.getElementById("Felevator");
    arrow.setAttribute('d', "M" + 0 * meter2pix + " " + 0 * meter2pix + " L" + XYZ_elev_body_FSD.x * Newton2meter*meter2pix +" "+ XYZ_elev_body_FSD.z*Newton2meter*meter2pix);
}
function plotFall() {
  var arrow = document.getElementById("Fall");
  arrow.setAttribute('d', "M" + 0*meter2pix + " " + 0 * meter2pix+ " L"+ XYZ_all_body_FSD.x*Newton2meter * meter2pix +" "+ XYZ_all_body_FSD.z*Newton2meter*meter2pix);
}
function updateView() {
    var mySelect = document.getElementById("viewSelect");
    var my2Dframe = document.getElementById("2DFrame");
    var my3Dframe = document.getElementById("3DFrame");

    switch( mySelect.value) {
        case "2DviewSelect":
            my2Dframe.style = "display:blocked;";
            my3Dframe.style = "display:none;";
            break
        case "3DviewSelect":
            my2Dframe.style = "display:none;";
            my3Dframe.style = "display:blocked;";
            break
        default:
            return
    }
}
function updateTargetHeight() {
        //get elements
        var myRange = document.getElementById("targetHeightRange");
        var myOutput = document.getElementById("targetHeight");
        targetHeight = -1.*myRange.value;
        myOutput.value = myRange.value;
}

function updateVirtualForceLongi() {
        //get elements
        var myRange = document.getElementById("virtualForceLongiRange");
        var myOutput = document.getElementById("virtualForceLongi");
        x_virtual = myRange.value;
        myOutput.value = myRange.value;
}
function updateAllowedElevatorRakeForControl() {
        //get elements
        var myRange = document.getElementById("allowedElevatorRakeForControlRange");
        var myOutput = document.getElementById("allowedElevatorRakeForControl");
        allowedElevatorRakeForControl = myRange.value*Math.PI/180;
        myOutput.value = myRange.value
}
function updateAllowedElevatorRakeForControlTimeConstant() {
        //get elements
        var myRange = document.getElementById("allowedElevatorRakeForControlTimeConstantRange");
        var myOutput = document.getElementById("allowedElevatorRakeForControlTimeConstant");
        allowedElevatorRakeForControlTimeConstant = myRange.value;
        myOutput.value = myRange.value;
}
function updatePitch() {
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
function updatePitchDynamic() {
    //get elements
    var myCheck = document.getElementById("pitchDynamicCheck");
    isPitchDynamic = myCheck.checked;
}
function updatePitchRate() {
    //get elements
    var myRange = document.getElementById("pitchRateRange");
    var myOutput = document.getElementById("pitchRate");
    var myCheck = document.getElementById("pitchDynamicCheck");
    isPitchDynamic = myCheck.checked;
    if (isPitchDynamic==true)
    {
        myRange.value = pqr_body_grnd_FSD.y*180/Math.PI;
    }
    else
    {
        pqr_body_grnd_FSD.y = myRange.value*Math.PI/180;
    }
    //copy the value over
        myOutput.value = myRange.value;
}
function updatePitchStiffness() {
    //get elements
    var myRange = document.getElementById("pitchStiffnessRange");
    var myOutput = document.getElementById("pitchStiffness");
    //copy the value over
    myOutput.value = myRange.value;
    pitchStiffness  = myRange.value; // rad/rad
}
function updateHeave() {
    //get elements
    var myRange = document.getElementById("heaveRange");
    var myOutput = document.getElementById("heave");
    var myCheck = document.getElementById("heaveDynamicCheck");
    isHeaveDynamic = myCheck.checked;
    if (isHeaveDynamic==true) {
        myRange.value = -xyz_body_grnd_NED.z;
    }
    else
    {
        xyz_body_grnd_NED.z = -myRange.value;
    }
    //copy the value over
    myOutput.value = myRange.value;
}
function updateHeaveDynamic() {
    //get elements
    var myCheck = document.getElementById("heaveDynamicCheck");
    isHeaveDynamic = myCheck.checked;
}
function updateHeaveRate() {
    //get elements
    var myRange = document.getElementById("heaveRateRange");
    var myOutput = document.getElementById("heaveRate");
    var myCheck = document.getElementById("heaveDynamicCheck");
    isHeaveDynamic = myCheck.checked;
    if (isHeaveDynamic==true) {
        myRange.value = -uvw_body_grnd_NED.z;
    }
    else
    {
        uvw_body_grnd_NED.z = -myRange.value;
    }
    //copy the value over
    myOutput.value = myRange.value;
}
function updateHeaveStiffness() {
    //get elements
    var myRange = document.getElementById("heaveStiffnessRange");
    var myOutput = document.getElementById("heaveStiffness");
    //copy the value over
    myOutput.value = myRange.value;
    heaveStiffness  = myRange.value * Math.PI / 180; //rad/m
}
function updateSurface() {
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
function updateElevatorRake() {
    //get elements
    var myRange = document.getElementById("elevatorRakeRange");
    var myOutput = document.getElementById("elevatorRake");
    //copy the value over
    myOutput.value = myRange.value;
    elevatorRake  = myRange.value * Math.PI / 180;
}
function updateFoilRake() {
    //get elements
    var myRange = document.getElementById("foilRakeRange");
    var myOutput = document.getElementById("foilRake");
    //copy the value over
    myOutput.value = myRange.value;
    foilRake  = myRange.value * Math.PI / 180;
}
function updateFoilRakeDelay() {
    //get elements
    var myRange = document.getElementById("foilRakeDelayRange");
    var myOutput = document.getElementById("foilRakeDelay");
    //copy the value over
    myOutput.value = myRange.value;
    foilRakeDelay  = myRange.value * 1;
}
function updateFoilRakeStep() {
    //get elements
    var myRange = document.getElementById("foilRakeStepRange");
    var myOutput = document.getElementById("foilRakeStep");
    var myTarget = document.getElementById("foilRakeRange");
    //copy the value over
    myOutput.value = myRange.value;
    foilRakeStep  = myRange.value * Math.PI / 180;
    myTarget.step = foilRakeStep * 180 / Math.PI;
}
function updateFlightSpeed() {
    //get elements
    var myRange = document.getElementById("flightSpeedRange");
    var myOutput = document.getElementById("flightSpeed");
    //copy the value over
    myOutput.value = myRange.value;
    V = 1 * myOutput.value * 1852 / 3600;
}
function updateFlightSpeedVariation() {
    //get elements
    var myRange = document.getElementById("flightSpeedVariationRange");
    var myOutput = document.getElementById("flightSpeedVariation");
    //copy the value over
    myOutput.value = myRange.value;
    flightSpeedVariation= 1 * myOutput.value * 1852 / 3600;
}
function updateMass() {
    //get elements
    var myRange = document.getElementById("massRange");
    var myOutput = document.getElementById("mass");
    //copy the value over
    myOutput.value = myRange.value;
    mass = myOutput.value;
}
function updateLongitudinalCenterOfInertiaPosition() {
    //get elements
    var myRange = document.getElementById("CGLongiRange");
    var myOutput = document.getElementById("CGLongi");
    //copy the value over
    myOutput.value = myRange.value;
    xyz_CoG_ref_FSD.x = myOutput.value * 1.;
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    var myRange = document.getElementById("bodyLongiRange");
    myRange.value = myOutput.value;
    updateBodyLongitudinalPosition();
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
    myRange.value = myOutput.value;
    updateBodyVerticalUpPosition();
}
function updateGravity() {
    //get elements
    var myCheck = document.getElementById("gravityCheck");
    g = earth_gravity*myCheck.checked;
}
function updatePitchInertia() {
    //get elements
    var myRange = document.getElementById("pitchInertiaRange");
    var myOutput = document.getElementById("pitchInertia");
    //copy the value over
    myOutput.value = myRange.value;
    pitchInertia = myOutput.value;
}
function updateElevatorArea() {
    //get elements
    var myRange = document.getElementById("elevatorAreaRange");
    var myOutput = document.getElementById("elevatorArea");
    //copy the value over
    myOutput.value = myRange.value;
    elevatorArea = myOutput.value;
}
function updateFoilArea() {
    //get elements
    var myRange = document.getElementById("foilAreaRange");
    var myOutput = document.getElementById("foilArea");
    //copy the value over
    myOutput.value = myRange.value;
    foilArea = myOutput.value;
}
function updateElevatorAspectRatio() {
    //get elements
    var myRange = document.getElementById("elevatorAspectRatioRange");
    var myOutput = document.getElementById("elevatorAspectRatio");
    //copy the value over
    myOutput.value = myRange.value;
    elevatorAspectRatio = myOutput.value;
}
function updateFoilAspectRatio() {
    //get elements
    var myRange = document.getElementById("foilAspectRatioRange");
    var myOutput = document.getElementById("foilAspectRatio");
    //copy the value over
    myOutput.value = myRange.value;
    foilAspectRatio = myOutput.value;
}
function updateStallAngle() {
    //get elements
    var myRange = document.getElementById("stallAngleRange");
    var myOutput = document.getElementById("stallAngle");
    //copy the value over
    myOutput.value = myRange.value;
    stallAngle = myOutput.value*Math.PI/180;
}
function updateStallRecoveryAngle() {
    //get elements
    var myRange = document.getElementById("stallRecoveryAngleRange");
    var myOutput = document.getElementById("stallRecoveryAngle");
    //copy the value over
    myOutput.value = myRange.value;
    stallRecoveryAngle = myOutput.value*Math.PI/180;
}
function updateElevatorLongitudinalPosition() {
    //get elements
    var myRange = document.getElementById("elevatorLongiRange");
    var myOutput = document.getElementById("elevatorLongi");
    //copy the value over
    myOutput.value = myRange.value;
    xyz_elev_ref_FSD.x = myOutput.value * 1.;
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateFoilLongitudinalPosition() {
    //get elements
    var myRange = document.getElementById("foilLongiRange");
    var myOutput = document.getElementById("foilLongi");
    //copy the value over
    myOutput.value = myRange.value;
    xyz_foil_ref_FSD.x = myOutput.value * 1.;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateElevatorVerticalUpPosition() {
    //get elements
    var myRange = document.getElementById("elevatorVertUpRange");
    var myOutput = document.getElementById("elevatorVertUp");
    //copy the value over
    myOutput.value = myRange.value;
    xyz_elev_ref_FSD.z = -myOutput.value * 1.;
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateFoilVerticalUpPosition() {
    //get elements
    var myRange = document.getElementById("foilVertUpRange");
    var myOutput = document.getElementById("foilVertUp");
    //copy the value over
    myOutput.value = myRange.value;
    xyz_foil_ref_FSD.z = -myOutput.value;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
}
function updateBodyLongitudinalPosition() {
    //get elements
    var myRange = document.getElementById("bodyLongiRange");
    var myOutput = document.getElementById("bodyLongi");
    //copy the value over
    myOutput.value = myRange.value;
    var xyz_body_ref_FSD_old = new THREE.Vector3( 0, 0, 0 );
    xyz_body_ref_FSD_old = xyz_body_ref_FSD.clone();
    xyz_body_ref_FSD.x = myOutput.value * 1.;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    var CTM = new THREE.Matrix4;
    var invCTM = new THREE.Matrix4;
    var rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
    xyz_body_grnd_NED.add((xyz_body_ref_FSD.clone().
        sub(xyz_body_ref_FSD_old)).applyMatrix4(invCTM));
    var myRange = document.getElementById("heaveRange");
    var myOutput = document.getElementById("heave");
    myRange.value = -xyz_body_grnd_NED.z;
    myOutput.value = -xyz_body_grnd_NED.z;
}
function updateBodyVerticalUpPosition() {
    //get elements
    var myRange = document.getElementById("bodyVertUpRange");
    var myOutput = document.getElementById("bodyVertUp");
    //copy the value over
    myOutput.value = myRange.value * 1.;
    var xyz_body_ref_FSD_old = new THREE.Vector3( 0, 0, 0 );
    xyz_body_ref_FSD_old=xyz_body_ref_FSD.clone()
    xyz_body_ref_FSD.z = -myOutput.value;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    
    var CTM = new THREE.Matrix4;
    var invCTM = new THREE.Matrix4;
    var rpy = new THREE.Euler( 0, -pitch, 0, 'XYZ' );
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
    xyz_body_grnd_NED.add((xyz_body_ref_FSD.clone().
        sub(xyz_body_ref_FSD_old)).applyMatrix4(invCTM));
    var myRange = document.getElementById("heaveRange");
    var myOutput = document.getElementById("heave");
    myRange.value = -xyz_body_grnd_NED.z;
    myOutput.value = -xyz_body_grnd_NED.z;
}
function updateOutputLongitudinalPosition() {
    var myRange = document.getElementById("outputLongiRange");
    var myOutput = document.getElementById("outputLongi");
    myOutput.value = myRange.value;
    xyz_output_ref_FSD.x = myRange.value*1.0;
}
function updateOutputVerticalUpPosition() {
    var myRange = document.getElementById("outputVertUpRange");
    var myOutput = document.getElementById("outputVertUp");
    myOutput.value = myRange.value;
    xyz_output_ref_FSD.z = -myRange.value*1.0;
}
    
function updateOutput() {
    var foil = document.getElementById("foilRakeDisplay");
    var elev = document.getElementById("elevatorRakeDisplay");
    var stepGainDisp = document.getElementById("stepGainDisplay");
    foil.innerHTML = (foilRake*180/Math.PI).toFixed(1) + "°";
    foil.style.fontSize= "40px"
    foil.style.color="red";
    elev.innerHTML = (APRake*180/Math.PI).toFixed(1) + "°";
    elev.style.fontSize= "40px"
    elev.style.color="red";
    stepGainDisp.innerHTML = stepGain;
}
function updateFluid() {
    var mySelect = document.getElementById("fluidSelect");
    if (mySelect.value == "Air") {
        density = 1.225;
    }
    else if (mySelect.value == "Water") {
        density = 1025;
    }
}

function computeBarycenter3(P1,P2,P3) {
  P = P1.clone().add(P2).add(P3).multiplyScalar(1/3.);
  return P;
}
function computeBuoyancy() {  
    var volume = 0;
    var force = 0;
    var CTM    = new THREE.Matrix4,
        invCTM = new THREE.Matrix4,
        rpy    = new THREE.Euler( 0, -pitch, 0, 'XYZ' ),
        tmp    = new THREE.Vector3( 0, 0, 0 ),
        tmp1   = new THREE.Vector3( 0, 0, 0 ),
        tmp2   = new THREE.Vector3( 0, 0, 0 ),
        xyz_ref_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        xyz_hAB_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        xyz_hFB_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        xyz_hAT_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        xyz_hFT_grnd_NED = new THREE.Vector3( 0, 0, 0 ),
        centroid = {x: 0, y:0};
    
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
    
    // Compute the position of the 4 points describing the boat in ground frame
    xyz_ref_grnd_NED = xyz_body_ref_FSD.clone().multiplyScalar(-1).
        applyMatrix4(invCTM).add(xyz_body_grnd_NED);
    xyz_hAB_grnd_NED = xyz_hAB_ref_FSD.clone().
        applyMatrix4(invCTM).add(xyz_ref_grnd_NED);
    xyz_hFB_grnd_NED = xyz_hFB_ref_FSD.clone().
        applyMatrix4(invCTM).add(xyz_ref_grnd_NED);
    xyz_hAT_grnd_NED = xyz_hAT_ref_FSD.clone().
        applyMatrix4(invCTM).add(xyz_ref_grnd_NED);
    xyz_hFT_grnd_NED = xyz_hFT_ref_FSD.clone().
        applyMatrix4(invCTM).add(xyz_ref_grnd_NED);
    
    // Create a polygon which represents the hull
    var hullPolygon = CSG.fromPolygons([[
            [xyz_hAB_grnd_NED.x, xyz_hAB_grnd_NED.z],
            [xyz_hAT_grnd_NED.x, xyz_hAT_grnd_NED.z],
            [xyz_hFT_grnd_NED.x, xyz_hFT_grnd_NED.z],
            [xyz_hFB_grnd_NED.x, xyz_hFB_grnd_NED.z]]]);
            
    // Create the polygon which represents the sea
    var waterPolygon = CSG.fromPolygons([[
            [-1e20, 0],
            [ 1e20, 0],
            [ 1e20, 1000],
            [-1e20, 1000]]]);
            
    // Compute the immersed part of hull
    var immersedHullPolygons = hullPolygon.intersect(waterPolygon).toPolygons();
    if (immersedHullPolygons.length>0) {
        volume = polyarea(immersedHullPolygons[0]);
        centroid = polycentroid(immersedHullPolygons[0]);
    }
    else {
        // volume is zero and centroid undefined, but take the most down point to ensure continuity
        volume = 0;
        var items = [
        {   x: xyz_hAB_grnd_NED.x, y : xyz_hAB_grnd_NED.z},
        {   x: xyz_hAT_grnd_NED.x, y : xyz_hAT_grnd_NED.z},
        {   x: xyz_hFT_grnd_NED.x, y : xyz_hFT_grnd_NED.z},
        {   x: xyz_hFB_grnd_NED.x, y : xyz_hFB_grnd_NED.z},
        ];
        items.sort(function (a, b) {
            return a.y - b.y;
        });
        centroid = items[3];
    }
    
    xyz_buoyancy_grnd_NED.x = centroid.x;
    xyz_buoyancy_grnd_NED.z = centroid.y;
    force = mass * g * volume / (Lpp * initialDraft) * isBuoyancy;
    XYZ_buoyancy_body_NED.z = -force;
    XYZ_buoyancy_body_FSD = XYZ_buoyancy_body_NED.clone().applyMatrix4(CTM);
    xyz_buoyancy_body_FSD = xyz_buoyancy_grnd_NED.clone().
        sub(xyz_body_grnd_NED).applyMatrix4(CTM);
    xyz_buoyancy_ref_FSD = xyz_buoyancy_body_FSD.clone().
        add(xyz_body_ref_FSD);
    KMN_buoyancy_body_FSD = tmp.clone().
        crossVectors(xyz_buoyancy_body_FSD, XYZ_buoyancy_body_FSD);
}
function localStore() {
    // use this to be able to send data to the 3D view in another page
    localStorage.setItem('x', xyz_output_grnd_NED.x);
    localStorage.setItem('y', xyz_output_grnd_NED.y);
    localStorage.setItem('z', xyz_output_grnd_NED.z);
    localStorage.setItem('roll', 0);
    localStorage.setItem('pitch', pitch);
    localStorage.setItem('yaw', 0);
    localStorage.setItem('t', simulation_time);
} 
function polyarea(vertices) {
    // Computes the area of a polygon
    // http://stackoverflow.com/questions/16285134/calculating-polygon-area
    var area = 0;
    var j = vertices.length-1;
    for (var i = 0, l = vertices.length; i < l; i++) {
            area = area +  (vertices[j].x + vertices[i].x) *
                (vertices[i].y-vertices[j].y);
            j = i;
    }
    return area / 2;
}
function polycentroid(vertices) {
    // Computes the centroid of a polygon
    // https://en.wikipedia.org/wiki/Centroid
    
    var area = polyarea(vertices);
    var totalx = 0;
    var totaly = 0;
    var j = vertices.length-1;
    for (var i = 0, l = vertices.length; i < l; i++) {
        totalx = totalx +  (vertices[j].x + vertices[i].x) *
            (vertices[j].x*vertices[i].y-vertices[i].x*vertices[j].y);
        totaly = totaly +  (vertices[j].y + vertices[i].y) *
            (vertices[j].x*vertices[i].y-vertices[i].x*vertices[j].y);   
        j = i;
    }
    var centroid = {x: 1 / (6 * area) * totalx, y: 1 / (6 * area) * totaly};
    return centroid;
}
function liftCoefficient(alpha, AR, CLs, alphas_deg) {
    // This is a simplified formula for lift coefficient
    // Maximum lift at 45°
    // No lift at 90°
    // Negative lift from 90°
    // http://people.clarkson.edu/~pmarzocc/AE429/AE-429-4.pdf
    var dCl= 2*Math.PI,//infinite elliptic wing for small angle
        e  = 1,// Oswald efficiency factor
        Cl,
        dCL;
    // AR= span^2/kite_surface;
    // alphai= Cl/(Math.PI*e*AR);
    // dCL = dCl*AR/(AR+2.5)
    dCL = dCl / (1 + dCl / (Math.PI * e * AR));
    Cl = dCL /2. * Math.sin (2 * alpha);
    if (CLs.length>0) {
        Cl = everpolate.linear(alpha * 180 / Math.PI, alphas_deg, CLs);
    }
    return Cl;
 }
function dragCoefficient(alpha, AR, CDs, alphas_deg) {
    var Cd0 = 0.1,
        e = 1,
        Cd,
        inducedDragCoefficient;
        
    inducedDragCoefficient = 
        Math.pow(2 * Math.PI * Math.sin(alpha), 2) / (Math.PI * AR * e);
    Cd = inducedDragCoefficient + Cd0;
    if (CDs.length>0) {
        Cd = everpolate.linear(alpha * 180 / Math.PI, alphas_deg, CDs);
    }
    return Cd;
}
function simple_moving_averager(period) {
    var nums = [];
    return function(num) {
        nums.push(num);
        if (nums.length > period)
            nums.splice(0,1);  // remove the first element of the array
        var sum = 0;
        for (var i in nums)
            sum += nums[i];
        var n = period;
        if (nums.length < period)
            n = nums.length;
        return(sum/n);
    }
}