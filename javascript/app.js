//"use strict"
console.log("This is a foil simulator");
var g_CLs = [],
    g_CDs = [],
    g_alphas_deg = [];
function liftCoefficient(alpha, AR, CLs, alphas_deg) {
    // This is a simplified formula for lift coefficient
    // Maximum lift at 45°
    // No lift at 90°
    // Negative lift from 90°
    // http://people.clarkson.edu/~pmarzocc/AE429/AE-429-4.pdf
    var dCl= 2*Math.PI,//infinite elliptic wing for small angle
        e  = 1,// Oswald efficiency factor
        Cl;
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
    initialDraft        = 0.25,
    Lpp                 = 13,

    isFoilStall         = false,
    isElevStall         = false;

// Simulation parameter
var sampleTime          = 0.0005, // Sample time
    isHeaveDynamic      = true,
    isPitchDyanmic      = true,
    isBuoyancy          = true,
    isSurfaceEffect     = true,
    isSurface           = true,
    foilRakeDelay       = 0,
    foilRakeStep        = 0.2 * Math.PI / 180.,
    elevatorRakeStep    = 0.1 * Math.PI / 180.,
    targetHeight        = 1;

    // Plot parameter
var meter2pix           = 50,
    Newton2meter        = 0.00001;

// Initial condition
var V                    = 10,
    pqr0                 = 0,    // Angular rate
    pitch                = 2.2 * Math.PI / 180,
    rakeMeanPower        = 0,
    flightSpeedVariation = 0;

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
    uvw_elev_grnd_NED   = new THREE.Vector3( 0, 0, 0 );

// Hull extremity for archimedian thrust
var xyz_hAB_ref_FSD = new THREE.Vector3( 0, 0, initialDraft),   // Hull Aft Bottom
    xyz_hFB_ref_FSD = new THREE.Vector3( Lpp, 0, initialDraft), // Hull fore Bottom
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
    foilRake      = 0;
    
pqr_body_grnd_FSD.x = pqr0;

document.getElementById("targetHeightRange")        .
    addEventListener("change", updateTargetHeight);
document.getElementById("pitchRange")               .
    addEventListener("change", updatePitch);
document.getElementById("pitchDynamicCheck")        .
    addEventListener("change", updatePitchDynamic);
document.getElementById("heaveRange")               .
    addEventListener("change", updateHeave);
document.getElementById("heaveDynamicCheck")        .
    addEventListener("change", updateHeaveDynamic);
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
document.getElementById("elevatorLongiRange")       .
    addEventListener("change", updateElevatorLongitudinalPosition);
document.getElementById("foilLongiRange")           .
    addEventListener("change", updateFoilLongitudinalPosition);
document.getElementById("elevatorVertUpRange")      .
    addEventListener("change", updateElevatorVerticalUpPosition);
document.getElementById("foilVertUpRange")          .
    addEventListener("change", updateFoilVerticalUpPosition);
//document.getElementById("bodyLongiRange")           . 
    //addEventListener("change", updateBodyLongitudinalPosition);
//document.getElementById("bodyVertUpRange")          .
    //addEventListener("change", updateBodyVerticalUpPosition);
document.getElementById("fluidSelect")              .
    addEventListener("change", updateFluid);

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
    if (event.defaultPrevented) {
        return; // Should do nothing if the key event was already consumed.
    }
    switch (event.key) {
        case "ArrowUp":
            foilRake = foilRake + foilRakeStep;
            break;
        case "ArrowDown":
            foilRake = foilRake - foilRakeStep;
            break;
        case "ArrowRight":
            elevatorRake = elevatorRake + elevatorRakeStep;
            break;
        case "ArrowLeft":
            elevatorRake = elevatorRake - elevatorRakeStep;
            break;
        default:
            return; // Quit when this doesn't handle the key event.
    }
    var myRange = document.getElementById("foilRakeRange");
    var myOutput = document.getElementById("foilRake");
    //copy the value over
    myOutput.value = Math.round(foilRake * 180 / Math.PI * 100) / 100;
    myRange.value = myOutput.value;
    
    var myRange = document.getElementById("elevatorRakeRange");
    var myOutput = document.getElementById("elevatorRake");
    //copy the value over
    myOutput.value = Math.round(elevatorRake * 180 / Math.PI * 100) / 100;
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
function saveRake() {
    i_rakeBuffer = i_rakeBuffer + 1
    if (i_rakeBuffer>20) {i_rakeBuffer = 0;}
    rake[i_rakeBuffer] = foilRake;
}
function getRakeDelayed(delay) {
    i_delay = i_rakeBuffer-Math.round(delay/0.1)
    if (i_delay<0){i_delay = i_delay + 20}
    return rake[i_delay]
}

function init() {
    // Do init to be sure values are the same as described in html page
    updateTargetHeight();
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
    updateElevatorLongitudinalPosition();
    updateFoilLongitudinalPosition();
    updateElevatorVerticalUpPosition();
    updateFoilVerticalUpPosition();
    updateBodyLongitudinalPosition();
    updateBodyVerticalUpPosition();
    updateFluid();
}
init();
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
    rotateFoil(foil_rake);
    rotateElevator(elevator_rake); 

    // For 3D view in other tab
    localStore()
}
function updatePlot() {
    plot(xyz_body_grnd_NED, xyz_foil_body_FSD, xyz_elev_body_FSD,
        foilRake, elevatorRake, pitch);
    updateOutput();
}
function updaten() {
    for (var i = 0;i<8;i++) { 
    // Dirty manual tuning to get close of real time on my computer
        update();
    }
}

function computeForcesOnLiftingSurface(CTM, pitch, uvw_fluid_grnd_NED,
    uvw_surf_grnd_NED, xyz_surf_grnd_NED, xyz_surf_body_FSD, AoK, density, chord, z_waterSurface,
    isSurfaceEffect, isSurface, isSurfStall, surfArea, surfAspectRatio) {
        
    var angle_uvw_surface_fluid_NED, q, AoA, chord, inverseMirrorEffect,
        dragSurfaceEffect, stallEffect;
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
    if (Math.abs(AoA)>10 * Math.PI / 180) {
        isSurfStall = true;
    }
    if (Math.abs(AoA)<5 * Math.PI / 180) { 
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
        tmp    = new THREE.Vector3( 0, 0, 0 );
      
    var z_waterSurface = 0;
      
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
    AoK = getRakeDelayed(foilRakeDelay);
    xyz_foil_grnd_NED = xyz_foil_body_FSD.clone().
        applyMatrix4(invCTM).add(xyz_body_grnd_NED);
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
    
}
function update() {
    computeForces();
    dt = sampleTime;// +0*dt;
    simulation_time = simulation_time + dt;
    inv_mass = 1. / mass;
    uvw_body_grnd_NED.add(XYZ_all_body_NED.clone().multiplyScalar(inv_mass*dt))
    if (false==isHeaveDynamic) {
        uvw_body_grnd_NED.z = 0
    }
    uvw_body_grnd_NED.x = V +
        flightSpeedVariation * Math.sin(2 * Math.PI / 30. * simulation_time)
    //console.log(uvw_body_grnd_NED)
    xyz_body_grnd_NED.add(uvw_body_grnd_NED.clone().multiplyScalar(dt))
    
    pqr_body_grnd_FSD.add(KMN_all_body_FSD.clone().
        multiplyScalar(1 / pitchInertia * dt));
    if (false==isPitchDynamic) {
        pqr_body_grnd_FSD.y = 0;
    }
    pitch = pitch + pqr_body_grnd_FSD.y * dt;
    
    //console.log(xyz_body_grnd_NED)
    updateHeave();
    updatePitch();
    
}
function rotateFoil(r) {
    var foilFrame = document.getElementById("foil");
    r_deg = r * 180 / Math.PI;
    foilFrame.setAttribute('transform', 'rotate(' +-r_deg + ')');
}
function rotateElevator(r) {
    var elevatorFrame = document.getElementById("elevator");
    r_deg = r * 180 / Math.PI;
    elevatorFrame.setAttribute('transform', 'rotate(' +-r_deg + ')');
}
function rotateBody(r) {
    var bodyFrame = document.getElementById("body__rotate_frame");
    r_deg = r * 180 / Math.PI;
    bodyFrame.setAttribute('transform', 'rotate(' +-r_deg + ')');
}
function translateFoil(x, z) {
    var foil_frame = document.getElementById("foil_frame");
    foil_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateElevator(x, z) {
    var elevator_frame = document.getElementById("elevator_frame");
    elevator_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateCenterOfInertia(x, z) {
    var elevator_frame = document.getElementById("CoG_frame");
    elevator_frame.setAttribute('transform', 'translate(' +x * meter2pix + ',' + z * meter2pix + ')');
}
function translateBuoyancy(x, z) {
    var B_frame = document.getElementById("B_frame");
    B_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateBody(x, z) {
    var body_frame = document.getElementById("body_frame");
    body_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}
function translateRef(x, z) {
    var ref_frame = document.getElementById("ref_frame");
    ref_frame.setAttribute('transform', 'translate(' + x * meter2pix + ',' + z * meter2pix + ')');
}

function plotFoil() {
    var chord = Math.sqrt(foilArea / foilAspectRatio);
    var foil = document.getElementById("foil");
    foil.setAttribute('x1', -2 / 3. * chord * meter2pix);
    foil.setAttribute('x2', 1 / 3 * chord * meter2pix);
}
function plotElevator() {
    var chord = Math.sqrt(elevatorArea/elevatorAspectRatio);
    var elevator = document.getElementById("elevator");
    elevator.setAttribute('x1', -2 / 3. * chord * meter2pix);
    elevator.setAttribute('x2', 1 / 3 * chord * meter2pix);
}
function plotShip() {
    var length = 13;
    var elevator = document.getElementById("keel");
    elevator.setAttribute('x1', 0 * meter2pix);
    elevator.setAttribute('x2', length * meter2pix);
}
function plotTargetHeight() {
    var targetHeightLine = document.getElementById("target_height");
    targetHeightLine.setAttribute('y1', -targetHeight*meter2pix);
    targetHeightLine.setAttribute('y2', -targetHeight*meter2pix);
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
function updateTargetHeight() {
        //get elements
        var myRange = document.getElementById("targetHeightRange");
        var myOutput = document.getElementById("targetHeight");
        targetHeight = myRange.value;
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
    xyz_body_ref_FSD_old = new THREE.Vector3( 0, 0, 0 );
    xyz_body_ref_FSD_old = xyz_body_ref_FSD.clone();
    xyz_body_ref_FSD.x = myOutput.value * 1.;
    xyz_foil_body_FSD = xyz_foil_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_elev_body_FSD = xyz_elev_ref_FSD.clone().sub(xyz_body_ref_FSD);
    xyz_CoG_body_FSD = xyz_CoG_ref_FSD.clone().sub(xyz_body_ref_FSD);
    CTM = new THREE.Matrix4;
    invCTM = new THREE.Matrix4;
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
function updateBodyVerticalUpPosition(){
    //get elements
    var myRange = document.getElementById("bodyVertUpRange");
    var myOutput = document.getElementById("bodyVertUp");
    //copy the value over
    myOutput.value = myRange.value * 1.;
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
    xyz_body_grnd_NED.add((xyz_body_ref_FSD.clone().
        sub(xyz_body_ref_FSD_old)).applyMatrix4(invCTM));
    var myRange = document.getElementById("heaveRange");
    var myOutput = document.getElementById("heave");
    myRange.value = -xyz_body_grnd_NED.z;
    myOutput.value = -xyz_body_grnd_NED.z;
    
}
function updateOutput() {
    
    myOutput = document.getElementById("time");
    myOutput.value = Math.round(simulation_time * 100) / 100;
    
    myOutput = document.getElementById("instantFlightSpeed");
    myOutput.value = Math.round(uvw_body_grnd_NED.x * 3600/1852 * 10) / 10;
    
    myOutput = document.getElementById("rakeMeanPower");
    myOutput.value = Math.round(rakeMeanPower * 10) / 10;
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
    var CTM    = new THREE.Matrix4,
        invCTM = new THREE.Matrix4,
        rpy    = new THREE.Euler( 0, -pitch, 0, 'XYZ' ),
        tmp    = new THREE.Vector3( 0, 0, 0 ),
        tmp1   = new THREE.Vector3( 0, 0, 0 ),
        tmp2   = new THREE.Vector3( 0, 0, 0 );
    
    CTM.makeRotationFromEuler(rpy);
    invCTM.getInverse(CTM);
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
    
    var hullPolygon = CSG.fromPolygons([[
            [xyz_hAB_grnd_NED.x, xyz_hAB_grnd_NED.z],
            [xyz_hAT_grnd_NED.x, xyz_hAT_grnd_NED.z],
            [xyz_hFT_grnd_NED.x, xyz_hFT_grnd_NED.z],
            [xyz_hFB_grnd_NED.x, xyz_hFB_grnd_NED.z]]]);
    var waterPolygon = CSG.fromPolygons([[
            [-1e20, 0],
            [ 1e20, 0],
            [ 1e20, 1000],
            [-1e20, 1000]]]);
    var immersedHullPolygons = hullPolygon.intersect(waterPolygon).toPolygons();
    if (immersedHullPolygons.length>0) {
        volume = polyarea(immersedHullPolygons[0])
        centroid = polycentroid(immersedHullPolygons[0])
    }
    else { 
        volume = 0;
        centroid = {x: 0, y: 0};
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

// Deals with 3D view
//var w2 = window.open("https://rawgit.com/baptistelabat/visu3D/master/visu3D.html")
//var w2 = window.open("C:/Users/labat/perso/visu3D/visu3D.html")

function localStore() {
    localStorage.setItem('x', xyz_body_grnd_NED.x);
    localStorage.setItem('y', xyz_body_grnd_NED.y);
    localStorage.setItem('z', xyz_body_grnd_NED.z);
    localStorage.setItem('pitch', pitch);
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