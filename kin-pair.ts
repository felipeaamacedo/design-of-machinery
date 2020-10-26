//This is a code that has the function to analyse the main types of kinematics pairs.
//

/* --- INPUT PARAMETERS --- */
let c:number = 3;
let b:number = 4;

let PHI:number = 30/180*Math.PI;
let PSI:number = 45/180*Math.PI;

let PHI_d:number = 4;
let PSI_d:number = 2;

let PHI_dd:number = 10;
let PSI_dd:number = 0;

/* --- CALCULATION --- */

function RR_Dir(c:number, b:number, PHI:number, PSI:number, PHI_d:number, PSI_d:number, PHI_dd:number, PSI_dd:number):number[] {
	let X = c*Math.cos(PHI) + b*Math.cos(PSI)	
	let Y = c*Math.sin(PHI) + b*Math.sin(PSI)

	let X_d = -c*PHI_d*Math.sin(PHI) - b*PSI_d*Math.sin(PSI)
	let Y_d = c*PHI_d*Math.cos(PHI) + b*PSI_d*Math.cos(PSI)

	let X_dd = -c*PHI_dd*Math.sin(PHI) - b*PSI_dd*Math.sin(PSI) - c*Math.cos(PHI)*PHI_d**2 - b*Math.cos(PSI)*PSI_d**2
	let Y_dd = c*PHI_dd*Math.cos(PHI) + b*PSI_dd*Math.cos(PSI) - c*Math.sin(PHI)*PHI_d**2 - b*Math.sin(PSI)*PSI_d**2

	return (
		[
	X,
	Y,
	X_d,
	Y_d,
	X_dd,
	Y_dd
		]
	)
}


console.log(RR_Dir(c,b,PHI,PSI,PHI_d,PSI_d,PHI_dd,PSI_dd))
