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

function RR_Fwd(c:number, b:number, PHI:number, PSI:number, PHI_d:number, PSI_d:number, PHI_dd:number, PSI_dd:number):number[] {
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

function RR_Inv(b:number, c:number, Xa:number, Ya:number, Xq:number, Yq:number, X_d:number, Y_d:number, X_dd:number, Y_dd:number):number[]{

	//INVERSE KINEMATICS (POSITION CALCULATION)
	let X = Xa - Xq
       	let Y = Ya - Yq

	let A:number = X**2 + Y**2 + c**2 - b**2 + 2*c*X
	let B:number = - 2*c*Y
	let C:number = X**2 + Y**2 + c**2 - b**2 - 2*c*X
	
	let t1 = (- B + Math.sqrt(B**2 - A*C))/A
	let t2 = (- B - Math.sqrt(B**2 - A*C))/A

	let PHI_1 = 2*Math.atan(t1)
	let PHI_2 = 2*Math.atan(t2)

	let U1:number = (X - c*Math.cos(PHI_1))/b
	let V1:number = (Y - c*Math.sin(PHI_1))/b	

	let U2:number = (X - c*Math.cos(PHI_2))/b
	let V2:number = (Y - c*Math.sin(PHI_2))/b	

	let PSI_1:number = Math.atan2(V1,U2)
	let PSI_2:number = Math.atan2(V2,U2)
	
	// INVERSE KINEMATICS (VELOCITY CALCULATION)

	return(
	[
		PHI_1,
		PHI_2,
		PSI_1,
		PSI_2
	]
	)
}



console.log(RR_Fwd(c,b,PHI,PSI,PHI_d,PSI_d,PHI_dd,PSI_dd))
