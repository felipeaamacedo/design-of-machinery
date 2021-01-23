import matrixts from "@felipeaamacedo/matrix-ts"

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

function RR_Inv(b:number, c:number, Xa:number, Ya:number, Xq:number, Yq:number, X_d:number, Y_d:number, X_dd:number, Y_dd:number, I:number):number[]{
	//INVERSE KINEMATICS (POSITION CALCULATION)
	let X = Xa - Xq
       	let Y = Ya - Yq

	let A:number = X**2 + Y**2 + c**2 - b**2 + 2*c*X
	let B:number = - 2*c*Y
	let C:number = X**2 + Y**2 + c**2 - b**2 - 2*c*X
	
	let t1 = (- B + Math.sqrt(B**2 - A*C))/A
	let t2 = (- B - Math.sqrt(B**2 - A*C))/A

	if(I==1){
		let t = t1
	}else{
		let t = t2
	}

	let PHI = 2*Math.atan(t1)

	let U:number = (X - c*Math.cos(PHI))/b
	let V:number = (Y - c*Math.sin(PHI))/b	

	let PSI:number = Math.atan2(V,U)
	
	// INVERSE KINEMATICS (VELOCITY CALCULATION)
	
	let D:matrixts.Matrix = new matrixts.Matrix(2,2)
    D.data = [
                [- c* Math.sin(PHI),	- b*Math.sin(PSI)],
			    [c*Math.cos(PHI), 		b*Math.cos(PSI)]
            ]	

	let E:matrixts.Matrix = new matrixts.Matrix(2,1)
    
	E.data = [
			[X_d],
			[Y_d]
	    ]

	let E_inv:matrixts.Matrix = matrixts.inv(E)
	let F:matrixts.Matrix = matrixts.multiply(D,E_inv) 

    
	let PHI_d:number = F.data[0][0]
	let PSI_d:number = F.data[1][0]

	//INVERSE KINEMATICS (ACCELERATION CALCULATION)
	let G:matrixts.Matrix = new matrixts.Matrix(2,2)
	G.data = [
			[-c*Math.sin(PHI), -b*Math.sin(PSI)], 
			[ c*Math.cos(PHI),  b*Math.cos(PSI)]
		]

	let H:matrixts.Matrix = new matrixts.Matrix(2,1)
	H.data = [
			[X_dd + c*Math.cos(PHI)*PHI_d**2 + b*Math.cos(PSI)*PSI_d**2],
			[Y_dd + c*Math.sin(PHI)*PHI_d**2 + b*Math.sin(PSI)*PSI_d**2]

		]

	let H_inv:matrixts.Matrix = matrixts.inv(H)
	let L:matrixts.Matrix = matrixts.multiply(G,H_inv)

	PHI_dd = L.data[0][0]
	PSI_dd = L.data[1][0]


	return [PHI, PSI, PHI_d, PSI_d, PHI_dd, PSI_dd]
}

function RP_Fwd(a:number, Xq:number, Yq:number, r:number, PHI:number, r_d:number, PHI_d:number, r_dd:number, PHI_dd:number):number[]{
	
	//FORWARD KINEMATICS (POSITION CALCULATION)
	let X:number = -a*Math.sin(PHI) + r*Math.cos(PHI)
	let Y:number = r*Math.sin(PHI) + a*Math.cos(PHI)
	
	let Xa:number = Xq + X
	let Ya:number = Yq + Y

	// FORWARD KINEMATICS (VELOCITY CALCULATION)
	let Xa_d:number = ( - a*Math.cos(PHI) - r*Math.sin(PHI))*PHI_d + Math.cos(PHI)*r_d
	let Ya_d:number = (r*Math.cos(PHI) - a*Math.sin(PHI))*PHI_d + Math.sin(PHI)*r_d

	// FORWARD KINEMATICS (ACCELERATION CALCULATION)
	let Xa_dd:number = ( - a*Math.cos(PHI) - r*Math.sin(PHI))*PHI_dd + Math.cos(PHI)*r_dd + (a*PHI_d*Math.sin(PHI) - r*PHI_d*Math.cos(PHI) - r_d*Math.sin(PHI)) * PHI_d + ( - PHI_d*Math.sin(PHI))*r_d
	let Ya_dd:number = (r*Math.cos(PHI) - a*Math.sin(PHI))*PHI_dd + Math.sin(PHI)*r_dd + (- a*PHI_d*Math.cos(PHI) - r*PHI_d*Math.sin(PHI) + r_d*Math.cos(PHI))*PHI_d + (PHI_d*Math.cos(PHI))*r_d

	return [Xa, Ya, Xa_d, Ya_d, Xa_dd, Ya_dd]
}

console.log(RR_Fwd(c,b,PHI,PSI,PHI_d,PSI_d,PHI_dd,PSI_dd))
