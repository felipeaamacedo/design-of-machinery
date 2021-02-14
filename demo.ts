import { RR_Fwd } from './src/kin-pair'
import { RR_Inv } from './src/kin-pair'


/* --- INPUT PARAMETERS --- */
let O2A:number = 75;
let AB:number = 195;
let BO4:number = 170

let O2:number[] = [0,0]
let O4:number[] = [150, 0]

let theta2:number = 60/180*Math.PI


let kp1 = RR_Fwd(0, O2A, 0, theta2, 0, 0, 0, 0)
let kp2 = RR_Inv(195, 170, 130, 0, kp1[0], kp1[1], kp1[2], kp1[3], kp1[4], kp1[5], 0 )
let kp2Space = RR_Fwd(195, 170, kp2[0], kp2[1], kp2[2], kp2[3], kp2[4], kp2[5])


let jointA:number[] = [kp1[0], kp1[1]]
let jointO4:number[] = O4
let jointB:number[] = [kp2Space[0], kp2Space[1]]


console.log(kp1) 
