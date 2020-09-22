/// <reference types="./RotationalMotorEquation.ts" />
/// <reference lib="dom" />
import { Equation } from "../equations/Equation.js";
import { Vec3 } from "../math/Vec3.js";
export class RotationalMotorEquation extends Equation {
    constructor(bodyA, bodyB, maxForce = 1e6) {
        super(bodyA, bodyB, -maxForce, maxForce);
        this.axisA = new Vec3();
        this.axisB = new Vec3();
        this.targetVelocity = 0;
    }
    computeB(h) {
        const a = this.a;
        const b = this.b;
        const bi = this.bi;
        const bj = this.bj;
        const axisA = this.axisA;
        const axisB = this.axisB;
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        GA.rotational.copy(axisA);
        axisB.negate(GB.rotational);
        const GW = this.computeGW() - this.targetVelocity;
        const GiMf = this.computeGiMf();
        const B = -GW * b - h * GiMf;
        return B;
    }
}
