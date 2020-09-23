/// <reference types="./ConeEquation.ts" />
/// <reference lib="dom" />
import { Vec3 } from "../math/Vec3.js";
import { Equation } from "../equations/Equation.js";
export class ConeEquation extends Equation {
    constructor(bodyA, bodyB, options = {}) {
        const maxForce = typeof options.maxForce !== "undefined"
            ? options.maxForce
            : 1e6;
        super(bodyA, bodyB, -maxForce, maxForce);
        this.axisA = options.axisA ? options.axisA.clone() : new Vec3(1, 0, 0);
        this.axisB = options.axisB ? options.axisB.clone() : new Vec3(0, 1, 0);
        this.angle = typeof options.angle !== "undefined" ? options.angle : 0;
    }
    computeB(h) {
        const a = this.a;
        const b = this.b;
        const ni = this.axisA;
        const nj = this.axisB;
        const nixnj = tmpVec1;
        const njxni = tmpVec2;
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        ni.cross(nj, nixnj);
        nj.cross(ni, njxni);
        GA.rotational.copy(njxni);
        GB.rotational.copy(nixnj);
        const g = Math.cos(this.angle) - ni.dot(nj);
        const GW = this.computeGW();
        const GiMf = this.computeGiMf();
        const B = -g * a - GW * b - h * GiMf;
        return B;
    }
}
const tmpVec1 = new Vec3();
const tmpVec2 = new Vec3();
