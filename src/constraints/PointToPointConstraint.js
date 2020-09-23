/// <reference types="./PointToPointConstraint.ts" />
/// <reference lib="dom" />
import { Constraint } from "../constraints/Constraint.js";
import { ContactEquation } from "../equations/ContactEquation.js";
import { Vec3 } from "../math/Vec3.js";
export class PointToPointConstraint extends Constraint {
    constructor(bodyA, pivotA = new Vec3(), bodyB, pivotB = new Vec3(), maxForce = 1e6) {
        super(bodyA, bodyB);
        this.pivotA = pivotA.clone();
        this.pivotB = pivotB.clone();
        const x = (this.equationX = new ContactEquation(bodyA, bodyB));
        const y = (this.equationY = new ContactEquation(bodyA, bodyB));
        const z = (this.equationZ = new ContactEquation(bodyA, bodyB));
        this.equations.push(x, y, z);
        x.minForce = y.minForce = z.minForce = -maxForce;
        x.maxForce = y.maxForce = z.maxForce = maxForce;
        x.ni.set(1, 0, 0);
        y.ni.set(0, 1, 0);
        z.ni.set(0, 0, 1);
    }
    update() {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const x = this.equationX;
        const y = this.equationY;
        const z = this.equationZ;
        bodyA.quaternion.vmult(this.pivotA, x.ri);
        bodyB.quaternion.vmult(this.pivotB, x.rj);
        y.ri.copy(x.ri);
        y.rj.copy(x.rj);
        z.ri.copy(x.ri);
        z.rj.copy(x.rj);
    }
}
