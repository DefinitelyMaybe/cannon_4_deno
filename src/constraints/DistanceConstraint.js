/// <reference lib="dom" />
import { Constraint } from "../constraints/Constraint.js";
import { ContactEquation } from "../equations/ContactEquation.js";
export class DistanceConstraint extends Constraint {
    constructor(bodyA, bodyB, distance, maxForce = 1e6) {
        super(bodyA, bodyB);
        if (typeof distance === "undefined") {
            distance = bodyA.position.distanceTo(bodyB.position);
        }
        this.distance = distance;
        const eq = (this.distanceEquation = new ContactEquation(bodyA, bodyB));
        this.equations.push(eq);
        eq.minForce = -maxForce;
        eq.maxForce = maxForce;
    }
    update() {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const eq = this.distanceEquation;
        const halfDist = this.distance * 0.5;
        const normal = eq.ni;
        bodyB.position.vsub(bodyA.position, normal);
        normal.normalize();
        normal.scale(halfDist, eq.ri);
        normal.scale(-halfDist, eq.rj);
    }
}
