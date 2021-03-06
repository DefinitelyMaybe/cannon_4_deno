/// <reference types="./ConeTwistConstraint.ts" />
/// <reference lib="dom" />
import { PointToPointConstraint } from "../constraints/PointToPointConstraint.js";
import { ConeEquation } from "../equations/ConeEquation.js";
import { RotationalEquation } from "../equations/RotationalEquation.js";
import { Vec3 } from "../math/Vec3.js";
export class ConeTwistConstraint extends PointToPointConstraint {
    constructor(bodyA, bodyB, options = {}) {
        const maxForce = typeof options.maxForce !== "undefined"
            ? options.maxForce
            : 1e6;
        const pivotA = options.pivotA ? options.pivotA.clone() : new Vec3();
        const pivotB = options.pivotB ? options.pivotB.clone() : new Vec3();
        super(bodyA, pivotA, bodyB, pivotB, maxForce);
        this.axisA = options.axisA ? options.axisA.clone() : new Vec3();
        this.axisB = options.axisB ? options.axisB.clone() : new Vec3();
        this.collideConnected = !!options.collideConnected;
        this.angle = typeof options.angle !== "undefined" ? options.angle : 0;
        const c = (this.coneEquation = new ConeEquation(bodyA, bodyB, options));
        const t = (this.twistEquation = new RotationalEquation(bodyA, bodyB, options));
        this.twistAngle = typeof options.twistAngle !== "undefined"
            ? options.twistAngle
            : 0;
        c.maxForce = 0;
        c.minForce = -maxForce;
        t.maxForce = 0;
        t.minForce = -maxForce;
        this.equations.push(c, t);
    }
    update() {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const cone = this.coneEquation;
        const twist = this.twistEquation;
        super.update();
        bodyA.vectorToWorldFrame(this.axisA, cone.axisA);
        bodyB.vectorToWorldFrame(this.axisB, cone.axisB);
        this.axisA.tangents(twist.axisA, twist.axisA);
        bodyA.vectorToWorldFrame(twist.axisA, twist.axisA);
        this.axisB.tangents(twist.axisB, twist.axisB);
        bodyB.vectorToWorldFrame(twist.axisB, twist.axisB);
        cone.angle = this.angle;
        twist.maxAngle = this.twistAngle;
    }
}
const ConeTwistConstraint_update_tmpVec1 = new Vec3();
const ConeTwistConstraint_update_tmpVec2 = new Vec3();
