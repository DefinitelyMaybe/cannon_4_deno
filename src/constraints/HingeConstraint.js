/// <reference types="./HingeConstraint.ts" />
/// <reference lib="dom" />
import { PointToPointConstraint } from "../constraints/PointToPointConstraint.js";
import { RotationalEquation } from "../equations/RotationalEquation.js";
import { RotationalMotorEquation } from "../equations/RotationalMotorEquation.js";
import { Vec3 } from "../math/Vec3.js";
export class HingeConstraint extends PointToPointConstraint {
    constructor(bodyA, bodyB, options = {}) {
        const maxForce = typeof options.maxForce !== "undefined"
            ? options.maxForce
            : 1e6;
        const pivotA = options.pivotA ? options.pivotA.clone() : new Vec3();
        const pivotB = options.pivotB ? options.pivotB.clone() : new Vec3();
        super(bodyA, pivotA, bodyB, pivotB, maxForce);
        const axisA = (this.axisA = options.axisA ? options.axisA.clone() : new Vec3(1, 0, 0));
        axisA.normalize();
        const axisB = (this.axisB = options.axisB ? options.axisB.clone() : new Vec3(1, 0, 0));
        axisB.normalize();
        this.collideConnected = !!options.collideConnected;
        const rotational1 = (this.rotationalEquation1 = new RotationalEquation(bodyA, bodyB, options));
        const rotational2 = (this.rotationalEquation2 = new RotationalEquation(bodyA, bodyB, options));
        const motor = (this.motorEquation = new RotationalMotorEquation(bodyA, bodyB, maxForce));
        motor.enabled = false;
        this.equations.push(rotational1, rotational2, motor);
    }
    enableMotor() {
        this.motorEquation.enabled = true;
    }
    disableMotor() {
        this.motorEquation.enabled = false;
    }
    setMotorSpeed(speed) {
        this.motorEquation.targetVelocity = speed;
    }
    setMotorMaxForce(maxForce) {
        this.motorEquation.maxForce = maxForce;
        this.motorEquation.minForce = -maxForce;
    }
    update() {
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const motor = this.motorEquation;
        const r1 = this.rotationalEquation1;
        const r2 = this.rotationalEquation2;
        const worldAxisA = HingeConstraint_update_tmpVec1;
        const worldAxisB = HingeConstraint_update_tmpVec2;
        const axisA = this.axisA;
        const axisB = this.axisB;
        super.update();
        bodyA.quaternion.vmult(axisA, worldAxisA);
        bodyB.quaternion.vmult(axisB, worldAxisB);
        worldAxisA.tangents(r1.axisA, r2.axisA);
        r1.axisB.copy(worldAxisB);
        r2.axisB.copy(worldAxisB);
        if (this.motorEquation.enabled) {
            bodyA.quaternion.vmult(this.axisA, motor.axisA);
            bodyB.quaternion.vmult(this.axisB, motor.axisB);
        }
    }
}
const HingeConstraint_update_tmpVec1 = new Vec3();
const HingeConstraint_update_tmpVec2 = new Vec3();
