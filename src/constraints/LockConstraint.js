/// <reference lib="dom" />
import { PointToPointConstraint } from "../constraints/PointToPointConstraint.js";
import { RotationalEquation } from "../equations/RotationalEquation.js";
import { Vec3 } from "../math/Vec3.js";
export class LockConstraint extends PointToPointConstraint {
  constructor(bodyA, bodyB, options = {}) {
    const maxForce = typeof options.maxForce !== "undefined"
      ? options.maxForce
      : 1e6;
    const pivotA = new Vec3();
    const pivotB = new Vec3();
    const halfWay = new Vec3();
    bodyA.position.vadd(bodyB.position, halfWay);
    halfWay.scale(0.5, halfWay);
    bodyB.pointToLocalFrame(halfWay, pivotB);
    bodyA.pointToLocalFrame(halfWay, pivotA);
    super(bodyA, pivotA, bodyB, pivotB, maxForce);
    this.xA = bodyA.vectorToLocalFrame(Vec3.UNIT_X);
    this.xB = bodyB.vectorToLocalFrame(Vec3.UNIT_X);
    this.yA = bodyA.vectorToLocalFrame(Vec3.UNIT_Y);
    this.yB = bodyB.vectorToLocalFrame(Vec3.UNIT_Y);
    this.zA = bodyA.vectorToLocalFrame(Vec3.UNIT_Z);
    this.zB = bodyB.vectorToLocalFrame(Vec3.UNIT_Z);
    const r1 =
      (this.rotationalEquation1 = new RotationalEquation(
        bodyA,
        bodyB,
        options,
      ));
    const r2 =
      (this.rotationalEquation2 = new RotationalEquation(
        bodyA,
        bodyB,
        options,
      ));
    const r3 =
      (this.rotationalEquation3 = new RotationalEquation(
        bodyA,
        bodyB,
        options,
      ));
    this.equations.push(r1, r2, r3);
  }
  update() {
    const bodyA = this.bodyA;
    const bodyB = this.bodyB;
    const motor = this.motorEquation;
    const r1 = this.rotationalEquation1;
    const r2 = this.rotationalEquation2;
    const r3 = this.rotationalEquation3;
    const worldAxisA = LockConstraint_update_tmpVec1;
    const worldAxisB = LockConstraint_update_tmpVec2;
    super.update();
    bodyA.vectorToWorldFrame(this.xA, r1.axisA);
    bodyB.vectorToWorldFrame(this.yB, r1.axisB);
    bodyA.vectorToWorldFrame(this.yA, r2.axisA);
    bodyB.vectorToWorldFrame(this.zB, r2.axisB);
    bodyA.vectorToWorldFrame(this.zA, r3.axisA);
    bodyB.vectorToWorldFrame(this.xB, r3.axisB);
  }
}
const LockConstraint_update_tmpVec1 = new Vec3();
const LockConstraint_update_tmpVec2 = new Vec3();
