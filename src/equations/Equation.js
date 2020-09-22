/// <reference lib="dom" />
import { JacobianElement } from "../math/JacobianElement.js";
import { Vec3 } from "../math/Vec3.js";
export class Equation {
  constructor(bi, bj, minForce = -1e6, maxForce = 1e6) {
    this.id = Equation.id++;
    this.minForce = minForce;
    this.maxForce = maxForce;
    this.bi = bi;
    this.bj = bj;
    this.a = 0.0;
    this.b = 0.0;
    this.eps = 0.0;
    this.jacobianElementA = new JacobianElement();
    this.jacobianElementB = new JacobianElement();
    this.enabled = true;
    this.multiplier = 0;
    this.setSpookParams(1e7, 4, 1 / 60);
  }
  setSpookParams(stiffness, relaxation, timeStep) {
    const d = relaxation;
    const k = stiffness;
    const h = timeStep;
    this.a = 4.0 / (h * (1 + 4 * d));
    this.b = (4.0 * d) / (1 + 4 * d);
    this.eps = 4.0 / (h * h * k * (1 + 4 * d));
  }
  computeB(a, b, h) {
    const GW = this.computeGW();
    const Gq = this.computeGq();
    const GiMf = this.computeGiMf();
    return -Gq * a - GW * b - GiMf * h;
  }
  computeGq() {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const xi = bi.position;
    const xj = bj.position;
    return GA.spatial.dot(xi) + GB.spatial.dot(xj);
  }
  computeGW() {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const vi = bi.velocity;
    const vj = bj.velocity;
    const wi = bi.angularVelocity;
    const wj = bj.angularVelocity;
    return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
  }
  computeGWlambda() {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const vi = bi.vlambda;
    const vj = bj.vlambda;
    const wi = bi.wlambda;
    const wj = bj.wlambda;
    return GA.multiplyVectors(vi, wi) + GB.multiplyVectors(vj, wj);
  }
  computeGiMf() {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const fi = bi.force;
    const ti = bi.torque;
    const fj = bj.force;
    const tj = bj.torque;
    const invMassi = bi.invMassSolve;
    const invMassj = bj.invMassSolve;
    fi.scale(invMassi, iMfi);
    fj.scale(invMassj, iMfj);
    bi.invInertiaWorldSolve.vmult(ti, invIi_vmult_taui);
    bj.invInertiaWorldSolve.vmult(tj, invIj_vmult_tauj);
    return GA.multiplyVectors(iMfi, invIi_vmult_taui) +
      GB.multiplyVectors(iMfj, invIj_vmult_tauj);
  }
  computeGiMGt() {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const invMassi = bi.invMassSolve;
    const invMassj = bj.invMassSolve;
    const invIi = bi.invInertiaWorldSolve;
    const invIj = bj.invInertiaWorldSolve;
    let result = invMassi + invMassj;
    invIi.vmult(GA.rotational, tmp);
    result += tmp.dot(GA.rotational);
    invIj.vmult(GB.rotational, tmp);
    result += tmp.dot(GB.rotational);
    return result;
  }
  addToWlambda(deltalambda) {
    const GA = this.jacobianElementA;
    const GB = this.jacobianElementB;
    const bi = this.bi;
    const bj = this.bj;
    const temp = addToWlambda_temp;
    bi.vlambda.addScaledVector(
      bi.invMassSolve * deltalambda,
      GA.spatial,
      bi.vlambda,
    );
    bj.vlambda.addScaledVector(
      bj.invMassSolve * deltalambda,
      GB.spatial,
      bj.vlambda,
    );
    bi.invInertiaWorldSolve.vmult(GA.rotational, temp);
    bi.wlambda.addScaledVector(deltalambda, temp, bi.wlambda);
    bj.invInertiaWorldSolve.vmult(GB.rotational, temp);
    bj.wlambda.addScaledVector(deltalambda, temp, bj.wlambda);
  }
  computeC() {
    return this.computeGiMGt() + this.eps;
  }
}
Equation.id = 0;
const iMfi = new Vec3();
const iMfj = new Vec3();
const invIi_vmult_taui = new Vec3();
const invIj_vmult_tauj = new Vec3();
const tmp = new Vec3();
const addToWlambda_temp = new Vec3();
