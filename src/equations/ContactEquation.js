/// <reference types="./ContactEquation.ts" />
import { Equation } from "../equations/Equation.js";
import { Vec3 } from "../math/Vec3.js";
export class ContactEquation extends Equation {
    constructor(bodyA, bodyB, maxForce = 1e6) {
        super(bodyA, bodyB, 0, maxForce);
        this.restitution = 0.0;
        this.ri = new Vec3();
        this.rj = new Vec3();
        this.ni = new Vec3();
    }
    computeB(h) {
        const a = this.a;
        const b = this.b;
        const bi = this.bi;
        const bj = this.bj;
        const ri = this.ri;
        const rj = this.rj;
        const rixn = ContactEquation_computeB_temp1;
        const rjxn = ContactEquation_computeB_temp2;
        const vi = bi.velocity;
        const wi = bi.angularVelocity;
        const fi = bi.force;
        const taui = bi.torque;
        const vj = bj.velocity;
        const wj = bj.angularVelocity;
        const fj = bj.force;
        const tauj = bj.torque;
        const penetrationVec = ContactEquation_computeB_temp3;
        const GA = this.jacobianElementA;
        const GB = this.jacobianElementB;
        const n = this.ni;
        ri.cross(n, rixn);
        rj.cross(n, rjxn);
        n.negate(GA.spatial);
        rixn.negate(GA.rotational);
        GB.spatial.copy(n);
        GB.rotational.copy(rjxn);
        penetrationVec.copy(bj.position);
        penetrationVec.vadd(rj, penetrationVec);
        penetrationVec.vsub(bi.position, penetrationVec);
        penetrationVec.vsub(ri, penetrationVec);
        const g = n.dot(penetrationVec);
        const ePlusOne = this.restitution + 1;
        const GW = ePlusOne * vj.dot(n) - ePlusOne * vi.dot(n) + wj.dot(rjxn) -
            wi.dot(rixn);
        const GiMf = this.computeGiMf();
        const B = -g * a - GW * b - h * GiMf;
        return B;
    }
    getImpactVelocityAlongNormal() {
        const vi = ContactEquation_getImpactVelocityAlongNormal_vi;
        const vj = ContactEquation_getImpactVelocityAlongNormal_vj;
        const xi = ContactEquation_getImpactVelocityAlongNormal_xi;
        const xj = ContactEquation_getImpactVelocityAlongNormal_xj;
        const relVel = ContactEquation_getImpactVelocityAlongNormal_relVel;
        this.bi.position.vadd(this.ri, xi);
        this.bj.position.vadd(this.rj, xj);
        this.bi.getVelocityAtWorldPoint(xi, vi);
        this.bj.getVelocityAtWorldPoint(xj, vj);
        vi.vsub(vj, relVel);
        return this.ni.dot(relVel);
    }
}
const ContactEquation_computeB_temp1 = new Vec3();
const ContactEquation_computeB_temp2 = new Vec3();
const ContactEquation_computeB_temp3 = new Vec3();
const ContactEquation_getImpactVelocityAlongNormal_vi = new Vec3();
const ContactEquation_getImpactVelocityAlongNormal_vj = new Vec3();
const ContactEquation_getImpactVelocityAlongNormal_xi = new Vec3();
const ContactEquation_getImpactVelocityAlongNormal_xj = new Vec3();
const ContactEquation_getImpactVelocityAlongNormal_relVel = new Vec3();
