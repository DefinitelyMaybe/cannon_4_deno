/// <reference types="./Spring.ts" />
import { Vec3 } from "../math/Vec3.js";
export class Spring {
    constructor(bodyA, bodyB, options = {}) {
        this.restLength = typeof options.restLength === "number"
            ? options.restLength
            : 1;
        this.stiffness = options.stiffness || 100;
        this.damping = options.damping || 1;
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.localAnchorA = new Vec3();
        this.localAnchorB = new Vec3();
        if (options.localAnchorA) {
            this.localAnchorA.copy(options.localAnchorA);
        }
        if (options.localAnchorB) {
            this.localAnchorB.copy(options.localAnchorB);
        }
        if (options.worldAnchorA) {
            this.setWorldAnchorA(options.worldAnchorA);
        }
        if (options.worldAnchorB) {
            this.setWorldAnchorB(options.worldAnchorB);
        }
    }
    setWorldAnchorA(worldAnchorA) {
        this.bodyA.pointToLocalFrame(worldAnchorA, this.localAnchorA);
    }
    setWorldAnchorB(worldAnchorB) {
        this.bodyB.pointToLocalFrame(worldAnchorB, this.localAnchorB);
    }
    getWorldAnchorA(result) {
        this.bodyA.pointToWorldFrame(this.localAnchorA, result);
    }
    getWorldAnchorB(result) {
        this.bodyB.pointToWorldFrame(this.localAnchorB, result);
    }
    applyForce() {
        const k = this.stiffness;
        const d = this.damping;
        const l = this.restLength;
        const bodyA = this.bodyA;
        const bodyB = this.bodyB;
        const r = applyForce_r;
        const r_unit = applyForce_r_unit;
        const u = applyForce_u;
        const f = applyForce_f;
        const tmp = applyForce_tmp;
        const worldAnchorA = applyForce_worldAnchorA;
        const worldAnchorB = applyForce_worldAnchorB;
        const ri = applyForce_ri;
        const rj = applyForce_rj;
        const ri_x_f = applyForce_ri_x_f;
        const rj_x_f = applyForce_rj_x_f;
        this.getWorldAnchorA(worldAnchorA);
        this.getWorldAnchorB(worldAnchorB);
        worldAnchorA.vsub(bodyA.position, ri);
        worldAnchorB.vsub(bodyB.position, rj);
        worldAnchorB.vsub(worldAnchorA, r);
        const rlen = r.length();
        r_unit.copy(r);
        r_unit.normalize();
        bodyB.velocity.vsub(bodyA.velocity, u);
        bodyB.angularVelocity.cross(rj, tmp);
        u.vadd(tmp, u);
        bodyA.angularVelocity.cross(ri, tmp);
        u.vsub(tmp, u);
        r_unit.scale(-k * (rlen - l) - d * u.dot(r_unit), f);
        bodyA.force.vsub(f, bodyA.force);
        bodyB.force.vadd(f, bodyB.force);
        ri.cross(f, ri_x_f);
        rj.cross(f, rj_x_f);
        bodyA.torque.vsub(ri_x_f, bodyA.torque);
        bodyB.torque.vadd(rj_x_f, bodyB.torque);
    }
}
const applyForce_r = new Vec3();
const applyForce_r_unit = new Vec3();
const applyForce_u = new Vec3();
const applyForce_f = new Vec3();
const applyForce_worldAnchorA = new Vec3();
const applyForce_worldAnchorB = new Vec3();
const applyForce_ri = new Vec3();
const applyForce_rj = new Vec3();
const applyForce_ri_x_f = new Vec3();
const applyForce_rj_x_f = new Vec3();
const applyForce_tmp = new Vec3();
