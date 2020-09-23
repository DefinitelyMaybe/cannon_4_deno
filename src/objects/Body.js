/// <reference types="./Body.ts" />
/// <reference lib="dom" />
import { EventTarget } from "../utils/EventTarget.js";
import { Vec3 } from "../math/Vec3.js";
import { Mat3 } from "../math/Mat3.js";
import { Quaternion } from "../math/Quaternion.js";
import { AABB } from "../collision/AABB.js";
import { Box } from "../shapes/Box.js";
export const BODY_TYPES = {
    DYNAMIC: 1,
    STATIC: 2,
    KINEMATIC: 4,
};
export const BODY_SLEEP_STATES = {
    AWAKE: 0,
    SLEEPY: 1,
    SLEEPING: 2,
};
export class Body extends EventTarget {
    constructor(options = {}) {
        super();
        this.id = Body.idCounter++;
        this.index = -1;
        this.world = null;
        this.preStep = null;
        this.postStep = null;
        this.vlambda = new Vec3();
        this.collisionFilterGroup = typeof options.collisionFilterGroup === "number"
            ? options.collisionFilterGroup
            : 1;
        this.collisionFilterMask = typeof options.collisionFilterMask === "number"
            ? options.collisionFilterMask
            : -1;
        this.collisionResponse = typeof options.collisionResponse === "boolean"
            ? options.collisionResponse
            : true;
        this.position = new Vec3();
        this.previousPosition = new Vec3();
        this.interpolatedPosition = new Vec3();
        this.initPosition = new Vec3();
        if (options.position) {
            this.position.copy(options.position);
            this.previousPosition.copy(options.position);
            this.interpolatedPosition.copy(options.position);
            this.initPosition.copy(options.position);
        }
        this.velocity = new Vec3();
        if (options.velocity) {
            this.velocity.copy(options.velocity);
        }
        this.initVelocity = new Vec3();
        this.force = new Vec3();
        const mass = typeof options.mass === "number" ? options.mass : 0;
        this.mass = mass;
        this.invMass = mass > 0 ? 1.0 / mass : 0;
        this.material = options.material || null;
        this.linearDamping = typeof options.linearDamping === "number"
            ? options.linearDamping
            : 0.01;
        this.type = mass <= 0.0 ? Body.STATIC : Body.DYNAMIC;
        if (typeof options.type === typeof Body.STATIC) {
            this.type = options.type;
        }
        this.allowSleep = typeof options.allowSleep !== "undefined"
            ? options.allowSleep
            : true;
        this.sleepState = 0;
        this.sleepSpeedLimit = typeof options.sleepSpeedLimit !== "undefined"
            ? options.sleepSpeedLimit
            : 0.1;
        this.sleepTimeLimit = typeof options.sleepTimeLimit !== "undefined"
            ? options.sleepTimeLimit
            : 1;
        this.timeLastSleepy = 0;
        this.wakeUpAfterNarrowphase = false;
        this.torque = new Vec3();
        this.quaternion = new Quaternion();
        this.initQuaternion = new Quaternion();
        this.previousQuaternion = new Quaternion();
        this.interpolatedQuaternion = new Quaternion();
        if (options.quaternion) {
            this.quaternion.copy(options.quaternion);
            this.initQuaternion.copy(options.quaternion);
            this.previousQuaternion.copy(options.quaternion);
            this.interpolatedQuaternion.copy(options.quaternion);
        }
        this.angularVelocity = new Vec3();
        if (options.angularVelocity) {
            this.angularVelocity.copy(options.angularVelocity);
        }
        this.initAngularVelocity = new Vec3();
        this.shapes = [];
        this.shapeOffsets = [];
        this.shapeOrientations = [];
        this.inertia = new Vec3();
        this.invInertia = new Vec3();
        this.invInertiaWorld = new Mat3();
        this.invMassSolve = 0;
        this.invInertiaSolve = new Vec3();
        this.invInertiaWorldSolve = new Mat3();
        this.fixedRotation = typeof options.fixedRotation !== "undefined"
            ? options.fixedRotation
            : false;
        this.angularDamping = typeof options.angularDamping !== "undefined"
            ? options.angularDamping
            : 0.01;
        this.linearFactor = new Vec3(1, 1, 1);
        if (options.linearFactor) {
            this.linearFactor.copy(options.linearFactor);
        }
        this.angularFactor = new Vec3(1, 1, 1);
        if (options.angularFactor) {
            this.angularFactor.copy(options.angularFactor);
        }
        this.aabb = new AABB();
        this.aabbNeedsUpdate = true;
        this.boundingRadius = 0;
        this.wlambda = new Vec3();
        if (options.shape) {
            this.addShape(options.shape);
        }
        this.updateMassProperties();
    }
    wakeUp() {
        const prevState = this.sleepState;
        this.sleepState = 0;
        this.wakeUpAfterNarrowphase = false;
        if (prevState === Body.SLEEPING) {
            this.dispatchEvent(Body.wakeupEvent);
        }
    }
    sleep() {
        this.sleepState = Body.SLEEPING;
        this.velocity.set(0, 0, 0);
        this.angularVelocity.set(0, 0, 0);
        this.wakeUpAfterNarrowphase = false;
    }
    sleepTick(time) {
        if (this.allowSleep) {
            const sleepState = this.sleepState;
            const speedSquared = this.velocity.lengthSquared() +
                this.angularVelocity.lengthSquared();
            const speedLimitSquared = this.sleepSpeedLimit ** 2;
            if (sleepState === Body.AWAKE && speedSquared < speedLimitSquared) {
                this.sleepState = Body.SLEEPY;
                this.timeLastSleepy = time;
                this.dispatchEvent(Body.sleepyEvent);
            }
            else if (sleepState === Body.SLEEPY && speedSquared > speedLimitSquared) {
                this.wakeUp();
            }
            else if (sleepState === Body.SLEEPY &&
                time - this.timeLastSleepy > this.sleepTimeLimit) {
                this.sleep();
                this.dispatchEvent(Body.sleepEvent);
            }
        }
    }
    updateSolveMassProperties() {
        if (this.sleepState === Body.SLEEPING || this.type === Body.KINEMATIC) {
            this.invMassSolve = 0;
            this.invInertiaSolve.setZero();
            this.invInertiaWorldSolve.setZero();
        }
        else {
            this.invMassSolve = this.invMass;
            this.invInertiaSolve.copy(this.invInertia);
            this.invInertiaWorldSolve.copy(this.invInertiaWorld);
        }
    }
    pointToLocalFrame(worldPoint, result = new Vec3()) {
        worldPoint.vsub(this.position, result);
        this.quaternion.conjugate().vmult(result, result);
        return result;
    }
    vectorToLocalFrame(worldVector, result = new Vec3()) {
        this.quaternion.conjugate().vmult(worldVector, result);
        return result;
    }
    pointToWorldFrame(localPoint, result = new Vec3()) {
        this.quaternion.vmult(localPoint, result);
        result.vadd(this.position, result);
        return result;
    }
    vectorToWorldFrame(localVector, result = new Vec3()) {
        this.quaternion.vmult(localVector, result);
        return result;
    }
    addShape(shape, _offset, _orientation) {
        const offset = new Vec3();
        const orientation = new Quaternion();
        if (_offset) {
            offset.copy(_offset);
        }
        if (_orientation) {
            orientation.copy(_orientation);
        }
        this.shapes.push(shape);
        this.shapeOffsets.push(offset);
        this.shapeOrientations.push(orientation);
        this.updateMassProperties();
        this.updateBoundingRadius();
        this.aabbNeedsUpdate = true;
        shape.body = this;
        return this;
    }
    updateBoundingRadius() {
        const shapes = this.shapes;
        const shapeOffsets = this.shapeOffsets;
        const N = shapes.length;
        let radius = 0;
        for (let i = 0; i !== N; i++) {
            const shape = shapes[i];
            shape.updateBoundingSphereRadius();
            const offset = shapeOffsets[i].length();
            const r = shape.boundingSphereRadius;
            if (offset + r > radius) {
                radius = offset + r;
            }
        }
        this.boundingRadius = radius;
    }
    computeAABB() {
        const shapes = this.shapes;
        const shapeOffsets = this.shapeOffsets;
        const shapeOrientations = this.shapeOrientations;
        const N = shapes.length;
        const offset = tmpVec;
        const orientation = tmpQuat;
        const bodyQuat = this.quaternion;
        const aabb = this.aabb;
        const shapeAABB = computeAABB_shapeAABB;
        for (let i = 0; i !== N; i++) {
            const shape = shapes[i];
            bodyQuat.vmult(shapeOffsets[i], offset);
            offset.vadd(this.position, offset);
            bodyQuat.mult(shapeOrientations[i], orientation);
            shape.calculateWorldAABB(offset, orientation, shapeAABB.lowerBound, shapeAABB.upperBound);
            if (i === 0) {
                aabb.copy(shapeAABB);
            }
            else {
                aabb.extend(shapeAABB);
            }
        }
        this.aabbNeedsUpdate = false;
    }
    updateInertiaWorld(force) {
        const I = this.invInertia;
        if (I.x === I.y && I.y === I.z && !force) {
        }
        else {
            const m1 = uiw_m1;
            const m2 = uiw_m2;
            const m3 = uiw_m3;
            m1.setRotationFromQuaternion(this.quaternion);
            m1.transpose(m2);
            m1.scale(I, m1);
            m1.mmult(m2, this.invInertiaWorld);
        }
    }
    applyForce(force, relativePoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        const rotForce = Body_applyForce_rotForce;
        relativePoint.cross(force, rotForce);
        this.force.vadd(force, this.force);
        this.torque.vadd(rotForce, this.torque);
    }
    applyLocalForce(localForce, localPoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        const worldForce = Body_applyLocalForce_worldForce;
        const relativePointWorld = Body_applyLocalForce_relativePointWorld;
        this.vectorToWorldFrame(localForce, worldForce);
        this.vectorToWorldFrame(localPoint, relativePointWorld);
        this.applyForce(worldForce, relativePointWorld);
    }
    applyImpulse(impulse, relativePoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        const r = relativePoint;
        const velo = Body_applyImpulse_velo;
        velo.copy(impulse);
        velo.scale(this.invMass, velo);
        this.velocity.vadd(velo, this.velocity);
        const rotVelo = Body_applyImpulse_rotVelo;
        r.cross(impulse, rotVelo);
        this.invInertiaWorld.vmult(rotVelo, rotVelo);
        this.angularVelocity.vadd(rotVelo, this.angularVelocity);
    }
    applyLocalImpulse(localImpulse, localPoint) {
        if (this.type !== Body.DYNAMIC) {
            return;
        }
        const worldImpulse = Body_applyLocalImpulse_worldImpulse;
        const relativePointWorld = Body_applyLocalImpulse_relativePoint;
        this.vectorToWorldFrame(localImpulse, worldImpulse);
        this.vectorToWorldFrame(localPoint, relativePointWorld);
        this.applyImpulse(worldImpulse, relativePointWorld);
    }
    updateMassProperties() {
        const halfExtents = Body_updateMassProperties_halfExtents;
        this.invMass = this.mass > 0 ? 1.0 / this.mass : 0;
        const I = this.inertia;
        const fixed = this.fixedRotation;
        this.computeAABB();
        halfExtents.set((this.aabb.upperBound.x - this.aabb.lowerBound.x) / 2, (this.aabb.upperBound.y - this.aabb.lowerBound.y) / 2, (this.aabb.upperBound.z - this.aabb.lowerBound.z) / 2);
        Box.calculateInertia(halfExtents, this.mass, I);
        this.invInertia.set(I.x > 0 && !fixed ? 1.0 / I.x : 0, I.y > 0 && !fixed ? 1.0 / I.y : 0, I.z > 0 && !fixed ? 1.0 / I.z : 0);
        this.updateInertiaWorld(true);
    }
    getVelocityAtWorldPoint(worldPoint, result) {
        const r = new Vec3();
        worldPoint.vsub(this.position, r);
        this.angularVelocity.cross(r, result);
        this.velocity.vadd(result, result);
        return result;
    }
    integrate(dt, quatNormalize, quatNormalizeFast) {
        this.previousPosition.copy(this.position);
        this.previousQuaternion.copy(this.quaternion);
        if (!(this.type === Body.DYNAMIC || this.type === Body.KINEMATIC) ||
            this.sleepState === Body.SLEEPING) {
            return;
        }
        const velo = this.velocity;
        const angularVelo = this.angularVelocity;
        const pos = this.position;
        const force = this.force;
        const torque = this.torque;
        const quat = this.quaternion;
        const invMass = this.invMass;
        const invInertia = this.invInertiaWorld;
        const linearFactor = this.linearFactor;
        const iMdt = invMass * dt;
        velo.x += force.x * iMdt * linearFactor.x;
        velo.y += force.y * iMdt * linearFactor.y;
        velo.z += force.z * iMdt * linearFactor.z;
        const e = invInertia.elements;
        const angularFactor = this.angularFactor;
        const tx = torque.x * angularFactor.x;
        const ty = torque.y * angularFactor.y;
        const tz = torque.z * angularFactor.z;
        angularVelo.x += dt * (e[0] * tx + e[1] * ty + e[2] * tz);
        angularVelo.y += dt * (e[3] * tx + e[4] * ty + e[5] * tz);
        angularVelo.z += dt * (e[6] * tx + e[7] * ty + e[8] * tz);
        pos.x += velo.x * dt;
        pos.y += velo.y * dt;
        pos.z += velo.z * dt;
        quat.integrate(this.angularVelocity, dt, this.angularFactor, quat);
        if (quatNormalize) {
            if (quatNormalizeFast) {
                quat.normalizeFast();
            }
            else {
                quat.normalize();
            }
        }
        this.aabbNeedsUpdate = true;
        this.updateInertiaWorld();
    }
}
Body.COLLIDE_EVENT_NAME = "collide";
Body.DYNAMIC = 1;
Body.STATIC = 2;
Body.KINEMATIC = 4;
Body.AWAKE = BODY_SLEEP_STATES.AWAKE;
Body.SLEEPY = BODY_SLEEP_STATES.SLEEPY;
Body.SLEEPING = BODY_SLEEP_STATES.SLEEPING;
Body.idCounter = 0;
Body.wakeupEvent = { type: "wakeup" };
Body.sleepyEvent = { type: "sleepy" };
Body.sleepEvent = { type: "sleep" };
const tmpVec = new Vec3();
const tmpQuat = new Quaternion();
const computeAABB_shapeAABB = new AABB();
const uiw_m1 = new Mat3();
const uiw_m2 = new Mat3();
const uiw_m3 = new Mat3();
const Body_applyForce_rotForce = new Vec3();
const Body_applyLocalForce_worldForce = new Vec3();
const Body_applyLocalForce_relativePointWorld = new Vec3();
const Body_applyImpulse_velo = new Vec3();
const Body_applyImpulse_rotVelo = new Vec3();
const Body_applyLocalImpulse_worldImpulse = new Vec3();
const Body_applyLocalImpulse_relativePoint = new Vec3();
const Body_updateMassProperties_halfExtents = new Vec3();
