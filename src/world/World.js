/// <reference lib="dom" />
import { EventTarget } from "../utils/EventTarget.js";
import { GSSolver } from "../solver/GSSolver.js";
import { NaiveBroadphase } from "../collision/NaiveBroadphase.js";
import { Narrowphase } from "../world/Narrowphase.js";
import { Vec3 } from "../math/Vec3.js";
import { Material } from "../material/Material.js";
import { ContactMaterial } from "../material/ContactMaterial.js";
import { ArrayCollisionMatrix } from "../collision/ArrayCollisionMatrix.js";
import { OverlapKeeper } from "../collision/OverlapKeeper.js";
import { TupleDictionary } from "../utils/TupleDictionary.js";
import { RaycastResult } from "../collision/RaycastResult.js";
import { Ray } from "../collision/Ray.js";
import { AABB } from "../collision/AABB.js";
import { Body } from "../objects/Body.js";
export class World extends EventTarget {
    constructor(options = {}) {
        super();
        this.dt = -1;
        this.allowSleep = !!options.allowSleep;
        this.contacts = [];
        this.frictionEquations = [];
        this.quatNormalizeSkip = options.quatNormalizeSkip !== undefined
            ? options.quatNormalizeSkip
            : 0;
        this.quatNormalizeFast = options.quatNormalizeFast !== undefined
            ? options.quatNormalizeFast
            : false;
        this.time = 0.0;
        this.stepnumber = 0;
        this.default_dt = 1 / 60;
        this.nextId = 0;
        this.gravity = new Vec3();
        if (options.gravity) {
            this.gravity.copy(options.gravity);
        }
        this.broadphase = options.broadphase !== undefined
            ? options.broadphase
            : new NaiveBroadphase();
        this.bodies = [];
        this.hasActiveBodies = false;
        this.solver = options.solver !== undefined
            ? options.solver
            : new GSSolver();
        this.constraints = [];
        this.narrowphase = new Narrowphase(this);
        this.collisionMatrix = new ArrayCollisionMatrix();
        this.collisionMatrixPrevious = new ArrayCollisionMatrix();
        this.bodyOverlapKeeper = new OverlapKeeper();
        this.shapeOverlapKeeper = new OverlapKeeper();
        this.materials = [];
        this.contactmaterials = [];
        this.contactMaterialTable = new TupleDictionary();
        this.defaultMaterial = new Material("default");
        this.defaultContactMaterial = new ContactMaterial(this.defaultMaterial, this.defaultMaterial, {
            friction: 0.3,
            restitution: 0.0,
        });
        this.doProfiling = false;
        this.profile = {
            solve: 0,
            makeContactConstraints: 0,
            broadphase: 0,
            integrate: 0,
            narrowphase: 0,
        };
        this.accumulator = 0;
        this.subsystems = [];
        this.addBodyEvent = { type: "addBody", body: null };
        this.removeBodyEvent = { type: "removeBody", body: null };
        this.idToBodyMap = {};
        this.broadphase.setWorld(this);
    }
    getContactMaterial(m1, m2) {
        return this.contactMaterialTable.get(m1.id, m2.id);
    }
    numObjects() {
        return this.bodies.length;
    }
    collisionMatrixTick() {
        const temp = this.collisionMatrixPrevious;
        this.collisionMatrixPrevious = this.collisionMatrix;
        this.collisionMatrix = temp;
        this.collisionMatrix.reset();
        this.bodyOverlapKeeper.tick();
        this.shapeOverlapKeeper.tick();
    }
    addConstraint(c) {
        this.constraints.push(c);
    }
    removeConstraint(c) {
        const idx = this.constraints.indexOf(c);
        if (idx !== -1) {
            this.constraints.splice(idx, 1);
        }
    }
    rayTest(from, to, result) {
        if (result instanceof RaycastResult) {
            this.raycastClosest(from, to, { skipBackfaces: true }, result);
        }
        else {
            this.raycastAll(from, to, { skipBackfaces: true }, result);
        }
    }
    raycastAll(from, to, options = {}, callback) {
        options.mode = Ray.ALL;
        options.from = from;
        options.to = to;
        options.callback = callback;
        return tmpRay.intersectWorld(this, options);
    }
    raycastAny(from, to, options = {}, result) {
        options.mode = Ray.ANY;
        options.from = from;
        options.to = to;
        options.result = result;
        return tmpRay.intersectWorld(this, options);
    }
    raycastClosest(from, to, options = {}, result) {
        options.mode = Ray.CLOSEST;
        options.from = from;
        options.to = to;
        options.result = result;
        return tmpRay.intersectWorld(this, options);
    }
    addBody(body) {
        if (this.bodies.includes(body)) {
            return;
        }
        body.index = this.bodies.length;
        this.bodies.push(body);
        body.world = this;
        body.initPosition.copy(body.position);
        body.initVelocity.copy(body.velocity);
        body.timeLastSleepy = this.time;
        if (body instanceof Body) {
            body.initAngularVelocity.copy(body.angularVelocity);
            body.initQuaternion.copy(body.quaternion);
        }
        this.collisionMatrix.setNumObjects(this.bodies.length);
        this.addBodyEvent.body = body;
        this.idToBodyMap[body.id] = body;
        this.dispatchEvent(this.addBodyEvent);
    }
    removeBody(body) {
        body.world = null;
        const n = this.bodies.length - 1;
        const bodies = this.bodies;
        const idx = bodies.indexOf(body);
        if (idx !== -1) {
            bodies.splice(idx, 1);
            for (let i = 0; i !== bodies.length; i++) {
                bodies[i].index = i;
            }
            this.collisionMatrix.setNumObjects(n);
            this.removeBodyEvent.body = body;
            delete this.idToBodyMap[body.id];
            this.dispatchEvent(this.removeBodyEvent);
        }
    }
    getBodyById(id) {
        return this.idToBodyMap[id];
    }
    getShapeById(id) {
        const bodies = this.bodies;
        for (let i = 0, bl = bodies.length; i < bl; i++) {
            const shapes = bodies[i].shapes;
            for (let j = 0, sl = shapes.length; j < sl; j++) {
                const shape = shapes[j];
                if (shape.id === id) {
                    return shape;
                }
            }
        }
    }
    addMaterial(m) {
        this.materials.push(m);
    }
    addContactMaterial(cmat) {
        this.contactmaterials.push(cmat);
        this.contactMaterialTable.set(cmat.materials[0].id, cmat.materials[1].id, cmat);
    }
    step(dt, timeSinceLastCalled, maxSubSteps = 10) {
        if (timeSinceLastCalled === undefined) {
            this.internalStep(dt);
            this.time += dt;
        }
        else {
            this.accumulator += timeSinceLastCalled;
            const t0 = performance.now();
            let substeps = 0;
            while (this.accumulator >= dt && substeps < maxSubSteps) {
                this.internalStep(dt);
                this.accumulator -= dt;
                substeps++;
                if (performance.now() - t0 > dt * 2 * 1000) {
                    break;
                }
            }
            this.accumulator = this.accumulator % dt;
            const t = this.accumulator / dt;
            for (let j = 0; j !== this.bodies.length; j++) {
                const b = this.bodies[j];
                b.previousPosition.lerp(b.position, t, b.interpolatedPosition);
                b.previousQuaternion.slerp(b.quaternion, t, b.interpolatedQuaternion);
                b.previousQuaternion.normalize();
            }
            this.time += timeSinceLastCalled;
        }
    }
    internalStep(dt) {
        this.dt = dt;
        const world = this;
        const that = this;
        const contacts = this.contacts;
        const p1 = World_step_p1;
        const p2 = World_step_p2;
        const N = this.numObjects();
        const bodies = this.bodies;
        const solver = this.solver;
        const gravity = this.gravity;
        const doProfiling = this.doProfiling;
        const profile = this.profile;
        const DYNAMIC = Body.DYNAMIC;
        let profilingStart = -Infinity;
        const constraints = this.constraints;
        const frictionEquationPool = World_step_frictionEquationPool;
        const gnorm = gravity.length();
        const gx = gravity.x;
        const gy = gravity.y;
        const gz = gravity.z;
        let i = 0;
        if (doProfiling) {
            profilingStart = performance.now();
        }
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.type === DYNAMIC) {
                const f = bi.force;
                const m = bi.mass;
                f.x += m * gx;
                f.y += m * gy;
                f.z += m * gz;
            }
        }
        for (let i = 0, Nsubsystems = this.subsystems.length; i !== Nsubsystems; i++) {
            this.subsystems[i].update();
        }
        if (doProfiling) {
            profilingStart = performance.now();
        }
        p1.length = 0;
        p2.length = 0;
        this.broadphase.collisionPairs(this, p1, p2);
        if (doProfiling) {
            profile.broadphase = performance.now() - profilingStart;
        }
        let Nconstraints = constraints.length;
        for (i = 0; i !== Nconstraints; i++) {
            const c = constraints[i];
            if (!c.collideConnected) {
                for (let j = p1.length - 1; j >= 0; j -= 1) {
                    if ((c.bodyA === p1[j] && c.bodyB === p2[j]) ||
                        (c.bodyB === p1[j] && c.bodyA === p2[j])) {
                        p1.splice(j, 1);
                        p2.splice(j, 1);
                    }
                }
            }
        }
        this.collisionMatrixTick();
        if (doProfiling) {
            profilingStart = performance.now();
        }
        const oldcontacts = World_step_oldContacts;
        const NoldContacts = contacts.length;
        for (i = 0; i !== NoldContacts; i++) {
            oldcontacts.push(contacts[i]);
        }
        contacts.length = 0;
        const NoldFrictionEquations = this.frictionEquations.length;
        for (i = 0; i !== NoldFrictionEquations; i++) {
            frictionEquationPool.push(this.frictionEquations[i]);
        }
        this.frictionEquations.length = 0;
        this.narrowphase.getContacts(p1, p2, this, contacts, oldcontacts, this.frictionEquations, frictionEquationPool);
        if (doProfiling) {
            profile.narrowphase = performance.now() - profilingStart;
        }
        if (doProfiling) {
            profilingStart = performance.now();
        }
        for (i = 0; i < this.frictionEquations.length; i++) {
            solver.addEquation(this.frictionEquations[i]);
        }
        const ncontacts = contacts.length;
        for (let k = 0; k !== ncontacts; k++) {
            const c = contacts[k];
            const bi = c.bi;
            const bj = c.bj;
            const si = c.si;
            const sj = c.sj;
            let cm;
            if (bi.material && bj.material) {
                cm = this.getContactMaterial(bi.material, bj.material) ||
                    this.defaultContactMaterial;
            }
            else {
                cm = this.defaultContactMaterial;
            }
            let mu = cm.friction;
            if (bi.material && bj.material) {
                if (bi.material.friction >= 0 && bj.material.friction >= 0) {
                    mu = bi.material.friction * bj.material.friction;
                }
                if (bi.material.restitution >= 0 && bj.material.restitution >= 0) {
                    c.restitution = bi.material.restitution * bj.material.restitution;
                }
            }
            solver.addEquation(c);
            if (bi.allowSleep &&
                bi.type === Body.DYNAMIC &&
                bi.sleepState === Body.SLEEPING &&
                bj.sleepState === Body.AWAKE &&
                bj.type !== Body.STATIC) {
                const speedSquaredB = bj.velocity.lengthSquared() +
                    bj.angularVelocity.lengthSquared();
                const speedLimitSquaredB = bj.sleepSpeedLimit ** 2;
                if (speedSquaredB >= speedLimitSquaredB * 2) {
                    bi.wakeUpAfterNarrowphase = true;
                }
            }
            if (bj.allowSleep &&
                bj.type === Body.DYNAMIC &&
                bj.sleepState === Body.SLEEPING &&
                bi.sleepState === Body.AWAKE &&
                bi.type !== Body.STATIC) {
                const speedSquaredA = bi.velocity.lengthSquared() +
                    bi.angularVelocity.lengthSquared();
                const speedLimitSquaredA = bi.sleepSpeedLimit ** 2;
                if (speedSquaredA >= speedLimitSquaredA * 2) {
                    bj.wakeUpAfterNarrowphase = true;
                }
            }
            this.collisionMatrix.set(bi, bj, true);
            if (!this.collisionMatrixPrevious.get(bi, bj)) {
                World_step_collideEvent.body = bj;
                World_step_collideEvent.contact = c;
                bi.dispatchEvent(World_step_collideEvent);
                World_step_collideEvent.body = bi;
                bj.dispatchEvent(World_step_collideEvent);
            }
            this.bodyOverlapKeeper.set(bi.id, bj.id);
            this.shapeOverlapKeeper.set(si.id, sj.id);
        }
        this.emitContactEvents();
        if (doProfiling) {
            profile.makeContactConstraints = performance.now() - profilingStart;
            profilingStart = performance.now();
        }
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.wakeUpAfterNarrowphase) {
                bi.wakeUp();
                bi.wakeUpAfterNarrowphase = false;
            }
        }
        Nconstraints = constraints.length;
        for (i = 0; i !== Nconstraints; i++) {
            const c = constraints[i];
            c.update();
            for (let j = 0, Neq = c.equations.length; j !== Neq; j++) {
                const eq = c.equations[j];
                solver.addEquation(eq);
            }
        }
        solver.solve(dt, this);
        if (doProfiling) {
            profile.solve = performance.now() - profilingStart;
        }
        solver.removeAllEquations();
        const pow = Math.pow;
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.type & DYNAMIC) {
                const ld = pow(1.0 - bi.linearDamping, dt);
                const v = bi.velocity;
                v.scale(ld, v);
                const av = bi.angularVelocity;
                if (av) {
                    const ad = pow(1.0 - bi.angularDamping, dt);
                    av.scale(ad, av);
                }
            }
        }
        this.dispatchEvent(World_step_preStepEvent);
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            if (bi.preStep) {
                bi.preStep.call(bi);
            }
        }
        if (doProfiling) {
            profilingStart = performance.now();
        }
        const stepnumber = this.stepnumber;
        const quatNormalize = stepnumber % (this.quatNormalizeSkip + 1) === 0;
        const quatNormalizeFast = this.quatNormalizeFast;
        for (i = 0; i !== N; i++) {
            bodies[i].integrate(dt, quatNormalize, quatNormalizeFast);
        }
        this.clearForces();
        this.broadphase.dirty = true;
        if (doProfiling) {
            profile.integrate = performance.now() - profilingStart;
        }
        this.time += dt;
        this.stepnumber += 1;
        this.dispatchEvent(World_step_postStepEvent);
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            const postStep = bi.postStep;
            if (postStep) {
                postStep.call(bi);
            }
        }
        let hasActiveBodies = true;
        if (this.allowSleep) {
            hasActiveBodies = false;
            for (i = 0; i !== N; i++) {
                const bi = bodies[i];
                bi.sleepTick(this.time);
                if (bi.sleepState !== Body.SLEEPING) {
                    hasActiveBodies = true;
                }
            }
        }
        this.hasActiveBodies = hasActiveBodies;
    }
    clearForces() {
        const bodies = this.bodies;
        const N = bodies.length;
        for (let i = 0; i !== N; i++) {
            const b = bodies[i];
            const force = b.force;
            const tau = b.torque;
            b.force.set(0, 0, 0);
            b.torque.set(0, 0, 0);
        }
    }
}
const tmpAABB1 = new AABB();
const tmpArray1 = [];
const tmpRay = new Ray();
const performance = (globalThis.performance || {});
if (!performance.now) {
    let nowOffset = Date.now();
    if (performance.timing && performance.timing.navigationStart) {
        nowOffset = performance.timing.navigationStart;
    }
    performance.now = () => Date.now() - nowOffset;
}
const step_tmp1 = new Vec3();
const World_step_postStepEvent = { type: "postStep" };
const World_step_preStepEvent = { type: "preStep" };
const World_step_collideEvent = { type: Body.COLLIDE_EVENT_NAME, body: null, contact: null };
const World_step_oldContacts = [];
const World_step_frictionEquationPool = [];
const World_step_p1 = [];
const World_step_p2 = [];
World.prototype.emitContactEvents = (() => {
    const additions = [];
    const removals = [];
    const beginContactEvent = {
        type: "beginContact",
        bodyA: null,
        bodyB: null,
    };
    const endContactEvent = {
        type: "endContact",
        bodyA: null,
        bodyB: null,
    };
    const beginShapeContactEvent = {
        type: "beginShapeContact",
        bodyA: null,
        bodyB: null,
        shapeA: null,
        shapeB: null,
    };
    const endShapeContactEvent = {
        type: "endShapeContact",
        bodyA: null,
        bodyB: null,
        shapeA: null,
        shapeB: null,
    };
    return function () {
        const hasBeginContact = this.hasAnyEventListener("beginContact");
        const hasEndContact = this.hasAnyEventListener("endContact");
        if (hasBeginContact || hasEndContact) {
            this.bodyOverlapKeeper.getDiff(additions, removals);
        }
        if (hasBeginContact) {
            for (let i = 0, l = additions.length; i < l; i += 2) {
                beginContactEvent.bodyA = this.getBodyById(additions[i]);
                beginContactEvent.bodyB = this.getBodyById(additions[i + 1]);
                this.dispatchEvent(beginContactEvent);
            }
            beginContactEvent.bodyA = beginContactEvent.bodyB = null;
        }
        if (hasEndContact) {
            for (let i = 0, l = removals.length; i < l; i += 2) {
                endContactEvent.bodyA = this.getBodyById(removals[i]);
                endContactEvent.bodyB = this.getBodyById(removals[i + 1]);
                this.dispatchEvent(endContactEvent);
            }
            endContactEvent.bodyA = endContactEvent.bodyB = null;
        }
        additions.length = removals.length = 0;
        const hasBeginShapeContact = this.hasAnyEventListener("beginShapeContact");
        const hasEndShapeContact = this.hasAnyEventListener("endShapeContact");
        if (hasBeginShapeContact || hasEndShapeContact) {
            this.shapeOverlapKeeper.getDiff(additions, removals);
        }
        if (hasBeginShapeContact) {
            for (let i = 0, l = additions.length; i < l; i += 2) {
                const shapeA = this.getShapeById(additions[i]);
                const shapeB = this.getShapeById(additions[i + 1]);
                beginShapeContactEvent.shapeA = shapeA;
                beginShapeContactEvent.shapeB = shapeB;
                beginShapeContactEvent.bodyA = shapeA.body;
                beginShapeContactEvent.bodyB = shapeB.body;
                this.dispatchEvent(beginShapeContactEvent);
            }
            beginShapeContactEvent.bodyA = beginShapeContactEvent.bodyB =
                beginShapeContactEvent.shapeA = beginShapeContactEvent.shapeB = null;
        }
        if (hasEndShapeContact) {
            for (let i = 0, l = removals.length; i < l; i += 2) {
                const shapeA = this.getShapeById(removals[i]);
                const shapeB = this.getShapeById(removals[i + 1]);
                endShapeContactEvent.shapeA = shapeA;
                endShapeContactEvent.shapeB = shapeB;
                endShapeContactEvent.bodyA = shapeA.body;
                endShapeContactEvent.bodyB = shapeB.body;
                this.dispatchEvent(endShapeContactEvent);
            }
            endShapeContactEvent.bodyA = endShapeContactEvent.bodyB =
                endShapeContactEvent.shapeA = endShapeContactEvent.shapeB = null;
        }
    };
})();
