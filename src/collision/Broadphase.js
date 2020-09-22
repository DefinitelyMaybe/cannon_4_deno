/// <reference lib="dom" />
import { Body } from "../objects/Body.js";
import { Vec3 } from "../math/Vec3.js";
import { Quaternion } from "../math/Quaternion.js";
export class Broadphase {
    constructor() {
        this.world = null;
        this.useBoundingBoxes = false;
        this.dirty = true;
    }
    collisionPairs(world, p1, p2) {
        throw new Error("collisionPairs not implemented for this BroadPhase class!");
    }
    needBroadphaseCollision(bodyA, bodyB) {
        if ((bodyA.collisionFilterGroup & bodyB.collisionFilterMask) === 0 ||
            (bodyB.collisionFilterGroup & bodyA.collisionFilterMask) === 0) {
            return false;
        }
        if (((bodyA.type & Body.STATIC) !== 0 ||
            bodyA.sleepState === Body.SLEEPING) &&
            ((bodyB.type & Body.STATIC) !== 0 || bodyB.sleepState === Body.SLEEPING)) {
            return false;
        }
        return true;
    }
    intersectionTest(bodyA, bodyB, pairs1, pairs2) {
        if (this.useBoundingBoxes) {
            this.doBoundingBoxBroadphase(bodyA, bodyB, pairs1, pairs2);
        }
        else {
            this.doBoundingSphereBroadphase(bodyA, bodyB, pairs1, pairs2);
        }
    }
    doBoundingSphereBroadphase(bodyA, bodyB, pairs1, pairs2) {
        const r = Broadphase_collisionPairs_r;
        bodyB.position.vsub(bodyA.position, r);
        const boundingRadiusSum2 = (bodyA.boundingRadius + bodyB.boundingRadius) **
            2;
        const norm2 = r.lengthSquared();
        if (norm2 < boundingRadiusSum2) {
            pairs1.push(bodyA);
            pairs2.push(bodyB);
        }
    }
    doBoundingBoxBroadphase(bodyA, bodyB, pairs1, pairs2) {
        if (bodyA.aabbNeedsUpdate) {
            bodyA.computeAABB();
        }
        if (bodyB.aabbNeedsUpdate) {
            bodyB.computeAABB();
        }
        if (bodyA.aabb.overlaps(bodyB.aabb)) {
            pairs1.push(bodyA);
            pairs2.push(bodyB);
        }
    }
    makePairsUnique(pairs1, pairs2) {
        const t = Broadphase_makePairsUnique_temp;
        const p1 = Broadphase_makePairsUnique_p1;
        const p2 = Broadphase_makePairsUnique_p2;
        const N = pairs1.length;
        for (let i = 0; i !== N; i++) {
            p1[i] = pairs1[i];
            p2[i] = pairs2[i];
        }
        pairs1.length = 0;
        pairs2.length = 0;
        for (let i = 0; i !== N; i++) {
            const id1 = p1[i].id;
            const id2 = p2[i].id;
            const key = id1 < id2 ? `${id1},${id2}` : `${id2},${id1}`;
            t[key] = i;
            t.keys.push(key);
        }
        for (let i = 0; i !== t.keys.length; i++) {
            const key = t.keys.pop();
            const pairIndex = t[key];
            pairs1.push(p1[pairIndex]);
            pairs2.push(p2[pairIndex]);
            delete t[key];
        }
    }
    setWorld(world) { }
    aabbQuery(world, aabb, result) {
        console.warn(".aabbQuery is not implemented in this Broadphase subclass.");
        return [];
    }
}
const Broadphase_collisionPairs_r = new Vec3();
const Broadphase_collisionPairs_normal = new Vec3();
const Broadphase_collisionPairs_quat = new Quaternion();
const Broadphase_collisionPairs_relpos = new Vec3();
const Broadphase_makePairsUnique_temp = { keys: [] };
const Broadphase_makePairsUnique_p1 = [];
const Broadphase_makePairsUnique_p2 = [];
const bsc_dist = new Vec3();
Broadphase.boundingSphereCheck = (bodyA, bodyB) => {
    const dist = new Vec3();
    bodyA.position.vsub(bodyB.position, dist);
    const sa = bodyA.shapes[0];
    const sb = bodyB.shapes[0];
    return Math.pow(sa.boundingSphereRadius + sb.boundingSphereRadius, 2) >
        dist.lengthSquared();
};
