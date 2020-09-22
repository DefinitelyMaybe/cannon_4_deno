/// <reference lib="dom" />
import { Broadphase } from "../collision/Broadphase.js";
export class NaiveBroadphase extends Broadphase {
    constructor() {
        super();
    }
    collisionPairs(world, pairs1, pairs2) {
        const bodies = world.bodies;
        const n = bodies.length;
        let bi;
        let bj;
        for (let i = 0; i !== n; i++) {
            for (let j = 0; j !== i; j++) {
                bi = bodies[i];
                bj = bodies[j];
                if (!this.needBroadphaseCollision(bi, bj)) {
                    continue;
                }
                this.intersectionTest(bi, bj, pairs1, pairs2);
            }
        }
    }
    aabbQuery(world, aabb, result = []) {
        for (let i = 0; i < world.bodies.length; i++) {
            const b = world.bodies[i];
            if (b.aabbNeedsUpdate) {
                b.computeAABB();
            }
            if (b.aabb.overlaps(aabb)) {
                result.push(b);
            }
        }
        return result;
    }
}
