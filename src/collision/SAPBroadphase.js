/// <reference types="./SAPBroadphase.ts" />
/// <reference lib="dom" />
import { Broadphase } from "../collision/Broadphase.js";
export class SAPBroadphase extends Broadphase {
    constructor(world) {
        super();
        this.axisList = [];
        this.world = null;
        this.axisIndex = 0;
        const axisList = this.axisList;
        this._addBodyHandler = (event) => {
            axisList.push(event.body);
        };
        this._removeBodyHandler = (event) => {
            const idx = axisList.indexOf(event.body);
            if (idx !== -1) {
                axisList.splice(idx, 1);
            }
        };
        if (world) {
            this.setWorld(world);
        }
    }
    setWorld(world) {
        this.axisList.length = 0;
        for (let i = 0; i < world.bodies.length; i++) {
            this.axisList.push(world.bodies[i]);
        }
        world.removeEventListener("addBody", this._addBodyHandler);
        world.removeEventListener("removeBody", this._removeBodyHandler);
        world.addEventListener("addBody", this._addBodyHandler);
        world.addEventListener("removeBody", this._removeBodyHandler);
        this.world = world;
        this.dirty = true;
    }
    collisionPairs(world, p1, p2) {
        const bodies = this.axisList;
        const N = bodies.length;
        const axisIndex = this.axisIndex;
        let i;
        let j;
        if (this.dirty) {
            this.sortList();
            this.dirty = false;
        }
        for (i = 0; i !== N; i++) {
            const bi = bodies[i];
            for (j = i + 1; j < N; j++) {
                const bj = bodies[j];
                if (!this.needBroadphaseCollision(bi, bj)) {
                    continue;
                }
                if (!SAPBroadphase.checkBounds(bi, bj, axisIndex)) {
                    break;
                }
                this.intersectionTest(bi, bj, p1, p2);
            }
        }
    }
    sortList() {
        const axisList = this.axisList;
        const axisIndex = this.axisIndex;
        const N = axisList.length;
        for (let i = 0; i !== N; i++) {
            const bi = axisList[i];
            if (bi.aabbNeedsUpdate) {
                bi.computeAABB();
            }
        }
        if (axisIndex === 0) {
            SAPBroadphase.insertionSortX(axisList);
        }
        else if (axisIndex === 1) {
            SAPBroadphase.insertionSortY(axisList);
        }
        else if (axisIndex === 2) {
            SAPBroadphase.insertionSortZ(axisList);
        }
    }
    autoDetectAxis() {
        let sumX = 0;
        let sumX2 = 0;
        let sumY = 0;
        let sumY2 = 0;
        let sumZ = 0;
        let sumZ2 = 0;
        const bodies = this.axisList;
        const N = bodies.length;
        const invN = 1 / N;
        for (let i = 0; i !== N; i++) {
            const b = bodies[i];
            const centerX = b.position.x;
            sumX += centerX;
            sumX2 += centerX * centerX;
            const centerY = b.position.y;
            sumY += centerY;
            sumY2 += centerY * centerY;
            const centerZ = b.position.z;
            sumZ += centerZ;
            sumZ2 += centerZ * centerZ;
        }
        const varianceX = sumX2 - sumX * sumX * invN;
        const varianceY = sumY2 - sumY * sumY * invN;
        const varianceZ = sumZ2 - sumZ * sumZ * invN;
        if (varianceX > varianceY) {
            if (varianceX > varianceZ) {
                this.axisIndex = 0;
            }
            else {
                this.axisIndex = 2;
            }
        }
        else if (varianceY > varianceZ) {
            this.axisIndex = 1;
        }
        else {
            this.axisIndex = 2;
        }
    }
    aabbQuery(world, aabb, result = []) {
        if (this.dirty) {
            this.sortList();
            this.dirty = false;
        }
        const axisIndex = this.axisIndex;
        let axis = "x";
        if (axisIndex === 1) {
            axis = "y";
        }
        if (axisIndex === 2) {
            axis = "z";
        }
        const axisList = this.axisList;
        const lower = aabb.lowerBound[axis];
        const upper = aabb.upperBound[axis];
        for (let i = 0; i < axisList.length; i++) {
            const b = axisList[i];
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
SAPBroadphase.insertionSortX = (a) => {
    for (let i = 1, l = a.length; i < l; i++) {
        const v = a[i];
        let j;
        for (j = i - 1; j >= 0; j--) {
            if (a[j].aabb.lowerBound.x <= v.aabb.lowerBound.x) {
                break;
            }
            a[j + 1] = a[j];
        }
        a[j + 1] = v;
    }
    return a;
};
SAPBroadphase.insertionSortY = (a) => {
    for (let i = 1, l = a.length; i < l; i++) {
        const v = a[i];
        let j;
        for (j = i - 1; j >= 0; j--) {
            if (a[j].aabb.lowerBound.y <= v.aabb.lowerBound.y) {
                break;
            }
            a[j + 1] = a[j];
        }
        a[j + 1] = v;
    }
    return a;
};
SAPBroadphase.insertionSortZ = (a) => {
    for (let i = 1, l = a.length; i < l; i++) {
        const v = a[i];
        let j;
        for (j = i - 1; j >= 0; j--) {
            if (a[j].aabb.lowerBound.z <= v.aabb.lowerBound.z) {
                break;
            }
            a[j + 1] = a[j];
        }
        a[j + 1] = v;
    }
    return a;
};
SAPBroadphase.checkBounds = (bi, bj, axisIndex) => {
    let biPos;
    let bjPos;
    if (axisIndex === 0) {
        biPos = bi.position.x;
        bjPos = bj.position.x;
    }
    else if (axisIndex === 1) {
        biPos = bi.position.y;
        bjPos = bj.position.y;
    }
    else if (axisIndex === 2) {
        biPos = bi.position.z;
        bjPos = bj.position.z;
    }
    const ri = bi.boundingRadius, rj = bj.boundingRadius, boundA2 = biPos + ri, boundB1 = bjPos - rj;
    return boundB1 < boundA2;
};
