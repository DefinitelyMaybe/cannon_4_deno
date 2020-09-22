/// <reference lib="dom" />
import { AABB } from "../collision/AABB.js";
import { Vec3 } from "../math/Vec3.js";
class OctreeNode {
    constructor(options = {}) {
        this.root = options.root || null;
        this.aabb = options.aabb ? options.aabb.clone() : new AABB();
        this.data = [];
        this.children = [];
    }
    reset() {
        this.children.length = this.data.length = 0;
    }
    insert(aabb, elementData, level = 0) {
        const nodeData = this.data;
        if (!this.aabb.contains(aabb)) {
            return false;
        }
        const children = this.children;
        const maxDepth = this.maxDepth || this.root.maxDepth;
        if (level < maxDepth) {
            let subdivided = false;
            if (!children.length) {
                this.subdivide();
                subdivided = true;
            }
            for (let i = 0; i !== 8; i++) {
                if (children[i].insert(aabb, elementData, level + 1)) {
                    return true;
                }
            }
            if (subdivided) {
                children.length = 0;
            }
        }
        nodeData.push(elementData);
        return true;
    }
    subdivide() {
        const aabb = this.aabb;
        const l = aabb.lowerBound;
        const u = aabb.upperBound;
        const children = this.children;
        children.push(new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(0, 0, 0) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(1, 0, 0) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(1, 1, 0) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(1, 1, 1) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(0, 1, 1) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(0, 0, 1) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(1, 0, 1) }) }), new OctreeNode({ aabb: new AABB({ lowerBound: new Vec3(0, 1, 0) }) }));
        u.vsub(l, halfDiagonal);
        halfDiagonal.scale(0.5, halfDiagonal);
        const root = this.root || this;
        for (let i = 0; i !== 8; i++) {
            const child = children[i];
            child.root = root;
            const lowerBound = child.aabb.lowerBound;
            lowerBound.x *= halfDiagonal.x;
            lowerBound.y *= halfDiagonal.y;
            lowerBound.z *= halfDiagonal.z;
            lowerBound.vadd(l, lowerBound);
            lowerBound.vadd(halfDiagonal, child.aabb.upperBound);
        }
    }
    aabbQuery(aabb, result) {
        const nodeData = this.data;
        const children = this.children;
        const queue = [this];
        while (queue.length) {
            const node = queue.pop();
            if (node.aabb.overlaps(aabb)) {
                Array.prototype.push.apply(result, node.data);
            }
            Array.prototype.push.apply(queue, node.children);
        }
        return result;
    }
    rayQuery(ray, treeTransform, result) {
        ray.getAABB(tmpAABB);
        tmpAABB.toLocalFrame(treeTransform, tmpAABB);
        this.aabbQuery(tmpAABB, result);
        return result;
    }
    removeEmptyNodes() {
        for (let i = this.children.length - 1; i >= 0; i--) {
            this.children[i].removeEmptyNodes();
            if (!this.children[i].children.length && !this.children[i].data.length) {
                this.children.splice(i, 1);
            }
        }
    }
}
export class Octree extends OctreeNode {
    constructor(aabb, options = {}) {
        super({ root: null, aabb });
        this.maxDepth = typeof options.maxDepth !== "undefined"
            ? options.maxDepth
            : 8;
    }
}
const halfDiagonal = new Vec3();
const tmpAABB = new AABB();
