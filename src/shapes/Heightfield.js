/// <reference lib="dom" />
import { Shape } from "../shapes/Shape.js";
import { ConvexPolyhedron } from "../shapes/ConvexPolyhedron.js";
import { Vec3 } from "../math/Vec3.js";
import { Utils } from "../utils/Utils.js";
export class Heightfield extends Shape {
    constructor(data, options = {}) {
        options = Utils.defaults(options, {
            maxValue: null,
            minValue: null,
            elementSize: 1,
        });
        super({ type: Shape.types.HEIGHTFIELD });
        this.data = data;
        this.maxValue = options.maxValue;
        this.minValue = options.minValue;
        this.elementSize = options.elementSize;
        if (options.minValue === null) {
            this.updateMinValue();
        }
        if (options.maxValue === null) {
            this.updateMaxValue();
        }
        this.cacheEnabled = true;
        this.pillarConvex = new ConvexPolyhedron();
        this.pillarOffset = new Vec3();
        this.updateBoundingSphereRadius();
        this._cachedPillars = {};
    }
    update() {
        this._cachedPillars = {};
    }
    updateMinValue() {
        const data = this.data;
        let minValue = data[0][0];
        for (let i = 0; i !== data.length; i++) {
            for (let j = 0; j !== data[i].length; j++) {
                const v = data[i][j];
                if (v < minValue) {
                    minValue = v;
                }
            }
        }
        this.minValue = minValue;
    }
    updateMaxValue() {
        const data = this.data;
        let maxValue = data[0][0];
        for (let i = 0; i !== data.length; i++) {
            for (let j = 0; j !== data[i].length; j++) {
                const v = data[i][j];
                if (v > maxValue) {
                    maxValue = v;
                }
            }
        }
        this.maxValue = maxValue;
    }
    setHeightValueAtIndex(xi, yi, value) {
        const data = this.data;
        data[xi][yi] = value;
        this.clearCachedConvexTrianglePillar(xi, yi, false);
        if (xi > 0) {
            this.clearCachedConvexTrianglePillar(xi - 1, yi, true);
            this.clearCachedConvexTrianglePillar(xi - 1, yi, false);
        }
        if (yi > 0) {
            this.clearCachedConvexTrianglePillar(xi, yi - 1, true);
            this.clearCachedConvexTrianglePillar(xi, yi - 1, false);
        }
        if (yi > 0 && xi > 0) {
            this.clearCachedConvexTrianglePillar(xi - 1, yi - 1, true);
        }
    }
    getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, result = []) {
        const data = this.data;
        let max = this.minValue;
        for (let i = iMinX; i <= iMaxX; i++) {
            for (let j = iMinY; j <= iMaxY; j++) {
                const height = data[i][j];
                if (height > max) {
                    max = height;
                }
            }
        }
        result[0] = this.minValue;
        result[1] = max;
    }
    getIndexOfPosition(x, y, result, clamp) {
        const w = this.elementSize;
        const data = this.data;
        let xi = Math.floor(x / w);
        let yi = Math.floor(y / w);
        result[0] = xi;
        result[1] = yi;
        if (clamp) {
            if (xi < 0) {
                xi = 0;
            }
            if (yi < 0) {
                yi = 0;
            }
            if (xi >= data.length - 1) {
                xi = data.length - 1;
            }
            if (yi >= data[0].length - 1) {
                yi = data[0].length - 1;
            }
        }
        if (xi < 0 || yi < 0 || xi >= data.length - 1 || yi >= data[0].length - 1) {
            return false;
        }
        return true;
    }
    getTriangleAt(x, y, edgeClamp, a, b, c) {
        const idx = getHeightAt_idx;
        this.getIndexOfPosition(x, y, idx, edgeClamp);
        let xi = idx[0];
        let yi = idx[1];
        const data = this.data;
        if (edgeClamp) {
            xi = Math.min(data.length - 2, Math.max(0, xi));
            yi = Math.min(data[0].length - 2, Math.max(0, yi));
        }
        const elementSize = this.elementSize;
        const lowerDist2 = (x / elementSize - xi) ** 2 +
            (y / elementSize - yi) ** 2;
        const upperDist2 = (x / elementSize - (xi + 1)) ** 2 +
            (y / elementSize - (yi + 1)) ** 2;
        const upper = lowerDist2 > upperDist2;
        this.getTriangle(xi, yi, upper, a, b, c);
        return upper;
    }
    getNormalAt(x, y, edgeClamp, result) {
        const a = getNormalAt_a;
        const b = getNormalAt_b;
        const c = getNormalAt_c;
        const e0 = getNormalAt_e0;
        const e1 = getNormalAt_e1;
        this.getTriangleAt(x, y, edgeClamp, a, b, c);
        b.vsub(a, e0);
        c.vsub(a, e1);
        e0.cross(e1, result);
        result.normalize();
    }
    getAabbAtIndex(xi, yi, { lowerBound, upperBound }) {
        const data = this.data;
        const elementSize = this.elementSize;
        lowerBound.set(xi * elementSize, yi * elementSize, data[xi][yi]);
        upperBound.set((xi + 1) * elementSize, (yi + 1) * elementSize, data[xi + 1][yi + 1]);
    }
    getHeightAt(x, y, edgeClamp) {
        const data = this.data;
        const a = getHeightAt_a;
        const b = getHeightAt_b;
        const c = getHeightAt_c;
        const idx = getHeightAt_idx;
        this.getIndexOfPosition(x, y, idx, edgeClamp);
        let xi = idx[0];
        let yi = idx[1];
        if (edgeClamp) {
            xi = Math.min(data.length - 2, Math.max(0, xi));
            yi = Math.min(data[0].length - 2, Math.max(0, yi));
        }
        const upper = this.getTriangleAt(x, y, edgeClamp, a, b, c);
        barycentricWeights(x, y, a.x, a.y, b.x, b.y, c.x, c.y, getHeightAt_weights);
        const w = getHeightAt_weights;
        if (upper) {
            return data[xi + 1][yi + 1] * w.x + data[xi][yi + 1] * w.y +
                data[xi + 1][yi] * w.z;
        }
        else {
            return data[xi][yi] * w.x + data[xi + 1][yi] * w.y +
                data[xi][yi + 1] * w.z;
        }
    }
    getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle) {
        return `${xi}_${yi}_${getUpperTriangle ? 1 : 0}`;
    }
    getCachedConvexTrianglePillar(xi, yi, getUpperTriangle) {
        return this
            ._cachedPillars[this.getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle)];
    }
    setCachedConvexTrianglePillar(xi, yi, getUpperTriangle, convex, offset) {
        this
            ._cachedPillars[this.getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle)] = {
            convex,
            offset,
        };
    }
    clearCachedConvexTrianglePillar(xi, yi, getUpperTriangle) {
        delete this
            ._cachedPillars[this.getCacheConvexTrianglePillarKey(xi, yi, getUpperTriangle)];
    }
    getTriangle(xi, yi, upper, a, b, c) {
        const data = this.data;
        const elementSize = this.elementSize;
        if (upper) {
            a.set((xi + 1) * elementSize, (yi + 1) * elementSize, data[xi + 1][yi + 1]);
            b.set(xi * elementSize, (yi + 1) * elementSize, data[xi][yi + 1]);
            c.set((xi + 1) * elementSize, yi * elementSize, data[xi + 1][yi]);
        }
        else {
            a.set(xi * elementSize, yi * elementSize, data[xi][yi]);
            b.set((xi + 1) * elementSize, yi * elementSize, data[xi + 1][yi]);
            c.set(xi * elementSize, (yi + 1) * elementSize, data[xi][yi + 1]);
        }
    }
    getConvexTrianglePillar(xi, yi, getUpperTriangle) {
        let result = this.pillarConvex;
        let offsetResult = this.pillarOffset;
        if (this.cacheEnabled) {
            const data = this.getCachedConvexTrianglePillar(xi, yi, getUpperTriangle);
            if (data) {
                this.pillarConvex = data.convex;
                this.pillarOffset = data.offset;
                return;
            }
            result = new ConvexPolyhedron();
            offsetResult = new Vec3();
            this.pillarConvex = result;
            this.pillarOffset = offsetResult;
        }
        const data = this.data;
        const elementSize = this.elementSize;
        const faces = result.faces;
        result.vertices.length = 6;
        for (let i = 0; i < 6; i++) {
            if (!result.vertices[i]) {
                result.vertices[i] = new Vec3();
            }
        }
        faces.length = 5;
        for (let i = 0; i < 5; i++) {
            if (!faces[i]) {
                faces[i] = [];
            }
        }
        const verts = result.vertices;
        const h = (Math.min(data[xi][yi], data[xi + 1][yi], data[xi][yi + 1], data[xi + 1][yi + 1]) - this.minValue) / 2 +
            this.minValue;
        if (!getUpperTriangle) {
            offsetResult.set((xi + 0.25) * elementSize, (yi + 0.25) * elementSize, h);
            verts[0].set(-0.25 * elementSize, -0.25 * elementSize, data[xi][yi] - h);
            verts[1].set(0.75 * elementSize, -0.25 * elementSize, data[xi + 1][yi] - h);
            verts[2].set(-0.25 * elementSize, 0.75 * elementSize, data[xi][yi + 1] - h);
            verts[3].set(-0.25 * elementSize, -0.25 * elementSize, -Math.abs(h) - 1);
            verts[4].set(0.75 * elementSize, -0.25 * elementSize, -Math.abs(h) - 1);
            verts[5].set(-0.25 * elementSize, 0.75 * elementSize, -Math.abs(h) - 1);
            faces[0][0] = 0;
            faces[0][1] = 1;
            faces[0][2] = 2;
            faces[1][0] = 5;
            faces[1][1] = 4;
            faces[1][2] = 3;
            faces[2][0] = 0;
            faces[2][1] = 2;
            faces[2][2] = 5;
            faces[2][3] = 3;
            faces[3][0] = 1;
            faces[3][1] = 0;
            faces[3][2] = 3;
            faces[3][3] = 4;
            faces[4][0] = 4;
            faces[4][1] = 5;
            faces[4][2] = 2;
            faces[4][3] = 1;
        }
        else {
            offsetResult.set((xi + 0.75) * elementSize, (yi + 0.75) * elementSize, h);
            verts[0].set(0.25 * elementSize, 0.25 * elementSize, data[xi + 1][yi + 1] - h);
            verts[1].set(-0.75 * elementSize, 0.25 * elementSize, data[xi][yi + 1] - h);
            verts[2].set(0.25 * elementSize, -0.75 * elementSize, data[xi + 1][yi] - h);
            verts[3].set(0.25 * elementSize, 0.25 * elementSize, -Math.abs(h) - 1);
            verts[4].set(-0.75 * elementSize, 0.25 * elementSize, -Math.abs(h) - 1);
            verts[5].set(0.25 * elementSize, -0.75 * elementSize, -Math.abs(h) - 1);
            faces[0][0] = 0;
            faces[0][1] = 1;
            faces[0][2] = 2;
            faces[1][0] = 5;
            faces[1][1] = 4;
            faces[1][2] = 3;
            faces[2][0] = 2;
            faces[2][1] = 5;
            faces[2][2] = 3;
            faces[2][3] = 0;
            faces[3][0] = 3;
            faces[3][1] = 4;
            faces[3][2] = 1;
            faces[3][3] = 0;
            faces[4][0] = 1;
            faces[4][1] = 4;
            faces[4][2] = 5;
            faces[4][3] = 2;
        }
        result.computeNormals();
        result.computeEdges();
        result.updateBoundingSphereRadius();
        this.setCachedConvexTrianglePillar(xi, yi, getUpperTriangle, result, offsetResult);
    }
    calculateLocalInertia(mass, target = new Vec3()) {
        target.set(0, 0, 0);
        return target;
    }
    volume() {
        return (Number.MAX_VALUE);
    }
    calculateWorldAABB(pos, quat, min, max) {
        min.set(-Number.MAX_VALUE, -Number.MAX_VALUE, -Number.MAX_VALUE);
        max.set(Number.MAX_VALUE, Number.MAX_VALUE, Number.MAX_VALUE);
    }
    updateBoundingSphereRadius() {
        const data = this.data;
        const s = this.elementSize;
        this.boundingSphereRadius = new Vec3(data.length * s, data[0].length * s, Math.max(Math.abs(this.maxValue), Math.abs(this.minValue))).length();
    }
    setHeightsFromImage(image, scale) {
        const { x, z, y } = scale;
        const canvas = document.createElement("canvas");
        canvas.width = image.width;
        canvas.height = image.height;
        const context = canvas.getContext("2d");
        context.drawImage(image, 0, 0);
        const imageData = context.getImageData(0, 0, image.width, image.height);
        const matrix = this.data;
        matrix.length = 0;
        this.elementSize = Math.abs(x) / imageData.width;
        for (let i = 0; i < imageData.height; i++) {
            const row = [];
            for (let j = 0; j < imageData.width; j++) {
                const a = imageData.data[(i * imageData.height + j) * 4];
                const b = imageData.data[(i * imageData.height + j) * 4 + 1];
                const c = imageData.data[(i * imageData.height + j) * 4 + 2];
                const height = ((a + b + c) / 4 / 255) * z;
                if (x < 0) {
                    row.push(height);
                }
                else {
                    row.unshift(height);
                }
            }
            if (y < 0) {
                matrix.unshift(row);
            }
            else {
                matrix.push(row);
            }
        }
        this.updateMaxValue();
        this.updateMinValue();
        this.update();
    }
}
const getHeightAt_idx = [];
const getHeightAt_weights = new Vec3();
const getHeightAt_a = new Vec3();
const getHeightAt_b = new Vec3();
const getHeightAt_c = new Vec3();
const getNormalAt_a = new Vec3();
const getNormalAt_b = new Vec3();
const getNormalAt_c = new Vec3();
const getNormalAt_e0 = new Vec3();
const getNormalAt_e1 = new Vec3();
function barycentricWeights(x, y, ax, ay, bx, by, cx, cy, result) {
    result.x = ((by - cy) * (x - cx) + (cx - bx) * (y - cy)) /
        ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
    result.y = ((cy - ay) * (x - cx) + (ax - cx) * (y - cy)) /
        ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
    result.z = 1 - result.x - result.y;
}
