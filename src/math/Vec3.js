/// <reference lib="dom" />
import { Mat3 } from "../math/Mat3.js";
export class Vec3 {
    constructor(x = 0.0, y = 0.0, z = 0.0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    cross(vector, target = new Vec3()) {
        const vx = vector.x;
        const vy = vector.y;
        const vz = vector.z;
        const x = this.x;
        const y = this.y;
        const z = this.z;
        target.x = y * vz - z * vy;
        target.y = z * vx - x * vz;
        target.z = x * vy - y * vx;
        return target;
    }
    set(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    setZero() {
        this.x = this.y = this.z = 0;
    }
    vadd(vector, target) {
        if (target) {
            target.x = vector.x + this.x;
            target.y = vector.y + this.y;
            target.z = vector.z + this.z;
        }
        else {
            return new Vec3(this.x + vector.x, this.y + vector.y, this.z + vector.z);
        }
    }
    vsub(vector, target) {
        if (target) {
            target.x = this.x - vector.x;
            target.y = this.y - vector.y;
            target.z = this.z - vector.z;
        }
        else {
            return new Vec3(this.x - vector.x, this.y - vector.y, this.z - vector.z);
        }
    }
    crossmat() {
        return new Mat3([0, -this.z, this.y, this.z, 0, -this.x, -this.y, this.x, 0]);
    }
    normalize() {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const n = Math.sqrt(x * x + y * y + z * z);
        if (n > 0.0) {
            const invN = 1 / n;
            this.x *= invN;
            this.y *= invN;
            this.z *= invN;
        }
        else {
            this.x = 0;
            this.y = 0;
            this.z = 0;
        }
        return n;
    }
    unit(target = new Vec3()) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        let ninv = Math.sqrt(x * x + y * y + z * z);
        if (ninv > 0.0) {
            ninv = 1.0 / ninv;
            target.x = x * ninv;
            target.y = y * ninv;
            target.z = z * ninv;
        }
        else {
            target.x = 1;
            target.y = 0;
            target.z = 0;
        }
        return target;
    }
    length() {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        return Math.sqrt(x * x + y * y + z * z);
    }
    lengthSquared() {
        return this.dot(this);
    }
    distanceTo(p) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const px = p.x;
        const py = p.y;
        const pz = p.z;
        return Math.sqrt((px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z));
    }
    distanceSquared(p) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const px = p.x;
        const py = p.y;
        const pz = p.z;
        return (px - x) * (px - x) + (py - y) * (py - y) + (pz - z) * (pz - z);
    }
    scale(scalar, target = new Vec3()) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        target.x = scalar * x;
        target.y = scalar * y;
        target.z = scalar * z;
        return target;
    }
    vmul(vector, target = new Vec3()) {
        target.x = vector.x * this.x;
        target.y = vector.y * this.y;
        target.z = vector.z * this.z;
        return target;
    }
    addScaledVector(scalar, vector, target = new Vec3()) {
        target.x = this.x + scalar * vector.x;
        target.y = this.y + scalar * vector.y;
        target.z = this.z + scalar * vector.z;
        return target;
    }
    dot(vector) {
        return this.x * vector.x + this.y * vector.y + this.z * vector.z;
    }
    isZero() {
        return this.x === 0 && this.y === 0 && this.z === 0;
    }
    negate(target = new Vec3()) {
        target.x = -this.x;
        target.y = -this.y;
        target.z = -this.z;
        return target;
    }
    tangents(t1, t2) {
        const norm = this.length();
        if (norm > 0.0) {
            const n = Vec3_tangents_n;
            const inorm = 1 / norm;
            n.set(this.x * inorm, this.y * inorm, this.z * inorm);
            const randVec = Vec3_tangents_randVec;
            if (Math.abs(n.x) < 0.9) {
                randVec.set(1, 0, 0);
                n.cross(randVec, t1);
            }
            else {
                randVec.set(0, 1, 0);
                n.cross(randVec, t1);
            }
            n.cross(t1, t2);
        }
        else {
            t1.set(1, 0, 0);
            t2.set(0, 1, 0);
        }
    }
    toString() {
        return `${this.x},${this.y},${this.z}`;
    }
    toArray() {
        return [this.x, this.y, this.z];
    }
    copy(vector) {
        this.x = vector.x;
        this.y = vector.y;
        this.z = vector.z;
        return this;
    }
    lerp(vector, t, target) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        target.x = x + (vector.x - x) * t;
        target.y = y + (vector.y - y) * t;
        target.z = z + (vector.z - z) * t;
    }
    almostEquals(vector, precision = 1e-6) {
        if (Math.abs(this.x - vector.x) > precision ||
            Math.abs(this.y - vector.y) > precision ||
            Math.abs(this.z - vector.z) > precision) {
            return false;
        }
        return true;
    }
    almostZero(precision = 1e-6) {
        if (Math.abs(this.x) > precision || Math.abs(this.y) > precision ||
            Math.abs(this.z) > precision) {
            return false;
        }
        return true;
    }
    isAntiparallelTo(vector, precision) {
        this.negate(antip_neg);
        return antip_neg.almostEquals(vector, precision);
    }
    clone() {
        return new Vec3(this.x, this.y, this.z);
    }
}
Vec3.ZERO = new Vec3(0, 0, 0);
Vec3.UNIT_X = new Vec3(1, 0, 0);
Vec3.UNIT_Y = new Vec3(0, 1, 0);
Vec3.UNIT_Z = new Vec3(0, 0, 1);
const Vec3_tangents_n = new Vec3();
const Vec3_tangents_randVec = new Vec3();
const antip_neg = new Vec3();
