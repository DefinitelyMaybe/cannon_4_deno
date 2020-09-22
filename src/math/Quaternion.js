/// <reference lib="dom" />
import { Vec3 } from "../math/Vec3.js";
export class Quaternion {
    constructor(x = 0, y = 0, z = 0, w = 1) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }
    set(x, y, z, w) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
        return this;
    }
    toString() {
        return `${this.x},${this.y},${this.z},${this.w}`;
    }
    toArray() {
        return [this.x, this.y, this.z, this.w];
    }
    setFromAxisAngle(vector, angle) {
        const s = Math.sin(angle * 0.5);
        this.x = vector.x * s;
        this.y = vector.y * s;
        this.z = vector.z * s;
        this.w = Math.cos(angle * 0.5);
        return this;
    }
    toAxisAngle(targetAxis = new Vec3()) {
        this.normalize();
        const angle = 2 * Math.acos(this.w);
        const s = Math.sqrt(1 - this.w * this.w);
        if (s < 0.001) {
            targetAxis.x = this.x;
            targetAxis.y = this.y;
            targetAxis.z = this.z;
        }
        else {
            targetAxis.x = this.x / s;
            targetAxis.y = this.y / s;
            targetAxis.z = this.z / s;
        }
        return [targetAxis, angle];
    }
    setFromVectors(u, v) {
        if (u.isAntiparallelTo(v)) {
            const t1 = sfv_t1;
            const t2 = sfv_t2;
            u.tangents(t1, t2);
            this.setFromAxisAngle(t1, Math.PI);
        }
        else {
            const a = u.cross(v);
            this.x = a.x;
            this.y = a.y;
            this.z = a.z;
            this.w = Math.sqrt(u.length() ** 2 * v.length() ** 2) + u.dot(v);
            this.normalize();
        }
        return this;
    }
    mult(quat, target = new Quaternion()) {
        const ax = this.x;
        const ay = this.y;
        const az = this.z;
        const aw = this.w;
        const bx = quat.x;
        const by = quat.y;
        const bz = quat.z;
        const bw = quat.w;
        target.x = ax * bw + aw * bx + ay * bz - az * by;
        target.y = ay * bw + aw * by + az * bx - ax * bz;
        target.z = az * bw + aw * bz + ax * by - ay * bx;
        target.w = aw * bw - ax * bx - ay * by - az * bz;
        return target;
    }
    inverse(target = new Quaternion()) {
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const w = this.w;
        this.conjugate(target);
        const inorm2 = 1 / (x * x + y * y + z * z + w * w);
        target.x *= inorm2;
        target.y *= inorm2;
        target.z *= inorm2;
        target.w *= inorm2;
        return target;
    }
    conjugate(target = new Quaternion()) {
        target.x = -this.x;
        target.y = -this.y;
        target.z = -this.z;
        target.w = this.w;
        return target;
    }
    normalize() {
        let l = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z + this.w * this.w);
        if (l === 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        }
        else {
            l = 1 / l;
            this.x *= l;
            this.y *= l;
            this.z *= l;
            this.w *= l;
        }
        return this;
    }
    normalizeFast() {
        const f = (3.0 -
            (this.x * this.x + this.y * this.y + this.z * this.z +
                this.w * this.w)) / 2.0;
        if (f === 0) {
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.w = 0;
        }
        else {
            this.x *= f;
            this.y *= f;
            this.z *= f;
            this.w *= f;
        }
        return this;
    }
    vmult(v, target = new Vec3()) {
        const x = v.x;
        const y = v.y;
        const z = v.z;
        const qx = this.x;
        const qy = this.y;
        const qz = this.z;
        const qw = this.w;
        const ix = qw * x + qy * z - qz * y;
        const iy = qw * y + qz * x - qx * z;
        const iz = qw * z + qx * y - qy * x;
        const iw = -qx * x - qy * y - qz * z;
        target.x = ix * qw + iw * -qx + iy * -qz - iz * -qy;
        target.y = iy * qw + iw * -qy + iz * -qx - ix * -qz;
        target.z = iz * qw + iw * -qz + ix * -qy - iy * -qx;
        return target;
    }
    copy(quat) {
        this.x = quat.x;
        this.y = quat.y;
        this.z = quat.z;
        this.w = quat.w;
        return this;
    }
    toEuler(target, order = "YZX") {
        let heading;
        let attitude;
        let bank;
        const x = this.x;
        const y = this.y;
        const z = this.z;
        const w = this.w;
        switch (order) {
            case "YZX":
                const test = x * y + z * w;
                if (test > 0.499) {
                    heading = 2 * Math.atan2(x, w);
                    attitude = Math.PI / 2;
                    bank = 0;
                }
                if (test < -0.499) {
                    heading = -2 * Math.atan2(x, w);
                    attitude = -Math.PI / 2;
                    bank = 0;
                }
                if (heading === undefined) {
                    const sqx = x * x;
                    const sqy = y * y;
                    const sqz = z * z;
                    heading = Math.atan2(2 * y * w - 2 * x * z, 1 - 2 * sqy - 2 * sqz);
                    attitude = Math.asin(2 * test);
                    bank = Math.atan2(2 * x * w - 2 * y * z, 1 - 2 * sqx - 2 * sqz);
                }
                break;
            default:
                throw new Error(`Euler order ${order} not supported yet.`);
        }
        target.y = heading;
        target.z = attitude;
        target.x = bank;
    }
    setFromEuler(x, y, z, order = "XYZ") {
        const c1 = Math.cos(x / 2);
        const c2 = Math.cos(y / 2);
        const c3 = Math.cos(z / 2);
        const s1 = Math.sin(x / 2);
        const s2 = Math.sin(y / 2);
        const s3 = Math.sin(z / 2);
        if (order === "XYZ") {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        }
        else if (order === "YXZ") {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        else if (order === "ZXY") {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        }
        else if (order === "ZYX") {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        else if (order === "YZX") {
            this.x = s1 * c2 * c3 + c1 * s2 * s3;
            this.y = c1 * s2 * c3 + s1 * c2 * s3;
            this.z = c1 * c2 * s3 - s1 * s2 * c3;
            this.w = c1 * c2 * c3 - s1 * s2 * s3;
        }
        else if (order === "XZY") {
            this.x = s1 * c2 * c3 - c1 * s2 * s3;
            this.y = c1 * s2 * c3 - s1 * c2 * s3;
            this.z = c1 * c2 * s3 + s1 * s2 * c3;
            this.w = c1 * c2 * c3 + s1 * s2 * s3;
        }
        return this;
    }
    clone() {
        return new Quaternion(this.x, this.y, this.z, this.w);
    }
    slerp(toQuat, t, target = new Quaternion()) {
        const ax = this.x;
        const ay = this.y;
        const az = this.z;
        const aw = this.w;
        let bx = toQuat.x;
        let by = toQuat.y;
        let bz = toQuat.z;
        let bw = toQuat.w;
        let omega;
        let cosom;
        let sinom;
        let scale0;
        let scale1;
        cosom = ax * bx + ay * by + az * bz + aw * bw;
        if (cosom < 0.0) {
            cosom = -cosom;
            bx = -bx;
            by = -by;
            bz = -bz;
            bw = -bw;
        }
        if (1.0 - cosom > 0.000001) {
            omega = Math.acos(cosom);
            sinom = Math.sin(omega);
            scale0 = Math.sin((1.0 - t) * omega) / sinom;
            scale1 = Math.sin(t * omega) / sinom;
        }
        else {
            scale0 = 1.0 - t;
            scale1 = t;
        }
        target.x = scale0 * ax + scale1 * bx;
        target.y = scale0 * ay + scale1 * by;
        target.z = scale0 * az + scale1 * bz;
        target.w = scale0 * aw + scale1 * bw;
        return target;
    }
    integrate(angularVelocity, dt, angularFactor, target = new Quaternion()) {
        const ax = angularVelocity.x * angularFactor.x, ay = angularVelocity.y * angularFactor.y, az = angularVelocity.z * angularFactor.z, bx = this.x, by = this.y, bz = this.z, bw = this.w;
        const half_dt = dt * 0.5;
        target.x += half_dt * (ax * bw + ay * bz - az * by);
        target.y += half_dt * (ay * bw + az * bx - ax * bz);
        target.z += half_dt * (az * bw + ax * by - ay * bx);
        target.w += half_dt * (-ax * bx - ay * by - az * bz);
        return target;
    }
}
const sfv_t1 = new Vec3();
const sfv_t2 = new Vec3();
