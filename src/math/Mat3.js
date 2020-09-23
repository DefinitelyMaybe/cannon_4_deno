/// <reference types="./Mat3.ts" />
import { Vec3 } from "../math/Vec3.js";
export class Mat3 {
    constructor(elements = [0, 0, 0, 0, 0, 0, 0, 0, 0]) {
        this.elements = elements;
    }
    identity() {
        const e = this.elements;
        e[0] = 1;
        e[1] = 0;
        e[2] = 0;
        e[3] = 0;
        e[4] = 1;
        e[5] = 0;
        e[6] = 0;
        e[7] = 0;
        e[8] = 1;
    }
    setZero() {
        const e = this.elements;
        e[0] = 0;
        e[1] = 0;
        e[2] = 0;
        e[3] = 0;
        e[4] = 0;
        e[5] = 0;
        e[6] = 0;
        e[7] = 0;
        e[8] = 0;
    }
    setTrace(vector) {
        const e = this.elements;
        e[0] = vector.x;
        e[4] = vector.y;
        e[8] = vector.z;
    }
    getTrace(target = new Vec3()) {
        const e = this.elements;
        target.x = e[0];
        target.y = e[4];
        target.z = e[8];
    }
    vmult(v, target = new Vec3()) {
        const e = this.elements;
        const x = v.x;
        const y = v.y;
        const z = v.z;
        target.x = e[0] * x + e[1] * y + e[2] * z;
        target.y = e[3] * x + e[4] * y + e[5] * z;
        target.z = e[6] * x + e[7] * y + e[8] * z;
        return target;
    }
    smult(s) {
        for (let i = 0; i < this.elements.length; i++) {
            this.elements[i] *= s;
        }
    }
    mmult(matrix, target = new Mat3()) {
        const { elements } = matrix;
        for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 3; j++) {
                let sum = 0.0;
                for (let k = 0; k < 3; k++) {
                    sum += elements[i + k * 3] * this.elements[k + j * 3];
                }
                target.elements[i + j * 3] = sum;
            }
        }
        return target;
    }
    scale(vector, target = new Mat3()) {
        const e = this.elements;
        const t = target.elements;
        for (let i = 0; i !== 3; i++) {
            t[3 * i + 0] = vector.x * e[3 * i + 0];
            t[3 * i + 1] = vector.y * e[3 * i + 1];
            t[3 * i + 2] = vector.z * e[3 * i + 2];
        }
        return target;
    }
    solve(b, target = new Vec3()) {
        const nr = 3;
        const nc = 4;
        const eqns = [];
        let i;
        let j;
        for (i = 0; i < nr * nc; i++) {
            eqns.push(0);
        }
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                eqns[i + nc * j] = this.elements[i + 3 * j];
            }
        }
        eqns[3 + 4 * 0] = b.x;
        eqns[3 + 4 * 1] = b.y;
        eqns[3 + 4 * 2] = b.z;
        let n = 3;
        const k = n;
        let np;
        const kp = 4;
        let p;
        do {
            i = k - n;
            if (eqns[i + nc * i] === 0) {
                for (j = i + 1; j < k; j++) {
                    if (eqns[i + nc * j] !== 0) {
                        np = kp;
                        do {
                            p = kp - np;
                            eqns[p + nc * i] += eqns[p + nc * j];
                        } while (--np);
                        break;
                    }
                }
            }
            if (eqns[i + nc * i] !== 0) {
                for (j = i + 1; j < k; j++) {
                    const multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                    np = kp;
                    do {
                        p = kp - np;
                        eqns[p + nc * j] = p <= i
                            ? 0
                            : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                    } while (--np);
                }
            }
        } while (--n);
        target.z = eqns[2 * nc + 3] / eqns[2 * nc + 2];
        target.y = (eqns[1 * nc + 3] - eqns[1 * nc + 2] * target.z) /
            eqns[1 * nc + 1];
        target.x = (eqns[0 * nc + 3] - eqns[0 * nc + 2] * target.z -
            eqns[0 * nc + 1] * target.y) / eqns[0 * nc + 0];
        if (isNaN(target.x) ||
            isNaN(target.y) ||
            isNaN(target.z) ||
            target.x === Infinity ||
            target.y === Infinity ||
            target.z === Infinity) {
            throw `Could not solve equation! Got x=[${target.toString()}], b=[${b.toString()}], A=[${this.toString()}]`;
        }
        return target;
    }
    e(row, column, value) {
        if (value === undefined) {
            return this.elements[column + 3 * row];
        }
        else {
            this.elements[column + 3 * row] = value;
        }
    }
    copy(matrix) {
        for (let i = 0; i < matrix.elements.length; i++) {
            this.elements[i] = matrix.elements[i];
        }
        return this;
    }
    toString() {
        let r = "";
        const sep = ",";
        for (let i = 0; i < 9; i++) {
            r += this.elements[i] + sep;
        }
        return r;
    }
    reverse(target = new Mat3()) {
        const nr = 3;
        const nc = 6;
        const eqns = [];
        let i;
        let j;
        for (i = 0; i < nr * nc; i++) {
            eqns.push(0);
        }
        for (i = 0; i < 3; i++) {
            for (j = 0; j < 3; j++) {
                eqns[i + nc * j] = this.elements[i + 3 * j];
            }
        }
        eqns[3 + 6 * 0] = 1;
        eqns[3 + 6 * 1] = 0;
        eqns[3 + 6 * 2] = 0;
        eqns[4 + 6 * 0] = 0;
        eqns[4 + 6 * 1] = 1;
        eqns[4 + 6 * 2] = 0;
        eqns[5 + 6 * 0] = 0;
        eqns[5 + 6 * 1] = 0;
        eqns[5 + 6 * 2] = 1;
        let n = 3;
        const k = n;
        let np;
        const kp = nc;
        let p;
        do {
            i = k - n;
            if (eqns[i + nc * i] === 0) {
                for (j = i + 1; j < k; j++) {
                    if (eqns[i + nc * j] !== 0) {
                        np = kp;
                        do {
                            p = kp - np;
                            eqns[p + nc * i] += eqns[p + nc * j];
                        } while (--np);
                        break;
                    }
                }
            }
            if (eqns[i + nc * i] !== 0) {
                for (j = i + 1; j < k; j++) {
                    const multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                    np = kp;
                    do {
                        p = kp - np;
                        eqns[p + nc * j] = p <= i
                            ? 0
                            : eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                    } while (--np);
                }
            }
        } while (--n);
        i = 2;
        do {
            j = i - 1;
            do {
                const multiplier = eqns[i + nc * j] / eqns[i + nc * i];
                np = nc;
                do {
                    p = nc - np;
                    eqns[p + nc * j] = eqns[p + nc * j] - eqns[p + nc * i] * multiplier;
                } while (--np);
            } while (j--);
        } while (--i);
        i = 2;
        do {
            const multiplier = 1 / eqns[i + nc * i];
            np = nc;
            do {
                p = nc - np;
                eqns[p + nc * i] = eqns[p + nc * i] * multiplier;
            } while (--np);
        } while (i--);
        i = 2;
        do {
            j = 2;
            do {
                p = eqns[nr + j + nc * i];
                if (isNaN(p) || p === Infinity) {
                    throw `Could not reverse! A=[${this.toString()}]`;
                }
                target.e(i, j, p);
            } while (j--);
        } while (i--);
        return target;
    }
    setRotationFromQuaternion(q) {
        const x = q.x;
        const y = q.y;
        const z = q.z;
        const w = q.w;
        const x2 = x + x;
        const y2 = y + y;
        const z2 = z + z;
        const xx = x * x2;
        const xy = x * y2;
        const xz = x * z2;
        const yy = y * y2;
        const yz = y * z2;
        const zz = z * z2;
        const wx = w * x2;
        const wy = w * y2;
        const wz = w * z2;
        const e = this.elements;
        e[3 * 0 + 0] = 1 - (yy + zz);
        e[3 * 0 + 1] = xy - wz;
        e[3 * 0 + 2] = xz + wy;
        e[3 * 1 + 0] = xy + wz;
        e[3 * 1 + 1] = 1 - (xx + zz);
        e[3 * 1 + 2] = yz - wx;
        e[3 * 2 + 0] = xz - wy;
        e[3 * 2 + 1] = yz + wx;
        e[3 * 2 + 2] = 1 - (xx + yy);
        return this;
    }
    transpose(target = new Mat3()) {
        const Mt = target.elements;
        const M = this.elements;
        for (let i = 0; i !== 3; i++) {
            for (let j = 0; j !== 3; j++) {
                Mt[3 * i + j] = M[3 * j + i];
            }
        }
        return target;
    }
}
