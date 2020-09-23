/// <reference types="./Transform.ts" />
import { Vec3 } from "../math/Vec3.js";
import { Quaternion } from "../math/Quaternion.js";
export class Transform {
    constructor(options = {}) {
        this.position = new Vec3();
        this.quaternion = new Quaternion();
        if (options.position) {
            this.position.copy(options.position);
        }
        if (options.quaternion) {
            this.quaternion.copy(options.quaternion);
        }
    }
    pointToLocal(worldPoint, result) {
        return Transform.pointToLocalFrame(this.position, this.quaternion, worldPoint, result);
    }
    pointToWorld(localPoint, result) {
        return Transform.pointToWorldFrame(this.position, this.quaternion, localPoint, result);
    }
    vectorToWorldFrame(localVector, result = new Vec3()) {
        this.quaternion.vmult(localVector, result);
        return result;
    }
    static pointToLocalFrame(position, quaternion, worldPoint, result = new Vec3()) {
        worldPoint.vsub(position, result);
        quaternion.conjugate(tmpQuat);
        tmpQuat.vmult(result, result);
        return result;
    }
    static pointToWorldFrame(position, quaternion, localPoint, result = new Vec3()) {
        quaternion.vmult(localPoint, result);
        result.vadd(position, result);
        return result;
    }
    static vectorToWorldFrame(quaternion, localVector, result = new Vec3()) {
        quaternion.vmult(localVector, result);
        return result;
    }
    static vectorToLocalFrame(position, quaternion, worldVector, result = new Vec3()) {
        quaternion.w *= -1;
        quaternion.vmult(worldVector, result);
        quaternion.w *= -1;
        return result;
    }
}
const tmpQuat = new Quaternion();
