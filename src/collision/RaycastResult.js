/// <reference types="./RaycastResult.ts" />
import { Vec3 } from "../math/Vec3.js";
export class RaycastResult {
    constructor() {
        this.rayFromWorld = new Vec3();
        this.rayToWorld = new Vec3();
        this.hitNormalWorld = new Vec3();
        this.hitPointWorld = new Vec3();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this.shouldStop = false;
    }
    reset() {
        this.rayFromWorld.setZero();
        this.rayToWorld.setZero();
        this.hitNormalWorld.setZero();
        this.hitPointWorld.setZero();
        this.hasHit = false;
        this.shape = null;
        this.body = null;
        this.hitFaceIndex = -1;
        this.distance = -1;
        this.shouldStop = false;
    }
    abort() {
        this.shouldStop = true;
    }
    set(rayFromWorld, rayToWorld, hitNormalWorld, hitPointWorld, shape, body, distance) {
        this.rayFromWorld.copy(rayFromWorld);
        this.rayToWorld.copy(rayToWorld);
        this.hitNormalWorld.copy(hitNormalWorld);
        this.hitPointWorld.copy(hitPointWorld);
        this.shape = shape;
        this.body = body;
        this.distance = distance;
    }
}
