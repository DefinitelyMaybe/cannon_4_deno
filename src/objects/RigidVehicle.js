/// <reference types="./RigidVehicle.ts" />
import { Vec3 } from "../math/Vec3.js";
import { Body } from "../objects/Body.js";
import { Sphere } from "../shapes/Sphere.js";
import { Box } from "../shapes/Box.js";
import { HingeConstraint } from "../constraints/HingeConstraint.js";
export class RigidVehicle {
    constructor(options = {}) {
        this.wheelBodies = [];
        this.coordinateSystem = typeof options.coordinateSystem !== "undefined"
            ? options.coordinateSystem.clone()
            : new Vec3(1, 2, 3);
        if (options.chassisBody) {
            this.chassisBody = options.chassisBody;
        }
        else {
            this.chassisBody = new Body({ mass: 1, shape: new Box(new Vec3(5, 2, 0.5)) });
        }
        this.constraints = [];
        this.wheelAxes = [];
        this.wheelForces = [];
    }
    addWheel(options = {}) {
        let wheelBody;
        if (options.body) {
            wheelBody = options.body;
        }
        else {
            wheelBody = new Body({ mass: 1, shape: new Sphere(1.2) });
        }
        this.wheelBodies.push(wheelBody);
        this.wheelForces.push(0);
        const zero = new Vec3();
        const position = typeof options.position !== "undefined"
            ? options.position.clone()
            : new Vec3();
        const worldPosition = new Vec3();
        this.chassisBody.pointToWorldFrame(position, worldPosition);
        wheelBody.position.set(worldPosition.x, worldPosition.y, worldPosition.z);
        const axis = typeof options.axis !== "undefined"
            ? options.axis.clone()
            : new Vec3(0, 1, 0);
        this.wheelAxes.push(axis);
        const hingeConstraint = new HingeConstraint(this.chassisBody, wheelBody, {
            pivotA: position,
            axisA: axis,
            pivotB: Vec3.ZERO,
            axisB: axis,
            collideConnected: false,
        });
        this.constraints.push(hingeConstraint);
        return this.wheelBodies.length - 1;
    }
    setSteeringValue(value, wheelIndex) {
        const axis = this.wheelAxes[wheelIndex];
        const c = Math.cos(value);
        const s = Math.sin(value);
        const x = axis.x;
        const y = axis.y;
        this.constraints[wheelIndex].axisA.set(c * x - s * y, s * x + c * y, 0);
    }
    setMotorSpeed(value, wheelIndex) {
        const hingeConstraint = this.constraints[wheelIndex];
        hingeConstraint.enableMotor();
        hingeConstraint.motorTargetVelocity = value;
    }
    disableMotor(wheelIndex) {
        const hingeConstraint = this.constraints[wheelIndex];
        hingeConstraint.disableMotor();
    }
    setWheelForce(value, wheelIndex) {
        this.wheelForces[wheelIndex] = value;
    }
    applyWheelForce(value, wheelIndex) {
        const axis = this.wheelAxes[wheelIndex];
        const wheelBody = this.wheelBodies[wheelIndex];
        const bodyTorque = wheelBody.torque;
        axis.scale(value, torque);
        wheelBody.vectorToWorldFrame(torque, torque);
        bodyTorque.vadd(torque, bodyTorque);
    }
    addToWorld(world) {
        const constraints = this.constraints;
        const bodies = this.wheelBodies.concat([this.chassisBody]);
        for (let i = 0; i < bodies.length; i++) {
            world.addBody(bodies[i]);
        }
        for (let i = 0; i < constraints.length; i++) {
            world.addConstraint(constraints[i]);
        }
        world.addEventListener("preStep", this._update.bind(this));
    }
    _update() {
        const wheelForces = this.wheelForces;
        for (let i = 0; i < wheelForces.length; i++) {
            this.applyWheelForce(wheelForces[i], i);
        }
    }
    removeFromWorld(world) {
        const constraints = this.constraints;
        const bodies = this.wheelBodies.concat([this.chassisBody]);
        for (let i = 0; i < bodies.length; i++) {
            world.removeBody(bodies[i]);
        }
        for (let i = 0; i < constraints.length; i++) {
            world.removeConstraint(constraints[i]);
        }
    }
    getWheelSpeed(wheelIndex) {
        const axis = this.wheelAxes[wheelIndex];
        const wheelBody = this.wheelBodies[wheelIndex];
        const w = wheelBody.angularVelocity;
        this.chassisBody.vectorToWorldFrame(axis, worldAxis);
        return w.dot(worldAxis);
    }
}
const torque = new Vec3();
const worldAxis = new Vec3();
