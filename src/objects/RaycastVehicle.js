/// <reference lib="dom" />
import { Vec3 } from "../math/Vec3.js";
import { Quaternion } from "../math/Quaternion.js";
import { Ray } from "../collision/Ray.js";
import { WheelInfo } from "../objects/WheelInfo.js";
export class RaycastVehicle {
    constructor(options) {
        this.chassisBody = options.chassisBody;
        this.wheelInfos = [];
        this.sliding = false;
        this.world = null;
        this.indexRightAxis = typeof options.indexRightAxis !== "undefined"
            ? options.indexRightAxis
            : 1;
        this.indexForwardAxis = typeof options.indexForwardAxis !== "undefined"
            ? options.indexForwardAxis
            : 0;
        this.indexUpAxis = typeof options.indexUpAxis !== "undefined"
            ? options.indexUpAxis
            : 2;
        this.constraints = [];
        this.preStepCallback = () => { };
        this.currentVehicleSpeedKmHour = 0;
    }
    addWheel(options = {}) {
        const info = new WheelInfo(options);
        const index = this.wheelInfos.length;
        this.wheelInfos.push(info);
        return index;
    }
    setSteeringValue(value, wheelIndex) {
        const wheel = this.wheelInfos[wheelIndex];
        wheel.steering = value;
    }
    applyEngineForce(value, wheelIndex) {
        this.wheelInfos[wheelIndex].engineForce = value;
    }
    setBrake(brake, wheelIndex) {
        this.wheelInfos[wheelIndex].brake = brake;
    }
    addToWorld(world) {
        const constraints = this.constraints;
        world.addBody(this.chassisBody);
        const that = this;
        this.preStepCallback = () => {
            that.updateVehicle(world.dt);
        };
        world.addEventListener("preStep", this.preStepCallback);
        this.world = world;
    }
    getVehicleAxisWorld(axisIndex, result) {
        result.set(axisIndex === 0 ? 1 : 0, axisIndex === 1 ? 1 : 0, axisIndex === 2 ? 1 : 0);
        this.chassisBody.vectorToWorldFrame(result, result);
    }
    updateVehicle(timeStep) {
        const wheelInfos = this.wheelInfos;
        const numWheels = wheelInfos.length;
        const chassisBody = this.chassisBody;
        for (let i = 0; i < numWheels; i++) {
            this.updateWheelTransform(i);
        }
        this.currentVehicleSpeedKmHour = 3.6 * chassisBody.velocity.length();
        const forwardWorld = new Vec3();
        this.getVehicleAxisWorld(this.indexForwardAxis, forwardWorld);
        if (forwardWorld.dot(chassisBody.velocity) < 0) {
            this.currentVehicleSpeedKmHour *= -1;
        }
        for (let i = 0; i < numWheels; i++) {
            this.castRay(wheelInfos[i]);
        }
        this.updateSuspension(timeStep);
        const impulse = new Vec3();
        const relpos = new Vec3();
        for (let i = 0; i < numWheels; i++) {
            const wheel = wheelInfos[i];
            let suspensionForce = wheel.suspensionForce;
            if (suspensionForce > wheel.maxSuspensionForce) {
                suspensionForce = wheel.maxSuspensionForce;
            }
            wheel.raycastResult.hitNormalWorld.scale(suspensionForce * timeStep, impulse);
            wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, relpos);
            chassisBody.applyImpulse(impulse, relpos);
        }
        this.updateFriction(timeStep);
        const hitNormalWorldScaledWithProj = new Vec3();
        const fwd = new Vec3();
        const vel = new Vec3();
        for (let i = 0; i < numWheels; i++) {
            const wheel = wheelInfos[i];
            chassisBody.getVelocityAtWorldPoint(wheel.chassisConnectionPointWorld, vel);
            let m = 1;
            switch (this.indexUpAxis) {
                case 1:
                    m = -1;
                    break;
            }
            if (wheel.isInContact) {
                this.getVehicleAxisWorld(this.indexForwardAxis, fwd);
                const proj = fwd.dot(wheel.raycastResult.hitNormalWorld);
                wheel.raycastResult.hitNormalWorld.scale(proj, hitNormalWorldScaledWithProj);
                fwd.vsub(hitNormalWorldScaledWithProj, fwd);
                const proj2 = fwd.dot(vel);
                wheel.deltaRotation = (m * proj2 * timeStep) / wheel.radius;
            }
            if ((wheel.sliding || !wheel.isInContact) && wheel.engineForce !== 0 &&
                wheel.useCustomSlidingRotationalSpeed) {
                wheel.deltaRotation = (wheel.engineForce > 0 ? 1 : -1) *
                    wheel.customSlidingRotationalSpeed * timeStep;
            }
            if (Math.abs(wheel.brake) > Math.abs(wheel.engineForce)) {
                wheel.deltaRotation = 0;
            }
            wheel.rotation += wheel.deltaRotation;
            wheel.deltaRotation *= 0.99;
        }
    }
    updateSuspension(deltaTime) {
        const chassisBody = this.chassisBody;
        const chassisMass = chassisBody.mass;
        const wheelInfos = this.wheelInfos;
        const numWheels = wheelInfos.length;
        for (let w_it = 0; w_it < numWheels; w_it++) {
            const wheel = wheelInfos[w_it];
            if (wheel.isInContact) {
                let force;
                const susp_length = wheel.suspensionRestLength;
                const current_length = wheel.suspensionLength;
                const length_diff = susp_length - current_length;
                force = wheel.suspensionStiffness * length_diff *
                    wheel.clippedInvContactDotSuspension;
                const projected_rel_vel = wheel.suspensionRelativeVelocity;
                let susp_damping;
                if (projected_rel_vel < 0) {
                    susp_damping = wheel.dampingCompression;
                }
                else {
                    susp_damping = wheel.dampingRelaxation;
                }
                force -= susp_damping * projected_rel_vel;
                wheel.suspensionForce = force * chassisMass;
                if (wheel.suspensionForce < 0) {
                    wheel.suspensionForce = 0;
                }
            }
            else {
                wheel.suspensionForce = 0;
            }
        }
    }
    removeFromWorld(world) {
        const constraints = this.constraints;
        world.removeBody(this.chassisBody);
        world.removeEventListener("preStep", this.preStepCallback);
        this.world = null;
    }
    castRay(wheel) {
        const rayvector = castRay_rayvector;
        const target = castRay_target;
        this.updateWheelTransformWorld(wheel);
        const chassisBody = this.chassisBody;
        let depth = -1;
        const raylen = wheel.suspensionRestLength + wheel.radius;
        wheel.directionWorld.scale(raylen, rayvector);
        const source = wheel.chassisConnectionPointWorld;
        source.vadd(rayvector, target);
        const raycastResult = wheel.raycastResult;
        const param = 0;
        raycastResult.reset();
        const oldState = chassisBody.collisionResponse;
        chassisBody.collisionResponse = false;
        this.world.rayTest(source, target, raycastResult);
        chassisBody.collisionResponse = oldState;
        const object = raycastResult.body;
        wheel.raycastResult.groundObject = 0;
        if (object) {
            depth = raycastResult.distance;
            wheel.raycastResult.hitNormalWorld = raycastResult.hitNormalWorld;
            wheel.isInContact = true;
            const hitDistance = raycastResult.distance;
            wheel.suspensionLength = hitDistance - wheel.radius;
            const minSuspensionLength = wheel.suspensionRestLength -
                wheel.maxSuspensionTravel;
            const maxSuspensionLength = wheel.suspensionRestLength +
                wheel.maxSuspensionTravel;
            if (wheel.suspensionLength < minSuspensionLength) {
                wheel.suspensionLength = minSuspensionLength;
            }
            if (wheel.suspensionLength > maxSuspensionLength) {
                wheel.suspensionLength = maxSuspensionLength;
                wheel.raycastResult.reset();
            }
            const denominator = wheel.raycastResult.hitNormalWorld.dot(wheel.directionWorld);
            const chassis_velocity_at_contactPoint = new Vec3();
            chassisBody.getVelocityAtWorldPoint(wheel.raycastResult.hitPointWorld, chassis_velocity_at_contactPoint);
            const projVel = wheel.raycastResult.hitNormalWorld.dot(chassis_velocity_at_contactPoint);
            if (denominator >= -0.1) {
                wheel.suspensionRelativeVelocity = 0;
                wheel.clippedInvContactDotSuspension = 1 / 0.1;
            }
            else {
                const inv = -1 / denominator;
                wheel.suspensionRelativeVelocity = projVel * inv;
                wheel.clippedInvContactDotSuspension = inv;
            }
        }
        else {
            wheel.suspensionLength = wheel.suspensionRestLength +
                0 * wheel.maxSuspensionTravel;
            wheel.suspensionRelativeVelocity = 0.0;
            wheel.directionWorld.scale(-1, wheel.raycastResult.hitNormalWorld);
            wheel.clippedInvContactDotSuspension = 1.0;
        }
        return depth;
    }
    updateWheelTransformWorld(wheel) {
        wheel.isInContact = false;
        const chassisBody = this.chassisBody;
        chassisBody.pointToWorldFrame(wheel.chassisConnectionPointLocal, wheel.chassisConnectionPointWorld);
        chassisBody.vectorToWorldFrame(wheel.directionLocal, wheel.directionWorld);
        chassisBody.vectorToWorldFrame(wheel.axleLocal, wheel.axleWorld);
    }
    updateWheelTransform(wheelIndex) {
        const up = tmpVec4;
        const right = tmpVec5;
        const fwd = tmpVec6;
        const wheel = this.wheelInfos[wheelIndex];
        this.updateWheelTransformWorld(wheel);
        wheel.directionLocal.scale(-1, up);
        right.copy(wheel.axleLocal);
        up.cross(right, fwd);
        fwd.normalize();
        right.normalize();
        const steering = wheel.steering;
        const steeringOrn = new Quaternion();
        steeringOrn.setFromAxisAngle(up, steering);
        const rotatingOrn = new Quaternion();
        rotatingOrn.setFromAxisAngle(right, wheel.rotation);
        const q = wheel.worldTransform.quaternion;
        this.chassisBody.quaternion.mult(steeringOrn, q);
        q.mult(rotatingOrn, q);
        q.normalize();
        const p = wheel.worldTransform.position;
        p.copy(wheel.directionWorld);
        p.scale(wheel.suspensionLength, p);
        p.vadd(wheel.chassisConnectionPointWorld, p);
    }
    getWheelTransformWorld(wheelIndex) {
        return this.wheelInfos[wheelIndex].worldTransform;
    }
    updateFriction(timeStep) {
        const surfNormalWS_scaled_proj = updateFriction_surfNormalWS_scaled_proj;
        const wheelInfos = this.wheelInfos;
        const numWheels = wheelInfos.length;
        const chassisBody = this.chassisBody;
        const forwardWS = updateFriction_forwardWS;
        const axle = updateFriction_axle;
        let numWheelsOnGround = 0;
        for (let i = 0; i < numWheels; i++) {
            const wheel = wheelInfos[i];
            const groundObject = wheel.raycastResult.body;
            if (groundObject) {
                numWheelsOnGround++;
            }
            wheel.sideImpulse = 0;
            wheel.forwardImpulse = 0;
            if (!forwardWS[i]) {
                forwardWS[i] = new Vec3();
            }
            if (!axle[i]) {
                axle[i] = new Vec3();
            }
        }
        for (let i = 0; i < numWheels; i++) {
            const wheel = wheelInfos[i];
            const groundObject = wheel.raycastResult.body;
            if (groundObject) {
                const axlei = axle[i];
                const wheelTrans = this.getWheelTransformWorld(i);
                wheelTrans.vectorToWorldFrame(directions[this.indexRightAxis], axlei);
                const surfNormalWS = wheel.raycastResult.hitNormalWorld;
                const proj = axlei.dot(surfNormalWS);
                surfNormalWS.scale(proj, surfNormalWS_scaled_proj);
                axlei.vsub(surfNormalWS_scaled_proj, axlei);
                axlei.normalize();
                surfNormalWS.cross(axlei, forwardWS[i]);
                forwardWS[i].normalize();
                wheel.sideImpulse = resolveSingleBilateral(chassisBody, wheel.raycastResult.hitPointWorld, groundObject, wheel.raycastResult.hitPointWorld, axlei);
                wheel.sideImpulse *= sideFrictionStiffness2;
            }
        }
        const sideFactor = 1;
        const fwdFactor = 0.5;
        this.sliding = false;
        for (let i = 0; i < numWheels; i++) {
            const wheel = wheelInfos[i];
            const groundObject = wheel.raycastResult.body;
            let rollingFriction = 0;
            wheel.slipInfo = 1;
            if (groundObject) {
                const defaultRollingFrictionImpulse = 0;
                const maxImpulse = wheel.brake
                    ? wheel.brake
                    : defaultRollingFrictionImpulse;
                rollingFriction = calcRollingFriction(chassisBody, groundObject, wheel.raycastResult.hitPointWorld, forwardWS[i], maxImpulse);
                rollingFriction += wheel.engineForce * timeStep;
                const factor = maxImpulse / rollingFriction;
                wheel.slipInfo *= factor;
            }
            wheel.forwardImpulse = 0;
            wheel.skidInfo = 1;
            if (groundObject) {
                wheel.skidInfo = 1;
                const maximp = wheel.suspensionForce * timeStep * wheel.frictionSlip;
                const maximpSide = maximp;
                const maximpSquared = maximp * maximpSide;
                wheel.forwardImpulse = rollingFriction;
                const x = wheel.forwardImpulse * fwdFactor;
                const y = wheel.sideImpulse * sideFactor;
                const impulseSquared = x * x + y * y;
                wheel.sliding = false;
                if (impulseSquared > maximpSquared) {
                    this.sliding = true;
                    wheel.sliding = true;
                    const factor = maximp / Math.sqrt(impulseSquared);
                    wheel.skidInfo *= factor;
                }
            }
        }
        if (this.sliding) {
            for (let i = 0; i < numWheels; i++) {
                const wheel = wheelInfos[i];
                if (wheel.sideImpulse !== 0) {
                    if (wheel.skidInfo < 1) {
                        wheel.forwardImpulse *= wheel.skidInfo;
                        wheel.sideImpulse *= wheel.skidInfo;
                    }
                }
            }
        }
        for (let i = 0; i < numWheels; i++) {
            const wheel = wheelInfos[i];
            const rel_pos = new Vec3();
            wheel.raycastResult.hitPointWorld.vsub(chassisBody.position, rel_pos);
            if (wheel.forwardImpulse !== 0) {
                const impulse = new Vec3();
                forwardWS[i].scale(wheel.forwardImpulse, impulse);
                chassisBody.applyImpulse(impulse, rel_pos);
            }
            if (wheel.sideImpulse !== 0) {
                const groundObject = wheel.raycastResult.body;
                const rel_pos2 = new Vec3();
                wheel.raycastResult.hitPointWorld.vsub(groundObject.position, rel_pos2);
                const sideImp = new Vec3();
                axle[i].scale(wheel.sideImpulse, sideImp);
                chassisBody.vectorToLocalFrame(rel_pos, rel_pos);
                rel_pos["xyz"[this.indexUpAxis]] *=
                    wheel.rollInfluence;
                chassisBody.vectorToWorldFrame(rel_pos, rel_pos);
                chassisBody.applyImpulse(sideImp, rel_pos);
                sideImp.scale(-1, sideImp);
                groundObject.applyImpulse(sideImp, rel_pos2);
            }
        }
    }
}
const tmpVec1 = new Vec3();
const tmpVec2 = new Vec3();
const tmpVec3 = new Vec3();
const tmpVec4 = new Vec3();
const tmpVec5 = new Vec3();
const tmpVec6 = new Vec3();
const tmpRay = new Ray();
const torque = new Vec3();
const castRay_rayvector = new Vec3();
const castRay_target = new Vec3();
const directions = [new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)];
const updateFriction_surfNormalWS_scaled_proj = new Vec3();
const updateFriction_axle = [];
const updateFriction_forwardWS = [];
const sideFrictionStiffness2 = 1;
const calcRollingFriction_vel1 = new Vec3();
const calcRollingFriction_vel2 = new Vec3();
const calcRollingFriction_vel = new Vec3();
function calcRollingFriction(body0, body1, frictionPosWorld, frictionDirectionWorld, maxImpulse) {
    let j1 = 0;
    const contactPosWorld = frictionPosWorld;
    const vel1 = calcRollingFriction_vel1;
    const vel2 = calcRollingFriction_vel2;
    const vel = calcRollingFriction_vel;
    body0.getVelocityAtWorldPoint(contactPosWorld, vel1);
    body1.getVelocityAtWorldPoint(contactPosWorld, vel2);
    vel1.vsub(vel2, vel);
    const vrel = frictionDirectionWorld.dot(vel);
    const denom0 = computeImpulseDenominator(body0, frictionPosWorld, frictionDirectionWorld);
    const denom1 = computeImpulseDenominator(body1, frictionPosWorld, frictionDirectionWorld);
    const relaxation = 1;
    const jacDiagABInv = relaxation / (denom0 + denom1);
    j1 = -vrel * jacDiagABInv;
    if (maxImpulse < j1) {
        j1 = maxImpulse;
    }
    if (j1 < -maxImpulse) {
        j1 = -maxImpulse;
    }
    return j1;
}
const computeImpulseDenominator_r0 = new Vec3();
const computeImpulseDenominator_c0 = new Vec3();
const computeImpulseDenominator_vec = new Vec3();
const computeImpulseDenominator_m = new Vec3();
function computeImpulseDenominator(body, pos, normal) {
    const r0 = computeImpulseDenominator_r0;
    const c0 = computeImpulseDenominator_c0;
    const vec = computeImpulseDenominator_vec;
    const m = computeImpulseDenominator_m;
    pos.vsub(body.position, r0);
    r0.cross(normal, c0);
    body.invInertiaWorld.vmult(c0, m);
    m.cross(r0, vec);
    return body.invMass + normal.dot(vec);
}
const resolveSingleBilateral_vel1 = new Vec3();
const resolveSingleBilateral_vel2 = new Vec3();
const resolveSingleBilateral_vel = new Vec3();
function resolveSingleBilateral(body1, pos1, body2, pos2, normal) {
    const normalLenSqr = normal.lengthSquared();
    if (normalLenSqr > 1.1) {
        return 0;
    }
    const vel1 = resolveSingleBilateral_vel1;
    const vel2 = resolveSingleBilateral_vel2;
    const vel = resolveSingleBilateral_vel;
    body1.getVelocityAtWorldPoint(pos1, vel1);
    body2.getVelocityAtWorldPoint(pos2, vel2);
    vel1.vsub(vel2, vel);
    const rel_vel = normal.dot(vel);
    const contactDamping = 0.2;
    const massTerm = 1 / (body1.invMass + body2.invMass);
    const impulse = -contactDamping * rel_vel * massTerm;
    return impulse;
}
