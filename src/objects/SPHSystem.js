/// <reference types="./SPHSystem.ts" />
/// <reference lib="dom" />
import { Vec3 } from "../math/Vec3.js";
export class SPHSystem {
    constructor() {
        this.particles = [];
        this.density = 1;
        this.smoothingRadius = 1;
        this.speedOfSound = 1;
        this.viscosity = 0.01;
        this.eps = 0.000001;
        this.pressures = [];
        this.densities = [];
        this.neighbors = [];
    }
    add(particle) {
        this.particles.push(particle);
        if (this.neighbors.length < this.particles.length) {
            this.neighbors.push([]);
        }
    }
    remove(particle) {
        const idx = this.particles.indexOf(particle);
        if (idx !== -1) {
            this.particles.splice(idx, 1);
            if (this.neighbors.length > this.particles.length) {
                this.neighbors.pop();
            }
        }
    }
    getNeighbors(particle, neighbors) {
        const N = this.particles.length;
        const id = particle.id;
        const R2 = this.smoothingRadius * this.smoothingRadius;
        const dist = SPHSystem_getNeighbors_dist;
        for (let i = 0; i !== N; i++) {
            const p = this.particles[i];
            p.position.vsub(particle.position, dist);
            if (id !== p.id && dist.lengthSquared() < R2) {
                neighbors.push(p);
            }
        }
    }
    update() {
        const N = this.particles.length;
        const dist = SPHSystem_update_dist;
        const cs = this.speedOfSound;
        const eps = this.eps;
        for (let i = 0; i !== N; i++) {
            const p = this.particles[i];
            const neighbors = this.neighbors[i];
            neighbors.length = 0;
            this.getNeighbors(p, neighbors);
            neighbors.push(this.particles[i]);
            const numNeighbors = neighbors.length;
            let sum = 0.0;
            for (let j = 0; j !== numNeighbors; j++) {
                p.position.vsub(neighbors[j].position, dist);
                const len = dist.length();
                const weight = this.w(len);
                sum += neighbors[j].mass * weight;
            }
            this.densities[i] = sum;
            this.pressures[i] = cs * cs * (this.densities[i] - this.density);
        }
        const a_pressure = SPHSystem_update_a_pressure;
        const a_visc = SPHSystem_update_a_visc;
        const gradW = SPHSystem_update_gradW;
        const r_vec = SPHSystem_update_r_vec;
        const u = SPHSystem_update_u;
        for (let i = 0; i !== N; i++) {
            const particle = this.particles[i];
            a_pressure.set(0, 0, 0);
            a_visc.set(0, 0, 0);
            let Pij;
            let nabla;
            let Vij;
            const neighbors = this.neighbors[i];
            const numNeighbors = neighbors.length;
            for (let j = 0; j !== numNeighbors; j++) {
                const neighbor = neighbors[j];
                particle.position.vsub(neighbor.position, r_vec);
                const r = r_vec.length();
                Pij = -neighbor.mass *
                    (this.pressures[i] / (this.densities[i] * this.densities[i] + eps) +
                        this.pressures[j] / (this.densities[j] * this.densities[j] + eps));
                this.gradw(r_vec, gradW);
                gradW.scale(Pij, gradW);
                a_pressure.vadd(gradW, a_pressure);
                neighbor.velocity.vsub(particle.velocity, u);
                u.scale((1.0 / (0.0001 + this.densities[i] * this.densities[j])) *
                    this.viscosity * neighbor.mass, u);
                nabla = this.nablaw(r);
                u.scale(nabla, u);
                a_visc.vadd(u, a_visc);
            }
            a_visc.scale(particle.mass, a_visc);
            a_pressure.scale(particle.mass, a_pressure);
            particle.force.vadd(a_visc, particle.force);
            particle.force.vadd(a_pressure, particle.force);
        }
    }
    w(r) {
        const h = this.smoothingRadius;
        return (315.0 / (64.0 * Math.PI * h ** 9)) * (h * h - r * r) ** 3;
    }
    gradw(rVec, resultVec) {
        const r = rVec.length();
        const h = this.smoothingRadius;
        rVec.scale((945.0 / (32.0 * Math.PI * h ** 9)) * (h * h - r * r) ** 2, resultVec);
    }
    nablaw(r) {
        const h = this.smoothingRadius;
        const nabla = (945.0 / (32.0 * Math.PI * h ** 9)) * (h * h - r * r) *
            (7 * r * r - 3 * h * h);
        return nabla;
    }
}
const SPHSystem_getNeighbors_dist = new Vec3();
const SPHSystem_update_dist = new Vec3();
const SPHSystem_update_a_pressure = new Vec3();
const SPHSystem_update_a_visc = new Vec3();
const SPHSystem_update_gradW = new Vec3();
const SPHSystem_update_r_vec = new Vec3();
const SPHSystem_update_u = new Vec3();
