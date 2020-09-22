/// <reference lib="dom" />
import { Utils } from "../utils/Utils.js";
export class Constraint {
    constructor(bodyA, bodyB, options = {}) {
        options = Utils.defaults(options, {
            collideConnected: true,
            wakeUpBodies: true,
        });
        this.equations = [];
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.id = Constraint.idCounter++;
        this.collideConnected = options.collideConnected;
        if (options.wakeUpBodies) {
            if (bodyA) {
                bodyA.wakeUp();
            }
            if (bodyB) {
                bodyB.wakeUp();
            }
        }
    }
    update() {
        throw new Error("method update() not implmemented in this Constraint subclass!");
    }
    enable() {
        const eqs = this.equations;
        for (let i = 0; i < eqs.length; i++) {
            eqs[i].enabled = true;
        }
    }
    disable() {
        const eqs = this.equations;
        for (let i = 0; i < eqs.length; i++) {
            eqs[i].enabled = false;
        }
    }
}
Constraint.idCounter = 0;
