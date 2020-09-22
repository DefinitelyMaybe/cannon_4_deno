import { Shape } from "../shapes/Shape.js";
import { Vec3 } from "../math/Vec3.js";
export class Particle extends Shape {
  constructor() {
    super({ type: Shape.types.PARTICLE });
  }
  calculateLocalInertia(mass, target = new Vec3()) {
    target.set(0, 0, 0);
    return target;
  }
  volume() {
    return 0;
  }
  updateBoundingSphereRadius() {
    this.boundingSphereRadius = 0;
  }
  calculateWorldAABB(pos, quat, min, max) {
    min.copy(pos);
    max.copy(pos);
  }
}
