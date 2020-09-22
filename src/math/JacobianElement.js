import { Vec3 } from "../math/Vec3.js";
export class JacobianElement {
  constructor() {
    this.spatial = new Vec3();
    this.rotational = new Vec3();
  }
  multiplyElement(element) {
    return element.spatial.dot(this.spatial) +
      element.rotational.dot(this.rotational);
  }
  multiplyVectors(spatial, rotational) {
    return spatial.dot(this.spatial) + rotational.dot(this.rotational);
  }
}
