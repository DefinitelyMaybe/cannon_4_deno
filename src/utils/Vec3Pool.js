import { Pool } from "../utils/Pool.js";
import { Vec3 } from "../math/Vec3.js";
export class Vec3Pool extends Pool {
  constructor() {
    super();
    this.type = Vec3;
  }
  constructObject() {
    return new Vec3();
  }
}
