/// <reference lib="dom" />
export const SHAPE_TYPES = {
  SPHERE: 1,
  PLANE: 2,
  BOX: 4,
  COMPOUND: 8,
  CONVEXPOLYHEDRON: 16,
  HEIGHTFIELD: 32,
  PARTICLE: 64,
  CYLINDER: 128,
  TRIMESH: 256,
};
export class Shape {
  constructor(options = {}) {
    this.id = Shape.idCounter++;
    this.type = options.type || 0;
    this.boundingSphereRadius = 0;
    this.collisionResponse = options.collisionResponse
      ? options.collisionResponse
      : true;
    this.collisionFilterGroup = options.collisionFilterGroup !== undefined
      ? options.collisionFilterGroup
      : 1;
    this.collisionFilterMask = options.collisionFilterMask !== undefined
      ? options.collisionFilterMask
      : -1;
    this.material = options.material ? options.material : null;
    this.body = null;
  }
  updateBoundingSphereRadius() {
    throw `computeBoundingSphereRadius() not implemented for shape type ${this.type}`;
  }
  volume() {
    throw `volume() not implemented for shape type ${this.type}`;
  }
  calculateLocalInertia(mass, target) {
    throw `calculateLocalInertia() not implemented for shape type ${this.type}`;
  }
  calculateWorldAABB(pos, quat, min, max) {
    throw `calculateWorldAABB() not implemented for shape type ${this.type}`;
  }
}
Shape.idCounter = 0;
Shape.types = SHAPE_TYPES;
