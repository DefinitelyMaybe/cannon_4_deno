/// <reference lib="dom" />
import { Shape } from "../shapes/Shape.js";
import { Vec3 } from "../math/Vec3.js";
import { Transform } from "../math/Transform.js";
import { AABB } from "../collision/AABB.js";
import { Octree } from "../utils/Octree.js";
export class Trimesh extends Shape {
  constructor(vertices, indices) {
    super({ type: Shape.types.TRIMESH });
    this.vertices = new Float32Array(vertices);
    this.indices = new Int16Array(indices);
    this.normals = new Float32Array(indices.length);
    this.aabb = new AABB();
    this.edges = null;
    this.scale = new Vec3(1, 1, 1);
    this.tree = new Octree();
    this.updateEdges();
    this.updateNormals();
    this.updateAABB();
    this.updateBoundingSphereRadius();
    this.updateTree();
  }
  updateTree() {
    const tree = this.tree;
    tree.reset();
    tree.aabb.copy(this.aabb);
    const scale = this.scale;
    tree.aabb.lowerBound.x *= 1 / scale.x;
    tree.aabb.lowerBound.y *= 1 / scale.y;
    tree.aabb.lowerBound.z *= 1 / scale.z;
    tree.aabb.upperBound.x *= 1 / scale.x;
    tree.aabb.upperBound.y *= 1 / scale.y;
    tree.aabb.upperBound.z *= 1 / scale.z;
    const triangleAABB = new AABB();
    const a = new Vec3();
    const b = new Vec3();
    const c = new Vec3();
    const points = [a, b, c];
    for (let i = 0; i < this.indices.length / 3; i++) {
      const i3 = i * 3;
      this._getUnscaledVertex(this.indices[i3], a);
      this._getUnscaledVertex(this.indices[i3 + 1], b);
      this._getUnscaledVertex(this.indices[i3 + 2], c);
      triangleAABB.setFromPoints(points);
      tree.insert(triangleAABB, i);
    }
    tree.removeEmptyNodes();
  }
  getTrianglesInAABB(aabb, result) {
    unscaledAABB.copy(aabb);
    const scale = this.scale;
    const isx = scale.x;
    const isy = scale.y;
    const isz = scale.z;
    const l = unscaledAABB.lowerBound;
    const u = unscaledAABB.upperBound;
    l.x /= isx;
    l.y /= isy;
    l.z /= isz;
    u.x /= isx;
    u.y /= isy;
    u.z /= isz;
    return this.tree.aabbQuery(unscaledAABB, result);
  }
  setScale(scale) {
    const wasUniform = this.scale.x === this.scale.y &&
      this.scale.y === this.scale.z;
    const isUniform = scale.x === scale.y && scale.y === scale.z;
    if (!(wasUniform && isUniform)) {
      this.updateNormals();
    }
    this.scale.copy(scale);
    this.updateAABB();
    this.updateBoundingSphereRadius();
  }
  updateNormals() {
    const n = computeNormals_n;
    const normals = this.normals;
    for (let i = 0; i < this.indices.length / 3; i++) {
      const i3 = i * 3;
      const a = this.indices[i3];
      const b = this.indices[i3 + 1];
      const c = this.indices[i3 + 2];
      this.getVertex(a, va);
      this.getVertex(b, vb);
      this.getVertex(c, vc);
      Trimesh.computeNormal(vb, va, vc, n);
      normals[i3] = n.x;
      normals[i3 + 1] = n.y;
      normals[i3 + 2] = n.z;
    }
  }
  updateEdges() {
    const edges = {};
    const add = (a, b) => {
      const key = a < b ? `${a}_${b}` : `${b}_${a}`;
      edges[key] = true;
    };
    for (let i = 0; i < this.indices.length / 3; i++) {
      const i3 = i * 3;
      const a = this.indices[i3];
      const b = this.indices[i3 + 1];
      const c = this.indices[i3 + 2];
      add(a, b);
      add(b, c);
      add(c, a);
    }
    const keys = Object.keys(edges);
    this.edges = new Int16Array(keys.length * 2);
    for (let i = 0; i < keys.length; i++) {
      const indices = keys[i].split("_");
      this.edges[2 * i] = parseInt(indices[0], 10);
      this.edges[2 * i + 1] = parseInt(indices[1], 10);
    }
  }
  getEdgeVertex(edgeIndex, firstOrSecond, vertexStore) {
    const vertexIndex = this.edges[edgeIndex * 2 + (firstOrSecond ? 1 : 0)];
    this.getVertex(vertexIndex, vertexStore);
  }
  getEdgeVector(edgeIndex, vectorStore) {
    const va = getEdgeVector_va;
    const vb = getEdgeVector_vb;
    this.getEdgeVertex(edgeIndex, 0, va);
    this.getEdgeVertex(edgeIndex, 1, vb);
    vb.vsub(va, vectorStore);
  }
  getVertex(i, out) {
    const scale = this.scale;
    this._getUnscaledVertex(i, out);
    out.x *= scale.x;
    out.y *= scale.y;
    out.z *= scale.z;
    return out;
  }
  _getUnscaledVertex(i, out) {
    const i3 = i * 3;
    const vertices = this.vertices;
    return out.set(vertices[i3], vertices[i3 + 1], vertices[i3 + 2]);
  }
  getWorldVertex(i, pos, quat, out) {
    this.getVertex(i, out);
    Transform.pointToWorldFrame(pos, quat, out, out);
    return out;
  }
  getTriangleVertices(i, a, b, c) {
    const i3 = i * 3;
    this.getVertex(this.indices[i3], a);
    this.getVertex(this.indices[i3 + 1], b);
    this.getVertex(this.indices[i3 + 2], c);
  }
  getNormal(i, target) {
    const i3 = i * 3;
    return target.set(
      this.normals[i3],
      this.normals[i3 + 1],
      this.normals[i3 + 2],
    );
  }
  calculateLocalInertia(mass, target) {
    this.computeLocalAABB(cli_aabb);
    const x = cli_aabb.upperBound.x - cli_aabb.lowerBound.x;
    const y = cli_aabb.upperBound.y - cli_aabb.lowerBound.y;
    const z = cli_aabb.upperBound.z - cli_aabb.lowerBound.z;
    return target.set(
      (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * z * 2 * z),
      (1.0 / 12.0) * mass * (2 * x * 2 * x + 2 * z * 2 * z),
      (1.0 / 12.0) * mass * (2 * y * 2 * y + 2 * x * 2 * x),
    );
  }
  computeLocalAABB(aabb) {
    const l = aabb.lowerBound;
    const u = aabb.upperBound;
    const n = this.vertices.length;
    const vertices = this.vertices;
    const v = computeLocalAABB_worldVert;
    this.getVertex(0, v);
    l.copy(v);
    u.copy(v);
    for (let i = 0; i !== n; i++) {
      this.getVertex(i, v);
      if (v.x < l.x) {
        l.x = v.x;
      } else if (v.x > u.x) {
        u.x = v.x;
      }
      if (v.y < l.y) {
        l.y = v.y;
      } else if (v.y > u.y) {
        u.y = v.y;
      }
      if (v.z < l.z) {
        l.z = v.z;
      } else if (v.z > u.z) {
        u.z = v.z;
      }
    }
  }
  updateAABB() {
    this.computeLocalAABB(this.aabb);
  }
  updateBoundingSphereRadius() {
    let max2 = 0;
    const vertices = this.vertices;
    const v = new Vec3();
    for (let i = 0, N = vertices.length / 3; i !== N; i++) {
      this.getVertex(i, v);
      const norm2 = v.lengthSquared();
      if (norm2 > max2) {
        max2 = norm2;
      }
    }
    this.boundingSphereRadius = Math.sqrt(max2);
  }
  calculateWorldAABB(pos, quat, min, max) {
    const frame = calculateWorldAABB_frame;
    const result = calculateWorldAABB_aabb;
    frame.position = pos;
    frame.quaternion = quat;
    this.aabb.toWorldFrame(frame, result);
    min.copy(result.lowerBound);
    max.copy(result.upperBound);
  }
  volume() {
    return (4.0 * Math.PI * this.boundingSphereRadius) / 3.0;
  }
}
const computeNormals_n = new Vec3();
const unscaledAABB = new AABB();
const getEdgeVector_va = new Vec3();
const getEdgeVector_vb = new Vec3();
const cb = new Vec3();
const ab = new Vec3();
Trimesh.computeNormal = (va, vb, vc, target) => {
  vb.vsub(va, ab);
  vc.vsub(vb, cb);
  cb.cross(ab, target);
  if (!target.isZero()) {
    target.normalize();
  }
};
const va = new Vec3();
const vb = new Vec3();
const vc = new Vec3();
const cli_aabb = new AABB();
const computeLocalAABB_worldVert = new Vec3();
const calculateWorldAABB_frame = new Transform();
const calculateWorldAABB_aabb = new AABB();
Trimesh.createTorus = (
  radius = 1,
  tube = 0.5,
  radialSegments = 8,
  tubularSegments = 6,
  arc = Math.PI * 2,
) => {
  const vertices = [];
  const indices = [];
  for (let j = 0; j <= radialSegments; j++) {
    for (let i = 0; i <= tubularSegments; i++) {
      const u = (i / tubularSegments) * arc;
      const v = (j / radialSegments) * Math.PI * 2;
      const x = (radius + tube * Math.cos(v)) * Math.cos(u);
      const y = (radius + tube * Math.cos(v)) * Math.sin(u);
      const z = tube * Math.sin(v);
      vertices.push(x, y, z);
    }
  }
  for (let j = 1; j <= radialSegments; j++) {
    for (let i = 1; i <= tubularSegments; i++) {
      const a = (tubularSegments + 1) * j + i - 1;
      const b = (tubularSegments + 1) * (j - 1) + i - 1;
      const c = (tubularSegments + 1) * (j - 1) + i;
      const d = (tubularSegments + 1) * j + i;
      indices.push(a, b, d);
      indices.push(b, c, d);
    }
  }
  return new Trimesh(vertices, indices);
};
