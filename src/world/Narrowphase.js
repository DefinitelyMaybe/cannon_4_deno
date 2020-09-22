import { Shape } from "../shapes/Shape.js";
import { Vec3 } from "../math/Vec3.js";
import { Transform } from "../math/Transform.js";
import { Quaternion } from "../math/Quaternion.js";
import { Body } from "../objects/Body.js";
import { AABB } from "../collision/AABB.js";
import { Ray } from "../collision/Ray.js";
import { Vec3Pool } from "../utils/Vec3Pool.js";
import { ContactEquation } from "../equations/ContactEquation.js";
import { FrictionEquation } from "../equations/FrictionEquation.js";
export const COLLISION_TYPES = {
  sphereSphere: Shape.types.SPHERE,
  spherePlane: (Shape.types.SPHERE | Shape.types.PLANE),
  boxBox: (Shape.types.BOX | Shape.types.BOX),
  sphereBox: (Shape.types.SPHERE | Shape.types.BOX),
  planeBox: (Shape.types.PLANE | Shape.types.BOX),
  convexConvex: Shape.types.CONVEXPOLYHEDRON,
  sphereConvex: (Shape.types.SPHERE | Shape.types.CONVEXPOLYHEDRON),
  planeConvex: (Shape.types.PLANE | Shape.types.CONVEXPOLYHEDRON),
  boxConvex: (Shape.types.BOX | Shape.types.CONVEXPOLYHEDRON),
  sphereHeightfield: (Shape.types.SPHERE | Shape.types.HEIGHTFIELD),
  boxHeightfield: (Shape.types.BOX | Shape.types.HEIGHTFIELD),
  convexHeightfield: (Shape.types.CONVEXPOLYHEDRON | Shape.types.HEIGHTFIELD),
  sphereParticle: (Shape.types.PARTICLE | Shape.types.SPHERE),
  planeParticle: (Shape.types.PLANE | Shape.types.PARTICLE),
  boxParticle: (Shape.types.BOX | Shape.types.PARTICLE),
  convexParticle: (Shape.types.PARTICLE | Shape.types.CONVEXPOLYHEDRON),
  sphereTrimesh: (Shape.types.SPHERE | Shape.types.TRIMESH),
  planeTrimesh: (Shape.types.PLANE | Shape.types.TRIMESH),
};
export class Narrowphase {
  constructor(world) {
    this.contactPointPool = [];
    this.frictionEquationPool = [];
    this.result = [];
    this.frictionResult = [];
    this.v3pool = new Vec3Pool();
    this.world = world;
    this.currentContactMaterial = world.defaultContactMaterial;
    this.enableFrictionReduction = false;
  }
  createContactEquation(bi, bj, si, sj, overrideShapeA, overrideShapeB) {
    let c;
    if (this.contactPointPool.length) {
      c = this.contactPointPool.pop();
      c.bi = bi;
      c.bj = bj;
    } else {
      c = new ContactEquation(bi, bj);
    }
    c.enabled = bi.collisionResponse && bj.collisionResponse &&
      si.collisionResponse && sj.collisionResponse;
    const cm = this.currentContactMaterial;
    c.restitution = cm.restitution;
    c.setSpookParams(
      cm.contactEquationStiffness,
      cm.contactEquationRelaxation,
      this.world.dt,
    );
    const matA = si.material || bi.material;
    const matB = sj.material || bj.material;
    if (matA && matB && matA.restitution >= 0 && matB.restitution >= 0) {
      c.restitution = matA.restitution * matB.restitution;
    }
    c.si = overrideShapeA || si;
    c.sj = overrideShapeB || sj;
    return c;
  }
  createFrictionEquationsFromContact(contactEquation, outArray) {
    const bodyA = contactEquation.bi;
    const bodyB = contactEquation.bj;
    const shapeA = contactEquation.si;
    const shapeB = contactEquation.sj;
    const world = this.world;
    const cm = this.currentContactMaterial;
    let friction = cm.friction;
    const matA = shapeA.material || bodyA.material;
    const matB = shapeB.material || bodyB.material;
    if (matA && matB && matA.friction >= 0 && matB.friction >= 0) {
      friction = matA.friction * matB.friction;
    }
    if (friction > 0) {
      const mug = friction * world.gravity.length();
      let reducedMass = bodyA.invMass + bodyB.invMass;
      if (reducedMass > 0) {
        reducedMass = 1 / reducedMass;
      }
      const pool = this.frictionEquationPool;
      const c1 = pool.length
        ? pool.pop()
        : new FrictionEquation(bodyA, bodyB, mug * reducedMass);
      const c2 = pool.length
        ? pool.pop()
        : new FrictionEquation(bodyA, bodyB, mug * reducedMass);
      c1.bi = c2.bi = bodyA;
      c1.bj = c2.bj = bodyB;
      c1.minForce = c2.minForce = -mug * reducedMass;
      c1.maxForce = c2.maxForce = mug * reducedMass;
      c1.ri.copy(contactEquation.ri);
      c1.rj.copy(contactEquation.rj);
      c2.ri.copy(contactEquation.ri);
      c2.rj.copy(contactEquation.rj);
      contactEquation.ni.tangents(c1.t, c2.t);
      c1.setSpookParams(
        cm.frictionEquationStiffness,
        cm.frictionEquationRelaxation,
        world.dt,
      );
      c2.setSpookParams(
        cm.frictionEquationStiffness,
        cm.frictionEquationRelaxation,
        world.dt,
      );
      c1.enabled = c2.enabled = contactEquation.enabled;
      outArray.push(c1, c2);
      return true;
    }
    return false;
  }
  createFrictionFromAverage(numContacts) {
    let c = this.result[this.result.length - 1];
    if (
      !this.createFrictionEquationsFromContact(c, this.frictionResult) ||
      numContacts === 1
    ) {
      return;
    }
    const f1 = this.frictionResult[this.frictionResult.length - 2];
    const f2 = this.frictionResult[this.frictionResult.length - 1];
    averageNormal.setZero();
    averageContactPointA.setZero();
    averageContactPointB.setZero();
    const bodyA = c.bi;
    const bodyB = c.bj;
    for (let i = 0; i !== numContacts; i++) {
      c = this.result[this.result.length - 1 - i];
      if (c.bi !== bodyA) {
        averageNormal.vadd(c.ni, averageNormal);
        averageContactPointA.vadd(c.ri, averageContactPointA);
        averageContactPointB.vadd(c.rj, averageContactPointB);
      } else {
        averageNormal.vsub(c.ni, averageNormal);
        averageContactPointA.vadd(c.rj, averageContactPointA);
        averageContactPointB.vadd(c.ri, averageContactPointB);
      }
    }
    const invNumContacts = 1 / numContacts;
    averageContactPointA.scale(invNumContacts, f1.ri);
    averageContactPointB.scale(invNumContacts, f1.rj);
    f2.ri.copy(f1.ri);
    f2.rj.copy(f1.rj);
    averageNormal.normalize();
    averageNormal.tangents(f1.t, f2.t);
  }
  getContacts(
    p1,
    p2,
    world,
    result,
    oldcontacts,
    frictionResult,
    frictionPool,
  ) {
    this.contactPointPool = oldcontacts;
    this.frictionEquationPool = frictionPool;
    this.result = result;
    this.frictionResult = frictionResult;
    const qi = tmpQuat1;
    const qj = tmpQuat2;
    const xi = tmpVec1;
    const xj = tmpVec2;
    for (let k = 0, N = p1.length; k !== N; k++) {
      const bi = p1[k];
      const bj = p2[k];
      let bodyContactMaterial = null;
      if (bi.material && bj.material) {
        bodyContactMaterial =
          world.getContactMaterial(bi.material, bj.material) || null;
      }
      const justTest = (bi.type & Body.KINEMATIC && bj.type & Body.STATIC) ||
        (bi.type & Body.STATIC && bj.type & Body.KINEMATIC) ||
        (bi.type & Body.KINEMATIC && bj.type & Body.KINEMATIC);
      for (let i = 0; i < bi.shapes.length; i++) {
        bi.quaternion.mult(bi.shapeOrientations[i], qi);
        bi.quaternion.vmult(bi.shapeOffsets[i], xi);
        xi.vadd(bi.position, xi);
        const si = bi.shapes[i];
        for (let j = 0; j < bj.shapes.length; j++) {
          bj.quaternion.mult(bj.shapeOrientations[j], qj);
          bj.quaternion.vmult(bj.shapeOffsets[j], xj);
          xj.vadd(bj.position, xj);
          const sj = bj.shapes[j];
          if (
            !(si.collisionFilterMask & sj.collisionFilterGroup &&
              sj.collisionFilterMask & si.collisionFilterGroup)
          ) {
            continue;
          }
          if (
            xi.distanceTo(xj) >
              si.boundingSphereRadius + sj.boundingSphereRadius
          ) {
            continue;
          }
          let shapeContactMaterial = null;
          if (si.material && sj.material) {
            shapeContactMaterial =
              world.getContactMaterial(si.material, sj.material) || null;
          }
          this.currentContactMaterial = shapeContactMaterial ||
            bodyContactMaterial || world.defaultContactMaterial;
          const resolverIndex = (si.type | sj.type);
          const resolver = this[resolverIndex];
          if (resolver) {
            let retval = false;
            if (si.type < sj.type) {
              retval = resolver.call(
                this,
                si,
                sj,
                xi,
                xj,
                qi,
                qj,
                bi,
                bj,
                si,
                sj,
                justTest,
              );
            } else {
              retval = resolver.call(
                this,
                sj,
                si,
                xj,
                xi,
                qj,
                qi,
                bj,
                bi,
                si,
                sj,
                justTest,
              );
            }
            if (retval && justTest) {
              world.shapeOverlapKeeper.set(si.id, sj.id);
              world.bodyOverlapKeeper.set(bi.id, bj.id);
            }
          }
        }
      }
    }
  }
  sphereSphere(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    if (justTest) {
      return xi.distanceSquared(xj) < (si.radius + sj.radius) ** 2;
    }
    const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
    xj.vsub(xi, r.ni);
    r.ni.normalize();
    r.ri.copy(r.ni);
    r.rj.copy(r.ni);
    r.ri.scale(si.radius, r.ri);
    r.rj.scale(-sj.radius, r.rj);
    r.ri.vadd(xi, r.ri);
    r.ri.vsub(bi.position, r.ri);
    r.rj.vadd(xj, r.rj);
    r.rj.vsub(bj.position, r.rj);
    this.result.push(r);
    this.createFrictionEquationsFromContact(r, this.frictionResult);
  }
  spherePlane(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
    r.ni.set(0, 0, 1);
    qj.vmult(r.ni, r.ni);
    r.ni.negate(r.ni);
    r.ni.normalize();
    r.ni.scale(si.radius, r.ri);
    xi.vsub(xj, point_on_plane_to_sphere);
    r.ni.scale(r.ni.dot(point_on_plane_to_sphere), plane_to_sphere_ortho);
    point_on_plane_to_sphere.vsub(plane_to_sphere_ortho, r.rj);
    if (-point_on_plane_to_sphere.dot(r.ni) <= si.radius) {
      if (justTest) {
        return true;
      }
      const ri = r.ri;
      const rj = r.rj;
      ri.vadd(xi, ri);
      ri.vsub(bi.position, ri);
      rj.vadd(xj, rj);
      rj.vsub(bj.position, rj);
      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }
  }
  boxBox(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    si.convexPolyhedronRepresentation.material = si.material;
    sj.convexPolyhedronRepresentation.material = sj.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
    return this.convexConvex(
      si.convexPolyhedronRepresentation,
      sj.convexPolyhedronRepresentation,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      si,
      sj,
      justTest,
    );
  }
  sphereBox(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    const v3pool = this.v3pool;
    const sides = sphereBox_sides;
    xi.vsub(xj, box_to_sphere);
    sj.getSideNormals(sides, qj);
    const R = si.radius;
    const penetrating_sides = [];
    let found = false;
    const side_ns = sphereBox_side_ns;
    const side_ns1 = sphereBox_side_ns1;
    const side_ns2 = sphereBox_side_ns2;
    let side_h = null;
    let side_penetrations = 0;
    let side_dot1 = 0;
    let side_dot2 = 0;
    let side_distance = null;
    for (
      let idx = 0, nsides = sides.length;
      idx !== nsides && found === false;
      idx++
    ) {
      const ns = sphereBox_ns;
      ns.copy(sides[idx]);
      const h = ns.length();
      ns.normalize();
      const dot = box_to_sphere.dot(ns);
      if (dot < h + R && dot > 0) {
        const ns1 = sphereBox_ns1;
        const ns2 = sphereBox_ns2;
        ns1.copy(sides[(idx + 1) % 3]);
        ns2.copy(sides[(idx + 2) % 3]);
        const h1 = ns1.length();
        const h2 = ns2.length();
        ns1.normalize();
        ns2.normalize();
        const dot1 = box_to_sphere.dot(ns1);
        const dot2 = box_to_sphere.dot(ns2);
        if (dot1 < h1 && dot1 > -h1 && dot2 < h2 && dot2 > -h2) {
          const dist = Math.abs(dot - h - R);
          if (side_distance === null || dist < side_distance) {
            side_distance = dist;
            side_dot1 = dot1;
            side_dot2 = dot2;
            side_h = h;
            side_ns.copy(ns);
            side_ns1.copy(ns1);
            side_ns2.copy(ns2);
            side_penetrations++;
            if (justTest) {
              return true;
            }
          }
        }
      }
    }
    if (side_penetrations) {
      found = true;
      const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      side_ns.scale(-R, r.ri);
      r.ni.copy(side_ns);
      r.ni.negate(r.ni);
      side_ns.scale(side_h, side_ns);
      side_ns1.scale(side_dot1, side_ns1);
      side_ns.vadd(side_ns1, side_ns);
      side_ns2.scale(side_dot2, side_ns2);
      side_ns.vadd(side_ns2, r.rj);
      r.ri.vadd(xi, r.ri);
      r.ri.vsub(bi.position, r.ri);
      r.rj.vadd(xj, r.rj);
      r.rj.vsub(bj.position, r.rj);
      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }
    let rj = v3pool.get();
    const sphere_to_corner = sphereBox_sphere_to_corner;
    for (let j = 0; j !== 2 && !found; j++) {
      for (let k = 0; k !== 2 && !found; k++) {
        for (let l = 0; l !== 2 && !found; l++) {
          rj.set(0, 0, 0);
          if (j) {
            rj.vadd(sides[0], rj);
          } else {
            rj.vsub(sides[0], rj);
          }
          if (k) {
            rj.vadd(sides[1], rj);
          } else {
            rj.vsub(sides[1], rj);
          }
          if (l) {
            rj.vadd(sides[2], rj);
          } else {
            rj.vsub(sides[2], rj);
          }
          xj.vadd(rj, sphere_to_corner);
          sphere_to_corner.vsub(xi, sphere_to_corner);
          if (sphere_to_corner.lengthSquared() < R * R) {
            if (justTest) {
              return true;
            }
            found = true;
            const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            r.ri.copy(sphere_to_corner);
            r.ri.normalize();
            r.ni.copy(r.ri);
            r.ri.scale(R, r.ri);
            r.rj.copy(rj);
            r.ri.vadd(xi, r.ri);
            r.ri.vsub(bi.position, r.ri);
            r.rj.vadd(xj, r.rj);
            r.rj.vsub(bj.position, r.rj);
            this.result.push(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
          }
        }
      }
    }
    v3pool.release(rj);
    rj = null;
    const edgeTangent = v3pool.get();
    const edgeCenter = v3pool.get();
    const r = v3pool.get();
    const orthogonal = v3pool.get();
    const dist = v3pool.get();
    const Nsides = sides.length;
    for (let j = 0; j !== Nsides && !found; j++) {
      for (let k = 0; k !== Nsides && !found; k++) {
        if (j % 3 !== k % 3) {
          sides[k].cross(sides[j], edgeTangent);
          edgeTangent.normalize();
          sides[j].vadd(sides[k], edgeCenter);
          r.copy(xi);
          r.vsub(edgeCenter, r);
          r.vsub(xj, r);
          const orthonorm = r.dot(edgeTangent);
          edgeTangent.scale(orthonorm, orthogonal);
          let l = 0;
          while (l === j % 3 || l === k % 3) {
            l++;
          }
          dist.copy(xi);
          dist.vsub(orthogonal, dist);
          dist.vsub(edgeCenter, dist);
          dist.vsub(xj, dist);
          const tdist = Math.abs(orthonorm);
          const ndist = dist.length();
          if (tdist < sides[l].length() && ndist < R) {
            if (justTest) {
              return true;
            }
            found = true;
            const res = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
            edgeCenter.vadd(orthogonal, res.rj);
            res.rj.copy(res.rj);
            dist.negate(res.ni);
            res.ni.normalize();
            res.ri.copy(res.rj);
            res.ri.vadd(xj, res.ri);
            res.ri.vsub(xi, res.ri);
            res.ri.normalize();
            res.ri.scale(R, res.ri);
            res.ri.vadd(xi, res.ri);
            res.ri.vsub(bi.position, res.ri);
            res.rj.vadd(xj, res.rj);
            res.rj.vsub(bj.position, res.rj);
            this.result.push(res);
            this.createFrictionEquationsFromContact(res, this.frictionResult);
          }
        }
      }
    }
    v3pool.release(edgeTangent, edgeCenter, r, orthogonal, dist);
  }
  planeBox(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    sj.convexPolyhedronRepresentation.material = sj.material;
    sj.convexPolyhedronRepresentation.collisionResponse = sj.collisionResponse;
    sj.convexPolyhedronRepresentation.id = sj.id;
    return this.planeConvex(
      si,
      sj.convexPolyhedronRepresentation,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      si,
      sj,
      justTest,
    );
  }
  convexConvex(
    si,
    sj,
    xi,
    xj,
    qi,
    qj,
    bi,
    bj,
    rsi,
    rsj,
    justTest,
    faceListA,
    faceListB,
  ) {
    const sepAxis = convexConvex_sepAxis;
    if (xi.distanceTo(xj) > si.boundingSphereRadius + sj.boundingSphereRadius) {
      return;
    }
    if (
      si.findSeparatingAxis(sj, xi, qi, xj, qj, sepAxis, faceListA, faceListB)
    ) {
      const res = [];
      const q = convexConvex_q;
      si.clipAgainstHull(xi, qi, sj, xj, qj, sepAxis, -100, 100, res);
      let numContacts = 0;
      for (let j = 0; j !== res.length; j++) {
        if (justTest) {
          return true;
        }
        const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        const ri = r.ri;
        const rj = r.rj;
        sepAxis.negate(r.ni);
        res[j].normal.negate(q);
        q.scale(res[j].depth, q);
        res[j].point.vadd(q, ri);
        rj.copy(res[j].point);
        ri.vsub(xi, ri);
        rj.vsub(xj, rj);
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);
        this.result.push(r);
        numContacts++;
        if (!this.enableFrictionReduction) {
          this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
      }
      if (this.enableFrictionReduction && numContacts) {
        this.createFrictionFromAverage(numContacts);
      }
    }
  }
  sphereConvex(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    const v3pool = this.v3pool;
    xi.vsub(xj, convex_to_sphere);
    const normals = sj.faceNormals;
    const faces = sj.faces;
    const verts = sj.vertices;
    const R = si.radius;
    const penetrating_sides = [];
    let found = false;
    for (let i = 0; i !== verts.length; i++) {
      const v = verts[i];
      const worldCorner = sphereConvex_worldCorner;
      qj.vmult(v, worldCorner);
      xj.vadd(worldCorner, worldCorner);
      const sphere_to_corner = sphereConvex_sphereToCorner;
      worldCorner.vsub(xi, sphere_to_corner);
      if (sphere_to_corner.lengthSquared() < R * R) {
        if (justTest) {
          return true;
        }
        found = true;
        const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        r.ri.copy(sphere_to_corner);
        r.ri.normalize();
        r.ni.copy(r.ri);
        r.ri.scale(R, r.ri);
        worldCorner.vsub(xj, r.rj);
        r.ri.vadd(xi, r.ri);
        r.ri.vsub(bi.position, r.ri);
        r.rj.vadd(xj, r.rj);
        r.rj.vsub(bj.position, r.rj);
        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
        return;
      }
    }
    for (
      let i = 0, nfaces = faces.length; i !== nfaces && found === false; i++
    ) {
      const normal = normals[i];
      const face = faces[i];
      const worldNormal = sphereConvex_worldNormal;
      qj.vmult(normal, worldNormal);
      const worldPoint = sphereConvex_worldPoint;
      qj.vmult(verts[face[0]], worldPoint);
      worldPoint.vadd(xj, worldPoint);
      const worldSpherePointClosestToPlane =
        sphereConvex_worldSpherePointClosestToPlane;
      worldNormal.scale(-R, worldSpherePointClosestToPlane);
      xi.vadd(worldSpherePointClosestToPlane, worldSpherePointClosestToPlane);
      const penetrationVec = sphereConvex_penetrationVec;
      worldSpherePointClosestToPlane.vsub(worldPoint, penetrationVec);
      const penetration = penetrationVec.dot(worldNormal);
      const worldPointToSphere = sphereConvex_sphereToWorldPoint;
      xi.vsub(worldPoint, worldPointToSphere);
      if (penetration < 0 && worldPointToSphere.dot(worldNormal) > 0) {
        const faceVerts = [];
        for (let j = 0, Nverts = face.length; j !== Nverts; j++) {
          const worldVertex = v3pool.get();
          qj.vmult(verts[face[j]], worldVertex);
          xj.vadd(worldVertex, worldVertex);
          faceVerts.push(worldVertex);
        }
        if (pointInPolygon(faceVerts, worldNormal, xi)) {
          if (justTest) {
            return true;
          }
          found = true;
          const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
          worldNormal.scale(-R, r.ri);
          worldNormal.negate(r.ni);
          const penetrationVec2 = v3pool.get();
          worldNormal.scale(-penetration, penetrationVec2);
          const penetrationSpherePoint = v3pool.get();
          worldNormal.scale(-R, penetrationSpherePoint);
          xi.vsub(xj, r.rj);
          r.rj.vadd(penetrationSpherePoint, r.rj);
          r.rj.vadd(penetrationVec2, r.rj);
          r.rj.vadd(xj, r.rj);
          r.rj.vsub(bj.position, r.rj);
          r.ri.vadd(xi, r.ri);
          r.ri.vsub(bi.position, r.ri);
          v3pool.release(penetrationVec2);
          v3pool.release(penetrationSpherePoint);
          this.result.push(r);
          this.createFrictionEquationsFromContact(r, this.frictionResult);
          for (
            let j = 0, Nfaceverts = faceVerts.length; j !== Nfaceverts; j++
          ) {
            v3pool.release(faceVerts[j]);
          }
          return;
        } else {
          for (let j = 0; j !== face.length; j++) {
            const v1 = v3pool.get();
            const v2 = v3pool.get();
            qj.vmult(verts[face[(j + 1) % face.length]], v1);
            qj.vmult(verts[face[(j + 2) % face.length]], v2);
            xj.vadd(v1, v1);
            xj.vadd(v2, v2);
            const edge = sphereConvex_edge;
            v2.vsub(v1, edge);
            const edgeUnit = sphereConvex_edgeUnit;
            edge.unit(edgeUnit);
            const p = v3pool.get();
            const v1_to_xi = v3pool.get();
            xi.vsub(v1, v1_to_xi);
            const dot = v1_to_xi.dot(edgeUnit);
            edgeUnit.scale(dot, p);
            p.vadd(v1, p);
            const xi_to_p = v3pool.get();
            p.vsub(xi, xi_to_p);
            if (
              dot > 0 && dot * dot < edge.lengthSquared() &&
              xi_to_p.lengthSquared() < R * R
            ) {
              if (justTest) {
                return true;
              }
              const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
              p.vsub(xj, r.rj);
              p.vsub(xi, r.ni);
              r.ni.normalize();
              r.ni.scale(R, r.ri);
              r.rj.vadd(xj, r.rj);
              r.rj.vsub(bj.position, r.rj);
              r.ri.vadd(xi, r.ri);
              r.ri.vsub(bi.position, r.ri);
              this.result.push(r);
              this.createFrictionEquationsFromContact(r, this.frictionResult);
              for (
                let j = 0, Nfaceverts = faceVerts.length; j !== Nfaceverts; j++
              ) {
                v3pool.release(faceVerts[j]);
              }
              v3pool.release(v1);
              v3pool.release(v2);
              v3pool.release(p);
              v3pool.release(xi_to_p);
              v3pool.release(v1_to_xi);
              return;
            }
            v3pool.release(v1);
            v3pool.release(v2);
            v3pool.release(p);
            v3pool.release(xi_to_p);
            v3pool.release(v1_to_xi);
          }
        }
        for (let j = 0, Nfaceverts = faceVerts.length; j !== Nfaceverts; j++) {
          v3pool.release(faceVerts[j]);
        }
      }
    }
  }
  planeConvex(
    planeShape,
    convexShape,
    planePosition,
    convexPosition,
    planeQuat,
    convexQuat,
    planeBody,
    convexBody,
    si,
    sj,
    justTest,
  ) {
    const worldVertex = planeConvex_v;
    const worldNormal = planeConvex_normal;
    worldNormal.set(0, 0, 1);
    planeQuat.vmult(worldNormal, worldNormal);
    let numContacts = 0;
    const relpos = planeConvex_relpos;
    for (let i = 0; i !== convexShape.vertices.length; i++) {
      worldVertex.copy(convexShape.vertices[i]);
      convexQuat.vmult(worldVertex, worldVertex);
      convexPosition.vadd(worldVertex, worldVertex);
      worldVertex.vsub(planePosition, relpos);
      const dot = worldNormal.dot(relpos);
      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }
        const r = this.createContactEquation(
          planeBody,
          convexBody,
          planeShape,
          convexShape,
          si,
          sj,
        );
        const projected = planeConvex_projected;
        worldNormal.scale(worldNormal.dot(relpos), projected);
        worldVertex.vsub(projected, projected);
        projected.vsub(planePosition, r.ri);
        r.ni.copy(worldNormal);
        worldVertex.vsub(convexPosition, r.rj);
        r.ri.vadd(planePosition, r.ri);
        r.ri.vsub(planeBody.position, r.ri);
        r.rj.vadd(convexPosition, r.rj);
        r.rj.vsub(convexBody.position, r.rj);
        this.result.push(r);
        numContacts++;
        if (!this.enableFrictionReduction) {
          this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
      }
    }
    if (this.enableFrictionReduction && numContacts) {
      this.createFrictionFromAverage(numContacts);
    }
  }
  boxConvex(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return this.convexConvex(
      si.convexPolyhedronRepresentation,
      sj,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      si,
      sj,
      justTest,
    );
  }
  sphereHeightfield(
    sphereShape,
    hfShape,
    spherePos,
    hfPos,
    sphereQuat,
    hfQuat,
    sphereBody,
    hfBody,
    rsi,
    rsj,
    justTest,
  ) {
    const data = hfShape.data;
    const radius = sphereShape.radius;
    const w = hfShape.elementSize;
    const worldPillarOffset = sphereHeightfield_tmp2;
    const localSpherePos = sphereHeightfield_tmp1;
    Transform.pointToLocalFrame(hfPos, hfQuat, spherePos, localSpherePos);
    let iMinX = Math.floor((localSpherePos.x - radius) / w) - 1;
    let iMaxX = Math.ceil((localSpherePos.x + radius) / w) + 1;
    let iMinY = Math.floor((localSpherePos.y - radius) / w) - 1;
    let iMaxY = Math.ceil((localSpherePos.y + radius) / w) + 1;
    if (
      iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length
    ) {
      return;
    }
    if (iMinX < 0) {
      iMinX = 0;
    }
    if (iMaxX < 0) {
      iMaxX = 0;
    }
    if (iMinY < 0) {
      iMinY = 0;
    }
    if (iMaxY < 0) {
      iMaxY = 0;
    }
    if (iMinX >= data.length) {
      iMinX = data.length - 1;
    }
    if (iMaxX >= data.length) {
      iMaxX = data.length - 1;
    }
    if (iMaxY >= data[0].length) {
      iMaxY = data[0].length - 1;
    }
    if (iMinY >= data[0].length) {
      iMinY = data[0].length - 1;
    }
    const minMax = [];
    hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    const min = minMax[0];
    const max = minMax[1];
    if (localSpherePos.z - radius > max || localSpherePos.z + radius < min) {
      return;
    }
    const result = this.result;
    for (let i = iMinX; i < iMaxX; i++) {
      for (let j = iMinY; j < iMaxY; j++) {
        const numContactsBefore = result.length;
        let intersecting = false;
        hfShape.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(
          hfPos,
          hfQuat,
          hfShape.pillarOffset,
          worldPillarOffset,
        );
        if (
          spherePos.distanceTo(worldPillarOffset) <
            hfShape.pillarConvex.boundingSphereRadius +
              sphereShape.boundingSphereRadius
        ) {
          intersecting = this.sphereConvex(
            sphereShape,
            hfShape.pillarConvex,
            spherePos,
            worldPillarOffset,
            sphereQuat,
            hfQuat,
            sphereBody,
            hfBody,
            sphereShape,
            hfShape,
            justTest,
          );
        }
        if (justTest && intersecting) {
          return true;
        }
        hfShape.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(
          hfPos,
          hfQuat,
          hfShape.pillarOffset,
          worldPillarOffset,
        );
        if (
          spherePos.distanceTo(worldPillarOffset) <
            hfShape.pillarConvex.boundingSphereRadius +
              sphereShape.boundingSphereRadius
        ) {
          intersecting = this.sphereConvex(
            sphereShape,
            hfShape.pillarConvex,
            spherePos,
            worldPillarOffset,
            sphereQuat,
            hfQuat,
            sphereBody,
            hfBody,
            sphereShape,
            hfShape,
            justTest,
          );
        }
        if (justTest && intersecting) {
          return true;
        }
        const numContacts = result.length - numContactsBefore;
        if (numContacts > 2) {
          return;
        }
      }
    }
  }
  boxHeightfield(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return this.convexHeightfield(
      si.convexPolyhedronRepresentation,
      sj,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      si,
      sj,
      justTest,
    );
  }
  convexHeightfield(
    convexShape,
    hfShape,
    convexPos,
    hfPos,
    convexQuat,
    hfQuat,
    convexBody,
    hfBody,
    rsi,
    rsj,
    justTest,
  ) {
    const data = hfShape.data;
    const w = hfShape.elementSize;
    const radius = convexShape.boundingSphereRadius;
    const worldPillarOffset = convexHeightfield_tmp2;
    const faceList = convexHeightfield_faceList;
    const localConvexPos = convexHeightfield_tmp1;
    Transform.pointToLocalFrame(hfPos, hfQuat, convexPos, localConvexPos);
    let iMinX = Math.floor((localConvexPos.x - radius) / w) - 1;
    let iMaxX = Math.ceil((localConvexPos.x + radius) / w) + 1;
    let iMinY = Math.floor((localConvexPos.y - radius) / w) - 1;
    let iMaxY = Math.ceil((localConvexPos.y + radius) / w) + 1;
    if (
      iMaxX < 0 || iMaxY < 0 || iMinX > data.length || iMinY > data[0].length
    ) {
      return;
    }
    if (iMinX < 0) {
      iMinX = 0;
    }
    if (iMaxX < 0) {
      iMaxX = 0;
    }
    if (iMinY < 0) {
      iMinY = 0;
    }
    if (iMaxY < 0) {
      iMaxY = 0;
    }
    if (iMinX >= data.length) {
      iMinX = data.length - 1;
    }
    if (iMaxX >= data.length) {
      iMaxX = data.length - 1;
    }
    if (iMaxY >= data[0].length) {
      iMaxY = data[0].length - 1;
    }
    if (iMinY >= data[0].length) {
      iMinY = data[0].length - 1;
    }
    const minMax = [];
    hfShape.getRectMinMax(iMinX, iMinY, iMaxX, iMaxY, minMax);
    const min = minMax[0];
    const max = minMax[1];
    if (localConvexPos.z - radius > max || localConvexPos.z + radius < min) {
      return;
    }
    for (let i = iMinX; i < iMaxX; i++) {
      for (let j = iMinY; j < iMaxY; j++) {
        let intersecting = false;
        hfShape.getConvexTrianglePillar(i, j, false);
        Transform.pointToWorldFrame(
          hfPos,
          hfQuat,
          hfShape.pillarOffset,
          worldPillarOffset,
        );
        if (
          convexPos.distanceTo(worldPillarOffset) <
            hfShape.pillarConvex.boundingSphereRadius +
              convexShape.boundingSphereRadius
        ) {
          intersecting = this.convexConvex(
            convexShape,
            hfShape.pillarConvex,
            convexPos,
            worldPillarOffset,
            convexQuat,
            hfQuat,
            convexBody,
            hfBody,
            null,
            null,
            justTest,
            faceList,
            null,
          );
        }
        if (justTest && intersecting) {
          return true;
        }
        hfShape.getConvexTrianglePillar(i, j, true);
        Transform.pointToWorldFrame(
          hfPos,
          hfQuat,
          hfShape.pillarOffset,
          worldPillarOffset,
        );
        if (
          convexPos.distanceTo(worldPillarOffset) <
            hfShape.pillarConvex.boundingSphereRadius +
              convexShape.boundingSphereRadius
        ) {
          intersecting = this.convexConvex(
            convexShape,
            hfShape.pillarConvex,
            convexPos,
            worldPillarOffset,
            convexQuat,
            hfQuat,
            convexBody,
            hfBody,
            null,
            null,
            justTest,
            faceList,
            null,
          );
        }
        if (justTest && intersecting) {
          return true;
        }
      }
    }
  }
  sphereParticle(sj, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest) {
    const normal = particleSphere_normal;
    normal.set(0, 0, 1);
    xi.vsub(xj, normal);
    const lengthSquared = normal.lengthSquared();
    if (lengthSquared <= sj.radius * sj.radius) {
      if (justTest) {
        return true;
      }
      const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      normal.normalize();
      r.rj.copy(normal);
      r.rj.scale(sj.radius, r.rj);
      r.ni.copy(normal);
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0);
      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }
  }
  planeParticle(sj, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest) {
    const normal = particlePlane_normal;
    normal.set(0, 0, 1);
    bj.quaternion.vmult(normal, normal);
    const relpos = particlePlane_relpos;
    xi.vsub(bj.position, relpos);
    const dot = normal.dot(relpos);
    if (dot <= 0.0) {
      if (justTest) {
        return true;
      }
      const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
      r.ni.copy(normal);
      r.ni.negate(r.ni);
      r.ri.set(0, 0, 0);
      const projected = particlePlane_projected;
      normal.scale(normal.dot(xi), projected);
      xi.vsub(projected, projected);
      r.rj.copy(projected);
      this.result.push(r);
      this.createFrictionEquationsFromContact(r, this.frictionResult);
    }
  }
  boxParticle(si, sj, xi, xj, qi, qj, bi, bj, rsi, rsj, justTest) {
    si.convexPolyhedronRepresentation.material = si.material;
    si.convexPolyhedronRepresentation.collisionResponse = si.collisionResponse;
    return this.convexParticle(
      si.convexPolyhedronRepresentation,
      sj,
      xi,
      xj,
      qi,
      qj,
      bi,
      bj,
      si,
      sj,
      justTest,
    );
  }
  convexParticle(sj, si, xj, xi, qj, qi, bj, bi, rsi, rsj, justTest) {
    let penetratedFaceIndex = -1;
    const penetratedFaceNormal = convexParticle_penetratedFaceNormal;
    const worldPenetrationVec = convexParticle_worldPenetrationVec;
    let minPenetration = null;
    let numDetectedFaces = 0;
    const local = convexParticle_local;
    local.copy(xi);
    local.vsub(xj, local);
    qj.conjugate(cqj);
    cqj.vmult(local, local);
    if (sj.pointIsInside(local)) {
      if (sj.worldVerticesNeedsUpdate) {
        sj.computeWorldVertices(xj, qj);
      }
      if (sj.worldFaceNormalsNeedsUpdate) {
        sj.computeWorldFaceNormals(qj);
      }
      for (let i = 0, nfaces = sj.faces.length; i !== nfaces; i++) {
        const verts = [sj.worldVertices[sj.faces[i][0]]];
        const normal = sj.worldFaceNormals[i];
        xi.vsub(verts[0], convexParticle_vertexToParticle);
        const penetration = -normal.dot(convexParticle_vertexToParticle);
        if (
          minPenetration === null ||
          Math.abs(penetration) < Math.abs(minPenetration)
        ) {
          if (justTest) {
            return true;
          }
          minPenetration = penetration;
          penetratedFaceIndex = i;
          penetratedFaceNormal.copy(normal);
          numDetectedFaces++;
        }
      }
      if (penetratedFaceIndex !== -1) {
        const r = this.createContactEquation(bi, bj, si, sj, rsi, rsj);
        penetratedFaceNormal.scale(minPenetration, worldPenetrationVec);
        worldPenetrationVec.vadd(xi, worldPenetrationVec);
        worldPenetrationVec.vsub(xj, worldPenetrationVec);
        r.rj.copy(worldPenetrationVec);
        penetratedFaceNormal.negate(r.ni);
        r.ri.set(0, 0, 0);
        const ri = r.ri;
        const rj = r.rj;
        ri.vadd(xi, ri);
        ri.vsub(bi.position, ri);
        rj.vadd(xj, rj);
        rj.vsub(bj.position, rj);
        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      } else {
        console.warn(
          "Point found inside convex, but did not find penetrating face!",
        );
      }
    }
  }
  sphereTrimesh(
    sphereShape,
    trimeshShape,
    spherePos,
    trimeshPos,
    sphereQuat,
    trimeshQuat,
    sphereBody,
    trimeshBody,
    rsi,
    rsj,
    justTest,
  ) {
    const edgeVertexA = sphereTrimesh_edgeVertexA;
    const edgeVertexB = sphereTrimesh_edgeVertexB;
    const edgeVector = sphereTrimesh_edgeVector;
    const edgeVectorUnit = sphereTrimesh_edgeVectorUnit;
    const localSpherePos = sphereTrimesh_localSpherePos;
    const tmp = sphereTrimesh_tmp;
    const localSphereAABB = sphereTrimesh_localSphereAABB;
    const v2 = sphereTrimesh_v2;
    const relpos = sphereTrimesh_relpos;
    const triangles = sphereTrimesh_triangles;
    Transform.pointToLocalFrame(
      trimeshPos,
      trimeshQuat,
      spherePos,
      localSpherePos,
    );
    const sphereRadius = sphereShape.radius;
    localSphereAABB.lowerBound.set(
      localSpherePos.x - sphereRadius,
      localSpherePos.y - sphereRadius,
      localSpherePos.z - sphereRadius,
    );
    localSphereAABB.upperBound.set(
      localSpherePos.x + sphereRadius,
      localSpherePos.y + sphereRadius,
      localSpherePos.z + sphereRadius,
    );
    trimeshShape.getTrianglesInAABB(localSphereAABB, triangles);
    const v = sphereTrimesh_v;
    const radiusSquared = sphereShape.radius * sphereShape.radius;
    for (let i = 0; i < triangles.length; i++) {
      for (let j = 0; j < 3; j++) {
        trimeshShape.getVertex(trimeshShape.indices[triangles[i] * 3 + j], v);
        v.vsub(localSpherePos, relpos);
        if (relpos.lengthSquared() <= radiusSquared) {
          v2.copy(v);
          Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);
          v.vsub(spherePos, relpos);
          if (justTest) {
            return true;
          }
          let r = this.createContactEquation(
            sphereBody,
            trimeshBody,
            sphereShape,
            trimeshShape,
            rsi,
            rsj,
          );
          r.ni.copy(relpos);
          r.ni.normalize();
          r.ri.copy(r.ni);
          r.ri.scale(sphereShape.radius, r.ri);
          r.ri.vadd(spherePos, r.ri);
          r.ri.vsub(sphereBody.position, r.ri);
          r.rj.copy(v);
          r.rj.vsub(trimeshBody.position, r.rj);
          this.result.push(r);
          this.createFrictionEquationsFromContact(r, this.frictionResult);
        }
      }
    }
    for (let i = 0; i < triangles.length; i++) {
      for (let j = 0; j < 3; j++) {
        trimeshShape.getVertex(
          trimeshShape.indices[triangles[i] * 3 + j],
          edgeVertexA,
        );
        trimeshShape.getVertex(
          trimeshShape.indices[triangles[i] * 3 + ((j + 1) % 3)],
          edgeVertexB,
        );
        edgeVertexB.vsub(edgeVertexA, edgeVector);
        localSpherePos.vsub(edgeVertexB, tmp);
        const positionAlongEdgeB = tmp.dot(edgeVector);
        localSpherePos.vsub(edgeVertexA, tmp);
        let positionAlongEdgeA = tmp.dot(edgeVector);
        if (positionAlongEdgeA > 0 && positionAlongEdgeB < 0) {
          localSpherePos.vsub(edgeVertexA, tmp);
          edgeVectorUnit.copy(edgeVector);
          edgeVectorUnit.normalize();
          positionAlongEdgeA = tmp.dot(edgeVectorUnit);
          edgeVectorUnit.scale(positionAlongEdgeA, tmp);
          tmp.vadd(edgeVertexA, tmp);
          const dist = tmp.distanceTo(localSpherePos);
          if (dist < sphereShape.radius) {
            if (justTest) {
              return true;
            }
            const r = this.createContactEquation(
              sphereBody,
              trimeshBody,
              sphereShape,
              trimeshShape,
              rsi,
              rsj,
            );
            tmp.vsub(localSpherePos, r.ni);
            r.ni.normalize();
            r.ni.scale(sphereShape.radius, r.ri);
            r.ri.vadd(spherePos, r.ri);
            r.ri.vsub(sphereBody.position, r.ri);
            Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
            tmp.vsub(trimeshBody.position, r.rj);
            Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
            Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);
            this.result.push(r);
            this.createFrictionEquationsFromContact(r, this.frictionResult);
          }
        }
      }
    }
    const va = sphereTrimesh_va;
    const vb = sphereTrimesh_vb;
    const vc = sphereTrimesh_vc;
    const normal = sphereTrimesh_normal;
    for (let i = 0, N = triangles.length; i !== N; i++) {
      trimeshShape.getTriangleVertices(triangles[i], va, vb, vc);
      trimeshShape.getNormal(triangles[i], normal);
      localSpherePos.vsub(va, tmp);
      let dist = tmp.dot(normal);
      normal.scale(dist, tmp);
      localSpherePos.vsub(tmp, tmp);
      dist = tmp.distanceTo(localSpherePos);
      if (Ray.pointInTriangle(tmp, va, vb, vc) && dist < sphereShape.radius) {
        if (justTest) {
          return true;
        }
        let r = this.createContactEquation(
          sphereBody,
          trimeshBody,
          sphereShape,
          trimeshShape,
          rsi,
          rsj,
        );
        tmp.vsub(localSpherePos, r.ni);
        r.ni.normalize();
        r.ni.scale(sphereShape.radius, r.ri);
        r.ri.vadd(spherePos, r.ri);
        r.ri.vsub(sphereBody.position, r.ri);
        Transform.pointToWorldFrame(trimeshPos, trimeshQuat, tmp, tmp);
        tmp.vsub(trimeshBody.position, r.rj);
        Transform.vectorToWorldFrame(trimeshQuat, r.ni, r.ni);
        Transform.vectorToWorldFrame(trimeshQuat, r.ri, r.ri);
        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      }
    }
    triangles.length = 0;
  }
  planeTrimesh(
    planeShape,
    trimeshShape,
    planePos,
    trimeshPos,
    planeQuat,
    trimeshQuat,
    planeBody,
    trimeshBody,
    rsi,
    rsj,
    justTest,
  ) {
    const v = new Vec3();
    const normal = planeTrimesh_normal;
    normal.set(0, 0, 1);
    planeQuat.vmult(normal, normal);
    for (let i = 0; i < trimeshShape.vertices.length / 3; i++) {
      trimeshShape.getVertex(i, v);
      const v2 = new Vec3();
      v2.copy(v);
      Transform.pointToWorldFrame(trimeshPos, trimeshQuat, v2, v);
      const relpos = planeTrimesh_relpos;
      v.vsub(planePos, relpos);
      const dot = normal.dot(relpos);
      if (dot <= 0.0) {
        if (justTest) {
          return true;
        }
        const r = this.createContactEquation(
          planeBody,
          trimeshBody,
          planeShape,
          trimeshShape,
          rsi,
          rsj,
        );
        r.ni.copy(normal);
        const projected = planeTrimesh_projected;
        normal.scale(relpos.dot(normal), projected);
        v.vsub(projected, projected);
        r.ri.copy(projected);
        r.ri.vsub(planeBody.position, r.ri);
        r.rj.copy(v);
        r.rj.vsub(trimeshBody.position, r.rj);
        this.result.push(r);
        this.createFrictionEquationsFromContact(r, this.frictionResult);
      }
    }
  }
}
COLLISION_TYPES.sphereSphere,
  COLLISION_TYPES.spherePlane,
  COLLISION_TYPES.boxBox,
  COLLISION_TYPES.sphereBox,
  COLLISION_TYPES.planeBox,
  COLLISION_TYPES.convexConvex,
  COLLISION_TYPES.sphereConvex,
  COLLISION_TYPES.planeConvex,
  COLLISION_TYPES.boxConvex,
  COLLISION_TYPES.sphereHeightfield,
  COLLISION_TYPES.boxHeightfield,
  COLLISION_TYPES.convexHeightfield,
  COLLISION_TYPES.sphereParticle,
  COLLISION_TYPES.planeParticle,
  COLLISION_TYPES.boxParticle,
  COLLISION_TYPES.convexParticle,
  COLLISION_TYPES.sphereTrimesh,
  COLLISION_TYPES.planeTrimesh;
const averageNormal = new Vec3();
const averageContactPointA = new Vec3();
const averageContactPointB = new Vec3();
const tmpVec1 = new Vec3();
const tmpVec2 = new Vec3();
const tmpQuat1 = new Quaternion();
const tmpQuat2 = new Quaternion();
let numWarnings = 0;
const maxWarnings = 10;
function warn(msg) {
  if (numWarnings > maxWarnings) {
    return;
  }
  numWarnings++;
  console.warn(msg);
}
Narrowphase.prototype[COLLISION_TYPES.boxBox] = Narrowphase.prototype.boxBox;
Narrowphase.prototype[COLLISION_TYPES.boxConvex] =
  Narrowphase.prototype.boxConvex;
Narrowphase.prototype[COLLISION_TYPES.boxParticle] =
  Narrowphase.prototype.boxParticle;
Narrowphase.prototype[COLLISION_TYPES.sphereSphere] =
  Narrowphase.prototype.sphereSphere;
const planeTrimesh_normal = new Vec3();
const planeTrimesh_relpos = new Vec3();
const planeTrimesh_projected = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.planeTrimesh] =
  Narrowphase.prototype.planeTrimesh;
const sphereTrimesh_normal = new Vec3();
const sphereTrimesh_relpos = new Vec3();
const sphereTrimesh_projected = new Vec3();
const sphereTrimesh_v = new Vec3();
const sphereTrimesh_v2 = new Vec3();
const sphereTrimesh_edgeVertexA = new Vec3();
const sphereTrimesh_edgeVertexB = new Vec3();
const sphereTrimesh_edgeVector = new Vec3();
const sphereTrimesh_edgeVectorUnit = new Vec3();
const sphereTrimesh_localSpherePos = new Vec3();
const sphereTrimesh_tmp = new Vec3();
const sphereTrimesh_va = new Vec3();
const sphereTrimesh_vb = new Vec3();
const sphereTrimesh_vc = new Vec3();
const sphereTrimesh_localSphereAABB = new AABB();
const sphereTrimesh_triangles = [];
Narrowphase.prototype[COLLISION_TYPES.sphereTrimesh] =
  Narrowphase.prototype.sphereTrimesh;
const point_on_plane_to_sphere = new Vec3();
const plane_to_sphere_ortho = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.spherePlane] =
  Narrowphase.prototype.spherePlane;
const pointInPolygon_edge = new Vec3();
const pointInPolygon_edge_x_normal = new Vec3();
const pointInPolygon_vtp = new Vec3();
function pointInPolygon(verts, normal, p) {
  let positiveResult = null;
  const N = verts.length;
  for (let i = 0; i !== N; i++) {
    const v = verts[i];
    const edge = pointInPolygon_edge;
    verts[(i + 1) % N].vsub(v, edge);
    const edge_x_normal = pointInPolygon_edge_x_normal;
    edge.cross(normal, edge_x_normal);
    const vertex_to_p = pointInPolygon_vtp;
    p.vsub(v, vertex_to_p);
    const r = edge_x_normal.dot(vertex_to_p);
    if (
      positiveResult === null || (r > 0 && positiveResult === true) ||
      (r <= 0 && positiveResult === false)
    ) {
      if (positiveResult === null) {
        positiveResult = r > 0;
      }
      continue;
    } else {
      return false;
    }
  }
  return true;
}
const box_to_sphere = new Vec3();
const sphereBox_ns = new Vec3();
const sphereBox_ns1 = new Vec3();
const sphereBox_ns2 = new Vec3();
const sphereBox_sides = [
  new Vec3(),
  new Vec3(),
  new Vec3(),
  new Vec3(),
  new Vec3(),
  new Vec3(),
];
const sphereBox_sphere_to_corner = new Vec3();
const sphereBox_side_ns = new Vec3();
const sphereBox_side_ns1 = new Vec3();
const sphereBox_side_ns2 = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.sphereBox] =
  Narrowphase.prototype.sphereBox;
const convex_to_sphere = new Vec3();
const sphereConvex_edge = new Vec3();
const sphereConvex_edgeUnit = new Vec3();
const sphereConvex_sphereToCorner = new Vec3();
const sphereConvex_worldCorner = new Vec3();
const sphereConvex_worldNormal = new Vec3();
const sphereConvex_worldPoint = new Vec3();
const sphereConvex_worldSpherePointClosestToPlane = new Vec3();
const sphereConvex_penetrationVec = new Vec3();
const sphereConvex_sphereToWorldPoint = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.sphereConvex] =
  Narrowphase.prototype.sphereConvex;
const planeBox_normal = new Vec3();
const plane_to_corner = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.planeBox] =
  Narrowphase.prototype.planeBox;
const planeConvex_v = new Vec3();
const planeConvex_normal = new Vec3();
const planeConvex_relpos = new Vec3();
const planeConvex_projected = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.planeConvex] =
  Narrowphase.prototype.planeConvex;
const convexConvex_sepAxis = new Vec3();
const convexConvex_q = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.convexConvex] =
  Narrowphase.prototype.convexConvex;
const particlePlane_normal = new Vec3();
const particlePlane_relpos = new Vec3();
const particlePlane_projected = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.planeParticle] =
  Narrowphase.prototype.planeParticle;
const particleSphere_normal = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.sphereParticle] =
  Narrowphase.prototype.sphereParticle;
const cqj = new Quaternion();
const convexParticle_local = new Vec3();
const convexParticle_normal = new Vec3();
const convexParticle_penetratedFaceNormal = new Vec3();
const convexParticle_vertexToParticle = new Vec3();
const convexParticle_worldPenetrationVec = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.convexParticle] =
  Narrowphase.prototype.convexParticle;
Narrowphase.prototype[COLLISION_TYPES.boxHeightfield] =
  Narrowphase.prototype.boxHeightfield;
const convexHeightfield_tmp1 = new Vec3();
const convexHeightfield_tmp2 = new Vec3();
const convexHeightfield_faceList = [0];
Narrowphase.prototype[COLLISION_TYPES.convexHeightfield] =
  Narrowphase.prototype.convexHeightfield;
const sphereHeightfield_tmp1 = new Vec3();
const sphereHeightfield_tmp2 = new Vec3();
Narrowphase.prototype[COLLISION_TYPES.sphereHeightfield] =
  Narrowphase.prototype.sphereHeightfield;
