/// <reference lib="dom" />
import { Vec3 } from "../math/Vec3.js";
import { Quaternion } from "../math/Quaternion.js";
import { Transform } from "../math/Transform.js";
import { RaycastResult } from "../collision/RaycastResult.js";
import { Shape } from "../shapes/Shape.js";
import { AABB } from "../collision/AABB.js";
export const RAY_MODES = {
    CLOSEST: 1,
    ANY: 2,
    ALL: 4,
};
export class Ray {
    constructor(from = new Vec3(), to = new Vec3()) {
        this.from = from.clone();
        this.to = to.clone();
        this.direction = new Vec3();
        this.precision = 0.0001;
        this.checkCollisionResponse = true;
        this.skipBackfaces = false;
        this.collisionFilterMask = -1;
        this.collisionFilterGroup = -1;
        this.mode = Ray.ANY;
        this.result = new RaycastResult();
        this.hasHit = false;
        this.callback = (result) => { };
    }
    intersectWorld(world, options) {
        this.mode = options.mode || Ray.ANY;
        this.result = options.result || new RaycastResult();
        this.skipBackfaces = !!options.skipBackfaces;
        this.collisionFilterMask =
            typeof options.collisionFilterMask !== "undefined"
                ? options.collisionFilterMask
                : -1;
        this.collisionFilterGroup =
            typeof options.collisionFilterGroup !== "undefined"
                ? options.collisionFilterGroup
                : -1;
        this.checkCollisionResponse =
            typeof options.checkCollisionResponse !== "undefined"
                ? options.checkCollisionResponse
                : true;
        if (options.from) {
            this.from.copy(options.from);
        }
        if (options.to) {
            this.to.copy(options.to);
        }
        this.callback = options.callback || (() => { });
        this.hasHit = false;
        this.result.reset();
        this.updateDirection();
        this.getAABB(tmpAABB);
        tmpArray.length = 0;
        world.broadphase.aabbQuery(world, tmpAABB, tmpArray);
        this.intersectBodies(tmpArray);
        return this.hasHit;
    }
    intersectBody(body, result) {
        if (result) {
            this.result = result;
            this.updateDirection();
        }
        const checkCollisionResponse = this.checkCollisionResponse;
        if (checkCollisionResponse && !body.collisionResponse) {
            return;
        }
        if ((this.collisionFilterGroup & body.collisionFilterMask) === 0 ||
            (body.collisionFilterGroup & this.collisionFilterMask) === 0) {
            return;
        }
        const xi = intersectBody_xi;
        const qi = intersectBody_qi;
        for (let i = 0, N = body.shapes.length; i < N; i++) {
            const shape = body.shapes[i];
            if (checkCollisionResponse && !shape.collisionResponse) {
                continue;
            }
            body.quaternion.mult(body.shapeOrientations[i], qi);
            body.quaternion.vmult(body.shapeOffsets[i], xi);
            xi.vadd(body.position, xi);
            this.intersectShape(shape, qi, xi, body);
            if (this.result.shouldStop) {
                break;
            }
        }
    }
    intersectBodies(bodies, result) {
        if (result) {
            this.result = result;
            this.updateDirection();
        }
        for (let i = 0, l = bodies.length; !this.result.shouldStop && i < l; i++) {
            this.intersectBody(bodies[i]);
        }
    }
    updateDirection() {
        this.to.vsub(this.from, this.direction);
        this.direction.normalize();
    }
    intersectShape(shape, quat, position, body) {
        const from = this.from;
        const distance = distanceFromIntersection(from, this.direction, position);
        if (distance > shape.boundingSphereRadius) {
            return;
        }
        const intersectMethod = this[shape.type];
        if (intersectMethod) {
            intersectMethod.call(this, shape, quat, position, body, shape);
        }
    }
    _intersectBox(box, quat, position, body, reportedShape) {
        return this._intersectConvex(box.convexPolyhedronRepresentation, quat, position, body, reportedShape);
    }
    _intersectPlane(shape, quat, position, body, reportedShape) {
        const from = this.from;
        const to = this.to;
        const direction = this.direction;
        const worldNormal = new Vec3(0, 0, 1);
        quat.vmult(worldNormal, worldNormal);
        const len = new Vec3();
        from.vsub(position, len);
        const planeToFrom = len.dot(worldNormal);
        to.vsub(position, len);
        const planeToTo = len.dot(worldNormal);
        if (planeToFrom * planeToTo > 0) {
            return;
        }
        if (from.distanceTo(to) < planeToFrom) {
            return;
        }
        const n_dot_dir = worldNormal.dot(direction);
        if (Math.abs(n_dot_dir) < this.precision) {
            return;
        }
        const planePointToFrom = new Vec3();
        const dir_scaled_with_t = new Vec3();
        const hitPointWorld = new Vec3();
        from.vsub(position, planePointToFrom);
        const t = -worldNormal.dot(planePointToFrom) / n_dot_dir;
        direction.scale(t, dir_scaled_with_t);
        from.vadd(dir_scaled_with_t, hitPointWorld);
        this.reportIntersection(worldNormal, hitPointWorld, reportedShape, body, -1);
    }
    getAABB(aabb) {
        const { lowerBound, upperBound } = aabb;
        const to = this.to;
        const from = this.from;
        lowerBound.x = Math.min(to.x, from.x);
        lowerBound.y = Math.min(to.y, from.y);
        lowerBound.z = Math.min(to.z, from.z);
        upperBound.x = Math.max(to.x, from.x);
        upperBound.y = Math.max(to.y, from.y);
        upperBound.z = Math.max(to.z, from.z);
    }
    _intersectHeightfield(shape, quat, position, body, reportedShape) {
        const data = shape.data;
        const w = shape.elementSize;
        const localRay = intersectHeightfield_localRay;
        localRay.from.copy(this.from);
        localRay.to.copy(this.to);
        Transform.pointToLocalFrame(position, quat, localRay.from, localRay.from);
        Transform.pointToLocalFrame(position, quat, localRay.to, localRay.to);
        localRay.updateDirection();
        const index = intersectHeightfield_index;
        let iMinX;
        let iMinY;
        let iMaxX;
        let iMaxY;
        iMinX = iMinY = 0;
        iMaxX = iMaxY = shape.data.length - 1;
        const aabb = new AABB();
        localRay.getAABB(aabb);
        shape.getIndexOfPosition(aabb.lowerBound.x, aabb.lowerBound.y, index, true);
        iMinX = Math.max(iMinX, index[0]);
        iMinY = Math.max(iMinY, index[1]);
        shape.getIndexOfPosition(aabb.upperBound.x, aabb.upperBound.y, index, true);
        iMaxX = Math.min(iMaxX, index[0] + 1);
        iMaxY = Math.min(iMaxY, index[1] + 1);
        for (let i = iMinX; i < iMaxX; i++) {
            for (let j = iMinY; j < iMaxY; j++) {
                if (this.result.shouldStop) {
                    return;
                }
                shape.getAabbAtIndex(i, j, aabb);
                if (!aabb.overlapsRay(localRay)) {
                    continue;
                }
                shape.getConvexTrianglePillar(i, j, false);
                Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
                this._intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, reportedShape, intersectConvexOptions);
                if (this.result.shouldStop) {
                    return;
                }
                shape.getConvexTrianglePillar(i, j, true);
                Transform.pointToWorldFrame(position, quat, shape.pillarOffset, worldPillarOffset);
                this._intersectConvex(shape.pillarConvex, quat, worldPillarOffset, body, reportedShape, intersectConvexOptions);
            }
        }
    }
    _intersectSphere(sphere, quat, position, body, reportedShape) {
        const from = this.from;
        const to = this.to;
        const r = sphere.radius;
        const a = (to.x - from.x) ** 2 + (to.y - from.y) ** 2 +
            (to.z - from.z) ** 2;
        const b = 2 *
            ((to.x - from.x) * (from.x - position.x) +
                (to.y - from.y) * (from.y - position.y) +
                (to.z - from.z) * (from.z - position.z));
        const c = (from.x - position.x) ** 2 + (from.y - position.y) ** 2 +
            (from.z - position.z) ** 2 - r ** 2;
        const delta = b ** 2 - 4 * a * c;
        const intersectionPoint = Ray_intersectSphere_intersectionPoint;
        const normal = Ray_intersectSphere_normal;
        if (delta < 0) {
            return;
        }
        else if (delta === 0) {
            from.lerp(to, delta, intersectionPoint);
            intersectionPoint.vsub(position, normal);
            normal.normalize();
            this.reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
        }
        else {
            const d1 = (-b - Math.sqrt(delta)) / (2 * a);
            const d2 = (-b + Math.sqrt(delta)) / (2 * a);
            if (d1 >= 0 && d1 <= 1) {
                from.lerp(to, d1, intersectionPoint);
                intersectionPoint.vsub(position, normal);
                normal.normalize();
                this.reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
            }
            if (this.result.shouldStop) {
                return;
            }
            if (d2 >= 0 && d2 <= 1) {
                from.lerp(to, d2, intersectionPoint);
                intersectionPoint.vsub(position, normal);
                normal.normalize();
                this.reportIntersection(normal, intersectionPoint, reportedShape, body, -1);
            }
        }
    }
    _intersectConvex(shape, quat, position, body, reportedShape, options) {
        const minDistNormal = intersectConvex_minDistNormal;
        const normal = intersectConvex_normal;
        const vector = intersectConvex_vector;
        const minDistIntersect = intersectConvex_minDistIntersect;
        const faceList = (options && options.faceList) || null;
        const faces = shape.faces;
        const vertices = shape.vertices;
        const normals = shape.faceNormals;
        const direction = this.direction;
        const from = this.from;
        const to = this.to;
        const fromToDistance = from.distanceTo(to);
        const minDist = -1;
        const Nfaces = faceList ? faceList.length : faces.length;
        const result = this.result;
        for (let j = 0; !result.shouldStop && j < Nfaces; j++) {
            const fi = faceList ? faceList[j] : j;
            const face = faces[fi];
            const faceNormal = normals[fi];
            const q = quat;
            const x = position;
            vector.copy(vertices[face[0]]);
            q.vmult(vector, vector);
            vector.vadd(x, vector);
            vector.vsub(from, vector);
            q.vmult(faceNormal, normal);
            const dot = direction.dot(normal);
            if (Math.abs(dot) < this.precision) {
                continue;
            }
            const scalar = normal.dot(vector) / dot;
            if (scalar < 0) {
                continue;
            }
            direction.scale(scalar, intersectPoint);
            intersectPoint.vadd(from, intersectPoint);
            a.copy(vertices[face[0]]);
            q.vmult(a, a);
            x.vadd(a, a);
            for (let i = 1; !result.shouldStop && i < face.length - 1; i++) {
                b.copy(vertices[face[i]]);
                c.copy(vertices[face[i + 1]]);
                q.vmult(b, b);
                q.vmult(c, c);
                x.vadd(b, b);
                x.vadd(c, c);
                const distance = intersectPoint.distanceTo(from);
                if (!(pointInTriangle(intersectPoint, a, b, c) ||
                    pointInTriangle(intersectPoint, b, a, c)) ||
                    distance > fromToDistance) {
                    continue;
                }
                this.reportIntersection(normal, intersectPoint, reportedShape, body, fi);
            }
        }
    }
    _intersectTrimesh(mesh, quat, position, body, reportedShape, options) {
        const normal = intersectTrimesh_normal;
        const triangles = intersectTrimesh_triangles;
        const treeTransform = intersectTrimesh_treeTransform;
        const minDistNormal = intersectConvex_minDistNormal;
        const vector = intersectConvex_vector;
        const minDistIntersect = intersectConvex_minDistIntersect;
        const localAABB = intersectTrimesh_localAABB;
        const localDirection = intersectTrimesh_localDirection;
        const localFrom = intersectTrimesh_localFrom;
        const localTo = intersectTrimesh_localTo;
        const worldIntersectPoint = intersectTrimesh_worldIntersectPoint;
        const worldNormal = intersectTrimesh_worldNormal;
        const faceList = (options && options.faceList) || null;
        const indices = mesh.indices;
        const vertices = mesh.vertices;
        const from = this.from;
        const to = this.to;
        const direction = this.direction;
        const minDist = -1;
        treeTransform.position.copy(position);
        treeTransform.quaternion.copy(quat);
        Transform.vectorToLocalFrame(position, quat, direction, localDirection);
        Transform.pointToLocalFrame(position, quat, from, localFrom);
        Transform.pointToLocalFrame(position, quat, to, localTo);
        localTo.x *= mesh.scale.x;
        localTo.y *= mesh.scale.y;
        localTo.z *= mesh.scale.z;
        localFrom.x *= mesh.scale.x;
        localFrom.y *= mesh.scale.y;
        localFrom.z *= mesh.scale.z;
        localTo.vsub(localFrom, localDirection);
        localDirection.normalize();
        const fromToDistanceSquared = localFrom.distanceSquared(localTo);
        mesh.tree.rayQuery(this, treeTransform, triangles);
        for (let i = 0, N = triangles.length; !this.result.shouldStop && i !== N; i++) {
            const trianglesIndex = triangles[i];
            mesh.getNormal(trianglesIndex, normal);
            mesh.getVertex(indices[trianglesIndex * 3], a);
            a.vsub(localFrom, vector);
            const dot = localDirection.dot(normal);
            const scalar = normal.dot(vector) / dot;
            if (scalar < 0) {
                continue;
            }
            localDirection.scale(scalar, intersectPoint);
            intersectPoint.vadd(localFrom, intersectPoint);
            mesh.getVertex(indices[trianglesIndex * 3 + 1], b);
            mesh.getVertex(indices[trianglesIndex * 3 + 2], c);
            const squaredDistance = intersectPoint.distanceSquared(localFrom);
            if (!(pointInTriangle(intersectPoint, b, a, c) ||
                pointInTriangle(intersectPoint, a, b, c)) ||
                squaredDistance > fromToDistanceSquared) {
                continue;
            }
            Transform.vectorToWorldFrame(quat, normal, worldNormal);
            Transform.pointToWorldFrame(position, quat, intersectPoint, worldIntersectPoint);
            this.reportIntersection(worldNormal, worldIntersectPoint, reportedShape, body, trianglesIndex);
        }
        triangles.length = 0;
    }
    reportIntersection(normal, hitPointWorld, shape, body, hitFaceIndex) {
        const from = this.from;
        const to = this.to;
        const distance = from.distanceTo(hitPointWorld);
        const result = this.result;
        if (this.skipBackfaces && normal.dot(this.direction) > 0) {
            return;
        }
        result.hitFaceIndex = typeof hitFaceIndex !== "undefined"
            ? hitFaceIndex
            : -1;
        switch (this.mode) {
            case Ray.ALL:
                this.hasHit = true;
                result.set(from, to, normal, hitPointWorld, shape, body, distance);
                result.hasHit = true;
                this.callback(result);
                break;
            case Ray.CLOSEST:
                if (distance < result.distance || !result.hasHit) {
                    this.hasHit = true;
                    result.hasHit = true;
                    result.set(from, to, normal, hitPointWorld, shape, body, distance);
                }
                break;
            case Ray.ANY:
                this.hasHit = true;
                result.hasHit = true;
                result.set(from, to, normal, hitPointWorld, shape, body, distance);
                result.shouldStop = true;
                break;
        }
    }
}
Shape.types.SPHERE, Shape.types.PLANE, Shape.types.BOX, Shape.types.CONVEXPOLYHEDRON, Shape.types.HEIGHTFIELD, Shape.types.TRIMESH;
Ray.CLOSEST = 1;
Ray.ANY = 2;
Ray.ALL = 4;
const tmpAABB = new AABB();
const tmpArray = [];
const v1 = new Vec3();
const v2 = new Vec3();
Ray.pointInTriangle = pointInTriangle;
function pointInTriangle(p, a, b, c) {
    c.vsub(a, v0);
    b.vsub(a, v1);
    p.vsub(a, v2);
    const dot00 = v0.dot(v0);
    const dot01 = v0.dot(v1);
    const dot02 = v0.dot(v2);
    const dot11 = v1.dot(v1);
    const dot12 = v1.dot(v2);
    let u;
    let v;
    return ((u = dot11 * dot02 - dot01 * dot12) >= 0 &&
        (v = dot00 * dot12 - dot01 * dot02) >= 0 &&
        u + v < dot00 * dot11 - dot01 * dot01);
}
const intersectBody_xi = new Vec3();
const intersectBody_qi = new Quaternion();
const vector = new Vec3();
const normal = new Vec3();
const intersectPoint = new Vec3();
const a = new Vec3();
const b = new Vec3();
const c = new Vec3();
const d = new Vec3();
const tmpRaycastResult = new RaycastResult();
Ray.prototype[Shape.types.BOX] = Ray.prototype._intersectBox;
Ray.prototype[Shape.types.PLANE] = Ray.prototype._intersectPlane;
const intersectConvexOptions = {
    faceList: [0],
};
const worldPillarOffset = new Vec3();
const intersectHeightfield_localRay = new Ray();
const intersectHeightfield_index = [];
const intersectHeightfield_minMax = [];
Ray.prototype[Shape.types.HEIGHTFIELD] = Ray.prototype._intersectHeightfield;
const Ray_intersectSphere_intersectionPoint = new Vec3();
const Ray_intersectSphere_normal = new Vec3();
Ray.prototype[Shape.types.SPHERE] = Ray.prototype._intersectSphere;
const intersectConvex_normal = new Vec3();
const intersectConvex_minDistNormal = new Vec3();
const intersectConvex_minDistIntersect = new Vec3();
const intersectConvex_vector = new Vec3();
Ray.prototype[Shape.types.CONVEXPOLYHEDRON] = Ray.prototype._intersectConvex;
const intersectTrimesh_normal = new Vec3();
const intersectTrimesh_localDirection = new Vec3();
const intersectTrimesh_localFrom = new Vec3();
const intersectTrimesh_localTo = new Vec3();
const intersectTrimesh_worldNormal = new Vec3();
const intersectTrimesh_worldIntersectPoint = new Vec3();
const intersectTrimesh_localAABB = new AABB();
const intersectTrimesh_triangles = [];
const intersectTrimesh_treeTransform = new Transform();
Ray.prototype[Shape.types.TRIMESH] = Ray.prototype._intersectTrimesh;
const v0 = new Vec3();
const intersect = new Vec3();
function distanceFromIntersection(from, direction, position) {
    position.vsub(from, v0);
    const dot = v0.dot(direction);
    direction.scale(dot, intersect);
    intersect.vadd(from, intersect);
    const distance = position.distanceTo(intersect);
    return distance;
}
