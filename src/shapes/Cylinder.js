/// <reference lib="dom" />
import { ConvexPolyhedron } from "../shapes/ConvexPolyhedron.js";
import { Vec3 } from "../math/Vec3.js";
export class Cylinder extends ConvexPolyhedron {
    constructor(radiusTop, radiusBottom, height, numSegments) {
        const N = numSegments;
        const vertices = [];
        const axes = [];
        const faces = [];
        const bottomface = [];
        const topface = [];
        const cos = Math.cos;
        const sin = Math.sin;
        vertices.push(new Vec3(-radiusBottom * sin(0), -height * 0.5, radiusBottom * cos(0)));
        bottomface.push(0);
        vertices.push(new Vec3(-radiusTop * sin(0), height * 0.5, radiusTop * cos(0)));
        topface.push(1);
        for (let i = 0; i < N; i++) {
            const theta = ((2 * Math.PI) / N) * (i + 1);
            const thetaN = ((2 * Math.PI) / N) * (i + 0.5);
            if (i < N - 1) {
                vertices.push(new Vec3(-radiusBottom * sin(theta), -height * 0.5, radiusBottom * cos(theta)));
                bottomface.push(2 * i + 2);
                vertices.push(new Vec3(-radiusTop * sin(theta), height * 0.5, radiusTop * cos(theta)));
                topface.push(2 * i + 3);
                faces.push([2 * i, 2 * i + 1, 2 * i + 3, 2 * i + 2]);
            }
            else {
                faces.push([2 * i, 2 * i + 1, 1, 0]);
            }
            if (N % 2 === 1 || i < N / 2) {
                axes.push(new Vec3(-sin(thetaN), 0, cos(thetaN)));
            }
        }
        faces.push(bottomface);
        axes.push(new Vec3(0, 1, 0));
        const temp = [];
        for (let i = 0; i < topface.length; i++) {
            temp.push(topface[topface.length - i - 1]);
        }
        faces.push(temp);
        super({ vertices, faces, axes });
    }
}
