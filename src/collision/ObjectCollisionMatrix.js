/// <reference types="./ObjectCollisionMatrix.ts" />
export class ObjectCollisionMatrix {
    constructor() {
        this.matrix = {};
    }
    get(bi, bj) {
        let { id: i } = bi;
        let { id: j } = bj;
        if (j > i) {
            const temp = j;
            j = i;
            i = temp;
        }
        return `${i}-${j}` in this.matrix;
    }
    set(bi, bj, value) {
        let { id: i } = bi;
        let { id: j } = bj;
        if (j > i) {
            const temp = j;
            j = i;
            i = temp;
        }
        if (value) {
            this.matrix[`${i}-${j}`] = true;
        }
        else {
            delete this.matrix[`${i}-${j}`];
        }
    }
    reset() {
        this.matrix = {};
    }
    setNumObjects(n) { }
}
