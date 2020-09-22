/// <reference lib="dom" />
export class ArrayCollisionMatrix {
  constructor() {
    this.matrix = [];
  }
  get(bi, bj) {
    let { index: i } = bi;
    let { index: j } = bj;
    if (j > i) {
      const temp = j;
      j = i;
      i = temp;
    }
    return this.matrix[((i * (i + 1)) >> 1) + j - 1];
  }
  set(bi, bj, value) {
    let { index: i } = bi;
    let { index: j } = bj;
    if (j > i) {
      const temp = j;
      j = i;
      i = temp;
    }
    this.matrix[((i * (i + 1)) >> 1) + j - 1] = value ? 1 : 0;
  }
  reset() {
    for (let i = 0, l = this.matrix.length; i !== l; i++) {
      this.matrix[i] = 0;
    }
  }
  setNumObjects(n) {
    this.matrix.length = (n * (n - 1)) >> 1;
  }
}
