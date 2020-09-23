/// <reference types="./OverlapKeeper.ts" />
/// <reference lib="dom" />
export class OverlapKeeper {
    constructor() {
        this.current = [];
        this.previous = [];
    }
    getKey(i, j) {
        if (j < i) {
            const temp = j;
            j = i;
            i = temp;
        }
        return (i << 16) | j;
    }
    set(i, j) {
        const key = this.getKey(i, j);
        const current = this.current;
        let index = 0;
        while (key > current[index]) {
            index++;
        }
        if (key === current[index]) {
            return;
        }
        for (let j = current.length - 1; j >= index; j--) {
            current[j + 1] = current[j];
        }
        current[index] = key;
    }
    tick() {
        const tmp = this.current;
        this.current = this.previous;
        this.previous = tmp;
        this.current.length = 0;
    }
    getDiff(additions, removals) {
        const a = this.current;
        const b = this.previous;
        const al = a.length;
        const bl = b.length;
        let j = 0;
        for (let i = 0; i < al; i++) {
            let found = false;
            const keyA = a[i];
            while (keyA > b[j]) {
                j++;
            }
            found = keyA === b[j];
            if (!found) {
                unpackAndPush(additions, keyA);
            }
        }
        j = 0;
        for (let i = 0; i < bl; i++) {
            let found = false;
            const keyB = b[i];
            while (keyB > a[j]) {
                j++;
            }
            found = a[j] === keyB;
            if (!found) {
                unpackAndPush(removals, keyB);
            }
        }
    }
}
function unpackAndPush(array, key) {
    array.push((key & 0xffff0000) >> 16, key & 0x0000ffff);
}
