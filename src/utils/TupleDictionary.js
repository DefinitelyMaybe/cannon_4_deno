/// <reference types="./TupleDictionary.ts" />
/// <reference lib="dom" />
export class TupleDictionary {
    constructor() {
        this.data = { keys: [] };
    }
    get(i, j) {
        if (i > j) {
            const temp = j;
            j = i;
            i = temp;
        }
        return this.data[`${i}-${j}`];
    }
    set(i, j, value) {
        if (i > j) {
            const temp = j;
            j = i;
            i = temp;
        }
        const key = `${i}-${j}`;
        if (!this.get(i, j)) {
            this.data.keys.push(key);
        }
        this.data[key] = value;
    }
    reset() {
        const data = this.data;
        const keys = data.keys;
        while (keys.length > 0) {
            const key = keys.pop();
            delete data[key];
        }
    }
}
