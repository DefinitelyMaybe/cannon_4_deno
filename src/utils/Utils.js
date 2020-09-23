/// <reference types="./Utils.ts" />
/// <reference lib="dom" />
export function Utils() { }
Utils.defaults = (options = {}, defaults) => {
    for (let key in defaults) {
        if (!(key in options)) {
            options[key] = defaults[key];
        }
    }
    return options;
};
