/// <reference types="./Material.ts" />
/// <reference lib="dom" />
export class Material {
    constructor(options = {}) {
        let name = "";
        if (typeof options === "string") {
            name = options;
            options = {};
        }
        this.name = name;
        this.id = Material.idCounter++;
        this.friction = typeof options.friction !== "undefined"
            ? options.friction
            : -1;
        this.restitution = typeof options.restitution !== "undefined"
            ? options.restitution
            : -1;
    }
}
Material.idCounter = 0;
