/// <reference lib="dom" />
export class Pool {
  constructor() {
    this.objects = [];
    this.type = Object;
  }
  release(...args) {
    const Nargs = args.length;
    for (let i = 0; i !== Nargs; i++) {
      this.objects.push(args[i]);
    }
    return this;
  }
  get() {
    if (this.objects.length === 0) {
      return this.constructObject();
    } else {
      return this.objects.pop();
    }
  }
  constructObject() {
    throw new Error(
      "constructObject() not implemented in this Pool subclass yet!",
    );
  }
  resize(size) {
    const objects = this.objects;
    while (objects.length > size) {
      objects.pop();
    }
    while (objects.length < size) {
      objects.push(this.constructObject());
    }
    return this;
  }
}
