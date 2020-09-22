/// <reference types="./EventTarget.ts" />
/// <reference lib="dom" />
export class EventTarget {
    constructor() { }
    addEventListener(type, listener) {
        if (this._listeners === undefined) {
            this._listeners = {};
        }
        const listeners = this._listeners;
        if (listeners[type] === undefined) {
            listeners[type] = [];
        }
        if (!listeners[type].includes(listener)) {
            listeners[type].push(listener);
        }
        return this;
    }
    hasEventListener(type, listener) {
        if (this._listeners === undefined) {
            return false;
        }
        const listeners = this._listeners;
        if (listeners[type] !== undefined && listeners[type].includes(listener)) {
            return true;
        }
        return false;
    }
    hasAnyEventListener(type) {
        if (this._listeners === undefined) {
            return false;
        }
        const listeners = this._listeners;
        return listeners[type] !== undefined;
    }
    removeEventListener(type, listener) {
        if (this._listeners === undefined) {
            return this;
        }
        const listeners = this._listeners;
        if (listeners[type] === undefined) {
            return this;
        }
        const index = listeners[type].indexOf(listener);
        if (index !== -1) {
            listeners[type].splice(index, 1);
        }
        return this;
    }
    dispatchEvent(event) {
        if (this._listeners === undefined) {
            return this;
        }
        const listeners = this._listeners;
        const listenerArray = listeners[event.type];
        if (listenerArray !== undefined) {
            event.target = this;
            for (let i = 0, l = listenerArray.length; i < l; i++) {
                listenerArray[i].call(this, event);
            }
        }
        return this;
    }
}
