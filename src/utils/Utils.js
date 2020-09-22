export function Utils() {}
Utils.defaults = (options = {}, defaults) => {
  for (let key in defaults) {
    if (!(key in options)) {
      options[key] = defaults[key];
    }
  }
  return options;
};
