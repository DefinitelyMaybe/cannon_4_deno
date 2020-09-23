import { walkSync } from "https://deno.land/std/fs/mod.ts";

// @ts-ignore
const [errors, emitted] = await Deno.compile(
  "./src/cannon-es.ts",
  undefined,
  {
    lib: ["esnext"],
  },
);

if (errors == null) {
  for (const obj in emitted) {
    const path = obj.split("/");
    const filename = path.slice(8).join("/");
    Deno.writeTextFileSync(`./${filename}`, emitted[obj]);
  }
}

// then go through the src directory.
// change all urls within .js files to reference the other .js files
const toRemove = []
for (const entry of walkSync("./src")) {
  if (entry.name.endsWith("js")) {
    let data = Deno.readTextFileSync(entry.path);
    data = data.replaceAll(/\.ts/g, ".js");
    data = data.replace(/^/, `/// <reference types="./${entry.name.split(".")[0]}.ts" />\n`)
    data = data.replace(/\/\/# sourceMappingURL=.+?map/g, "");
    Deno.writeTextFileSync(entry.path, data);
  } else if (entry.name.endsWith("map")) {
    toRemove.push(entry.path)
  } else if (entry.name.endsWith("ts")) {
    let data = Deno.readTextFileSync(entry.path);
    data = data.replace(/^/, `/// <reference lib="dom" />\n`);
    Deno.writeTextFileSync(entry.path, data);
  }
}

// remove map files
toRemove.forEach(path => {
  try {
    Deno.removeSync(path); 
  } catch (error) {
    console.log(error);
  }
});

// fmt afterwards
Deno.run({
  cmd: ["deno", "fmt"],
});