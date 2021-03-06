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
for (const entry of walkSync("./src")) {
  if (entry.isFile) {
    if (entry.name.endsWith("js")) {
      let data = Deno.readTextFileSync(entry.path);
      data = data.replaceAll(/\.ts/g, ".js");
      data = data.replace(
        /^/,
        `/// <reference types="./${entry.name.split(".")[0]}.ts" />\n/// <reference lib="dom" />\n`,
      );
      data = data.replace(/\/\/# sourceMappingURL=.+?map/g, "");
      Deno.writeTextFileSync(entry.path, data);
    } else if (entry.name.endsWith("map")) {
      Deno.removeSync(entry.path);
    } else if (entry.name.endsWith("ts")) {
      let data = Deno.readTextFileSync(entry.path);
      data = data.replaceAll(/\/\/\/ <reference lib="dom" \/>\n/g, "")
      data = data.replace(/^/, `/// <reference lib="dom" />\n`);
      Deno.writeTextFileSync(entry.path, data); 
    } 
  }
}

// fmt afterwards
Deno.run({
  cmd: ["deno", "fmt"],
});
