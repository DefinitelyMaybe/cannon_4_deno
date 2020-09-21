# cannon_4_deno
A simple transformation of the cannon-es scripts for use within a deno workflow

[Manage your external dependencies with care.](https://deno.land/manual/examples/manage_dependencies)

```typescript
// deps.ts
import { World } from "https://deno.land/x/cannon_4_deno/.../"

// main.ts
import { World } from "deps.ts"
//use as per normal
```

Check out [the repo](https://github.com/pmndrs/cannon-es) if you'd like to make improvements. This repo will be updated after there are new releases over there.
