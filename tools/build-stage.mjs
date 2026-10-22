import { build } from './stage/node_modules/esbuild/lib/main.js';
import { fileURLToPath } from 'node:url';
import { readFile, copyFile } from 'node:fs/promises';
const root = fileURLToPath(new URL('../', import.meta.url));
const manifest = JSON.parse(await readFile(root + 'tools/stage/package.json'));
if (manifest.dependencies.three !== '0.186.1') throw new Error('Stage dependency must stay pinned');
await build({ entryPoints: [root + 'web/stage.mjs'], outfile: root + 'web/stage.bundle.js',
  bundle: true, minify: true, format: 'iife', target: ['safari16.4', 'chrome110'], legalComments: 'eof' });
await copyFile(root + 'tools/stage/node_modules/three/LICENSE', root + 'web/THREE-LICENSE.txt');
