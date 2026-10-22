// Drone Show AGH backend plugin entry point.
// The implementation is split into dependency-free ES modules so AGH can install the package as a
// self-contained snapshot while keeping transport, schemas, and tool definitions reviewable.
export { droneShowPlugin, tools } from './src/tools.mjs'
