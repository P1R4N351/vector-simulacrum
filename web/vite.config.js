import { defineConfig } from "vite";
import { wireRelay } from "./tools/wire_relay.js";
import react from "@vitejs/plugin-react";

// the page. base "./" lets the build be served from any path (a file server,
// a container, a project page). Runtime asset URLs are base-relative.
export default defineConfig({
  base: process.env.VITE_BASE || "./",
  plugins: [react(), ...(process.env.WIREOS_BODY === "1" ? [wireRelay()] : [])],
  define: { __WIREOS_BODY__: JSON.stringify(process.env.WIREOS_BODY === "1") },
  server: { port: 5183, strictPort: true },
  preview: { port: 5183, strictPort: true },
  build: { assetsDir: "bundle", chunkSizeWarningLimit: 1500 },
});
