import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// the page. base "./" lets the build be served from any path (a file server,
// a container, a project page). Runtime asset URLs are base-relative.
export default defineConfig({
  base: process.env.VITE_BASE || "./",
  plugins: [react()],
  server: { port: 5183, strictPort: true },
  preview: { port: 5183, strictPort: true },
  build: { assetsDir: "bundle", chunkSizeWarningLimit: 1500 },
});
