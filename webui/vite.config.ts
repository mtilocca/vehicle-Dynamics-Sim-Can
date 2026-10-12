import { defineConfig } from "vite";
import { mockApi } from "./mock-api";

// Dev: `npm run dev` serves the UI against an in-process mock of the board API.
// Set HDV_PROXY=https://192.168.1.80 to talk to the real board instead
// (requires a session cookie: pass HDV_COOKIE="sid=<value>").
const proxy = process.env.HDV_PROXY;

export default defineConfig({
  build: {
    // Single-file-friendly output: one JS, one CSS, no hashed chunk splitting.
    cssCodeSplit: false,
    assetsInlineLimit: 0,
    rollupOptions: { output: { entryFileNames: "app.js", assetFileNames: "app[extname]" } },
  },
  server: proxy
    ? {
        proxy: {
          "/api": {
            target: proxy,
            secure: false,
            changeOrigin: true,
            headers: process.env.HDV_COOKIE ? { Cookie: process.env.HDV_COOKIE } : {},
          },
          "/dash": { target: proxy, secure: false, changeOrigin: true },
        },
      }
    : {},
  plugins: proxy ? [] : [mockApi()],
});
