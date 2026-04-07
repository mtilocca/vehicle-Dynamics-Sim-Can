// Initialise Mermaid after the page (and any SPA navigation) finishes loading.
// Works with MkDocs Material's instant-navigation (turbo-like) page transitions.
document$.subscribe(() => {
  if (typeof mermaid !== "undefined") {
    mermaid.initialize({ startOnLoad: false, theme: "dark" });
    mermaid.run({ querySelector: ".mermaid" });
  }
});
