// Links that carry a model (generator/fdgen/share.py writes the same formats):
//
//   #model=<base64url of the raw-deflated UTF-8 YAML>   compact (freedyn2 link, the link button)
//   #yaml=<the YAML, percent-encoded>                    readable; easy to write by hand
//   ?src=<URL of a YAML file or a zip with model and meshes>
//
// The part after '#' is not sent to the server of the page: the model stays in the browser.

async function pipe(bytes, stream) {
  return new Uint8Array(await new Response(new Blob([bytes]).stream().pipeThrough(stream)).arrayBuffer());
}

export async function encodeModel(yaml) {
  const data = await pipe(new TextEncoder().encode(yaml), new CompressionStream("deflate-raw"));
  let s = "";
  for (let i = 0; i < data.length; i += 0x8000) s += String.fromCharCode(...data.subarray(i, i + 0x8000));
  return btoa(s).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
}

export async function decodeModel(value) {
  const b64 = value.trim().replace(/-/g, "+").replace(/_/g, "/");
  const bytes = Uint8Array.from(atob(b64 + "=".repeat((4 - (b64.length % 4)) % 4)), (c) => c.charCodeAt(0));
  return new TextDecoder("utf-8", { fatal: true }).decode(await pipe(bytes, new DecompressionStream("deflate-raw")));
}

export async function linkFor(yaml, base) {
  return `${String(base).split("#")[0]}#model=${await encodeModel(yaml)}`;
}

// What a page address carries: { yaml } (from the hash), { src } (a URL to fetch) or null.
// Throws with a readable message when the hash is there but broken (a link cut off when copied).
export async function modelFrom(href) {
  const url = new URL(href);
  // the whole rest of the hash is the value, read by hand: URLSearchParams would read '+' as a
  // space and split at '&', and a hand-written YAML may have both
  const hash = url.hash.slice(1);
  if (hash.startsWith("model=")) {
    try {
      return { yaml: await decodeModel(hash.slice(6)) };
    } catch {
      throw new Error("the model in the link cannot be read: was the link cut off when it was copied?");
    }
  }
  if (hash.startsWith("yaml=")) {
    try {
      return { yaml: decodeURIComponent(hash.slice(5)) };
    } catch {
      throw new Error("the model in the link is not percent-encoded correctly (#yaml=)");
    }
  }
  const src = url.searchParams.get("src");
  if (src) return { src: new URL(src, url).href };
  return null;
}
