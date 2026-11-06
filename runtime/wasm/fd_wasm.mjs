// freedyn2 in WebAssembly: load freedyn2.wasm and simulate .fdm models, in a browser or in Node.
//
//   import { loadFreedyn2 } from "./fd_wasm.mjs";
//   const fd = await loadFreedyn2(bytesOrUrl);
//   const r = fd.run(fdmText, { endTime: 2 });   // { names, rows, cols, data, steps, rejected }
//
// The module needs a few WASI functions (wasi_snapshot_preview1). They are provided here without
// a file system: models come as text, stdout and stderr go to the console. Models that read files
// (splines from data files, OpenCRG roads) or need a model plugin are not supported.

const ERRNO_SUCCESS = 0, ERRNO_BADF = 8, ERRNO_NOENT = 44, ERRNO_NOSYS = 52;

function wasiShim(getMemory) {
  const dec = new TextDecoder();
  const pending = { 1: "", 2: "" };
  const view = () => new DataView(getMemory().buffer);
  const flush = (fd, text) => {
    pending[fd] += text;
    const lines = pending[fd].split("\n");
    pending[fd] = lines.pop();
    for (const l of lines) (fd === 2 ? console.warn : console.log)(`freedyn2: ${l}`);
  };
  return {
    fd_write(fd, iovs, iovsLen, nwritten) {
      if (fd !== 1 && fd !== 2) return ERRNO_BADF;
      const v = view();
      let n = 0;
      for (let i = 0; i < iovsLen; i++) {
        const ptr = v.getUint32(iovs + 8 * i, true), len = v.getUint32(iovs + 8 * i + 4, true);
        flush(fd, dec.decode(new Uint8Array(getMemory().buffer, ptr, len)));
        n += len;
      }
      v.setUint32(nwritten, n, true);
      return ERRNO_SUCCESS;
    },
    environ_sizes_get(count, size) {
      const v = view();
      v.setUint32(count, 0, true);
      v.setUint32(size, 0, true);
      return ERRNO_SUCCESS;
    },
    environ_get: () => ERRNO_SUCCESS,
    fd_prestat_get: () => ERRNO_BADF,  // no preopened directories
    fd_prestat_dir_name: () => ERRNO_BADF,
    path_open: () => ERRNO_NOENT,
    path_filestat_get: () => ERRNO_NOENT,
    fd_fdstat_get: (fd) => (fd <= 2 ? ERRNO_SUCCESS : ERRNO_BADF),
    fd_fdstat_set_flags: () => ERRNO_NOSYS,
    fd_read: () => ERRNO_BADF,
    fd_seek: () => ERRNO_BADF,
    fd_close: () => ERRNO_SUCCESS,
    proc_exit(code) {
      throw new Error(`freedyn2: exit ${code}`);
    },
  };
}

export async function loadFreedyn2(source) {
  let bytes = source;
  if (typeof source === "string" || source instanceof URL) bytes = await (await fetch(source)).arrayBuffer();
  let memory;
  const { instance } = await WebAssembly.instantiate(bytes, {
    wasi_snapshot_preview1: wasiShim(() => memory),
  });
  const x = instance.exports;
  memory = x.memory;
  x._initialize();
  const enc = new TextEncoder(), dec = new TextDecoder();
  const cstring = (ptr) => {
    const b = new Uint8Array(memory.buffer, ptr);
    return dec.decode(b.subarray(0, b.indexOf(0)));
  };

  return {
    // Simulates the model; throws with the runtime's message on an error.
    run(fdmText, { endTime = 0 } = {}) {
      const text = enc.encode(fdmText + "\0");
      const ptr = x.fd_alloc(text.length);
      new Uint8Array(memory.buffer, ptr, text.length).set(text);
      let status;
      try {
        status = x.fd_run(ptr, endTime);
      } finally {
        x.fd_free(ptr);
      }
      if (status !== 0) throw new Error(cstring(x.fd_error()));
      const rows = x.fd_result_rows(), cols = x.fd_result_cols();
      // copied: the module's memory may grow (and move) with the next run
      const data = new Float64Array(memory.buffer, x.fd_result_data(), rows * cols).slice();
      return {
        names: cstring(x.fd_result_names()).split(","),
        rows, cols, data,
        steps: Number(x.fd_steps()),
        rejected: Number(x.fd_rejected_steps()),
      };
    },
  };
}
