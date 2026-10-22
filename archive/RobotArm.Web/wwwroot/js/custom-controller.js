// T10b: Custom JavaScript controller runtime and Web Worker dry-run verifier.
// Runs entirely in the student's browser; nothing is sent over the network.

let compiledFn = null;
let dotNetBridge = null;
let currentKp = [6, 6, 6, 2];
let currentKd = [0.3, 0.6, 0.3, 0.05];
let currentKi = [0, 0, 0, 0];
let lastError = null;

const modelObject = {
    M: function (q) {
        if (!dotNetBridge) throw new Error("Model köprüsü hazır değil.");
        return dotNetBridge.invokeMethod("ComputeM", q);
    },
    gravity: function (q) {
        if (!dotNetBridge) throw new Error("Model köprüsü hazır değil.");
        return dotNetBridge.invokeMethod("ComputeGravity", q);
    },
    bias: function (q, qd) {
        if (!dotNetBridge) throw new Error("Model köprüsü hazır değil.");
        return dotNetBridge.invokeMethod("ComputeBias", q, qd);
    },
    get kp() { return currentKp; },
    get kd() { return currentKd; },
    get ki() { return currentKi; }
};

const refObject = {
    q: [0, 0, 0, 0],
    qd: [0, 0, 0, 0],
    qdd: [0, 0, 0, 0]
};

function compileCode(sourceCode) {
    // Wrap the student's source code and return the `kontrol` function
    const factory = new Function(
        `"use strict";\n${sourceCode}\nif (typeof kontrol !== "function") { throw new Error("'kontrol(t, q, qd, ref, model)' fonksiyonu tanımlanmalıdır."); }\nreturn kontrol;`
    );
    return factory();
}

function prepareRuntime(sourceCode, bridgeRef, kp, kd, ki) {
    lastError = null;
    dotNetBridge = bridgeRef;
    currentKp = Array.isArray(kp) ? kp.slice(0, 4) : [6, 6, 6, 2];
    currentKd = Array.isArray(kd) ? kd.slice(0, 4) : [0.3, 0.6, 0.3, 0.05];
    currentKi = Array.isArray(ki) ? ki.slice(0, 4) : [0, 0, 0, 0];
    try {
        compiledFn = compileCode(sourceCode);
        return { ok: true, error: null };
    } catch (err) {
        compiledFn = null;
        lastError = err && err.message ? String(err.message) : String(err);
        return { ok: false, error: "Sözdizimi veya derleme hatası: " + lastError };
    }
}

function updateGains(kp, kd, ki) {
    if (Array.isArray(kp)) currentKp = kp.slice(0, 4);
    if (Array.isArray(kd)) currentKd = kd.slice(0, 4);
    if (Array.isArray(ki)) currentKi = ki.slice(0, 4);
}

/**
 * Called synchronously from JsController.Compute every Ts via IJSInProcessRuntime.
 * Returns [u0, u1, u2, u3, okFlag] where okFlag === 1 when 4 finite numbers were returned,
 * or okFlag === 0 when the output was invalid / non-finite / threw an exception.
 */
function invokeKontrol(t, q, qd, qRef, vRef, aRef) {
    if (!compiledFn) {
        return [0, 0, 0, 0, 0];
    }
    try {
        for (let i = 0; i < 4; i++) {
            refObject.q[i] = qRef[i];
            refObject.qd[i] = vRef[i];
            refObject.qdd[i] = aRef[i];
        }
        const u = compiledFn(t, q, qd, refObject, modelObject);
        if (!u || typeof u.length !== "number" || u.length !== 4) {
            lastError = "Kontrolcün 4 elemanlı bir dizi döndürmedi.";
            return [0, 0, 0, 0, 0];
        }
        const u0 = Number(u[0]);
        const u1 = Number(u[1]);
        const u2 = Number(u[2]);
        const u3 = Number(u[3]);
        if (!Number.isFinite(u0) || !Number.isFinite(u1) || !Number.isFinite(u2) || !Number.isFinite(u3)) {
            lastError = "Kontrolcün sayı olmayan bir değer döndürdü";
            return [0, 0, 0, 0, 0];
        }
        return [u0, u1, u2, u3, 1];
    } catch (err) {
        lastError = err && err.message ? String(err.message) : String(err);
        return [0, 0, 0, 0, 0];
    }
}

/**
 * Dry run in an isolated Web Worker before starting a run (TASKS.md T10b item 3a):
 * Executes 2000 calls with representative inputs. If it exceeds 500 ms, throws,
 * or returns anything other than 4 finite numbers, terminates the worker and refuses to start.
 */
function dryRunInWorker(sourceCode, kp, kd, ki) {
    return new Promise((resolve) => {
        const workerScript = `
            "use strict";
            self.onmessage = function(ev) {
                const { sourceCode, kp, kd, ki } = ev.data;
                const tStart = performance.now();
                let fn;
                try {
                    const factory = new Function(
                        '"use strict";\\n' + sourceCode +
                        '\\nif (typeof kontrol !== "function") { throw new Error("\\x27kontrol(t, q, qd, ref, model)\\x27 fonksiyonu tanımlanmalıdır."); }\\nreturn kontrol;'
                    );
                    fn = factory();
                } catch (e) {
                    self.postMessage({
                        ok: false,
                        error: "Kuru çalıştırma derleme hatası: " + (e && e.message ? e.message : String(e))
                    });
                    return;
                }

                // Representative model functions for the 2000-call dry run
                const repModel = {
                    M: function(q) {
                        const c2 = Math.cos(q[1] || 0);
                        return [
                            [0.012 + 0.004 * c2 * c2, 0, 0, 0],
                            [0, 0.018, 0.006, 0.0015],
                            [0, 0.006, 0.005, 0.0009],
                            [0, 0.0015, 0.0009, 0.0004]
                        ];
                    },
                    gravity: function(q) {
                        const q2 = q[1] || 0, q3 = q[2] || 0, q4 = q[3] || 0;
                        return [
                            0.0,
                            -0.85 * Math.sin(q2) - 0.35 * Math.cos(q2 + q3),
                            -0.35 * Math.cos(q2 + q3) - 0.08 * Math.cos(q2 + q3 + q4),
                            -0.08 * Math.cos(q2 + q3 + q4)
                        ];
                    },
                    bias: function(q, qd) {
                        const g = this.gravity(q);
                        return [
                            g[0] + 0.002 * (qd[0] || 0),
                            g[1] + 0.002 * (qd[1] || 0),
                            g[2] + 0.001 * (qd[2] || 0),
                            g[3] + 0.0005 * (qd[3] || 0)
                        ];
                    },
                    kp: kp || [6, 6, 6, 2],
                    kd: kd || [0.3, 0.6, 0.3, 0.05],
                    ki: ki || [0, 0, 0, 0]
                };

                try {
                    for (let k = 0; k < 2000; k++) {
                        const t = (k % 50) * 0.002;
                        const s = Math.sin(0.01 * k);
                        const c = Math.cos(0.01 * k);
                        const q = [0.2 * s, -0.4 + 0.1 * c, -0.3 + 0.1 * s, 1.1 - 0.1 * c];
                        const qd = [0.1 * c, -0.05 * s, 0.08 * c, -0.06 * s];
                        const ref = {
                            q: [0.25 * s, -0.35 + 0.1 * c, -0.25 + 0.1 * s, 1.05 - 0.1 * c],
                            qd: [0.12 * c, -0.04 * s, 0.09 * c, -0.05 * s],
                            qdd: [-0.02 * s, -0.01 * c, -0.02 * s, 0.01 * c]
                        };
                        const out = fn(t, q, qd, ref, repModel);
                        if (!out || typeof out.length !== "number" || out.length !== 4) {
                            self.postMessage({
                                ok: false,
                                error: "Kuru çalıştırma hatası (çağrı #" + (k + 1) + "): Kontrolcün 4 elemanlı bir dizi döndürmelidir."
                            });
                            return;
                        }
                        for (let i = 0; i < 4; i++) {
                            const v = out[i];
                            if (typeof v !== "number" || !Number.isFinite(v)) {
                                self.postMessage({
                                    ok: false,
                                    error: "Kuru çalıştırma hatası (çağrı #" + (k + 1) + ", eklem " + (i + 1) + "): Kontrolcün sayı olmayan bir değer döndürdü."
                                });
                                return;
                            }
                        }
                        if ((k & 63) === 63 && (performance.now() - tStart) > 500.0) {
                            self.postMessage({
                                ok: false,
                                error: "Kuru çalıştırma 500 ms zaman sınırını aştı (2000 çağrı). Sonsuz döngü veya çok yavaş hesaplama nedeniyle deney başlatılmadı."
                            });
                            return;
                        }
                    }
                    const elapsedMs = performance.now() - tStart;
                    if (elapsedMs > 500.0) {
                        self.postMessage({
                            ok: false,
                            error: "Kuru çalıştırma 500 ms zaman sınırını aştı (" + elapsedMs.toFixed(1) + " ms). Deney başlatılmadı."
                        });
                        return;
                    }
                    self.postMessage({ ok: true, elapsedMs, calls: 2000, error: null });
                } catch (err) {
                    self.postMessage({
                        ok: false,
                        error: "Kuru çalıştırma çalışma zamanı hatası: " + (err && err.message ? err.message : String(err))
                    });
                }
            };
        `;

        const blob = new Blob([workerScript], { type: "application/javascript" });
        const blobUrl = URL.createObjectURL(blob);
        const worker = new Worker(blobUrl);
        let settled = false;

        const timer = setTimeout(() => {
            if (settled) return;
            settled = true;
            worker.terminate();
            URL.revokeObjectURL(blobUrl);
            resolve({
                ok: false,
                elapsedMs: 500,
                calls: 0,
                error: "Kuru çalıştırma 500 ms zaman aşımına uğradı: Sonsuz döngü tespit edildi, sayfa dondurulmadan deney reddedildi."
            });
        }, 500);

        worker.onmessage = (ev) => {
            if (settled) return;
            settled = true;
            clearTimeout(timer);
            worker.terminate();
            URL.revokeObjectURL(blobUrl);
            resolve(ev.data);
        };

        worker.onerror = (err) => {
            if (settled) return;
            settled = true;
            clearTimeout(timer);
            worker.terminate();
            URL.revokeObjectURL(blobUrl);
            resolve({
                ok: false,
                elapsedMs: 0,
                calls: 0,
                error: "Kuru çalıştırma hatası: " + (err && err.message ? err.message : "Bilinmeyen Web Worker hatası.")
            });
        };

        worker.postMessage({ sourceCode, kp, kd, ki });
    });
}

let hostRef = null;

function registerHost(dotNetHostRef) {
    hostRef = dotNetHostRef;
}

function verifyPdgEquivalence() {
    if (!hostRef) {
        throw new Error("Home.razor host referansı kayıtlı değil.");
    }
    return hostRef.invokeMethod("VerifyPdgEquivalence");
}

function disposeRuntime() {
    compiledFn = null;
    dotNetBridge = null;
}

window.RobotArmCustomController = {
    prepareRuntime,
    updateGains,
    invokeKontrol,
    dryRunInWorker,
    disposeRuntime,
    registerHost,
    verifyPdgEquivalence
};
