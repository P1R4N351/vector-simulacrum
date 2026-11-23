// Pure Float32 JavaScript NoMaD engine: Goal-Masked Transformer Encoder,
// FiLM 1D U-Net Denoiser, DDIM sampler, TopologicalGraph, and controller.
import { renderHeadCamRgb, IMG_H, IMG_W } from "./nomad_cam.js";
import { assert } from "./util.js";

export { IMG_H, IMG_W };
export const CONTEXT_FRAMES = 4;
export const HORIZON = 8;
export const ACT_DIM = 2;
export const EMBED_DIM = 64;
export const NUM_HEADS = 4;
export const HEAD_DIM = EMBED_DIM / NUM_HEADS;
export const FF_DIM = 128;

export const MAX_NODES = 64;
export const PLAN_INTERVAL_TICKS = 50;
export const EDGE_NEW_DIST_STEPS = 2.2;
export const EDGE_NEW_METRIC_MM = 75.0;
export const LOOP_CLOSE_DIST_STEPS = 3.0;
const DEG = Math.PI / 180;

function clamp(value, low, high) {
  return value < low ? low : value > high ? high : value;
}

function makeRng(seed = 0) {
  let s = (seed >>> 0) || 1;
  const uniform = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  return () => {
    const u1 = Math.max(1e-7, uniform());
    const u2 = uniform();
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  };
}

export function rgbToChw(imgU8) {
  const plain = IMG_H * IMG_W;
  assert(imgU8 && imgU8.length === plain * 3, `expected ${plain * 3} RGB bytes`);
  const out = new Float32Array(plain * 3);
  for (let p = 0; p < plain; p++) {
    out[p] = imgU8[3 * p] / 255;
    out[plain + p] = imgU8[3 * p + 1] / 255;
    out[2 * plain + p] = imgU8[3 * p + 2] / 255;
  }
  return out;
}

export function stackObsFrames(framesU8) {
  assert(Array.isArray(framesU8) && framesU8.length >= 1, "at least one frame required");
  const kept = framesU8.slice(-CONTEXT_FRAMES);
  for (let i = 0; i < CONTEXT_FRAMES && kept.length < CONTEXT_FRAMES; i++) {
    kept.unshift(kept[0]);
  }
  const frameLen = 3 * IMG_H * IMG_W;
  const out = new Float32Array(CONTEXT_FRAMES * frameLen);
  for (let i = 0; i < CONTEXT_FRAMES; i++) {
    out.set(rgbToChw(kept[i]), i * frameLen);
  }
  return out;
}

export function unpackWeights(meta, weightsBytes) {
  assert(meta && Array.isArray(meta.tensors), "invalid NoMaD metadata");
  assert(weightsBytes instanceof Uint8Array, "weights must be Uint8Array");
  assert(weightsBytes.byteLength === meta.weights_floats * 4, "weights length mismatch");
  const aligned = new Float32Array(meta.weights_floats);
  new Uint8Array(aligned.buffer).set(weightsBytes);
  const map = new Map();
  for (const t of meta.tensors) {
    assert(t.offset + t.length <= aligned.length, `tensor ${t.name} out of bounds`);
    map.set(t.name, aligned.subarray(t.offset, t.offset + t.length));
  }
  return map;
}

function siluInplace(arr) {
  for (let i = 0; i < arr.length; i++) {
    const x = arr[i];
    arr[i] = x / (1 + Math.exp(-x));
  }
  return arr;
}

function conv2d3x3(input, cIn, hIn, wIn, weight, bias, cOut, stride) {
  const hOut = Math.floor((hIn + 2 - 3) / stride) + 1;
  const wOut = Math.floor((wIn + 2 - 3) / stride) + 1;
  const out = new Float32Array(cOut * hOut * wOut);
  const inHW = hIn * wIn;
  const outHW = hOut * wOut;
  for (let oc = 0; oc < cOut; oc++) {
    const b = bias[oc];
    const wBase = oc * cIn * 9;
    const oBase = oc * outHW;
    for (let oh = 0; oh < hOut; oh++) {
      const ihBase = oh * stride - 1;
      for (let ow = 0; ow < wOut; ow++) {
        const iwBase = ow * stride - 1;
        let acc = b;
        for (let ic = 0; ic < cIn; ic++) {
          const icOff = ic * inHW;
          const kwOff = wBase + ic * 9;
          for (let kh = 0; kh < 3; kh++) {
            const ih = ihBase + kh;
            if (ih < 0 || ih >= hIn) continue;
            const rowOff = icOff + ih * wIn;
            const krOff = kwOff + kh * 3;
            for (let kw = 0; kw < 3; kw++) {
              const iw = iwBase + kw;
              if (iw >= 0 && iw < wIn) acc += input[rowOff + iw] * weight[krOff + kw];
            }
          }
        }
        out[oBase + oh * wOut + ow] = acc;
      }
    }
  }
  return { data: out, h: hOut, w: wOut };
}

function avgPool2d3x4(input, c, h, w) {
  const hOut = Math.floor(h / 3);
  const wOut = Math.floor(w / 4);
  const out = new Float32Array(c * hOut * wOut);
  const inHW = h * w;
  const outHW = hOut * wOut;
  for (let ch = 0; ch < c; ch++) {
    const inOff = ch * inHW;
    const oOff = ch * outHW;
    for (let oh = 0; oh < hOut; oh++) {
      const ih0 = oh * 3;
      for (let ow = 0; ow < wOut; ow++) {
        const iw0 = ow * 4;
        let sum = 0;
        for (let kh = 0; kh < 3; kh++) {
          const row = inOff + (ih0 + kh) * w + iw0;
          for (let kw = 0; kw < 4; kw++) sum += input[row + kw];
        }
        out[oOff + oh * wOut + ow] = sum / 12;
      }
    }
  }
  return out;
}

function linear(x, weight, bias, inDim, outDim) {
  const out = new Float32Array(outDim);
  for (let o = 0; o < outDim; o++) {
    let acc = bias[o];
    const row = o * inDim;
    for (let i = 0; i < inDim; i++) acc += weight[row + i] * x[i];
    out[o] = acc;
  }
  return out;
}

function layerNorm(x, weight, bias, dim) {
  let mean = 0;
  for (let i = 0; i < dim; i++) mean += x[i];
  mean /= dim;
  let varSum = 0;
  for (let i = 0; i < dim; i++) {
    const d = x[i] - mean;
    varSum += d * d;
  }
  const invStd = 1 / Math.sqrt(varSum / dim + 1e-5);
  const out = new Float32Array(dim);
  for (let i = 0; i < dim; i++) out[i] = weight[i] * ((x[i] - mean) * invStd) + bias[i];
  return out;
}

function convStem(w, prefix, xChw, cIn) {
  const c1 = conv2d3x3(xChw, cIn, IMG_H, IMG_W, w.get(`${prefix}.conv1.weight`), w.get(`${prefix}.conv1.bias`), 16, 2);
  siluInplace(c1.data);
  const c2 = conv2d3x3(c1.data, 16, c1.h, c1.w, w.get(`${prefix}.conv2.weight`), w.get(`${prefix}.conv2.bias`), 32, 2);
  siluInplace(c2.data);
  const c3 = conv2d3x3(c2.data, 32, c2.h, c2.w, w.get(`${prefix}.conv3.weight`), w.get(`${prefix}.conv3.bias`), 48, 1);
  siluInplace(c3.data);
  const pooled = avgPool2d3x4(c3.data, 48, c3.h, c3.w);
  const proj = linear(pooled, w.get(`${prefix}.proj.weight`), w.get(`${prefix}.proj.bias`), 48 * 3 * 4, EMBED_DIM);
  return layerNorm(proj, w.get(`${prefix}.norm.weight`), w.get(`${prefix}.norm.bias`), EMBED_DIM);
}

function transformerBlock(w, prefix, tokens, seqLen) {
  const normed = [];
  const q = [], k = [], v = [];
  for (let s = 0; s < seqLen; s++) {
    const tok = tokens.subarray(s * EMBED_DIM, (s + 1) * EMBED_DIM);
    const h = layerNorm(tok, w.get(`${prefix}.ln1.weight`), w.get(`${prefix}.ln1.bias`), EMBED_DIM);
    normed.push(h);
    q.push(linear(h, w.get(`${prefix}.q_proj.weight`), w.get(`${prefix}.q_proj.bias`), EMBED_DIM, EMBED_DIM));
    k.push(linear(h, w.get(`${prefix}.k_proj.weight`), w.get(`${prefix}.k_proj.bias`), EMBED_DIM, EMBED_DIM));
    v.push(linear(h, w.get(`${prefix}.v_proj.weight`), w.get(`${prefix}.v_proj.bias`), EMBED_DIM, EMBED_DIM));
  }
  const scale = 1 / Math.sqrt(HEAD_DIM);
  const attnConcat = new Float32Array(seqLen * EMBED_DIM);
  const logits = new Float32Array(seqLen);
  for (let head = 0; head < NUM_HEADS; head++) {
    const hOff = head * HEAD_DIM;
    for (let i = 0; i < seqLen; i++) {
      let maxL = -Infinity;
      for (let j = 0; j < seqLen; j++) {
        let dot = 0;
        for (let d = 0; d < HEAD_DIM; d++) dot += q[i][hOff + d] * k[j][hOff + d];
        logits[j] = dot * scale;
        if (logits[j] > maxL) maxL = logits[j];
      }
      let sumE = 0;
      for (let j = 0; j < seqLen; j++) {
        logits[j] = Math.exp(logits[j] - maxL);
        sumE += logits[j];
      }
      for (let d = 0; d < HEAD_DIM; d++) {
        let val = 0;
        for (let j = 0; j < seqLen; j++) val += (logits[j] / sumE) * v[j][hOff + d];
        attnConcat[i * EMBED_DIM + hOff + d] = val;
      }
    }
  }
  const out = new Float32Array(seqLen * EMBED_DIM);
  const outW = w.get(`${prefix}.out_proj.weight`), outB = w.get(`${prefix}.out_proj.bias`);
  const ln2W = w.get(`${prefix}.ln2.weight`), ln2B = w.get(`${prefix}.ln2.bias`);
  const ff1W = w.get(`${prefix}.ff1.weight`), ff1B = w.get(`${prefix}.ff1.bias`);
  const ff2W = w.get(`${prefix}.ff2.weight`), ff2B = w.get(`${prefix}.ff2.bias`);
  for (let s = 0; s < seqLen; s++) {
    const att = linear(attnConcat.subarray(s * EMBED_DIM, (s + 1) * EMBED_DIM), outW, outB, EMBED_DIM, EMBED_DIM);
    const x1 = new Float32Array(EMBED_DIM);
    for (let d = 0; d < EMBED_DIM; d++) x1[d] = tokens[s * EMBED_DIM + d] + att[d];
    const h2 = layerNorm(x1, ln2W, ln2B, EMBED_DIM);
    const f1 = siluInplace(linear(h2, ff1W, ff1B, EMBED_DIM, FF_DIM));
    const f2 = linear(f1, ff2W, ff2B, FF_DIM, EMBED_DIM);
    for (let d = 0; d < EMBED_DIM; d++) out[s * EMBED_DIM + d] = x1[d] + f2[d];
  }
  return out;
}

function groupNorm4_1d(x, weight, bias, ch, len) {
  const out = new Float32Array(ch * len);
  const gCh = ch / 4;
  const gCount = gCh * len;
  for (let g = 0; g < 4; g++) {
    const cStart = g * gCh;
    let mean = 0;
    for (let c = cStart; c < cStart + gCh; c++) {
      const base = c * len;
      for (let t = 0; t < len; t++) mean += x[base + t];
    }
    mean /= gCount;
    let varSum = 0;
    for (let c = cStart; c < cStart + gCh; c++) {
      const base = c * len;
      for (let t = 0; t < len; t++) {
        const d = x[base + t] - mean;
        varSum += d * d;
      }
    }
    const invStd = 1 / Math.sqrt(varSum / gCount + 1e-5);
    for (let c = cStart; c < cStart + gCh; c++) {
      const base = c * len;
      const wc = weight[c], bc = bias[c];
      for (let t = 0; t < len; t++) out[base + t] = wc * ((x[base + t] - mean) * invStd) + bc;
    }
  }
  return out;
}

function conv1d(x, cIn, len, weight, bias, cOut, kernel, stride = 1, pad = 0) {
  const lOut = Math.floor((len + 2 * pad - kernel) / stride) + 1;
  const out = new Float32Array(cOut * lOut);
  for (let oc = 0; oc < cOut; oc++) {
    const b = bias[oc];
    const wOff = oc * cIn * kernel;
    const oOff = oc * lOut;
    for (let ot = 0; ot < lOut; ot++) {
      const it0 = ot * stride - pad;
      let acc = b;
      for (let ic = 0; ic < cIn; ic++) {
        const xOff = ic * len;
        const kOff = wOff + ic * kernel;
        for (let k = 0; k < kernel; k++) {
          const it = it0 + k;
          if (it >= 0 && it < len) acc += x[xOff + it] * weight[kOff + k];
        }
      }
      out[oOff + ot] = acc;
    }
  }
  return { data: out, len: lOut };
}

function filmResBlock1d(w, prefix, x, cIn, len, cond, cOut) {
  const c1 = conv1d(x, cIn, len, w.get(`${prefix}.conv1.weight`), w.get(`${prefix}.conv1.bias`), cOut, 3, 1, 1);
  const n1 = groupNorm4_1d(c1.data, w.get(`${prefix}.norm1.weight`), w.get(`${prefix}.norm1.bias`), cOut, len);
  const gb = linear(cond, w.get(`${prefix}.film.weight`), w.get(`${prefix}.film.bias`), EMBED_DIM, cOut * 2);
  const mod = new Float32Array(cOut * len);
  for (let c = 0; c < cOut; c++) {
    const scale = 1 + gb[c], shift = gb[cOut + c], base = c * len;
    for (let t = 0; t < len; t++) {
      const v = scale * n1[base + t] + shift;
      mod[base + t] = v / (1 + Math.exp(-v));
    }
  }
  const c2 = conv1d(mod, cOut, len, w.get(`${prefix}.conv2.weight`), w.get(`${prefix}.conv2.bias`), cOut, 3, 1, 1);
  const n2 = siluInplace(groupNorm4_1d(c2.data, w.get(`${prefix}.norm2.weight`), w.get(`${prefix}.norm2.bias`), cOut, len));
  const skipW = w.get(`${prefix}.skip.weight`);
  const skip = skipW ? conv1d(x, cIn, len, skipW, w.get(`${prefix}.skip.bias`), cOut, 1, 1, 0).data : x;
  for (let i = 0; i < n2.length; i++) n2[i] += skip[i];
  return n2;
}

export class NomadPolicy {
  constructor(meta, weightsBytes) {
    this.meta = meta;
    this.w = unpackWeights(meta, weightsBytes);
    this.alphasCumprod = meta.alphas_cumprod;
    this.diffusionSteps = meta.diffusion_steps;
    this.ddimSteps = meta.ddim_steps;
    this.waypointScaleMm = meta.waypoint_scale_mm;
  }

  static fromBuffers(meta, weightsBytes) {
    return new NomadPolicy(meta, weightsBytes);
  }

  _obsTokens(obsOne) {
    const w = this.w;
    const single = 3 * IMG_H * IMG_W;
    const seqLen = CONTEXT_FRAMES + 1;
    const tokens = new Float32Array(seqLen * EMBED_DIM);
    for (let f = 0; f < CONTEXT_FRAMES; f++) {
      const tok = convStem(w, "encoder.obs_stem", obsOne.subarray(f * single, (f + 1) * single), 3);
      tokens.set(tok, f * EMBED_DIM);
    }
    return tokens;
  }

  _encodeOne(obsOne, goalOne, goalMask, cachedTokens = null) {
    const w = this.w;
    const single = 3 * IMG_H * IMG_W;
    const seqLen = CONTEXT_FRAMES + 1;
    const tokens = cachedTokens ? new Float32Array(cachedTokens) : this._obsTokens(obsOne);
    const maskToken = w.get("encoder.mask_token");
    const keep = 1 - goalMask;
    let goalTok = null;
    if (keep !== 0) {
      const pair = new Float32Array(6 * IMG_H * IMG_W);
      pair.set(obsOne.subarray((CONTEXT_FRAMES - 1) * single, CONTEXT_FRAMES * single), 0);
      pair.set(goalOne, single);
      goalTok = convStem(w, "encoder.goal_stem", pair, 6);
    }
    for (let d = 0; d < EMBED_DIM; d++) {
      tokens[CONTEXT_FRAMES * EMBED_DIM + d] = (goalTok ? keep * goalTok[d] : 0) + goalMask * maskToken[d];
    }
    const pos = w.get("encoder.pos_embed");
    for (let i = 0; i < tokens.length; i++) tokens[i] += pos[i];
    let h = transformerBlock(w, "encoder.blocks.0", tokens, seqLen);
    h = transformerBlock(w, "encoder.blocks.1", h, seqLen);
    const fnW = w.get("encoder.final_norm.weight"), fnB = w.get("encoder.final_norm.bias");
    const pooled = new Float32Array(EMBED_DIM * 2);
    pooled.set(layerNorm(h.subarray((CONTEXT_FRAMES - 1) * EMBED_DIM, CONTEXT_FRAMES * EMBED_DIM), fnW, fnB, EMBED_DIM), 0);
    pooled.set(layerNorm(h.subarray(CONTEXT_FRAMES * EMBED_DIM, seqLen * EMBED_DIM), fnW, fnB, EMBED_DIM), EMBED_DIM);
    const ctxProj = linear(pooled, w.get("encoder.ctx_proj.weight"), w.get("encoder.ctx_proj.bias"), EMBED_DIM * 2, EMBED_DIM);
    const context = layerNorm(ctxProj, w.get("encoder.ctx_norm.weight"), w.get("encoder.ctx_norm.bias"), EMBED_DIM);
    const d1 = siluInplace(linear(context, w.get("encoder.dist_fc1.weight"), w.get("encoder.dist_fc1.bias"), EMBED_DIM, EMBED_DIM));
    const predDist = linear(d1, w.get("encoder.dist_fc2.weight"), w.get("encoder.dist_fc2.bias"), EMBED_DIM, 1)[0];
    return { context, predDist };
  }

  encode(obsStack, goalImg, goalMask) {
    const obsStride = CONTEXT_FRAMES * 3 * IMG_H * IMG_W;
    const goalStride = 3 * IMG_H * IMG_W;
    const batch = Math.max(1, Math.round(obsStack.length / obsStride));
    assert(obsStack.length === batch * obsStride, "bad obsStack length");
    assert(goalImg.length === batch * goalStride, "bad goalImg length");
    const masks = typeof goalMask === "number" ? [goalMask] : goalMask;
    assert(masks.length === batch, "goalMask length must match batch");
    const context = new Float32Array(batch * EMBED_DIM);
    const predDist = new Float32Array(batch);
    const firstObs = obsStack.subarray(0, obsStride);
    const firstToks = batch > 1 ? this._obsTokens(firstObs) : null;
    for (let b = 0; b < batch; b++) {
      const obsB = obsStack.subarray(b * obsStride, (b + 1) * obsStride);
      let sameAsFirst = firstToks !== null;
      if (b > 0 && sameAsFirst) {
        for (let k = 0; k < obsStride; k++) {
          if (obsB[k] !== firstObs[k]) { sameAsFirst = false; break; }
        }
      }
      const res = this._encodeOne(
        obsB,
        goalImg.subarray(b * goalStride, (b + 1) * goalStride),
        Number(masks[b]),
        sameAsFirst ? firstToks : null,
      );
      context.set(res.context, b * EMBED_DIM);
      predDist[b] = res.predDist;
    }
    return { context, predDist };
  }

  _denoiseOne(actOne, ctxOne, t) {
    const w = this.w;
    const invFreq = w.get("denoiser.inv_freq");
    const half = EMBED_DIM / 2;
    const tEmb = new Float32Array(EMBED_DIM);
    for (let i = 0; i < half; i++) {
      const ang = t * invFreq[i];
      tEmb[i] = Math.sin(ang);
      tEmb[half + i] = Math.cos(ang);
    }
    const te = linear(tEmb, w.get("denoiser.time_fc.weight"), w.get("denoiser.time_fc.bias"), EMBED_DIM, EMBED_DIM);
    const ce = linear(ctxOne, w.get("denoiser.ctx_fc.weight"), w.get("denoiser.ctx_fc.bias"), EMBED_DIM, EMBED_DIM);
    const cond = new Float32Array(EMBED_DIM);
    for (let i = 0; i < EMBED_DIM; i++) cond[i] = te[i] + ce[i];
    siluInplace(cond);
    const xCh = new Float32Array(ACT_DIM * HORIZON);
    for (let h = 0; h < HORIZON; h++) {
      xCh[h] = actOne[2 * h];
      xCh[HORIZON + h] = actOne[2 * h + 1];
    }
    const h1 = filmResBlock1d(w, "denoiser.down1", xCh, ACT_DIM, HORIZON, cond, 32);
    const pooled = conv1d(h1, 32, HORIZON, w.get("denoiser.down_pool.weight"), w.get("denoiser.down_pool.bias"), 32, 3, 2, 1);
    const hMid = filmResBlock1d(w, "denoiser.mid", pooled.data, 32, pooled.len, cond, 64);
    const up = new Float32Array(64 * HORIZON);
    for (let c = 0; c < 64; c++) {
      const srcOff = c * pooled.len, dstOff = c * HORIZON;
      for (let h = 0; h < HORIZON; h++) up[dstOff + h] = hMid[srcOff + (h >> 1)];
    }
    const upC = conv1d(up, 64, HORIZON, w.get("denoiser.up_conv.weight"), w.get("denoiser.up_conv.bias"), 32, 3, 1, 1);
    const cat = new Float32Array(64 * HORIZON);
    cat.set(upC.data, 0);
    cat.set(h1, 32 * HORIZON);
    const hUp = filmResBlock1d(w, "denoiser.up1", cat, 64, HORIZON, cond, 32);
    const outCh = conv1d(hUp, 32, HORIZON, w.get("denoiser.out_conv.weight"), w.get("denoiser.out_conv.bias"), ACT_DIM, 1, 1, 0).data;
    const out = new Float32Array(HORIZON * ACT_DIM);
    for (let h = 0; h < HORIZON; h++) {
      out[2 * h] = outCh[h];
      out[2 * h + 1] = outCh[HORIZON + h];
    }
    return out;
  }

  denoise(noisyActions, context, timestep) {
    const stride = HORIZON * ACT_DIM;
    const batch = Math.max(1, Math.round(noisyActions.length / stride));
    assert(noisyActions.length === batch * stride, "bad noisyActions shape");
    assert(context.length === batch * EMBED_DIM, "bad context shape");
    const ts = typeof timestep === "number" ? new Float32Array(batch).fill(timestep) : timestep;
    const out = new Float32Array(batch * stride);
    for (let b = 0; b < batch; b++) {
      const eps = this._denoiseOne(
        noisyActions.subarray(b * stride, (b + 1) * stride),
        context.subarray(b * EMBED_DIM, (b + 1) * EMBED_DIM),
        Number(ts[b]),
      );
      out.set(eps, b * stride);
    }
    return out;
  }

  ddimSample(context, { numSamples = 4, ddimSteps = null, initNoise = null, seed = 0 } = {}) {
    const steps = Math.trunc(ddimSteps ?? this.ddimSteps);
    assert(numSamples >= 1 && numSamples <= 32, "numSamples out of bounds");
    assert(steps >= 1 && steps <= this.diffusionSteps, "ddimSteps out of bounds");
    const stride = HORIZON * ACT_DIM;
    const ctxBatch = new Float32Array(numSamples * EMBED_DIM);
    for (let b = 0; b < numSamples; b++) ctxBatch.set(context.subarray(0, EMBED_DIM), b * EMBED_DIM);
    let x;
    if (initNoise) {
      assert(initNoise.length === numSamples * stride, "initNoise length mismatch");
      x = new Float32Array(initNoise);
    } else {
      const randn = makeRng(seed);
      x = new Float32Array(numSamples * stride);
      for (let i = 0; i < x.length; i++) x[i] = randn();
    }
    const strideT = Math.max(1, Math.floor(this.diffusionSteps / steps));
    const tList = [];
    for (let i = 0; i < steps; i++) {
      const t = this.diffusionSteps - 1 - i * strideT;
      if (t >= 0) tList.push(t);
    }
    if (tList[tList.length - 1] !== 0) tList[tList.length - 1] = 0;
    for (let idx = 0; idx < tList.length; idx++) {
      const t = tList[idx];
      const tPrev = idx + 1 < tList.length ? tList[idx + 1] : -1;
      const eps = this.denoise(x, ctxBatch, t);
      const abT = this.alphasCumprod[t];
      const abPrev = tPrev >= 0 ? this.alphasCumprod[tPrev] : 1.0;
      const sAbT = Math.sqrt(abT), sOneT = Math.sqrt(1 - abT);
      const sAbP = Math.sqrt(abPrev), sOneP = Math.sqrt(Math.max(0, 1 - abPrev));
      for (let i = 0; i < x.length; i++) {
        const x0 = clamp((x[i] - sOneT * eps[i]) / sAbT, -1.8, 1.8);
        x[i] = sAbP * x0 + sOneP * eps[i];
      }
    }
    const trajs = [];
    for (let b = 0; b < numSamples; b++) {
      const tr = [];
      for (let h = 0; h < HORIZON; h++) {
        tr.push([x[b * stride + 2 * h] * this.waypointScaleMm, x[b * stride + 2 * h + 1] * this.waypointScaleMm]);
      }
      trajs.push(tr);
    }
    return trajs;
  }
}

export class TopologicalGraph {
  constructor() {
    this.nodes = [];
    this.edges = new Map();
    this.currentId = null;
  }

  reset() {
    this.nodes = [];
    this.edges = new Map();
    this.currentId = null;
  }

  addEdge(src, dst, weight) {
    assert(src >= 0 && src < this.nodes.length && dst >= 0 && dst < this.nodes.length, "bad topological edge id");
    assert(weight > 0, "edge weight must be positive");
    if (src === dst) return;
    const prev = this.edges.get(src)?.get(dst) ?? Infinity;
    const w = Math.min(prev, weight);
    this.edges.get(src).set(dst, w);
    this.edges.get(dst).set(src, w);
  }

  _addNode(imageU8, poseMm, tick, label = "") {
    const id = this.nodes.length;
    this.nodes.push({ id, imageU8: new Uint8Array(imageU8), poseMm: [...poseMm], label, visits: 1, seenTick: tick });
    this.edges.set(id, new Map());
    return id;
  }

  localize(obsOne, distBatchFn, poseMm = null) {
    if (this.nodes.length === 0) return { bestId: null, dists: new Float32Array(0) };
    const n = this.nodes.length;
    const obsStride = CONTEXT_FRAMES * 3 * IMG_H * IMG_W;
    const goalStride = 3 * IMG_H * IMG_W;
    const obsBatch = new Float32Array(n * obsStride);
    const goalBatch = new Float32Array(n * goalStride);
    for (let i = 0; i < n; i++) {
      obsBatch.set(obsOne, i * obsStride);
      goalBatch.set(rgbToChw(this.nodes[i].imageU8), i * goalStride);
    }
    const dists = distBatchFn(obsBatch, goalBatch, n);
    let bestId = 0, bestScore = Infinity;
    for (let i = 0; i < n; i++) {
      let score = dists[i];
      if (poseMm) score += 0.008 * Math.hypot(poseMm[0] - this.nodes[i].poseMm[0], poseMm[1] - this.nodes[i].poseMm[1]);
      if (score < bestScore) { bestScore = score; bestId = i; }
    }
    return { bestId, dists };
  }

  recordStep(obsOne, curImgU8, poseMm, distBatchFn, tick = 0, label = "") {
    if (this.nodes.length === 0) {
      const id = this._addNode(curImgU8, poseMm, tick, label);
      this.currentId = id;
      return id;
    }
    const { bestId, dists } = this.localize(obsOne, distBatchFn, poseMm);
    let minMetric = Infinity;
    for (const node of this.nodes) {
      const m = Math.hypot(poseMm[0] - node.poseMm[0], poseMm[1] - node.poseMm[1]);
      if (m < minMetric) minMetric = m;
    }
    const curId = this.currentId ?? bestId;
    if ((dists[bestId] > EDGE_NEW_DIST_STEPS || minMetric > EDGE_NEW_METRIC_MM) && this.nodes.length < MAX_NODES) {
      const nid = this._addNode(curImgU8, poseMm, tick, label);
      this.addEdge(curId, nid, Math.max(1.0, dists[curId]));
      for (let oid = 0; oid < nid; oid++) {
        const odMm = Math.hypot(poseMm[0] - this.nodes[oid].poseMm[0], poseMm[1] - this.nodes[oid].poseMm[1]);
        if (dists[oid] <= LOOP_CLOSE_DIST_STEPS && odMm < 190) {
          this.addEdge(oid, nid, Math.max(1.0, dists[oid]));
        }
      }
      this.currentId = nid;
      return nid;
    }
    this.nodes[bestId].visits += 1;
    this.nodes[bestId].seenTick = tick;
    if (curId !== bestId && dists[curId] <= LOOP_CLOSE_DIST_STEPS) {
      this.addEdge(curId, bestId, Math.max(1.0, dists[curId]));
    }
    this.currentId = bestId;
    return bestId;
  }

  shortestPath(src, dst) {
    const n = this.nodes.length;
    assert(src >= 0 && src < n && dst >= 0 && dst < n, "invalid topological node id");
    const dist = new Float64Array(n).fill(Infinity);
    const prev = new Int32Array(n).fill(-1);
    const visited = new Uint8Array(n);
    dist[src] = 0;
    for (let iter = 0; iter < n; iter++) {
      let u = -1, best = Infinity;
      for (let i = 0; i < n; i++) {
        if (!visited[i] && dist[i] < best) { best = dist[i]; u = i; }
      }
      if (u < 0 || u === dst) break;
      visited[u] = 1;
      for (const [vId, w] of this.edges.get(u)?.entries() ?? []) {
        const alt = dist[u] + w;
        if (!visited[vId] && alt < dist[vId]) {
          dist[vId] = alt;
          prev[vId] = u;
        }
      }
    }
    if (!Number.isFinite(dist[dst])) return src === dst ? [src] : [src, dst];
    const path = [];
    for (let cur = dst, step = 0; cur >= 0 && step < n; cur = prev[cur], step++) {
      path.push(cur);
      if (cur === src) break;
    }
    return path.reverse();
  }

  chooseSubgoal(obsOne, targetId, distBatchFn, poseMm = null) {
    assert(targetId >= 0 && targetId < this.nodes.length, "target node id out of range");
    const { bestId, dists } = this.localize(obsOne, distBatchFn, poseMm);
    const directDist = dists[targetId];
    if (directDist <= 3.5) {
      return { subId: targetId, subImg: this.nodes[targetId].imageU8, directDist };
    }
    const path = this.shortestPath(bestId, targetId);
    const subId = path.length >= 2 ? path[1] : targetId;
    return { subId, subImg: this.nodes[subId].imageU8, directDist };
  }

  snapshot() {
    const seen = new Set();
    const edges = [];
    for (const [u, nbrs] of this.edges.entries()) {
      for (const [v, weight] of nbrs.entries()) {
        const a = Math.min(u, v), b = Math.max(u, v);
        const key = `${a}:${b}`;
        if (!seen.has(key)) {
          seen.add(key);
          edges.push({ u: a, v: b, weight: Number(weight.toFixed(2)) });
        }
      }
    }
    return {
      nodes: this.nodes.map((n) => ({
        id: n.id, x_mm: n.poseMm[0], y_mm: n.poseMm[1], yaw_deg: n.poseMm[2], label: n.label, visits: n.visits,
      })),
      edges,
      currentId: this.currentId,
    };
  }
}

export function selectSafeTrajectory(candsMm, state, v) {
  assert(Array.isArray(candsMm) && candsMm.length >= 1, "candidates required");
  let bestIdx = 0, bestScore = -Infinity;
  const halfL = 0.5 * v.table_length_mm - 65;
  const halfW = 0.5 * v.table_width_mm - 65;
  const cYaw = Math.cos(state.pose_angle_rad), sYaw = Math.sin(state.pose_angle_rad);
  for (let idx = 0; idx < candsMm.length; idx++) {
    const traj = candsMm[idx];
    const reach = Math.hypot(traj[2][0], traj[2][1]);
    const endX = state.x_mm + cYaw * traj[4][0] - sYaw * traj[4][1];
    const endY = state.y_mm + sYaw * traj[4][0] + cYaw * traj[4][1];
    let score = 0.25 * traj[2][0] + 0.1 * reach;
    if (Math.abs(endX) > halfL || Math.abs(endY) > halfW) score -= 220;
    if (state.prox_found_object && state.prox_distance_mm < 135 && traj[2][0] > 25 && Math.abs(traj[2][1]) < 24) {
      score -= 180;
    }
    if (state.cliff_detected && traj[1][0] > 0) score -= 250;
    if (score > bestScore) { bestScore = score; bestIdx = idx; }
  }
  return { traj: candsMm[bestIdx], index: bestIdx };
}

export function waypointToWheels(trajMm, v, cliffAny = false, proxFound = false, proxMm = 400) {
  if (cliffAny) return [0, 0];
  const tx = 0.5 * (trajMm[1][0] + trajMm[2][0]);
  const ty = 0.5 * (trajMm[1][1] + trajMm[2][1]);
  const dist = Math.hypot(tx, ty);
  const alpha = Math.atan2(ty, tx);
  const top = v.wheel_speed_max_mmps, half = 0.5 * v.tread_spacing_mm;
  if (dist < 10) return [0, 0];
  if (Math.abs(alpha) > 0.70) {
    const omega = Math.sign(alpha || 1) * clamp(Math.abs(alpha) * 1.4, 0.8, 1.9);
    return [clamp(-omega * half, -top, top), clamp(omega * half, -top, top)];
  }
  const spdCap = proxFound && proxMm < 72 ? 0 : (!proxFound || proxMm > 130 ? 105 : 55);
  const lin = clamp(dist / 0.60, spdCap > 0 ? 22 : 0, spdCap);
  const omega = clamp(2.2 * alpha, -1.6, 1.6);
  return [clamp(lin - omega * half, -top, top), clamp(lin + omega * half, -top, top)];
}

export class NomadAutonomy {
  constructor(sim, spec = null) {
    this.sim = sim;
    this.spec = spec;
    this.policy = null;
    this.graph = new TopologicalGraph();
    this.framesU8 = [];
    this.goalImgU8 = null;
    this.goalNodeId = null;
    this.lastPredDist = 0;
    this.lastCandidatesMm = null;
    this.lastSelectedIdx = 0;
    this.lastObsU8 = null;
    this.phase = null;
    this._planSeed = 1;
  }

  attachPolicy(policy, spec = null) {
    this.policy = policy;
    if (spec) this.spec = spec;
  }

  reset() {
    this.graph.reset();
    this.framesU8 = [];
    this.goalImgU8 = null;
    this.goalNodeId = null;
    this.lastPredDist = 0;
    this.lastCandidatesMm = null;
    this.lastSelectedIdx = 0;
    this.lastObsU8 = null;
    this.phase = null;
    this._planSeed = 1;
  }

  _distBatch(obsBatch, goalBatch, n) {
    assert(this.policy, "NoMaD policy missing");
    const masks = new Float32Array(n);
    return this.policy.encode(obsBatch, goalBatch, masks).predDist;
  }

  _sampleCameraObs() {
    const img = renderHeadCamRgb(this.sim, this.spec ?? this.sim.cameraSpec);
    this.lastObsU8 = img;
    this.framesU8.push(img);
    if (this.framesU8.length > CONTEXT_FRAMES) this.framesU8.shift();
    return stackObsFrames(this.framesU8);
  }

  captureGoal(label = "goal") {
    assert(this.policy, "NoMaD policy missing");
    const obsOne = this._sampleCameraObs();
    const curImg = this.framesU8[this.framesU8.length - 1];
    const s = this.sim.state;
    const pose = [s.x_mm, s.y_mm, (s.pose_angle_rad / DEG)];
    const nid = this.graph.recordStep(
      obsOne, curImg, pose, (o, g, n) => this._distBatch(o, g, n), this.sim.tick_, label,
    );
    this.goalImgU8 = new Uint8Array(curImg);
    this.goalNodeId = nid;
    return nid;
  }

  startExplore(durationS = 10) {
    assert(this.policy, "NoMaD policy missing");
    assert(Number.isFinite(durationS) && durationS >= 0.5 && durationS <= 60, "bad explore duration");
    const onDock = this.sim.chargerContact || this.sim._onDockPlatform();
    return {
      mode: "nomad_explore",
      controller: this,
      phaseName: onDock ? "NOMAD UNDOCKING" : "NOMAD EXPLORING",
      stage: onDock ? "undock" : "nomad",
      undockTicks: 0,
      goalMask: 1,
      goalNodeId: null,
      goalImgU8: null,
      planTick: 0,
      wheelCmd: [0, 0],
      recoverTicks: 0,
      closeStreak: 0,
      deadlineTick: this.sim.tick_ + Math.round(durationS / this.sim.dt),
      savedLevel: null,
    };
  }

  startNavigate(maxS = 20, goalNodeId = null) {
    assert(this.policy, "NoMaD policy missing");
    assert(Number.isFinite(maxS) && maxS >= 0.5 && maxS <= 90, "bad navigate duration");
    const gid = goalNodeId ?? this.goalNodeId;
    const targetImg = (gid !== null && gid >= 0 && gid < this.graph.nodes.length)
      ? this.graph.nodes[gid].imageU8 : this.goalImgU8;
    assert(targetImg !== null && targetImg !== undefined, "no NoMaD goal captured");
    const onDock = this.sim.chargerContact || this.sim._onDockPlatform();
    return {
      mode: "nomad_navigate",
      controller: this,
      phaseName: onDock ? "NOMAD UNDOCKING" : "NOMAD NAVIGATING",
      stage: onDock ? "undock" : "nomad",
      undockTicks: 0,
      goalMask: 0,
      goalNodeId: gid,
      goalImgU8: targetImg,
      planTick: 0,
      wheelCmd: [0, 0],
      recoverTicks: 0,
      closeStreak: 0,
      deadlineTick: this.sim.tick_ + Math.round(maxS / this.sim.dt),
      savedLevel: null,
    };
  }

  restoreControlLevel(ctx) {
    if (ctx && ctx.savedLevel !== null) {
      this.sim.controlLevel = ctx.savedLevel;
      ctx.savedLevel = null;
    }
    this.phase = null;
  }

  _plan(ctx) {
    const sim = this.sim;
    const obsOne = this._sampleCameraObs();
    const curImg = this.framesU8[this.framesU8.length - 1];
    const s = sim.state;
    const pose = [s.x_mm, s.y_mm, s.pose_angle_rad / DEG];
    const distFn = (o, g, n) => this._distBatch(o, g, n);
    this.graph.recordStep(obsOne, curImg, pose, distFn, sim.tick_);
    let gChw;
    if (ctx.goalMask >= 0.5 || !ctx.goalImgU8) {
      gChw = rgbToChw(curImg);
    } else if (ctx.goalNodeId !== null && ctx.goalNodeId >= 0 && ctx.goalNodeId < this.graph.nodes.length) {
      const sub = this.graph.chooseSubgoal(obsOne, ctx.goalNodeId, distFn, pose);
      gChw = rgbToChw(sub.subImg);
      this.lastPredDist = sub.directDist;
    } else {
      gChw = rgbToChw(ctx.goalImgU8);
    }
    const enc = this.policy.encode(obsOne, gChw, ctx.goalMask);
    if (ctx.goalMask < 0.5 && ctx.goalNodeId === null) this.lastPredDist = enc.predDist[0];
    const cands = this.policy.ddimSample(enc.context, { numSamples: 4, seed: this._planSeed++ });
    this.lastCandidatesMm = cands;
    const { traj, index } = selectSafeTrajectory(cands, s, sim.v);
    this.lastSelectedIdx = index;
    ctx.selectedTraj = traj;
    ctx.wheelCmd = waypointToWheels(traj, sim.v, sim.hazards.cliffAny, s.prox_found_object, s.prox_distance_mm);
  }

  step(ctx) {
    const sim = this.sim;
    if (!ctx.headSet) {
      ctx.headSet = true;
      sim.commandHeadAngle(4);
      sim.commandLiftHeight(32);
    }
    this.phase = ctx.phaseName;
    if (sim._reactPhase !== "idle") return { running: true };
    if (!sim.hasControl) {
      sim.requestControl();
      ctx.recoverTicks = 90;
    }
    if (ctx.recoverTicks > 0) {
      ctx.recoverTicks -= 1;
      sim._wheelTarget = ctx.recoverTicks > 45 ? [-65, -65] : [-70, 70];
      return { running: true };
    }
    if (sim.tick_ >= ctx.deadlineTick) {
      return {
        running: false,
        result: ctx.mode === "nomad_explore" ? "completed" : "timeout",
        nodes: this.graph.nodes.length,
        edges: this.snapshot().edges.length,
        pred_dist: Number(this.lastPredDist.toFixed(3)),
      };
    }
    if (ctx.stage === "undock") {
      ctx.undockTicks += 1;
      sim._wheelTarget = [90, 90];
      if ((!sim.chargerContact && !sim._onDockPlatform() && ctx.undockTicks >= 160) || ctx.undockTicks >= 420) {
        sim._wheelTarget = [0, 0];
        ctx.stage = "nomad";
        ctx.phaseName = ctx.mode === "nomad_explore" ? "NOMAD EXPLORING" : "NOMAD NAVIGATING";
      }
      return { running: true };
    }
    if (ctx.planTick % PLAN_INTERVAL_TICKS === 0) {
      this._plan(ctx);
      if (ctx.goalMask < 0.5) {
        ctx.closeStreak = this.lastPredDist <= 1.25 ? ctx.closeStreak + 1 : 0;
        if (ctx.closeStreak >= 2) {
          return {
            running: false,
            result: "completed",
            nodes: this.graph.nodes.length,
            edges: this.snapshot().edges.length,
            pred_dist: Number(this.lastPredDist.toFixed(3)),
          };
        }
      }
    }
    ctx.planTick += 1;
    if (sim.hazards.cliffAny) {
      ctx.wheelCmd = [0, 0];
    } else if (ctx.selectedTraj) {
      ctx.wheelCmd = waypointToWheels(
        ctx.selectedTraj, sim.v, sim.hazards.cliffAny, sim.state.prox_found_object, sim.state.prox_distance_mm,
      );
    }
    sim._wheelTarget = [ctx.wheelCmd[0], ctx.wheelCmd[1]];
    return { running: true };
  }

  snapshot() {
    const g = this.graph.snapshot();
    return {
      ready: Boolean(this.policy),
      nodes: g.nodes,
      edges: g.edges,
      currentId: g.currentId,
      goalNodeId: this.goalNodeId,
      predDist: Number(this.lastPredDist.toFixed(2)),
      candidates: this.lastCandidatesMm,
      selectedIdx: this.lastSelectedIdx,
      hasGoal: Boolean(this.goalImgU8),
      obsU8: this.lastObsU8,
      goalU8: this.goalImgU8,
    };
  }
}
