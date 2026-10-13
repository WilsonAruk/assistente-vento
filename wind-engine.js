/**
 * WindEngine — motor de cálculo do Assistente de Vento (tiro com arco)
 * ---------------------------------------------------------------------
 * Modelo: flecha como ponto de massa com arrasto quadrático em relação ao AR
 * (não ao solo), integração numérica 3D. Assim o vento afeta a flecha do
 * jeito físico correto:
 *   - vento lateral  -> deriva lateral (≈ W · (t − D/v0), a "lei do atraso")
 *   - vento de cauda -> menos arrasto, chega mais rápido -> acerta mais ALTO
 *   - vento de frente-> mais arrasto, chega mais lento  -> acerta mais BAIXO
 *
 * A mira é "zerada" na distância sem vento; o resultado é o desvio causado
 * só pelo vento. Não modela weathervaning (a flecha girando para o vento),
 * rajadas reais nem turbulência — por isso o resultado é uma FAIXA.
 *
 * Sem dependências. Funciona no navegador (window.WindEngine), Node/React
 * Native (module.exports) e pode ser portado 1:1 para Dart/Kotlin/Swift.
 */
(function (root) {
  const G = 9.81;

  // v0 em m/s; k = coeficiente de arrasto por metro (a = k·v²).
  // k calibrado para perda típica de velocidade em 70 m (recurvo ~18%).
  const BOW_PRESETS = {
    recurve:  { v0: 62, k: 0.0029 }, // recurvo olímpico (~203 fps)
    compound: { v0: 88, k: 0.0022 }, // composto (~289 fps)
    barebow:  { v0: 57, k: 0.0031 },
    longbow:  { v0: 50, k: 0.0036 },
  };

  /** Converte direção "de onde o vento vem" (graus, 0 = vem do alvo,
   *  90 = vem da direita, 180 = vem das costas, 270 = vem da esquerda)
   *  em componentes no referencial do arqueiro (m/s). */
  function windVector(speedKmh, fromDeg) {
    const w = speedKmh / 3.6;
    const th = (fromDeg * Math.PI) / 180;
    return {
      along: -w * Math.cos(th), // + = vento de cauda (empurra para o alvo)
      cross: -w * Math.sin(th), // + = empurra para a DIREITA
    };
  }

  /** Voo até o plano do alvo. Retorna altura (y), lateral (z), tempo, vel. */
  function fly(v0, angle, k, wind, D, dt = 0.0005) {
    let x = 0, y = 0, z = 0, t = 0;
    let vx = v0 * Math.cos(angle), vy = v0 * Math.sin(angle), vz = 0;
    let px = 0, py = 0, pz = 0, pt = 0;
    while (x < D && t < 8) {
      px = x; py = y; pz = z; pt = t;
      const rx = vx - wind.along, ry = vy, rz = vz - wind.cross;
      const s = Math.hypot(rx, ry, rz);
      vx += -k * s * rx * dt;
      vy += (-G - k * s * ry) * dt;
      vz += -k * s * rz * dt;
      x += vx * dt; y += vy * dt; z += vz * dt; t += dt;
    }
    const f = (D - px) / (x - px || 1);
    return {
      y: py + (y - py) * f,
      z: pz + (z - pz) * f,
      t: pt + (t - pt) * f,
      v: Math.hypot(vx, vy, vz),
    };
  }

  /** Ângulo de lançamento que zera na distância D sem vento (bisseção). */
  function zeroAngle(v0, k, D) {
    let lo = 0, hi = 0.35;
    const calm = { along: 0, cross: 0 };
    for (let i = 0; i < 40; i++) {
      const mid = (lo + hi) / 2;
      if (fly(v0, mid, k, calm, D).y < 0) lo = mid; else hi = mid;
    }
    return (lo + hi) / 2;
  }

  function single(v0, k, D, angle, speedKmh, fromDeg) {
    const r = fly(v0, angle, k, windVector(speedKmh, fromDeg), D);
    return { lateralCm: r.z * 100, verticalCm: r.y * 100, t: r.t, v: r.v };
  }

  /**
   * Cálculo principal.
   * @param {object} p
   *  distance   m (ex.: 70)
   *  faceCm     diâmetro do alvo (122, 80, 60, 40)
   *  bow        'recurve' | 'compound' | 'barebow' | 'longbow'
   *  v0         m/s (opcional; sobrescreve o preset)
   *  windKmh    velocidade do vento
   *  windFrom   graus, de onde o vento VEM (0 alvo, 90 dir., 180 costas, 270 esq.)
   *  gust       variação de rajada 0–1 (ex.: 0.2 = ±20%)
   */
  function calculate(p) {
    const preset = BOW_PRESETS[p.bow] || BOW_PRESETS.recurve;
    const v0 = p.v0 || preset.v0;
    const k = preset.k;
    const D = p.distance;
    const gust = p.gust ?? 0.2;
    const angle = zeroAngle(v0, k, D);

    const nom = single(v0, k, D, angle, p.windKmh, p.windFrom);
    const lo = single(v0, k, D, angle, p.windKmh * (1 - gust), p.windFrom);
    const hi = single(v0, k, D, angle, p.windKmh * (1 + gust), p.windFrom);
    const calm = single(v0, k, D, angle, 0, 0);

    // ±10% de incerteza do modelo (arrasto real varia com flecha/penas)
    const band = (a, b) => {
      const m1 = Math.abs(a) * 0.9, m2 = Math.abs(b) * 1.1;
      return [Math.min(m1, m2), Math.max(m1, m2)];
    };

    const wv = windVector(p.windKmh, p.windFrom);
    const ringCm = p.faceCm / 20; // largura de cada anel (10 anéis no raio)

    // Confiança: piora com vento forte, rajada alta e distância longa
    let score = 0;
    if (p.windKmh > 15) score++;
    if (p.windKmh > 25) score++;
    if (gust > 0.3) score++;
    if (D >= 60) score++;
    const confidence = score <= 1 ? 'alta' : score <= 2 ? 'média' : 'baixa';

    return {
      v0, k, distance: D, faceCm: p.faceCm, ringCm,
      flightTime: [Math.min(lo.t, hi.t), Math.max(lo.t, hi.t)],
      impactSpeed: nom.v,
      crosswindKmh: Math.abs(wv.cross) * 3.6,
      alongwindKmh: wv.along * 3.6, // + cauda, − frente
      lateral: {
        cm: nom.lateralCm,
        range: band(lo.lateralCm, hi.lateralCm),
        dir: nom.lateralCm > 0.05 ? 'direita' : nom.lateralCm < -0.05 ? 'esquerda' : null,
        rings: Math.abs(nom.lateralCm) / ringCm,
      },
      vertical: {
        cm: nom.verticalCm - calm.verticalCm,
        range: band(lo.verticalCm - calm.verticalCm, hi.verticalCm - calm.verticalCm),
        dir: nom.verticalCm - calm.verticalCm > 0.05 ? 'cima'
           : nom.verticalCm - calm.verticalCm < -0.05 ? 'baixo' : null,
        rings: Math.abs(nom.verticalCm - calm.verticalCm) / ringCm,
      },
      // Correção de mira = o oposto do desvio
      aim: { xCm: -nom.lateralCm, yCm: -(nom.verticalCm - calm.verticalCm) },
      confidence,
    };
  }

  const api = { calculate, windVector, BOW_PRESETS };
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.WindEngine = api;
})(typeof window !== 'undefined' ? window : globalThis);
