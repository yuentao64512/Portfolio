/* live_dot.js - the liveness dot (v0.1.47, WP10k-a, docs/UI_QOL_2026-11-07.md item 9; OCS_V2_SPEC §2 rule 5: motion encodes an event).
   One small component for every page: a dot that LIGHTS FOR 200 ms ON EACH MESSAGE RECEIVED (not a blink, nothing moves between
   messages) + the age text; past the stream's limit the dot and the text turn --alarm with the word "stale" beside them. The colours are
   tokens of themes.css (--up lit, --dim idle, --alarm stale), so it follows the theme. Self-contained: the CSS is injected once.

   Serve it beside the page (judge_web serves /live_dot.js; copy the route to any server: it is this one file) and:
       <script src="/live_dot.js"></script>
       const d = LiveDot.create('heartbeat + reports of USV1');   // {el, feed, state}; d.el is a <span class="ld">
       someCell.appendChild(d.el);                                // keep the same object across redraws (re-append d.el)
       d.feed(times, now, limit_s);                               // on each poll: message times (epoch s, any order, repeats ok),
                                                                  // the server's clock, the stream's limit (s)
   A poll at 1 Hz of a 2 Hz stream brings two new times: each one lights the dot, replayed at its offset from the first new one, so the
   dot flashes with the stream. The first feed only takes the times in (no replay of history).
   LiveDot.LIGHT_MS = 200; LiveDot.ageText(age_s, limit_s) -> {text, stale} (pure, for tests; '0.4\u00a0s': the unit never wraps). */
(function () {
  'use strict';
  const LIGHT_MS = 200;
  const CSS = '.ld{display:inline-flex;align-items:center;gap:5px;white-space:nowrap;font-variant-numeric:tabular-nums;vertical-align:baseline}' +
    '.ld i{display:inline-block;width:8px;height:8px;border-radius:50%;border:1px solid var(--dim);background:transparent;box-sizing:border-box}' +
    '.ld.lit i{background:var(--up,var(--ok));border-color:var(--up,var(--ok))}' +
    '.ld .a{color:var(--muted,var(--dim));min-width:6.5em;font-size:11px}' +
    '.ld.stale i{background:var(--alarm,var(--bad));border-color:var(--alarm,var(--bad))}.ld.stale .a{color:var(--alarm,var(--bad));font-weight:700}';
  function inject() {
    if (document.getElementById('live-dot-css')) return;
    const st = document.createElement('style'); st.id = 'live-dot-css'; st.textContent = CSS; (document.head || document.documentElement).appendChild(st);
  }
  function ageText(age, limit) {
    if (age == null || !isFinite(age)) return { text: 'no message', stale: true };
    const stale = limit != null && age > limit;
    return { text: (stale ? 'stale ' : '') + age.toFixed(1) + '\u00a0s', stale };
  }
  function create(label) {
    inject();
    const el = document.createElement('span'); el.className = 'ld'; el.title = (label ? label + ': ' : '') + 'lights 200 ms per message received · the age of the last one';
    const dot = document.createElement('i'), a = document.createElement('span'); a.className = 'a'; a.textContent = 'no message';
    el.appendChild(dot); el.appendChild(a);
    const st = { last: null, until: 0, timer: 0, flashes: 0 };
    function light() {
      st.flashes++; el.classList.add('lit'); clearTimeout(st.timer);
      st.timer = setTimeout(() => el.classList.remove('lit'), LIGHT_MS);
    }
    function feed(times, now, limit) {
      const ts = (times || []).map(Number).filter(t => isFinite(t)).sort((x, y) => x - y);
      const newest = ts.length ? ts[ts.length - 1] : null;
      if (st.last !== null) {
        const fresh = ts.filter(t => t > st.last + 1e-6);
        for (const t of fresh) { const d = Math.max(0, (t - fresh[0]) * 1000); if (d === 0) light(); else setTimeout(light, d); }
      }
      if (newest !== null) st.last = st.last === null ? newest : Math.max(st.last, newest);
      const r = ageText(st.last === null ? null : (now - st.last), limit);
      a.textContent = r.text; el.classList.toggle('stale', r.stale);
      return r;
    }
    return { el, feed, state: st };
  }
  window.LiveDot = { create, ageText, LIGHT_MS };
})();
