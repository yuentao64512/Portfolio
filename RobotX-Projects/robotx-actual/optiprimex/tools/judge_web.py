"""judge_web: the judge's view in the browser (v0.1.26; Dmitrii's request beside ROADMAP WP7). The same decoding as rc-tui
(tools/rc_tui.RcModel: the bench JSON on rc/# and RoboNation's protobuf on robocommand/robotx/#), read-only, served as one
page that refreshes every second: the course, the declaration, the RunStart, per vehicle the whole Heartbeat (state, position,
speed, heading, roll, pitch, altitude, depth, task, type, flight phase, registration; rate and age), the task timeline, the
latest report of each kind with its content, and both message logs. v0.1.29 (WP8b): JUDGE ACTIONS - rc-tui's commands as buttons
with what each one means for us (course, RunStart, assistance, readiness confirm, keep-out, all clear, moving object), sent by the
same tools/rc_tui.JudgeSender: the fake judge is asked (default), --as-judge publishes the RxCommand itself (rc-test), --monitor never.
POST /cmd {"line": "start 3"} needs the header X-Judge-Web: 1 (a cross-site form cannot set it).

v0.1.47 (WP10k-a B): the fills are theme tokens; three columns (ACTIONS · JUDGE RECEIVES · JUDGE SENDS) under a strip of the declaration /
course / run, and below them the judge map (map.html?judge=1: the course, the geofence and its editor, the vehicles, the incidents; a
click places the next assist / keepout / moving position) - /map.html, /course.json and POST /geofence are served / proxied here
(same origin); the liveness dot beside our RC topic (tools/live_dot.js, /live_dot.js); the theme shared through /prefs (tools/prefs.py);
/status.json = which broker this page is wired to (the launcher's rc-test link).
v0.1.53: the COURSE strip's course select (GET /courses.json = sim/courses.py's catalogue) + "publish course" (POST /course {"course": ID},
X-Judge-Web: 1) - the fake judge re-publishes it, --as-judge publishes the RxCourse itself (publish_course()).
v0.1.54: SAFE PASSAGE - the buoys of each vehicle's last SafePassageReport (BeaconState, lat / lon, entry / exit) and, when the SIM
page answers (/world.json, SIM_PORT), the nearest truth buoy, its state and the distance to it (passage_buoys(); /state.json `passage`).

  judge-web [--host 127.0.0.1 --port 1885] [--team SGIT] [--as-judge [--wire protobuf|json] | --monitor] [--web-port 8095] [--bind 127.0.0.1] [--open]
  normally window `web` of tmux `judge` (scripts/run_judge.sh); scripts/run_rctest.sh opens it in the browser.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import socketserver
import subprocess
import sys
import time
from typing import Optional
from http.server import BaseHTTPRequestHandler, HTTPServer

import paho.mqtt.client as mqtt

from . import prefs as PREFS
from .rc_tui import HELP, JudgeSender, RcModel, monitor_subscriptions

HERE = os.path.dirname(os.path.abspath(__file__))
HB_LIMIT_S = 2.0                          # v0.1.47: our heartbeat stream's limit (2 Hz sent; RoboCommand wants 1 Hz) - past it the dot is stale

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Judge view</title>
<style>
/* v0.1.47 (WP10k-a B7): every fill is a token of /themes.css - no dark box on the light themes */
body{margin:0;background:var(--bg);color:var(--ink);font:13px ui-monospace,Menlo,Consolas,monospace}
header{padding:8px 14px;background:var(--panel);border-bottom:1px solid var(--line);display:flex;gap:18px;flex-wrap:wrap;align-items:baseline}
header b{font-size:15px}.dim{color:var(--dim)}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
.strip{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;padding:12px 12px 0}
main{display:grid;grid-template-columns:minmax(250px,.85fr) minmax(0,1.3fr) minmax(0,1.3fr);gap:12px;padding:12px}
@media(max-width:1100px){main,.strip{grid-template-columns:1fr}}
section{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:10px 12px;min-width:0}
h2{margin:0 0 8px;font-size:13px;color:var(--acc);letter-spacing:.04em}h3{margin:10px 0 4px;font-size:12px;color:var(--mag)}
table{border-collapse:collapse;width:100%}td{padding:2px 8px 2px 0;vertical-align:top}td.k{color:var(--dim);white-space:nowrap;width:1%}
.veh{border-top:1px solid var(--line);padding-top:8px;margin-top:8px}.log{max-height:340px;overflow:auto;white-space:pre-wrap}
.log div{border-bottom:1px solid var(--line2);padding:1px 0}table.mt td{border-bottom:1px solid var(--line2);padding:1px 6px 1px 0;white-space:nowrap}table.mt td.x{white-space:normal}
table.mt tr.warn td{color:var(--warn)}.pill{padding:1px 6px;border-radius:9px;border:1px solid var(--line)}
.agrid{display:grid;gap:8px;grid-template-columns:1fr}
.act{border:1px solid var(--line);border-radius:6px;padding:8px;background:var(--panel2)}.act b{color:var(--ink)}.act p{margin:4px 0 6px;color:var(--dim);font-size:12px}
.act.placing{border-color:var(--acc)}
.act input,.act select{width:84px;background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:4px;padding:2px 4px;font:12px ui-monospace,Menlo,monospace;margin:0 4px 4px 0}
button{background:var(--btnBg,var(--panel2));color:var(--ink);border:1px solid var(--btnLine,var(--line));border-radius:5px;padding:4px 10px;font:12px ui-monospace,Menlo,monospace;cursor:pointer}
button:hover{border-color:var(--acc)}button.go{background:var(--actionPale);border:2px solid var(--actionLine);font-weight:700}button:disabled{opacity:.45;cursor:not-allowed}
.mode{padding:2px 8px;border-radius:9px;font-weight:700}.mode.fake{background:var(--accBg);color:var(--accText)}.mode.judge{background:var(--warnBg);color:var(--warnText)}.mode.monitor{background:var(--chipBg);color:var(--dim)}
#res{margin-top:6px;min-height:16px}#tip{position:fixed;z-index:9;max-width:380px;background:var(--panel2);color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:7px 9px;
font:12px -apple-system,Segoe UI,sans-serif;pointer-events:none;display:none;white-space:pre-line}
#bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:6px 12px;border-bottom:1px solid var(--line);background:var(--panel)}
#bar b{font-size:15px;letter-spacing:.04em}#bar b.warn{color:var(--warn)}#bar b.bad{color:var(--alarm,var(--bad))}#bar b.acc{color:var(--acc)}
#bar .c{border:1px solid var(--line);border-radius:10px;padding:1px 8px;font:11px ui-monospace,Menlo,monospace}#bar .c.bad{color:var(--alarm,var(--bad));border-color:var(--alarm,var(--bad))}
#bar .sp{flex:1}#bar select,#place{background:var(--bg);color:var(--ink);border:1px solid var(--line);border-radius:4px;font:12px ui-monospace,Menlo,monospace;padding:2px 4px}
.jmap{margin:0 12px 12px}.jmap iframe{display:block;width:100%;height:560px;border:1px solid var(--line);border-radius:6px;background:var(--mapBg,var(--bg))}
.vh{display:flex;gap:10px;align-items:baseline;flex-wrap:wrap}
.veh.silent{opacity:.55}.veh.silent .vh b{text-decoration:line-through}
</style><link rel="stylesheet" href="/themes.css"></head><body>
<div id="bar" title="the OCS's global bar (OCS_V2_SPEC §3): the state word + why, who commands the FCU, the run, the alarms - from the OCS page's /state.json"><b id="b-w">OCS ?</b><span id="b-y" class="dim"></span><span class="c" id="b-f"></span><span class="c" id="b-r"></span><span class="c" id="b-a"></span><span class="sp"></span><select id="theme" title="the theme of every page (shared through the OCS page's /prefs; the launcher's choice by default - a change here goes to all open pages)"><option value="dark">dark</option><option value="light">light</option><option value="hc-dark">HC dark</option><option value="hc-light">HC light</option><option value="fun">fun</option></select></div>
<header><b>Judge view</b><span id="mode" class="mode"></span><span id="hdr" class="dim">connecting…</span><span id="flav" class="dim" title="the flavour: preset · judge · net · FCU · boat profile · CV (the launcher's record; ? = started by hand)"></span><span class="dim">the same commands typed: rc-tui (<code>tmux attach -t judge</code>)</span></header>
<div class="strip"><section><h2>DECLARATION</h2><div id="decl"></div></section><section><h2>COURSE</h2><div id="course"></div><div class="vh" id="crs-pick" style="margin-top:6px" title="v0.1.53: the course the judge publishes (sim/courses.py) - the RxCourse goes out as RoboCommand's: the fake judge re-publishes it (rc/sim/inject), --as-judge publishes it on this broker (rc-test: rctest_manual follows it for its geofence check)"><select id="crs-sel" title="the course to publish: EXTRA (ours, 120 x 80 m, every task) / ALPHA (rc-test's demo course)"></select><button id="crs-pub" title="publish this course (RxCourse, retained) - the OCS gets it; the SIM page then offers moving the world there">publish course</button><span id="crs-res" class="dim"></span></div></section><section><h2>RUN</h2><div id="run"></div></section></div>
<main>
<section class="acts"><h2>JUDGE ACTIONS <span class="dim" id="modetxt" style="font-weight:400;letter-spacing:0"></span></h2><div class="agrid" id="acts"></div><div id="res" class="dim"></div></section>
<section><h2>JUDGE RECEIVES (OCS → RoboCommand)</h2><div id="veh"></div>
<h3>RECEIVED <select id="fk-rx" title="show one kind of message (heartbeats are many: hidden by default)"><option value="-hb">all but heartbeats</option><option value="">all</option></select></h3>
<div class="log"><table class="mt" id="rx"></table></div></section>
<section><h2>JUDGE SENDS (RoboCommand → OCS)</h2>
<h3>SENT <select id="fk-tx" title="show one kind of message"><option value="">all</option></select></h3><div class="log"><table class="mt" id="tx"></table></div></section>
</main>
<section class="jmap" id="spb-sec" title="v0.1.54: the buoys of the last SafePassageReport of each vehicle - its BeaconState and lat / lon as the judge decoded them; with the simulator running, the nearest truth buoy (the SIM page's world), its state and how far the reported position is from it"><h2>SAFE PASSAGE · THE LAST REPORT'S BUOYS <span class="dim" id="spb-h" style="font-weight:400;letter-spacing:0"></span></h2><div id="spb"><span class="dim">no SafePassageReport yet</span></div></section>
<section class="jmap"><h2>THE JUDGE MAP <span class="dim" style="font-weight:400;letter-spacing:0">course · geofence · vehicles · incidents - a click places the position of
<select id="place" title="the command whose lat / lon a click on the map fills (then press its button above)"><option value="assist">assist</option><option value="keepout">keepout</option><option value="moving">moving</option></select>
· the geofence editor writes the OCS profile's geofence: as on the OCS map</span></h2><iframe id="jmapf" src="/map.html?judge=1" title="the judge map (map.html?judge=1)"></iframe></section>
<script src="/live_dot.js"></script>
<script>
const $=id=>document.getElementById(id), esc=s=>String(s??'').replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
const f=(x,d=1)=>x==null?'-':Number(x).toFixed(d), short=s=>String(s??'?').replace(/^(STATE_|TASK_|TIER_|FLIGHT_PHASE_|TYPE_)/,'');
const hhmmss=t=>new Date(t*1000).toTimeString().slice(0,8);
const ACTS=[
 {w:'course',t:'Course',d:'(Re)publish the course (RxCourse, retained). Our OCS checks it: R1 → R2 COURSE_VALID. Only --as-judge publishes one; the fake judge keeps its own.',f:[]},
 {w:'start',t:'RunStart',d:'Answer our RunDeclaration: the run starts (OCS R3/R4 → R5 RUNNING, run_id to the vehicle). Defaults to the last declaration seen. With rc-test (RCTEST_RUNSTART=manual, the default since v0.1.52) this button is the only RunStart; a second one for a declaration that already has its RunStart is ignored by the OCS.',f:[['seq','declaration','']],go:1},
 {w:'assist',t:'Assistance request (T4)',d:'A vessel needs help at LAT LON: the incident task. In AUTO_HOLD with a run it auto-starts m4t0; the vehicle answers with a ReadinessReport.',f:[['lat','lat',''],['lon','lon',''],['dom','domain','surface',['surface','air']]]},
 {w:'confirm',t:'Readiness confirm',d:'Confirm the last ReadinessReport: T4 leaves WAIT_CONFIRM and goes. Needs a ReadinessReport first.',f:[]},
 {w:'keepout',t:'Keep-out zone',d:'A circle the vehicles must avoid (LAT LON, radius m). The OCS acks with an IncidentAck.',f:[['lat','lat',''],['lon','lon',''],['r','radius m','10'],['dom','domain','surface',['surface','air']]]},
 {w:'allclear',t:'All clear',d:'Lift the incident / keep-out: back to normal operation.',f:[['dom','domain','surface',['surface','air']]]},
 {w:'moving',t:'Moving object',d:'Report a moving object at LAT LON with heading and speed; the vehicle should keep clear.',f:[['lat','lat',''],['lon','lon',''],['hdg','heading °','90'],['spd','speed m/s','0.5'],['dom','domain','surface',['surface','air']]]},
];
let S0=null,built=false;const DOTS={};
const tip=$('tip');document.addEventListener('mouseover',e=>{const el=e.target.closest('[data-tip]');if(!el){tip.style.display='none';return}tip.textContent=el.dataset.tip;tip.style.display='block'});
document.addEventListener('mousemove',e=>{if(tip.style.display==='block'){tip.style.left=Math.min(e.clientX+14,innerWidth-tip.offsetWidth-8)+'px';tip.style.top=(e.clientY+16)+'px'}});
function centre(s){const b=(s.course&&s.course.boundary)||[];if(b.length){return [b.reduce((a,p)=>a+p[0],0)/b.length,b.reduce((a,p)=>a+p[1],0)/b.length]}
  const v=Object.values(s.vehicles||{}).find(o=>o.heartbeat&&o.heartbeat.lat!=null);return v?[v.heartbeat.lat,v.heartbeat.lon]:[1.28113,103.85503]}
function buildActs(s){const c=centre(s);
  $('acts').innerHTML=ACTS.map(a=>`<div class="act" data-tip="${esc(a.d)}"><b>${esc(a.t)}</b><p>${esc(a.d)}</p>`+a.f.map(([k,l,def,ch])=>{
    const v=def||(k==='lat'?c[0].toFixed(6):k==='lon'?c[1].toFixed(6):'');
    return ch?`<select id="a-${a.w}-${k}" title="${l}">${ch.map(x=>`<option>${x}</option>`).join('')}</select>`:`<input id="a-${a.w}-${k}" placeholder="${l}" value="${v}" title="${l}">`}).join('')+
    `<div><button class="${a.go?'go':''}" id="b-${a.w}" title="${esc(a.t)}: ${esc(a.d)}">${a.w}</button> <span class="dim" id="p-${a.w}"></span></div></div>`).join('');
  for(const a of ACTS){const upd=()=>$('p-'+a.w).textContent=line(a);for(const [k] of a.f)$(`a-${a.w}-${k}`).oninput=upd;upd();$('b-'+a.w).onclick=()=>act(a)}built=true}
function line(a){let s=a.w;for(const [k] of a.f){const v=$(`a-${a.w}-${k}`).value.trim();if(k==='dom'){if(v==='air')s+=' air';continue}if(v)s+=' '+v}return s}
async function act(a){const l=line(a);$('res').textContent='> '+l+' …';try{const r=await (await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json','X-Judge-Web':'1'},body:JSON.stringify({line:l})})).json();
  $('res').innerHTML=`&gt; ${esc(l)} : <span class="${r.ok?'ok':'bad'}">${esc(r.text)}</span>`}catch(e){$('res').innerHTML='<span class="bad">judge_web not reachable</span>'}tick()}
function rows(o){return '<table>'+o.map(([k,v,c])=>`<tr><td class="k">${k}</td><td class="${c||''}">${v}</td></tr>`).join('')+'</table>'}
function drawPassage(P){if(!P||!P.length){$('spb').innerHTML='<span class="dim">no SafePassageReport yet</span>';$('spb-h').textContent='';return}
  const sh=b=>esc(String(b||'?').replace('BEACON_STATE_','')), dd=d=>d==null?'-':`<span class="${d>1?'bad':'ok'}">${Number(d).toFixed(2)} m</span>`;
  $('spb-h').textContent=P[0].truth?`truth: ${P[0].truth}`:'no truth (the SIM page does not answer)';
  $('spb').innerHTML=P.map(p=>`<h3>${esc(p.vehicle)} · ${p.buoys.length} buoys · ${p.ago_s!=null?p.ago_s+' s ago':''} · entry ${p.entry?`${Number(p.entry.lat).toFixed(7)} ${Number(p.entry.lon).toFixed(7)} (${dd(p.entry.d_m)})`:'-'} · exit ${p.exit?`${Number(p.exit.lat).toFixed(7)} ${Number(p.exit.lon).toFixed(7)} (${dd(p.exit.d_m)})`:'-'}</h3>`+
    '<table class="mt"><tr class="dim"><td>#</td><td>state</td><td>lat</td><td>lon</td><td>truth</td><td>truth state</td><td>off by</td></tr>'+
    p.buoys.map((b,i)=>`<tr class="${b.truth_beacon&&b.truth_beacon!==b.beacon?'warn':''}"><td>${i+1}</td><td>${sh(b.beacon)}</td><td>${Number(b.lat).toFixed(7)}</td><td>${Number(b.lon).toFixed(7)}</td>`+
      `<td>${b.truth_id!=null?'#'+esc(b.truth_id):'-'}</td><td>${b.truth_beacon?sh(b.truth_beacon):'-'}</td><td>${dd(b.d_m)}</td></tr>`).join('')+'</table>'+
    (p.missing&&p.missing.length?`<div class="warn">not reported: ${p.missing.map(m=>'#'+esc(m.id)+' '+sh(m.beacon)).join(', ')}</div>`:'')).join('')}
function draw(s){drawPassage(s.passage);
  if(!built)buildActs(s);
  const M={fake:['fake','asks OUR fake judge (rc/sim/inject) to send it, with its own numbering'],judge:['as judge','publishes the RoboCommand message itself ('+s.wire+' wire) - rc-test / a bare broker'],monitor:['monitor','read-only: the course network; nothing is sent']}[s.mode]||['?',''];
  $('mode').className='mode '+s.mode;$('mode').textContent=M[0];$('modetxt').textContent='· '+M[1];
  for(const a of ACTS){const b=$('b-'+a.w);if(b)b.disabled=s.mode==='monitor'||(a.w==='course'&&s.mode!=='judge')||(a.w==='confirm'&&!s.has_readiness)||(a.w==='start'&&!s.declaration&&!$('a-start-seq').value)}
  const sq=$('a-start-seq');if(sq&&!sq.value&&s.declaration&&document.activeElement!==sq)sq.placeholder='last: '+s.declaration.request_seq;
  $('hdr').innerHTML=`team <b>${esc(s.team)}</b> · <span class="${s.connected?'ok':'bad'}">${s.connected?'connected':'NOT connected'}</span> · wires json ${s.wires.json} / protobuf ${s.wires.protobuf} · undecodable ${s.undecodable}`;
  const d=s.declaration;
  $('decl').innerHTML=d?rows([['declaration',`seq ${d.request_seq} · ${esc((d.vehicle_ids||[]).join(', '))} · ${s.declarations} sent`,'ok'],
    ['tiers',Object.entries(d.tiers||{}).map(([k,v])=>`${k} ${short(v).toLowerCase()}`).join(' · ')],
    ['UAV geofence',`${(d.uav_geofence||[]).length} points · ceiling ${f(d.uav_geofence_max_altitude_hae_m)} m HAE · pilot ${esc(d.uav_pilot_callsign)||'-'}`]])
    :'<div class="dim">no RunDeclaration yet</div>';
  $('veh').innerHTML=Object.entries(s.vehicles).map(([v,o])=>{const h=o.heartbeat||{};const fresh=o.age_s!=null&&o.age_s<2;
    const rep=Object.values(o.reports||{}).map(r=>`<div>${esc(r.text)} <span class="dim">(${r.ago_s} s ago)</span></div>`).join('')||'<span class="dim">none</span>';
    const tl=(o.timeline||[]).slice(-6).map(x=>`${short(x.task)} <span class="dim">${x.ago_s}s</span>`).join(' → ')||'-';
    const silent=o.age_s==null||o.age_s>3;   // v0.1.54: no heartbeat for > 3 s - the card dims and says SILENT (reporting stopped / the OCS gone)
    return `<div class="veh${silent?' silent':''}"><div class="vh"><b>${esc(v)}</b>${silent?` <span class="pill bad" title="no heartbeat from the OCS for this vehicle for more than 3 s: reporting stopped (STOP REPORTING / STOP &amp; RESET), the OCS is down, or the vehicle's status is stale">SILENT${o.age_s!=null?' · '+Math.round(o.age_s)+' s':''}</span>`:''} <span class="lds" data-v="${esc(v)}" title="our RC topic: heartbeats + reports of ${esc(v)}"></span> <span class="pill ${fresh?'ok':'bad'}">${o.heartbeat?short(h.state):'no heartbeat'}</span> ${o.heartbeat?`${short(h.current_task)} · ${esc(h.vehicle_type)} · seq ${h.report_seq} · ${f(o.rate_hz)} Hz · age ${f(o.age_s)} s`:''}</div>`+
      (o.heartbeat?rows([['position',`${f(h.lat,6)} ${f(h.lon,6)}`],['speed / heading',`${f(h.spd_mps,2)} m/s · ${f(h.heading_deg,0)}°`],
        ['roll / pitch',`${f(h.roll_deg)}° · ${f(h.pitch_deg)}°`],['altitude HAE / depth',`${f(h.altitude_hae_m)} m · ${f(h.depth_m)} m`],
        ['flight phase / reg',`${short(h.flight_phase)} · ${esc(h.uav_registration)||'-'}`],['task timeline',tl],['latest reports',rep]]):rows([['latest reports',rep]]))+'</div>'}).join('')
    ||'<div class="dim">no vehicle heard yet</div>';
  // v0.1.47 (B9): the liveness dot beside our RC topic - one LiveDot per vehicle, kept across redraws (tools/live_dot.js)
  document.querySelectorAll('#veh .lds[data-v]').forEach(ph=>{const v=ph.dataset.v,L=(s.live||{})[v];if(!L||!window.LiveDot)return;
    const d=DOTS[v]||(DOTS[v]=LiveDot.create(L.topic||v));ph.appendChild(d.el);d.feed(L.times,s.now,L.limit_s)});
  const c=s.course;
  $('course').innerHTML=c?rows([['course',`<b>${esc(c.course_id)}</b> (${c.wire}, ${c.ago_s} s ago)`,'ok'],['corners',(c.boundary||[]).length],
    ['UAV transit area',(c.uav_transit_area||[]).length+' points'],['pinger',`${c.pinger_hz} Hz`]]):'<div class="warn">no RxCourse seen</div>';
  const r=s.run_start; $('run').innerHTML=rows([['RunStart',r?`run <b>${r.run_id}</b> for declaration ${r.declaration_seq}`:'<span class="dim">not yet</span>',r?'ok':'']]);
  {const dv=(d&&d.vehicle_ids)||[],out=dv.filter(v=>{const h=((s.vehicles||{})[v]||{}).heartbeat;return !h||h.state!=='STATE_AUTO'});   // v0.1.37: as RoboCommand shows it
   const ended=r&&dv.length&&out.length===dv.length;
   $('run').innerHTML+=rows([['run timeline',`${d?'Declared '+d.request_seq:'-'}${d&&!r?' → Held (waiting for RunStart)':''}${r?' → Started run '+r.run_id:''}`+
     (ended?` → <span class="bad"><b>Ended: ${esc(out.map(v=>v+' left AUTO ('+short((((s.vehicles||{})[v]||{}).heartbeat||{}).state||'no heartbeat')+')').join(', '))} - the last vehicle out</b></span>`:''),ended?'':'ok']])}
  for(const [id,arr] of [['rx',s.received],['tx',s.sent]]){const rows=arr.map(x=>parseMsg(x,id)),sel=$('fk-'+id),want=sel.value;
    const kinds=[...new Set(rows.map(r=>r.kind))].sort();const opts=(id==='rx'?['-hb','']:['']).concat(kinds);
    if(sel.dataset.k!==opts.join('|')){sel.dataset.k=opts.join('|');sel.innerHTML=opts.map(k=>`<option value="${esc(k)}">${k==='-hb'?'all but heartbeats':k===''?'all':esc(k)}</option>`).join('');sel.value=want||opts[0]}
    const el=$(id),box=el.parentElement,atEnd=box.scrollTop+box.clientHeight>=box.scrollHeight-4,f=sel.value;
    el.innerHTML='<tr class="dim"><td>time</td><td>dir</td><td>kind</td><td>seq</td><td>vehicle</td><td>the message</td></tr>'+rows.filter(r=>!f||(f==='-hb'?r.kind!=='heartbeat':r.kind===f))
      .map(r=>`<tr class="${r.warn?'warn':''}"><td>${hhmmss(r.t)}</td><td>${id==='rx'?'→ judge':'← judge'}</td><td>${esc(r.kind)}</td><td>${esc(r.seq)}</td><td>${esc(r.veh)}</td><td class="x">${esc(r.text)}</td></tr>`).join('');
    if(atEnd)box.scrollTop=box.scrollHeight}
}
// v0.1.37 (WP10d): the messages as table rows - kind / seq / vehicle read from the text rc-tui's model keeps
function parseMsg(x,id){const t=x.text;let m,kind='other',seq='',veh='';
  if((m=t.match(/^RunDeclaration seq (\d+)/)))[kind,seq]=['run_declaration',m[1]];
  else if((m=t.match(/^RxCourse/)))kind='course';
  else if((m=t.match(/^(\S+) current_task (\S+) -> (\S+)/)))[kind,veh]=['task_change',m[1]];
  else if((m=t.match(/^(\S+) #(\d+) (\S+)/)))[veh,seq,kind]=[m[1],m[2],m[3].replace(/[:,]$/,'')];
  else if((m=t.match(/^#(\d+) (\S+)/)))[seq,kind]=[m[1],m[2].replace(/[:,]$/,'')];
  return {t:x.t,kind,seq,veh,text:t,warn:/refus|reject|undecod|wrong|stale|error/i.test(t)}}
// v0.1.47 (A6): the theme is shared - /prefs (proxied to the OCS page's, the launcher's when the OCS is down) read on load and every
// 2 s; this page's select writes back there; localStorage only a cache for the first paint (and the map frame's, same origin)
let THEME=null;
function applyTheme(t){if(!t)return;THEME=t;document.documentElement.dataset.theme=t;if(document.activeElement!==$('theme'))$('theme').value=t;
  try{localStorage.setItem('ocs.theme',t)}catch(e){}try{const d=$('jmapf').contentDocument;if(d&&d.documentElement)d.documentElement.dataset.theme=t}catch(e){}}
try{let t=localStorage.getItem('ocs.theme')||localStorage.getItem('judge.theme');if(t==='contrast')t='hc-dark';applyTheme(t||'dark')}catch(e){applyTheme('dark')}
$('theme').onchange=()=>{applyTheme($('theme').value);fetch('/prefs',{method:'POST',headers:{'Content-Type':'application/json','X-Judge-Web':'1'},body:JSON.stringify({theme:THEME})}).catch(()=>{})};
async function prefs(){try{const d=await (await fetch('/prefs',{cache:'no-store'})).json();const t=(d.prefs||{}).theme;if(t&&t!==THEME)applyTheme(t)}catch(e){}}
prefs();setInterval(prefs,2000);
$('jmapf').addEventListener('load',()=>applyTheme(THEME));
// v0.1.47 (B8): the judge map - a click there places the lat / lon of the chosen command (assist / keepout / moving); the placed
// positions go back to the map as "next ..." marks
const PLACE=['assist','keepout','moving'];
function marks(){const w=$('jmapf').contentWindow;if(!w)return;
  w.postMessage({type:'judge-marks',marks:PLACE.map(k=>({w:k,lat:parseFloat(($('a-'+k+'-lat')||{}).value),lon:parseFloat(($('a-'+k+'-lon')||{}).value)})).filter(m=>isFinite(m.lat)&&isFinite(m.lon)&&placed[m.w])},'*')}
const placed={};
function selPlace(){for(const k of PLACE){const el=$('b-'+k);if(el)el.closest('.act').classList.toggle('placing',k===$('place').value)}}
$('place').onchange=selPlace;
window.addEventListener('message',e=>{const d=e.data||{};if(e.source!==$('jmapf').contentWindow||d.type!=='judge-pos'||!built)return;
  const w=$('place').value,a=ACTS.find(x=>x.w===w);$('a-'+w+'-lat').value=Number(d.lat).toFixed(7);$('a-'+w+'-lon').value=Number(d.lon).toFixed(7);
  $('p-'+w).textContent=line(a);placed[w]=true;marks();
  $('res').innerHTML=`placed <b>${esc(Number(d.lat).toFixed(7))} ${esc(Number(d.lon).toFixed(7))}</b> into ${esc(w)} - press <b>${esc(w)}</b> to send it`});
function drawBar(B){if(!B){$('b-w').textContent='OCS not reachable';$('b-w').className='warn';return}
  $('b-w').textContent=B.word+(B.why?' —':'');$('b-w').className=(B.kill||B.word==='LINK LOST')?'bad':(B.word==='NOT READY'||B.word==='HELD')?'warn':(B.word==='RUNNING'?'acc':'');
  $('b-y').textContent=B.why||'';$('b-f').textContent=`FCU: ${B.holder} · ${B.v} · ${B.mode} · ${B.armed===true?'armed':B.armed===false?'disarmed':'armed ?'} · ${B.kill?'KILL':'kill clear'}`;
  $('b-f').className='c'+(B.kill?' bad':'');$('b-r').textContent=`${B.r}${B.run_id?' · run '+B.run_id:''}`;
  $('b-a').textContent=`${B.alarms} alarm${B.alarms===1?'':'s'} · ${B.warnings} warning${B.warnings===1?'':'s'}`;$('b-a').className='c'+(B.alarms?' bad':'');
}
async function tick(){try{const d=await (await fetch('/state.json',{cache:'no-store'})).json();draw(d);selPlace();marks();if(d.flavour)$('flav').textContent=' '+d.flavour.text;drawBar(d.bar)}catch(e){$('hdr').textContent='judge_web not reachable'}}
tick(); setInterval(tick,1000);
// v0.1.53: the course select - /courses.json (the catalogue), POST /course {course} publishes it as the judge would
async function courses(){try{const d=await (await fetch('/courses.json',{cache:'no-store'})).json();
  $('crs-sel').innerHTML=d.courses.map(c=>`<option value="${esc(c.id)}" title="${esc(c.name)} · ${c.length_m} x ${c.width_m} m · ${esc(c.tasks.join(' '))}">${esc(c.id)} · ${c.length_m} x ${c.width_m} m</option>`).join('');
  $('crs-sel').value=d.default;$('crs-pub').disabled=d.mode==='monitor'}catch(e){$('crs-res').textContent='no course list'}}
$('crs-pub').onclick=async()=>{const c=$('crs-sel').value;$('crs-res').textContent='publishing '+c+' …';
  try{const r=await (await fetch('/course',{method:'POST',headers:{'Content-Type':'application/json','X-Judge-Web':'1'},body:JSON.stringify({course:c})})).json();
    $('crs-res').innerHTML=`<span class="${r.ok?'ok':'bad'}">${esc(r.text)}</span>`}catch(e){$('crs-res').innerHTML='<span class="bad">judge_web not reachable</span>'}tick()};
courses();
</script><div id="tip"></div></body></html>"""


def _ocs_bar() -> Optional[dict]:
    """v0.1.43: the OCS page's global bar (its /state.json `bar`): the same word + why on the judge page. None when the OCS page is down."""
    import urllib.request
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{os.environ.get('OCS_WEB_PORT', '8096')}/state.json", timeout=0.6) as r:
            return json.loads(r.read()).get("bar")
    except Exception:
        return None


def _flavour(sender) -> Optional[dict]:
    """v0.1.42 (WP9d): the flavour line from the launcher's record (None when it cannot be read)."""
    try:
        from .launcher import flavour
        return flavour(None, os.environ.get("NET_PROFILE") or None)
    except Exception:
        return None


# ---- v0.1.47 (WP10k-a B8, B9): the incidents for the map, the liveness of our RC topic, the OCS proxy ---------------------------------
_INC = (("assist", re.compile(r"^#(\d+) AssistanceRequest (\w+) at (-?[\d.]+) (-?[\d.]+)")),
        ("keepout", re.compile(r"^#(\d+) KeepOutZone (\w+) at (-?[\d.]+) (-?[\d.]+) r (-?[\d.]+) m")),
        ("moving", re.compile(r"^#(\d+) MovingObjectAlert at (-?[\d.]+) (-?[\d.]+) hdg (-?[\d.]+) spd (-?[\d.]+)")))
_ALLCLEAR = re.compile(r"^#(\d+) AllClear (\w+)")


def incidents(sent) -> list:
    """The judge's commands with a position (from rc-tui's text of each RxCommand sent): assist / keepout / moving; an AllClear of the
    same domain marks the earlier assist / keep-out cleared."""
    out = []
    for x in sent or []:
        t = x.get("text", "")
        m = _ALLCLEAR.match(t)
        if m:
            for it in out:
                if it.get("domain") == m.group(2) and it["kind"] in ("assist", "keepout"):
                    it["cleared"] = True
            continue
        for kind, rx in _INC:
            m = rx.match(t)
            if not m:
                continue
            g = m.groups()
            if kind == "moving":
                it = {"kind": kind, "seq": int(g[0]), "lat": float(g[1]), "lon": float(g[2]), "hdg": float(g[3]), "spd": float(g[4]), "domain": "surface"}
            else:
                it = {"kind": kind, "seq": int(g[0]), "domain": g[1], "lat": float(g[2]), "lon": float(g[3])}
                if kind == "keepout":
                    it["r"] = float(g[4])
            out.append(dict(it, t=x.get("t"), cleared=False))
            break
    return out


# ---- v0.1.54: the last SafePassageReport's buoys against the simulator's truth --------------------------------------------------
TRUTH_BEACON = {("red", "flash"): "BEACON_STATE_FLASHING_RED", ("green", "flash"): "BEACON_STATE_FLASHING_GREEN",
                ("blue", "flash"): "BEACON_STATE_FLASHING_BLUE", ("blue", "steady"): "BEACON_STATE_STEADY_BLUE"}
_TRUTH = {"t": -1e9, "w": None}


def sim_truth(max_age_s: float = 5.0) -> Optional[dict]:
    """The SIM page's world (/world.json on SIM_PORT, default 8097), cached a few seconds; None when it does not answer."""
    import urllib.request
    if time.time() - _TRUTH["t"] < max_age_s:
        return _TRUTH["w"]
    _TRUTH["t"] = time.time()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{os.environ.get('SIM_PORT', '8097')}/world.json", timeout=0.6) as r:
            _TRUTH["w"] = json.loads(r.read())
    except Exception:
        _TRUTH["w"] = None
    return _TRUTH["w"]


def truth_buoys(world: Optional[dict]) -> list:
    """[{id, beacon, lat, lon}] of a world's Task 1 buoys (the sensed ones: T2's are not reported) - BLACK / unlit = OFF."""
    if not world or not world.get("origin"):
        return []
    from ..sim import world as W
    fr = W.WorldFrame(world["origin"])
    out = []
    for o in world.get("objects") or []:
        if o.get("kind") != "buoy" or o.get("sensed", True) is False or o.get("x_n") is None:
            continue
        lat, lon = fr.to_latlon(float(o["x_n"]), float(o["y_e"]))
        out.append({"id": o.get("id"), "role": o.get("role"), "beacon": TRUTH_BEACON.get((o.get("colour"), o.get("light")), "BEACON_STATE_OFF"),
                    "lat": lat, "lon": lon})
    return out


def _m(a_lat, a_lon, b_lat, b_lon) -> float:
    import math
    k = math.radians(1.0) * 6371000.0
    return math.hypot((a_lat - b_lat) * k, (a_lon - b_lon) * k * math.cos(math.radians(a_lat)))


def passage_buoys(snap: dict, world: Optional[dict] = None) -> list:
    """Per vehicle with a SafePassageReport: its buoys (beacon, lat, lon) + entry / exit, each with the nearest truth buoy and the
    distance to it when `world` (the SIM page's) is given; `missing` = truth buoys no reported buoy is the nearest report of."""
    truth = truth_buoys(world)
    out = []
    for v, o in sorted((snap.get("vehicles") or {}).items()):
        r = (o.get("reports") or {}).get("safe_passage")
        if not r:
            continue
        rows, hit = [], set()
        for b in r.get("buoys") or []:
            lat, lon = b.get("lat"), b.get("lon")
            beacon = b.get("beacon") or b.get("state")
            row = {"beacon": beacon, "lat": lat, "lon": lon}
            if truth and lat is not None and lon is not None:
                t = min(truth, key=lambda t: _m(lat, lon, t["lat"], t["lon"]))
                row.update(truth_id=t["id"], truth_beacon=t["beacon"], d_m=round(_m(lat, lon, t["lat"], t["lon"]), 2))
                hit.add(t["id"])
            rows.append(row)

        def ll(p, role):
            if not p or p.get("lat") is None:
                return None
            d = dict(lat=p["lat"], lon=p["lon"])
            t = next((t for t in truth if t["role"] == role), None)
            if t is not None:
                d["d_m"] = round(_m(p["lat"], p["lon"], t["lat"], t["lon"]), 2)
            return d
        out.append({"vehicle": v, "ago_s": r.get("ago_s"), "buoys": rows, "entry": ll(r.get("entry_position"), "entry"),
                    "exit": ll(r.get("exit_position"), "exit"), "truth": (world or {}).get("name") if truth else None,
                    "missing": [{"id": t["id"], "beacon": t["beacon"]} for t in truth if t["id"] not in hit]})
    return out


def liveness(model: RcModel) -> dict:
    """Per vehicle: the times of the last messages on our RC topic (heartbeats + the latest report of each kind) for the liveness dot."""
    with model.lock:
        out = {}
        for v in set(model.hb_times) | set(model.last_report):
            ts = list(model.hb_times.get(v) or [])[-10:] + [r.get("_t") for r in (model.last_report.get(v) or {}).values() if r.get("_t")]
            out[v] = {"times": sorted(ts)[-12:], "limit_s": HB_LIMIT_S, "topic": f"our RC topic: {v}'s heartbeats + reports"}
        return out


def _ocs(path: str, body: Optional[bytes] = None, timeout: float = 2.0):
    """(status, bytes, content type) of the OCS page's path (GET, or POST with its header); (502, ...) when it does not answer."""
    import urllib.error
    import urllib.request
    req = urllib.request.Request(PREFS.ocs_url().rstrip("/") + path, data=body, method="POST" if body is not None else "GET",
                                 headers={"Content-Type": "application/json", "X-OCS-Web": "1"} if body is not None else {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, r.read(), r.headers.get("Content-Type", "application/json")
    except urllib.error.HTTPError as e:
        return e.code, e.read(), "application/json"
    except Exception:
        return 502, b"the OCS page does not answer", "text/plain"


RCTEST_NOTE = (" - rc-test through rctest_manual follows it (its geofence check too); a bare test_server.py keeps its own: "
               "scripts/stop_rctest.sh; RCTEST_COURSE={cid} scripts/run_rctest.sh")


def publish_course(sender: Optional[JudgeSender], cid: str) -> dict:
    """v0.1.53: the judge page's "publish course": the fake judge is asked (rc/sim/inject {"kind": "course"}: it re-publishes its retained
    RxCourse on its wire); --as-judge publishes the catalogue course's RxCourse itself, retained, on its wire (rc-test's broker: the OCS
    gets it at once); --monitor never."""
    from ..sim import courses as C
    crs = C.get(cid)
    if crs is None:
        return {"ok": False, "text": f"no course {cid!r} (one of {', '.join(C.COURSES)})"}
    if sender is None or sender.mode == "monitor":
        return {"ok": False, "text": "refused: this page is read-only (monitor)"}
    if sender.mode == "fake":
        sender.c.publish("rc/sim/inject", json.dumps({"kind": "course", "course": crs.id}), qos=0)
        return {"ok": True, "text": f"asked the fake judge to publish course {crs.id} (retained RxCourse)", "course_id": crs.id}
    body = dict(crs.body(), published_at=time.time())
    if sender.wire == "json":
        sender.c.publish("rc/course", json.dumps(body), qos=0, retain=True)
    else:
        from ..ocs import rc_wire
        sender.c.publish(rc_wire.t_course(), rc_wire.encode_course(body), qos=1, retain=True)
    return {"ok": True, "text": f"course {crs.id} published ({sender.wire}, retained)" + RCTEST_NOTE.format(cid=crs.id.lower()), "course_id": crs.id}


def make_handler(model: RcModel, sender: Optional[JudgeSender] = None, broker: Optional[tuple] = None):
    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _reply(self, code: int, body: bytes, ctype: str = "application/json") -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                ws = os.environ.get("HUB_WS")                   # v0.1.47: the judge map's websocket (the hub's, default 9001)
                body = (PAGE.replace('src="/map.html?judge=1"', f'src="/map.html?judge=1&port={int(ws)}"') if ws and ws.isdigit() else PAGE).encode()
                ctype = "text/html; charset=utf-8"
            elif path == "/themes.css":                     # v0.1.43: the five palettes
                with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "themes.css"), "rb") as fh:
                    body, ctype = fh.read(), "text/css; charset=utf-8"
            elif path == "/state.json":
                snap = dict(model.snapshot(), mode=sender.mode if sender else "monitor", wire=sender.wire if sender else "",
                            has_readiness=bool(model.last_readiness), help=HELP, flavour=_flavour(sender), bar=_ocs_bar())
                snap["incidents"], snap["live"] = incidents(snap.get("sent")), liveness(model)          # v0.1.47 (B8, B9)
                snap["passage"] = passage_buoys(snap, sim_truth())                                  # v0.1.54
                body, ctype = json.dumps(snap, default=str).encode(), "application/json"
            # ---- v0.1.47 (WP10k-a): the launcher's probe, the shared theme, the judge map (served / proxied: same origin) ----
            elif path == "/status.json":
                body = json.dumps({"ok": True, "page": "judge", "broker_host": broker[0] if broker else None, "broker_port": broker[1] if broker else None,
                                   "mode": sender.mode if sender else "monitor", "connected": model.connected}).encode()
                ctype = "application/json"
            elif path == "/prefs":
                sh = PREFS.read_shared()
                body, ctype = json.dumps({"ok": True, "prefs": {"theme": sh["theme"]}, "from": sh["from"]}).encode(), "application/json"
            elif path in ("/map.html", "/live_dot.js"):
                with open(os.path.join(HERE, path[1:]), "rb") as fh:
                    body = fh.read()
                ctype = "text/html; charset=utf-8" if path.endswith(".html") else "application/javascript; charset=utf-8"
            elif path == "/course.json":
                code, b, ct = _ocs("/course.json")
                return self._reply(code, b, ct)
            elif path == "/courses.json":                   # v0.1.53: the catalogue for the course select
                from ..sim import courses as C
                body = json.dumps({"courses": C.catalogue(), "default": C.DEFAULT, "mode": sender.mode if sender else "monitor"}).encode()
                ctype = "application/json"
            else:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                req = {}
            if self.path == "/prefs" and self.headers.get("X-Judge-Web") == "1":           # v0.1.47 (A6): this page's selector
                out = PREFS.write_shared(str(req.get("theme", "")))
                return self._reply(200, json.dumps(out).encode())
            if self.path == "/geofence" and "1" in (self.headers.get("X-Judge-Web"), self.headers.get("X-OCS-Web")):   # v0.1.47 (B8): the map's editor
                code, b, ct = _ocs("/geofence", json.dumps({"points": req.get("points")}).encode())
                return self._reply(code, b, ct)
            if self.path == "/course" and self.headers.get("X-Judge-Web") == "1":          # v0.1.53: the course select
                out = publish_course(sender, str(req.get("course", "")))
                if out["ok"]:
                    model.note(f"page> publish course {out.get('course_id')}: {out['text']}")
                return self._reply(200, json.dumps(out).encode())
            if self.headers.get("X-Judge-Web") != "1" or self.path != "/cmd":
                code, out = 403, {"ok": False, "text": "refused (needs POST /cmd with X-Judge-Web: 1)"}
            elif sender is None:
                code, out = 200, {"ok": False, "text": "this page is read-only"}
            else:
                line = str(req.get("line", ""))
                note = start_note(model, line)
                text = sender.do(line) if line.split() and line.split()[0].lower() not in ("quit", "q", "exit", "clear") else "not from the page"
                ok = not text.startswith(("refused", "unknown", "no ", "course: only", "keepout LAT", "moving LAT", "not from"))
                text += note if ok else ""
                model.note(f"page> {line}: {text}")
                code, out = 200, {"ok": ok, "text": text}
            body = json.dumps(out).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    return H


def start_note(model, line: str) -> str:
    """v0.1.52: a RunStart for a declaration that already has one does nothing on our side (the OCS is R5 already and ignores it,
    alert run_start_ignored) - found live: rc-test had auto-started the run, the page's RunStart then looked like it did nothing.
    Returns the warning to append to the page's answer, or ''."""
    p = line.split()
    if not p or p[0].lower() != "start":
        return ""
    decl = model.declarations[-1] if model.declarations else {}
    nums = [x for x in p[1:] if x.isdigit()]
    seq = int(nums[0]) if nums else decl.get("request_seq")
    rs = model.run_start or {}
    if seq is not None and rs.get("declaration_seq") is not None and int(rs["declaration_seq"]) == int(seq):
        return (f" - note: declaration {seq} already had its RunStart (run {rs.get('run_id')}): the OCS ignores this one; "
                "start the mission with start_task (or mission CODE --auto: in R5 it starts at once)")
    return ""


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):                       # no reverse-DNS getfqdn() (slow on macOS)
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="the broker the judge talks on")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--team", default="SGIT")
    ap.add_argument("--wire", choices=["protobuf", "json"], default="protobuf", help="what --as-judge publishes")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--monitor", action="store_true", help="read-only: the buttons are off (the course network)")
    g.add_argument("--as-judge", action="store_true", help="the buttons publish the RxCommand / RxCourse themselves (rc-test, a bare broker)")
    ap.add_argument("--web-port", type=int, default=8095)
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--open", action="store_true", help="open the page in the browser (macOS `open`)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    model = RcModel(a.team)
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"judge-web-{int(time.time() * 1000) % 1000000}", protocol=mqtt.MQTTv311)

    def on_connect(cl, u, f, rc, p=None):
        model.connected = True
        cl.subscribe(monitor_subscriptions(a.team))         # v0.1.59 (A3): our team's namespace only
    c.on_connect = on_connect
    c.on_disconnect = lambda *x: setattr(model, "connected", False)
    c.on_message = lambda cl, u, msg: model.on_wire(msg.topic, msg.payload)
    c.reconnect_delay_set(1, 5)
    c.connect_async(a.host, a.port, keepalive=15)
    c.loop_start()
    mode = "monitor" if a.monitor else ("judge" if a.as_judge else "fake")
    srv = Server((a.bind, a.web_port), make_handler(model, JudgeSender(c, model, mode, a.team, a.wire), broker=(a.host, a.port)))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    url = f"http://{'127.0.0.1' if a.bind in ('0.0.0.0', '') else a.bind}:{a.web_port}/"
    print(f"[judge_web] the judge's view of {a.host}:{a.port} (team {a.team}, buttons: {mode}) on {url}", flush=True)
    if a.open and sys.platform == "darwin":
        subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        c.loop_stop()
        c.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
