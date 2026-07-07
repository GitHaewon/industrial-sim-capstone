"""Browser UI for the factory monitoring dashboard."""

DASHBOARD_HTML = r"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Smart Factory Control</title>
  <style>
    :root {
      --bg:#07111e; --panel:#101d2d; --line:#26384e;
      --text:#eef6ff; --muted:#8ca2ba; --cyan:#26d7e8;
      --orange:#ffad32; --green:#39d98a; --red:#ff6577;
      --yellow:#ffd166; --blue:#4ba3ff;
    }
    * { box-sizing:border-box }
    body {
      margin:0; color:var(--text); background:
      radial-gradient(circle at 78% -10%,#183d55 0,transparent 38%),
      linear-gradient(180deg,#07111e,#050912 78%);
      font:15px/1.45 "Noto Sans KR","DejaVu Sans",sans-serif;
    }
    main { max-width:1340px; margin:auto; padding:28px }
    header { display:flex; align-items:end; justify-content:space-between;
      gap:20px; margin-bottom:22px }
    h1 { margin:0; letter-spacing:.04em; font-size:27px }
    .eyebrow { color:var(--cyan); font-size:12px; letter-spacing:.2em;
      font-weight:800 }
    #clock { color:var(--muted); font-variant-numeric:tabular-nums }
    .grid { display:grid; grid-template-columns:repeat(12,1fr); gap:14px }
    .card { background:linear-gradient(145deg,#122338ee,#0d1928ee);
      border:1px solid var(--line); border-radius:13px; padding:18px;
      box-shadow:0 14px 34px #0005 }
    .state { grid-column:span 4 } .andon { grid-column:span 3 }
    .progress-card { grid-column:span 5 }
    .metric { grid-column:span 3 } .vision { grid-column:span 3 }
    .boxes { grid-column:span 6 } .docks { grid-column:span 6 }
    .events { grid-column:span 5 } .history { grid-column:span 7 }
    .process { grid-column:span 12 }
    .label { color:var(--muted); font-size:12px; letter-spacing:.08em;
      text-transform:uppercase }
    .value { font-size:34px; font-weight:800; margin-top:6px }
    .state-value { color:var(--cyan); font-size:35px }
    .sub { color:var(--muted); margin-top:5px }
    .progress { height:13px; background:#07111e; border-radius:20px;
      overflow:hidden; margin:16px 0 8px }
    .progress > i { display:block; height:100%; width:0;
      background:linear-gradient(90deg,var(--cyan),var(--green));
      box-shadow:0 0 15px var(--cyan); transition:width .4s }
    .andon-wrap { display:flex; gap:16px; align-items:center; margin-top:12px }
    .tower { width:54px; padding:7px; border:1px solid #33465f;
      border-radius:28px; background:#07111e; box-shadow:inset 0 0 12px #000 }
    .lamp { width:38px; height:38px; border-radius:50%; margin:5px auto;
      opacity:.2; background:#526070 }
    .lamp.red { background:var(--red); color:var(--red) }
    .lamp.yellow { background:var(--yellow); color:var(--yellow) }
    .lamp.green { background:var(--green); color:var(--green) }
    .lamp.on { opacity:1; box-shadow:0 0 22px currentColor }
    .andon-text { font-size:25px; font-weight:850 }
    .andon-level { color:var(--muted) }
    .classes { display:flex; gap:10px; margin-top:13px }
    .class-pill { flex:1; padding:10px; border-radius:8px;
      background:#091522; border:1px solid var(--line); text-align:center }
    .class-pill b { font-size:22px; display:block }
    .class-a b { color:#ff6474 }.class-b b { color:#42df83 }
    .class-c b { color:#4ba3ff }
    .box-row,.dock-row { display:grid; grid-template-columns:repeat(3,1fr);
      gap:11px; margin-top:12px }
    .box,.dock { padding:14px; background:#091522; border-radius:9px;
      border-left:5px solid var(--line); min-height:82px }
    .box.a,.dock.a { border-color:#e44551 }.box.b,.dock.b { border-color:#35bd69 }
    .box.c,.dock.c { border-color:#377de2 }
    .box strong,.dock strong { font-size:19px }.box span,.dock span {
      display:block; color:var(--muted); margin-top:5px }
    .dock small { color:#d8e7ff }
    .events-list { list-style:none; padding:0; margin:12px 0 0;
      display:flex; flex-direction:column; gap:7px; max-height:270px;
      overflow:hidden }
    .events-list li { display:grid; grid-template-columns:46px 1fr; gap:9px;
      padding:8px 9px; border-radius:8px; background:#081522;
      border-left:4px solid var(--cyan) }
    .events-list li.success { border-color:var(--green) }
    .events-list li.warning { border-color:var(--yellow) }
    .events-list li.error { border-color:var(--red) }
    .events-list time { color:var(--muted); font-variant-numeric:tabular-nums }
    table { width:100%; border-collapse:collapse; margin-top:12px }
    th,td { padding:9px 7px; border-bottom:1px solid #26384e;
      text-align:left; vertical-align:top }
    th { color:var(--muted); font-size:12px; letter-spacing:.05em }
    td { color:#e8f2ff } td.state-cell { color:var(--cyan); font-weight:700 }
    .steps { display:grid; grid-template-columns:repeat(7,1fr);
      gap:8px; margin-top:14px }
    .step { color:#60778e; border:1px solid #26384e; border-radius:8px;
      padding:10px 5px; text-align:center; font-size:12px }
    .step.done { color:#dffff2; border-color:#2dbb76; background:#123527 }
    .step.active { color:#fff; border-color:var(--cyan);
      background:#123449; box-shadow:0 0 14px #26d7e833 }
    .status-dot { display:inline-block; width:9px; height:9px;
      border-radius:50%; background:var(--green); box-shadow:0 0 10px var(--green);
      margin-right:8px }
    @media(max-width:900px) {
      .state,.andon,.progress-card,.metric,.vision,.boxes,.docks,.events,
      .history,.process {grid-column:span 12}
      .steps {grid-template-columns:repeat(2,1fr)}
      .box-row,.dock-row,.classes{grid-template-columns:1fr;display:grid}
    }
  </style>
</head>
<body><main>
  <header>
    <div><div class="eyebrow">INDUSTRIAL SIM CAPSTONE</div>
      <h1>스마트 팩토리 공정 관제</h1></div>
    <div id="clock"><span class="status-dot"></span>ROS 2 LIVE</div>
  </header>
  <section class="grid">
    <article class="card state"><div class="label">현재 공정 상태</div>
      <div class="value state-value" id="factory">연결 중</div>
      <div class="sub" id="arm">로봇팔 상태 수신 대기</div></article>
    <article class="card andon"><div class="label">Andon 타워</div>
      <div class="andon-wrap">
        <div class="tower" id="andon-light">
          <i class="lamp red" id="andon-red"></i>
          <i class="lamp yellow" id="andon-yellow"></i>
          <i class="lamp green" id="andon-green"></i>
        </div>
        <div><div class="andon-text" id="andon-text">대기</div>
          <div class="andon-level" id="andon-level">idle</div></div>
      </div></article>
    <article class="card progress-card"><div class="label">배치 처리율</div>
      <div class="value"><span id="count">0</span> / <span id="target">3</span></div>
      <div class="progress"><i id="bar"></i></div>
      <div class="sub"><span id="cycle">1주기</span> ·
        <span id="order">투입 순서 수신 대기</span></div></article>
    <article class="card metric"><div class="label">성공</div>
      <div class="value" style="color:var(--green)" id="success">0</div></article>
    <article class="card metric"><div class="label">실패</div>
      <div class="value" style="color:var(--red)" id="failure">0</div></article>
    <article class="card metric"><div class="label">실행 시간</div>
      <div class="value" id="elapsed">0s</div></article>
    <article class="card vision"><div class="label">최근 비전 인식</div>
      <div class="value"><span id="vclass">-</span>
        <small style="font-size:16px;color:var(--muted)" id="confidence">0%</small></div>
      <div class="sub" id="vision-status">카메라 대기</div></article>
    <article class="card boxes"><div class="label">클래스별 처리량 / 박스 상태</div>
      <div class="classes">
        <div class="class-pill class-a">A · RED<b id="class-a">0</b></div>
        <div class="class-pill class-b">B · GREEN<b id="class-b">0</b></div>
        <div class="class-pill class-c">C · BLUE<b id="class-c">0</b></div>
      </div>
      <div class="box-row">
        <div class="box a"><strong>BOX A</strong><span id="box-a">적재 대기</span></div>
        <div class="box b"><strong>BOX B</strong><span id="box-b">적재 대기</span></div>
        <div class="box c"><strong>BOX C</strong><span id="box-c">적재 대기</span></div>
      </div></article>
    <article class="card docks"><div class="label">출하 도크 / 도착 표시</div>
      <div class="dock-row">
        <div class="dock a"><strong>DOCK A</strong><small>왼쪽 문</small>
          <span id="dock-a">도크 A 대기</span></div>
        <div class="dock b"><strong>DOCK B</strong><small>중앙 문</small>
          <span id="dock-b">도크 B 대기</span></div>
        <div class="dock c"><strong>DOCK C</strong><small>오른쪽 문</small>
          <span id="dock-c">도크 C 대기</span></div>
      </div></article>
    <article class="card events"><div class="label">이벤트 로그</div>
      <ul class="events-list" id="events"></ul></article>
    <article class="card history"><div class="label">작업 이력 테이블</div>
      <table><thead><tr><th>작업 ID</th><th>품목</th><th>박스</th>
        <th>도착지</th><th>상태</th></tr></thead>
        <tbody id="history"></tbody></table></article>
    <article class="card process"><div class="label">공정 흐름</div>
      <div class="steps" id="steps"></div></article>
  </section>
</main>
<script>
const flow=['IDLE','CONVEYOR_RUNNING','ITEM_READY','PICKING','BOX_READY',
            'AMR_MOVING','DELIVERED'];
const labels=['대기','컨베이어','도착 감지','픽앤플레이스','적재 완료','Nav2 운송','배송 완료'];
function text(id,value){document.getElementById(id).textContent=value}
function escapeHtml(value){
  return String(value ?? '').replace(/[&<>"']/g,c=>({
    '&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'
  }[c]));
}
function setAndon(level,state){
  text('andon-text',state || '대기'); text('andon-level',level || 'idle');
  ['red','yellow','green'].forEach(c=>document.getElementById(`andon-${c}`).classList.remove('on'));
  const lamp = level === 'error' ? 'red' :
    level === 'warning' ? 'yellow' :
    (level === 'running' || level === 'done') ? 'green' : '';
  if(lamp) document.getElementById(`andon-${lamp}`).classList.add('on');
}
function renderEvents(events){
  document.getElementById('events').innerHTML=(events||[]).map(e=>
    `<li class="${escapeHtml(e.level)}"><time>${escapeHtml(e.time)}</time>`+
    `<span>${escapeHtml(e.text)}</span></li>`).join('') ||
    '<li><time>--:--</time><span>이벤트 대기</span></li>';
}
function renderHistory(jobs){
  document.getElementById('history').innerHTML=(jobs||[]).map(job=>
    `<tr><td>${escapeHtml(job.work_id)}</td><td>${escapeHtml(job.class)} · ${escapeHtml(job.item)}</td>`+
    `<td>${escapeHtml(job.box)}</td><td>${escapeHtml(job.destination)}</td>`+
    `<td class="state-cell">${escapeHtml(job.state)}</td></tr>`).join('') ||
    '<tr><td colspan="5">작업 이력 수신 대기</td></tr>';
}
function render(s){
  text('factory',s.factory_state); text('arm',`로봇팔 · ${s.arm_state}`);
  setAndon(s.andon_level,s.andon_state);
  text('count',s.item_count); text('target',s.target_count);
  text('cycle',`${s.cycle_number}주기`);
  document.getElementById('bar').style.width=`${Math.min(100,100*s.item_count/s.target_count)}%`;
  text('order',s.arrival_order.length?`투입 순서 ${s.arrival_order.join(' → ')}`:'투입 순서 수신 대기');
  text('success',s.success_count); text('failure',s.failure_count);
  text('elapsed',`${s.elapsed_seconds}s`); text('vclass',s.vision_class||'-');
  text('confidence',`${Math.round(s.vision_confidence*100)}%`);
  text('vision-status',s.vision_status);
  ['A','B','C'].forEach(c=>{
    text(`class-${c.toLowerCase()}`,s.class_counts[c]);
    text(`box-${c.toLowerCase()}`,s.box_states[c]);
    text(`dock-${c.toLowerCase()}`,s.dock_states[c]);
  });
  renderEvents(s.events); renderHistory(s.jobs);
  const index=flow.indexOf(s.factory_state);
  document.getElementById('steps').innerHTML=flow.map((v,i)=>
    `<div class="step ${i<index?'done':i===index?'active':''}">${labels[i]}</div>`).join('');
  text('clock',new Date().toLocaleTimeString('ko-KR'));
}
async function update(){
  try{render(await (await fetch('/api/status',{cache:'no-store'})).json())}
  catch(e){text('factory','연결 끊김')}
}
setInterval(update,500); update();
</script></body></html>"""
