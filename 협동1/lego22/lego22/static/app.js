// =============================================================
// LEGO CAD Builder - Web (Three.js) 버전
// 원본 PyQt5 + pyqtgraph 데스크톱 앱의 로직을 그대로 이식했습니다.
//
// 좌표계 매핑:
//   원본(python)  : x, y = 바닥 그리드 두 축, z = 쌓이는 높이
//   Three.js(y-up): threeX = x, threeY = z(높이), threeZ = y
// =============================================================

const COLORS = {
  red:    0xdd3333,
  blue:   0x2255dd,
  yellow: 0xf2c11d,
  green:  0x2f9e44,
  white:  0xf2f2f2,
  black:  0x1a1a1a,
};

const BLOCK_SIZES = {
  // 2x2 블록만 사용하기로 함 (다른 종류는 비활성화)
  // "1x1": [1, 1],
  // "1x2": [1, 2],
  "2x2": [2, 2],
  // "2x3": [2, 3],
};

// 블록 종류 -> 번호 (요청된 규칙: 1x1=0, 1x2=1, 2x2=2, 2x3=3)
const BLOCK_TYPE_INDEX = {
  "1x1": 0,
  "1x2": 1,
  "2x2": 2,
  "2x3": 3,
};

const GRID_MIN = -10;
const GRID_MAX = 9; // 포함 (python range(-10,10)과 동일한 20x20 칸)
const COORD_LIMIT = 20;
const MAX_Z = 30; // z축(높이) 최대 제한 — 필요하면 이 값만 바꾸면 됨

// ---------------- 상태 ----------------
const state = {
  blocks: new Map(),       // key "x,y,z" -> {x,y,z,sx,sy,type,color,group}
  cellToBlock: new Map(),  // key "x,y,z" -> blockKey
  blockOrder: [],          // 추가된 순서 (마지막 블록 삭제용)
  selectedCell: null,      // {x,y,z} | null
  currentBlockType: "2x2",
  currentRotated: false,
  currentColorName: "red",
  highlightGroup: null,
};

function cellKey(x, y, z) { return `${x},${y},${z}`; }

function getCurrentBlockSize() {
  let [sx, sy] = BLOCK_SIZES[state.currentBlockType];
  if (state.currentRotated) [sx, sy] = [sy, sx];
  return [sx, sy];
}

// ---------------- Three.js 씬 구성 ----------------
const canvas = document.getElementById('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(window.devicePixelRatio || 1);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1114);

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 2000);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.mouseButtons = {
  LEFT: null,                    // 좌클릭은 셀 선택 전용, 카메라 조작 없음
  MIDDLE: THREE.MOUSE.PAN,
  RIGHT: THREE.MOUSE.ROTATE,     // 우클릭 드래그 = 화면 회전
};
controls.enablePan = true;
controls.screenSpacePanning = true;
controls.minDistance = 3;
controls.maxDistance = 120;
controls.target.set(0, 0, 0);

function setInitialCamera() {
  // 원본: distance=25, elevation=25, azimuth=45 (도 단위, z-up 기준)
  const distance = 25;
  const elevation = THREE.MathUtils.degToRad(25);
  const azimuth = THREE.MathUtils.degToRad(45);
  const x = distance * Math.cos(elevation) * Math.cos(azimuth);
  const z = distance * Math.cos(elevation) * Math.sin(azimuth); // python y -> three z
  const y = distance * Math.sin(elevation);                     // python z(높이) -> three y
  camera.position.set(x, y, z);
  camera.lookAt(0, 0, 0);
}
setInitialCamera();

// 조명
scene.add(new THREE.AmbientLight(0xffffff, 0.55));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(15, 25, 10);
scene.add(dirLight);
const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.35);
dirLight2.position.set(-10, 10, -15);
scene.add(dirLight2);

// 축 표시 (원본의 GLAxisItem)
scene.add(new THREE.AxesHelper(5));

// 시각용 그리드 (칸 경계에 맞춰 0.5 오프셋)
const gridHelper = new THREE.GridHelper(20, 20, 0x555a63, 0x33373f);
gridHelper.position.set(-0.5, 0, -0.5);
scene.add(gridHelper);

// 바닥 반투명 평면 (원본의 회색 반투명 셀들에 대응하는 은은한 바닥)
const groundVisual = new THREE.Mesh(
  new THREE.PlaneGeometry(20, 20),
  new THREE.MeshBasicMaterial({ color: 0x25282f, transparent: true, opacity: 0.35, side: THREE.DoubleSide })
);
groundVisual.rotation.x = -Math.PI / 2;
groundVisual.position.set(-0.5, -0.01, -0.5);
scene.add(groundVisual);

// 레이캐스트 전용 무한 바닥 (보이지 않음, z(높이)=0 평면)
const groundPick = new THREE.Mesh(
  new THREE.PlaneGeometry(2000, 2000),
  new THREE.MeshBasicMaterial({ visible: false })
);
groundPick.rotation.x = -Math.PI / 2;
groundPick.position.set(0, 0, 0);
scene.add(groundPick);

function resizeRenderer() {
  const w = canvas.clientWidth || canvas.parentElement.clientWidth;
  const h = canvas.clientHeight || canvas.parentElement.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resizeRenderer);
resizeRenderer();

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// ---------------- 블록/하이라이트 메시 생성 ----------------
function makeBoxGroup(x, y, z, sx, sy, colorHex, opacity, edgeColorHex) {
  const group = new THREE.Group();

  const geo = new THREE.BoxGeometry(sx, 1, sy);
  const mat = new THREE.MeshStandardMaterial({
    color: colorHex,
    transparent: opacity < 1,
    opacity: opacity,
    roughness: 0.55,
    metalness: 0.05,
  });
  const mesh = new THREE.Mesh(geo, mat);

  const edgesGeo = new THREE.EdgesGeometry(geo);
  const edgeMat = new THREE.LineBasicMaterial({ color: edgeColorHex !== undefined ? edgeColorHex : 0x000000 });
  const edges = new THREE.LineSegments(edgesGeo, edgeMat);

  group.add(mesh);
  group.add(edges);

  // anchor(x,y,z)는 최소 모서리이므로 중심으로 변환해서 배치
  group.position.set(x - 0.5 + sx / 2, z + 0.5, y - 0.5 + sy / 2);

  mesh.userData.isBlockFace = true;

  return { group, mesh, edges };
}

function createBlockMesh(x, y, z, sx, sy, colorHex) {
  const { group, mesh } = makeBoxGroup(x, y, z, sx, sy, colorHex, 1.0, 0x0a0a0a);
  scene.add(group);
  return { group, mesh };
}

function createHighlight(x, y, z, sx, sy, valid) {
  const color = valid ? 0xffffff : 0xff3b3b;
  const { group, mesh } = makeBoxGroup(x, y, z, sx, sy, color, 0.4, valid ? 0xffffff : 0xff3b3b);
  mesh.userData.isHighlight = true;
  scene.add(group);
  return group;
}

function clearHighlight() {
  if (state.highlightGroup) {
    scene.remove(state.highlightGroup);
    disposeGroup(state.highlightGroup);
    state.highlightGroup = null;
  }
}

function disposeGroup(group) {
  group.traverse(obj => {
    if (obj.geometry) obj.geometry.dispose();
    if (obj.material) obj.material.dispose();
  });
}

// ---------------- 배치 규칙 (원본과 동일) ----------------
function isOverlapping(x, y, z, sx, sy) {
  for (let i = 0; i < sx; i++) {
    for (let j = 0; j < sy; j++) {
      if (state.cellToBlock.has(cellKey(x + i, y + j, z))) return true;
    }
  }
  return false;
}

function hasSupport(x, y, z, sx, sy) {
  if (z === 0) return true;
  for (let i = 0; i < sx; i++) {
    for (let j = 0; j < sy; j++) {
      if (state.cellToBlock.has(cellKey(x + i, y + j, z - 1))) return true;
    }
  }
  return false;
}

function isPlacementValid(x, y, z, sx, sy) {
  if (isOverlapping(x, y, z, sx, sy)) return false;
  if (!hasSupport(x, y, z, sx, sy)) return false;
  return true;
}

// ---------------- 상태 표시 ----------------
const statusEl = document.getElementById('status');
function setStatus(text, kind) {
  statusEl.textContent = text;
  statusEl.className = kind || '';
}

// ---------------- 블록 추가/삭제 ----------------
function addBlock(x, y, z) {
  const [sx, sy] = getCurrentBlockSize();

  // z축 높이 제한
  if (z < 0 || z > MAX_Z) {
    setStatus(`높이 제한 초과: z는 0~${MAX_Z} 범위여야 합니다 (요청: ${z})`, 'bad');
    return false;
  }

  const footprint = [];
  for (let i = 0; i < sx; i++) {
    for (let j = 0; j < sy; j++) footprint.push([x + i, y + j, z]);
  }

  // x, y 절댓값 10 제한 (블록이 차지하는 모든 칸 기준으로 검사)
  for (const [fx, fy, _fz] of footprint) {
    if (Math.abs(fx) > 10 || Math.abs(fy) > 10) {
      setStatus(`범위 초과: x, y 절댓값은 10을 넘을 수 없습니다 (해당 칸: (${fx}, ${fy}))`, 'bad');
      return false;
    }
  }

  for (const [fx, fy, fz] of footprint) {
    if (state.cellToBlock.has(cellKey(fx, fy, fz))) {
      setStatus(`겹치는 블록 있음: (${fx}, ${fy}, ${fz})`, 'bad');
      return false;
    }
  }

  if (!hasSupport(x, y, z, sx, sy)) {
    setStatus(`받침 없음: (${x}, ${y}, ${z}) 아래에 바닥이나 블록이 필요합니다`, 'bad');
    return false;
  }

  const key = cellKey(x, y, z);
  const colorHex = COLORS[state.currentColorName];
  const { group, mesh } = createBlockMesh(x, y, z, sx, sy, colorHex);
  mesh.userData.blockKey = key;

  state.blocks.set(key, {
    x, y, z, sx, sy,
    type: state.currentBlockType,
    color: state.currentColorName,
    group, mesh,
  });
  state.blockOrder.push(key);

  for (const [fx, fy, fz] of footprint) {
    state.cellToBlock.set(cellKey(fx, fy, fz), key);
  }

  setStatus(`블록 추가: (${x}, ${y}, ${z}) [${state.currentBlockType}, ${sx}x${sy}]`, 'ok');
  return true;
}

function removeBlock(key) {
  const block = state.blocks.get(key);
  if (!block) return;

  scene.remove(block.group);
  disposeGroup(block.group);

  for (let i = 0; i < block.sx; i++) {
    for (let j = 0; j < block.sy; j++) {
      const ck = cellKey(block.x + i, block.y + j, block.z);
      if (state.cellToBlock.get(ck) === key) state.cellToBlock.delete(ck);
    }
  }

  state.blocks.delete(key);
  const idx = state.blockOrder.indexOf(key);
  if (idx !== -1) state.blockOrder.splice(idx, 1);
}

function deleteBlockAtInput() {
  const x = numVal('input-x'), y = numVal('input-y'), z = numVal('input-z');
  const key = state.cellToBlock.get(cellKey(x, y, z));
  if (!key) {
    setStatus(`해당 좌표에 블록 없음: (${x}, ${y}, ${z})`, 'bad');
    return;
  }
  removeBlock(key);
  setStatus(`삭제: ${key}`, 'ok');
}

function deleteLastBlock() {
  if (state.blockOrder.length === 0) {
    setStatus('삭제할 블록 없음', 'bad');
    return;
  }
  const key = state.blockOrder[state.blockOrder.length - 1];
  const b = state.blocks.get(key);
  removeBlock(key);
  if (b) {
    // 삭제한 블록 자리로 예상 블록 이동 -> 엔터로 다시 놓기(redo)도 가능
    updatePreview(b.x, b.y, b.z, '되돌리기');
    setInputs(b.x, b.y, b.z);
  } else {
    setStatus(`삭제: ${key}`, 'ok');
  }
}

// ---------------- 미리보기 / 선택 ----------------
function setHighlight(x, y, z) {
  clearHighlight();
  const [sx, sy] = getCurrentBlockSize();
  const valid = isPlacementValid(x, y, z, sx, sy);
  state.highlightGroup = createHighlight(x, y, z, sx, sy, valid);
  return valid;
}

function updatePreview(x, y, z, prefix) {
  prefix = prefix || '위치 미리보기';
  const valid = setHighlight(x, y, z);
  state.selectedCell = { x, y, z };

  if (valid) {
    setStatus(`${prefix}: (${x}, ${y}, ${z})`, 'ok');
  } else {
    const [sx, sy] = getCurrentBlockSize();
    let reason;
    if (isOverlapping(x, y, z, sx, sy)) reason = '기존 블록과 겹침';
    else if (!hasSupport(x, y, z, sx, sy)) reason = '받침 없음 (아래에 바닥/블록 필요)';
    else reason = '배치 불가';
    setStatus(`${prefix}: (${x}, ${y}, ${z}) (${reason})`, 'bad');
  }
  return valid;
}

function numVal(id) {
  return parseInt(document.getElementById(id).value || '0', 10);
}

function setInputs(x, y, z) {
  document.getElementById('input-x').value = x;
  document.getElementById('input-y').value = y;
  document.getElementById('input-z').value = z;
}

// ---------------- 레이캐스트 클릭 처리 (마인크래프트 방식) ----------------
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();

function snapToCell(value, lo, hi) {
  let idx = Math.floor(value + 0.5);
  if (idx < lo) idx = lo;
  else if (idx > hi) idx = hi;
  return idx;
}

function handleCanvasClick(evt) {
  const rect = canvas.getBoundingClientRect();
  ndc.x = ((evt.clientX - rect.left) / rect.width) * 2 - 1;
  ndc.y = -((evt.clientY - rect.top) / rect.height) * 2 + 1;

  raycaster.setFromCamera(ndc, camera);

  const blockMeshes = [];
  for (const block of state.blocks.values()) blockMeshes.push(block.mesh);

  const candidates = [groundPick, ...blockMeshes];
  const hits = raycaster.intersectObjects(candidates, false);

  if (hits.length === 0) {
    setStatus('클릭 대상 없음 (허공을 클릭함)', 'bad');
    return;
  }

  const hit = hits[0];
  let target = null;

  if (hit.object === groundPick) {
    const gx = Math.round(hit.point.x);
    const gy = Math.round(hit.point.z);
    if (gx >= GRID_MIN && gx <= GRID_MAX && gy >= GRID_MIN && gy <= GRID_MAX) {
      target = { x: gx, y: gy, z: 0 };
    }
  } else {
    const key = hit.object.userData.blockKey;
    const block = state.blocks.get(key);
    if (block) {
      const n = hit.face.normal.clone();
      // 박스는 회전이 없으므로 로컬 노멀 == 월드 노멀
      const { x: bx, y: by, z: bz, sx, sy } = block;

      if (Math.abs(n.x) > 0.5) {
        const tx = n.x > 0 ? bx + sx : bx - 1;
        const ty = snapToCell(hit.point.z, by, by + sy - 1);
        target = { x: tx, y: ty, z: bz };
      } else if (Math.abs(n.z) > 0.5) {
        const ty = n.z > 0 ? by + sy : by - 1;
        const tx = snapToCell(hit.point.x, bx, bx + sx - 1);
        target = { x: tx, y: ty, z: bz };
      } else {
        const tz = n.y > 0 ? bz + 1 : bz - 1;
        const tx = snapToCell(hit.point.x, bx, bx + sx - 1);
        const ty = snapToCell(hit.point.z, by, by + sy - 1);
        target = { x: tx, y: ty, z: tz };
      }
    }
  }

  if (!target) {
    setStatus('클릭 대상 없음 (허공을 클릭함)', 'bad');
    return;
  }

  const { x, y, z } = target;
  if (Math.abs(x) > COORD_LIMIT || Math.abs(y) > COORD_LIMIT || z < 0 || z > MAX_Z) {
    setStatus(`범위 밖 선택: (${x}, ${y}, ${z})`, 'bad');
    return;
  }

  const sel = state.selectedCell;
  if (sel && sel.x === x && sel.y === y && sel.z === z) {
    clearHighlight();
    state.selectedCell = null;
    setStatus(`셀 선택 해제: (${x}, ${y}, ${z})`, '');
  } else {
    updatePreview(x, y, z, '셀 선택');
    setInputs(x, y, z);
  }
}

canvas.addEventListener('pointerdown', (evt) => {
  console.log('[lego-cad] canvas pointerdown, button =', evt.button, 'pointerType =', evt.pointerType);
  // 좌표 입력창 등에 포커스가 남아 있으면 해제 -> 3D 화면 클릭 후 단축키가 바로 동작하도록
  const ae = document.activeElement;
  if (ae && typeof ae.blur === 'function' &&
      (ae.tagName === 'INPUT' || ae.tagName === 'TEXTAREA' || ae.tagName === 'SELECT')) {
    ae.blur();
  }
  if (evt.button !== 0) return; // 좌클릭만 처리 (0=좌, 1=휠, 2=우)
  try {
    handleCanvasClick(evt);
  } catch (e) {
    console.error('[lego-cad] handleCanvasClick 오류:', e);
    setStatus(`클릭 처리 오류: ${e.message}`, 'bad');
  }
});
// 우클릭 컨텍스트 메뉴 방지 (우클릭 드래그는 화면 회전용)
canvas.addEventListener('contextmenu', (evt) => evt.preventDefault());

// ---------------- 사이드 패널 UI ----------------
function buildColorChips() {
  const container = document.getElementById('color-list');
  container.innerHTML = '';
  Object.keys(COLORS).forEach((name, idx) => {
    const chip = document.createElement('button');
    chip.className = 'chip' + (idx === 0 ? ' selected' : '');
    chip.dataset.name = name;
    chip.innerHTML = `<span class="swatch" style="background:#${COLORS[name].toString(16).padStart(6, '0')}"></span>${name}`;
    chip.addEventListener('click', () => {
      state.currentColorName = name;
      container.querySelectorAll('.chip').forEach(c => c.classList.remove('selected'));
      chip.classList.add('selected');
      setStatus(`현재 색상: ${name}`, '');
    });
    container.appendChild(chip);
  });
}

function buildBlockTypeChips() {
  const container = document.getElementById('block-type-list');
  container.innerHTML = '';
  Object.keys(BLOCK_SIZES).forEach((name, idx) => {
    const chip = document.createElement('button');
    chip.className = 'chip' + (idx === 0 ? ' selected' : '');
    chip.dataset.name = name;
    chip.textContent = name;
    chip.addEventListener('click', () => {
      state.currentBlockType = name;
      state.currentRotated = false;
      container.querySelectorAll('.chip').forEach(c => c.classList.remove('selected'));
      chip.classList.add('selected');
      setStatus(`현재 블록 종류: ${name}`, '');
      if (state.selectedCell) {
        const { x, y, z } = state.selectedCell;
        setHighlight(x, y, z);
      }
    });
    container.appendChild(chip);
  });
}

document.getElementById('btn-rotate').addEventListener('click', () => {
  state.currentRotated = !state.currentRotated;
  const [sx, sy] = getCurrentBlockSize();
  setStatus(`블록 회전: ${state.currentBlockType} (${sx}x${sy})`, '');
  if (state.selectedCell) {
    const { x, y, z } = state.selectedCell;
    setHighlight(x, y, z);
  }
});

['input-x', 'input-y', 'input-z'].forEach(id => {
  document.getElementById(id).addEventListener('change', () => {
    const x = numVal('input-x'), y = numVal('input-y'), z = numVal('input-z');
    updatePreview(x, y, z, '위치 미리보기');
  });
});

document.getElementById('btn-add').addEventListener('click', () => {
  const x = numVal('input-x'), y = numVal('input-y'), z = numVal('input-z');
  if (addBlock(x, y, z)) {
    // 블록을 놓으면 예상 블록을 방금 놓은 블록 바로 위(z+1)로 이동
    updatePreview(x, y, z + 1, '예상 블록 위로 이동');
    setInputs(x, y, z + 1);
  }
});

document.getElementById('btn-delete-at').addEventListener('click', deleteBlockAtInput);
document.getElementById('btn-delete-last').addEventListener('click', deleteLastBlock);

const btnClearAll = document.getElementById('btn-clear-all');
if (btnClearAll) btnClearAll.addEventListener('click', async () => {
  const n = state.blockOrder.length;
  if (n === 0) {
    setStatus('삭제할 블록이 없습니다', 'bad');
    return;
  }
  const ok = await showConfirmDialog(`블록 ${n}개를 모두 삭제하고 맵을 초기화할까요?`, '초기화');
  if (!ok) return;
  clearAllBlocks();
  setStatus(`맵 초기화 완료: 블록 ${n}개 삭제`, 'ok');
});

document.getElementById('btn-export').addEventListener('click', async () => {
  // [블록타입 번호, [cx, cy, cz]] 리스트 생성
  // 중심 좌표 공식: cx = x + (sx-1)/2, cy = y + (sy-1)/2, cz = z
  const centers = [];
  for (const b of state.blocks.values()) {
    const typeIndex = BLOCK_TYPE_INDEX[b.type] !== undefined ? BLOCK_TYPE_INDEX[b.type] : -1;
    const cx = b.x + (b.sx - 1) / 2;
    const cy = b.y + (b.sy - 1) / 2;
    const cz = b.z;
    centers.push([typeIndex, [cx, cy, cz]]);
  }

  if (printState.active) return; // 이미 출력 중이면 무시

  try {
    const res = await fetch('/api/publish_centers', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(centers),
    });
    const json = await res.json();
    if (json.ok) {
      setStatus(`발행 완료 (${centers.length}개 블록)`, 'ok');
      startPrintMode(); // 출력(조립) 진행 표시 모드 진입
    } else {
      setStatus(`발행 실패: ${json.error}`, 'bad');
    }
  } catch (e) {
    setStatus(`발행 실패: ${e}`, 'bad');
  }
});

// ---------------- 초기화 ----------------
buildColorChips();
buildBlockTypeChips();
setStatus('현재 색상: red', '');

// ---------------- 저장 / 불러오기 ----------------
// (파일 맨 끝에 있으므로, 이 부분에서 오류가 나도 위의 3D 뷰/기존 기능은 영향 없음)
function serializeBlocks() {
  // 추가된 순서(blockOrder)대로 직렬화 -> 불러올 때 같은 순서로 복원하면
  // 아래층부터 쌓이므로 지지 조건이 자연스럽게 만족된다.
  const out = [];
  for (const key of state.blockOrder) {
    const b = state.blocks.get(key);
    if (!b) continue;
    out.push({ x: b.x, y: b.y, z: b.z, sx: b.sx, sy: b.sy, type: b.type, color: b.color });
  }
  return out;
}

function clearAllBlocks() {
  for (const key of [...state.blockOrder]) removeBlock(key);
  clearHighlight();
  state.selectedCell = null;
}

function restoreBlock(b) {
  // 저장 파일의 블록을 그대로 복원 (현재 선택된 색상/종류와 무관)
  const x = b.x, y = b.y, z = b.z, sx = b.sx, sy = b.sy;
  const key = cellKey(x, y, z);
  if (state.blocks.has(key)) return false;

  // 겹침 방지 (손상된 저장 파일 대비 최소한의 방어)
  for (let i = 0; i < sx; i++) {
    for (let j = 0; j < sy; j++) {
      if (state.cellToBlock.has(cellKey(x + i, y + j, z))) return false;
    }
  }

  const colorName = COLORS[b.color] !== undefined ? b.color : 'red';
  const { group, mesh } = createBlockMesh(x, y, z, sx, sy, COLORS[colorName]);
  mesh.userData.blockKey = key;

  state.blocks.set(key, { x, y, z, sx, sy, type: b.type, color: colorName, group, mesh });
  state.blockOrder.push(key);

  for (let i = 0; i < sx; i++) {
    for (let j = 0; j < sy; j++) {
      state.cellToBlock.set(cellKey(x + i, y + j, z), key);
    }
  }
  return true;
}

const btnSave = document.getElementById('btn-save');
const btnLoad = document.getElementById('btn-load');

// ----- 썸네일 캡처: 현재 3D 화면을 작은 이미지로 -----
function captureThumbnail() {
  try {
    // 배치 미리보기(하이라이트)는 썸네일에서 잠깐 숨김
    const hl = state.highlightGroup;
    if (hl) hl.visible = false;
    renderer.render(scene, camera);          // 캡처 직전에 한 번 그려서 버퍼 보장
    const src = renderer.domElement;
    const w = 240;
    const h = Math.max(1, Math.round(w * src.height / src.width));
    const c = document.createElement('canvas');
    c.width = w; c.height = h;
    c.getContext('2d').drawImage(src, 0, 0, w, h);
    if (hl) hl.visible = true;
    return c.toDataURL('image/png');
  } catch (e) {
    console.warn('[lego-cad] 썸네일 캡처 실패:', e);
    return null;
  }
}

// ----- 페이지 내장 대화상자 (window.prompt/confirm 은 환경에 따라 차단될 수 있음) -----
function makeDialogOverlay() {
  const overlay = document.createElement('div');
  overlay.dataset.dialog = '1'; // 모달 열림 감지용 (단축키 비활성화)
  overlay.style.cssText =
    'position:fixed;inset:0;background:rgba(0,0,0,.55);display:flex;' +
    'align-items:center;justify-content:center;z-index:1100;';
  const box = document.createElement('div');
  box.style.cssText =
    'background:#1b1e24;color:#e8e8e8;border:1px solid #3a3f49;border-radius:10px;' +
    'min-width:300px;max-width:380px;padding:16px;box-shadow:0 12px 40px rgba(0,0,0,.5);' +
    'font-family:inherit;';
  overlay.appendChild(box);
  return { overlay, box };
}

function dialogButton(text, primary) {
  const b = document.createElement('button');
  b.textContent = text;
  b.style.cssText =
    'padding:8px 18px;border:none;border-radius:8px;cursor:pointer;' +
    'font-family:inherit;font-size:14px;margin-left:8px;' +
    (primary ? 'background:#3b6cff;color:#fff;font-weight:600;' : 'background:#343a45;color:#e8e8e8;');
  return b;
}

// 이름 입력 창: 확인 -> 입력값(빈 문자열 가능), 취소 -> null
function showNameDialog() {
  return new Promise((resolve) => {
    const { overlay, box } = makeDialogOverlay();

    const title = document.createElement('div');
    title.textContent = '💾 맵 저장';
    title.style.cssText = 'font-weight:700;margin-bottom:10px;';
    box.appendChild(title);

    const desc = document.createElement('div');
    desc.textContent = '저장할 맵 이름을 입력하세요. 비워두면 map1, map2 ... 자동 이름으로 저장됩니다.';
    desc.style.cssText = 'color:#9aa0aa;font-size:13px;margin-bottom:10px;line-height:1.5;';
    box.appendChild(desc);

    const input = document.createElement('input');
    input.type = 'text';
    input.placeholder = '예: 우리집 성';
    input.style.cssText =
      'width:100%;box-sizing:border-box;padding:9px 10px;border-radius:8px;' +
      'border:1px solid #3a3f49;background:#12141a;color:#e8e8e8;' +
      'font-family:inherit;font-size:14px;outline:none;';
    box.appendChild(input);

    const footer = document.createElement('div');
    footer.style.cssText = 'margin-top:14px;text-align:right;';
    const cancel = dialogButton('취소', false);
    const ok = dialogButton('저장', true);
    footer.appendChild(cancel);
    footer.appendChild(ok);
    box.appendChild(footer);

    function done(val) { overlay.remove(); resolve(val); }
    cancel.addEventListener('click', () => done(null));
    ok.addEventListener('click', () => done(input.value));
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter') done(input.value);
      if (e.key === 'Escape') done(null);
    });
    overlay.addEventListener('click', (e) => { if (e.target === overlay) done(null); });

    document.body.appendChild(overlay);
    input.focus();
  });
}

// 확인 창: 확인 -> true, 취소 -> false
function showConfirmDialog(message, okText) {
  return new Promise((resolve) => {
    const { overlay, box } = makeDialogOverlay();

    const msg = document.createElement('div');
    msg.textContent = message;
    msg.style.cssText = 'font-size:14px;line-height:1.6;white-space:pre-line;';
    box.appendChild(msg);

    const footer = document.createElement('div');
    footer.style.cssText = 'margin-top:14px;text-align:right;';
    const cancel = dialogButton('취소', false);
    const ok = dialogButton(okText || '확인', true);
    footer.appendChild(cancel);
    footer.appendChild(ok);
    box.appendChild(footer);

    function done(val) { overlay.remove(); resolve(val); }
    cancel.addEventListener('click', () => done(false));
    ok.addEventListener('click', () => done(true));
    overlay.addEventListener('click', (e) => { if (e.target === overlay) done(false); });

    document.body.appendChild(overlay);
    ok.focus();
  });
}

async function requestSave(blocks, thumbnail, name, overwrite) {
  const res = await fetch('/api/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ blocks, thumbnail, name, overwrite }),
  });
  return res.json();
}

if (btnSave) btnSave.addEventListener('click', async () => {
  // 맵 이름 입력 (취소하면 저장 안 함, 비워두면 map번호로 자동 저장)
  const input = await showNameDialog();
  if (input === null) {
    setStatus('저장 취소됨', '');
    return;
  }
  const name = input.trim();

  const blocks = serializeBlocks();
  const thumbnail = captureThumbnail();
  try {
    let json = await requestSave(blocks, thumbnail, name || null, false);

    // 같은 이름이 이미 있으면 덮어쓸지 확인
    if (!json.ok && json.error === 'exists') {
      const yes = await showConfirmDialog(`"${json.name}" 이(가) 이미 있습니다.\n덮어쓸까요?`, '덮어쓰기');
      if (!yes) {
        setStatus('저장 취소됨 (같은 이름 존재)', '');
        return;
      }
      json = await requestSave(blocks, thumbnail, name || null, true);
    }

    if (json.ok) {
      setStatus(`저장 완료 (${json.count}개 블록) -> ${json.path}`, 'ok');
      refreshGallery();
    } else {
      setStatus(`저장 실패: ${json.error}`, 'bad');
    }
  } catch (e) {
    setStatus(`저장 실패: ${e}`, 'bad');
  }
});

// ----- 불러오기: 저장 폴더의 파일 목록을 띄워서 선택 (폴더 열기 방식) -----
let loadModal = null;

function closeLoadModal() {
  if (loadModal) {
    loadModal.remove();
    loadModal = null;
  }
}

function fmtTime(unixSec) {
  const d = new Date(unixSec * 1000);
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

async function loadMapByName(name) {
  try {
    const res = await fetch(`/api/load?name=${encodeURIComponent(name)}`);
    const json = await res.json();
    if (!json.ok) {
      setStatus(`불러오기 실패: ${json.error}`, 'bad');
      return;
    }
    clearAllBlocks();
    let restored = 0;
    for (const b of json.blocks) {
      if (restoreBlock(b)) restored++;
    }
    closeLoadModal();
    setStatus(`불러오기 완료: ${name} (${restored}/${json.blocks.length}개 블록)`, 'ok');
  } catch (e) {
    setStatus(`불러오기 실패: ${e}`, 'bad');
  }
}

function showLoadModal(maps) {
  closeLoadModal();

  const overlay = document.createElement('div');
  overlay.dataset.dialog = '1'; // 모달 열림 감지용 (단축키 비활성화)
  overlay.style.cssText =
    'position:fixed;inset:0;background:rgba(0,0,0,.55);display:flex;' +
    'align-items:center;justify-content:center;z-index:1000;';
  overlay.addEventListener('click', (e) => { if (e.target === overlay) closeLoadModal(); });

  const box = document.createElement('div');
  box.style.cssText =
    'background:#1b1e24;color:#e8e8e8;border:1px solid #3a3f49;border-radius:10px;' +
    'min-width:320px;max-width:440px;max-height:70vh;display:flex;flex-direction:column;' +
    'box-shadow:0 12px 40px rgba(0,0,0,.5);font-family:inherit;';

  const title = document.createElement('div');
  title.textContent = '📂 저장된 맵 불러오기';
  title.style.cssText = 'padding:14px 16px;font-weight:700;border-bottom:1px solid #3a3f49;';
  box.appendChild(title);

  const list = document.createElement('div');
  list.style.cssText = 'overflow-y:auto;padding:8px;flex:1;';

  if (maps.length === 0) {
    const empty = document.createElement('div');
    empty.textContent = '저장된 맵이 없습니다. 먼저 💾 저장하기를 눌러주세요.';
    empty.style.cssText = 'padding:18px 10px;color:#9aa0aa;text-align:center;';
    list.appendChild(empty);
  } else {
    for (const m of maps) {
      const item = document.createElement('button');
      const count = (m.blocks === null || m.blocks === undefined) ? '?' : m.blocks;
      const thumbHtml = m.thumb
        ? `<img src="/api/thumb?name=${encodeURIComponent(m.name)}" style="width:64px;height:44px;object-fit:cover;border-radius:5px;border:1px solid #3a3f49;flex:none" />`
        : `<span style="width:64px;height:44px;display:flex;align-items:center;justify-content:center;background:#1a1d23;border:1px solid #3a3f49;border-radius:5px;color:#5c6470;font-size:11px;flex:none">미리보기<br>없음</span>`;
      item.innerHTML =
        thumbHtml +
        `<span style="font-weight:600">${m.name}</span>` +
        `<span style="color:#9aa0aa;font-size:12px;margin-left:auto">블록 ${count}개 · ${fmtTime(m.mtime)}</span>`;
      item.style.cssText =
        'display:flex;gap:10px;align-items:center;width:100%;text-align:left;' +
        'padding:8px 10px;margin:2px 0;background:#232730;color:inherit;' +
        'border:1px solid #343a45;border-radius:8px;cursor:pointer;font-family:inherit;font-size:14px;';
      item.addEventListener('mouseenter', () => item.style.background = '#2c313c');
      item.addEventListener('mouseleave', () => item.style.background = '#232730');
      item.addEventListener('click', () => loadMapByName(m.name));
      list.appendChild(item);
    }
  }
  box.appendChild(list);

  const footer = document.createElement('div');
  footer.style.cssText = 'padding:10px 16px;border-top:1px solid #3a3f49;text-align:right;';
  const cancel = document.createElement('button');
  cancel.textContent = '취소';
  cancel.style.cssText =
    'padding:8px 18px;background:#343a45;color:#e8e8e8;border:none;border-radius:8px;' +
    'cursor:pointer;font-family:inherit;font-size:14px;';
  cancel.addEventListener('click', closeLoadModal);
  footer.appendChild(cancel);
  box.appendChild(footer);

  overlay.appendChild(box);
  document.body.appendChild(overlay);
  loadModal = overlay;
}

if (btnLoad) btnLoad.addEventListener('click', async () => {
  try {
    const res = await fetch('/api/maps');
    const json = await res.json();
    if (!json.ok) {
      setStatus(`목록 조회 실패: ${json.error}`, 'bad');
      return;
    }
    showLoadModal(json.maps);
  } catch (e) {
    setStatus(`목록 조회 실패: ${e}`, 'bad');
  }
});

// ----- 패널 하단 미리보기 갤러리 -----
const galleryEl = document.getElementById('map-gallery');

// ----- 카드 우측 상단 ⋯ 메뉴 -----
let openMenu = null;
function closeCardMenu() {
  if (openMenu) { openMenu.remove(); openMenu = null; }
}
document.addEventListener('pointerdown', (e) => {
  if (openMenu && !openMenu.contains(e.target)) closeCardMenu();
});

async function deleteMapByName(name) {
  const yes = await showConfirmDialog(`${name} 을(를) 삭제할까요?\n삭제한 맵은 되돌릴 수 없습니다.`, '삭제');
  if (!yes) return;
  try {
    const res = await fetch('/api/delete', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ name }),
    });
    const json = await res.json();
    if (json.ok) {
      setStatus(`삭제 완료: ${name}`, 'ok');
      refreshGallery();
    } else {
      setStatus(`삭제 실패: ${json.error}`, 'bad');
    }
  } catch (e) {
    setStatus(`삭제 실패: ${e}`, 'bad');
  }
}

function showCardMenu(anchorBtn, name) {
  closeCardMenu();
  const menu = document.createElement('div');
  menu.style.cssText =
    'position:absolute;top:26px;right:4px;z-index:20;background:#1b1e24;' +
    'border:1px solid #3a3f49;border-radius:8px;box-shadow:0 8px 24px rgba(0,0,0,.5);' +
    'overflow:hidden;min-width:110px;';
  const del = document.createElement('button');
  del.textContent = '🗑 삭제하기';
  del.style.cssText =
    'display:block;width:100%;padding:9px 14px;background:none;border:none;' +
    'color:#ff6b6b;cursor:pointer;font-family:inherit;font-size:13px;text-align:left;';
  del.addEventListener('mouseenter', () => del.style.background = '#2c313c');
  del.addEventListener('mouseleave', () => del.style.background = 'none');
  del.addEventListener('click', (e) => {
    e.stopPropagation();
    closeCardMenu();
    deleteMapByName(name);
  });
  menu.appendChild(del);
  anchorBtn.parentElement.appendChild(menu);
  openMenu = menu;
}

async function refreshGallery() {
  if (!galleryEl) return;
  let maps = [];
  try {
    const res = await fetch('/api/maps');
    const json = await res.json();
    if (json.ok) maps = json.maps;
  } catch (e) {
    return; // 서버 오류 시 조용히 무시 (갤러리는 부가 기능)
  }

  galleryEl.innerHTML = '';
  galleryEl.style.cssText =
    'display:grid;grid-template-columns:repeat(2,1fr);gap:8px;';

  if (maps.length === 0) {
    const empty = document.createElement('div');
    empty.textContent = '저장된 맵이 없습니다';
    empty.style.cssText = 'grid-column:1/-1;color:#5c6470;font-size:12px;padding:6px 2px;';
    galleryEl.appendChild(empty);
    return;
  }

  for (const m of maps) {
    const card = document.createElement('button');
    card.title = `${m.name} 불러오기`;
    card.style.cssText =
      'position:relative;padding:0;border:1px solid #343a45;border-radius:8px;overflow:hidden;' +
      'background:#232730;cursor:pointer;font-family:inherit;color:inherit;text-align:left;';

    const thumbWrap = document.createElement('div');
    thumbWrap.style.cssText =
      'width:100%;aspect-ratio:16/10;background:#1a1d23;display:flex;' +
      'align-items:center;justify-content:center;';
    if (m.thumb) {
      const img = document.createElement('img');
      img.src = `/api/thumb?name=${encodeURIComponent(m.name)}&t=${m.mtime}`;
      img.loading = 'lazy';
      img.style.cssText = 'width:100%;height:100%;object-fit:cover;display:block;';
      thumbWrap.appendChild(img);
    } else {
      const ph = document.createElement('span');
      ph.textContent = '미리보기 없음';
      ph.style.cssText = 'color:#5c6470;font-size:11px;';
      thumbWrap.appendChild(ph);
    }
    card.appendChild(thumbWrap);

    // 우측 상단 ⋯ 메뉴 버튼
    const more = document.createElement('span');
    more.textContent = '⋯';
    more.title = '메뉴';
    more.style.cssText =
      'position:absolute;top:4px;right:4px;width:22px;height:22px;line-height:20px;' +
      'text-align:center;border-radius:6px;background:rgba(15,17,20,.75);color:#cfd3da;' +
      'font-size:15px;cursor:pointer;user-select:none;';
    more.addEventListener('mouseenter', () => more.style.background = 'rgba(50,55,65,.9)');
    more.addEventListener('mouseleave', () => more.style.background = 'rgba(15,17,20,.75)');
    more.addEventListener('pointerdown', (e) => e.stopPropagation());
    more.addEventListener('click', (e) => {
      e.stopPropagation();
      showCardMenu(more, m.name);
    });
    card.appendChild(more);

    const label = document.createElement('div');
    const count = (m.blocks === null || m.blocks === undefined) ? '?' : m.blocks;
    label.innerHTML =
      `<div style="font-weight:600;font-size:12px">${m.name}</div>` +
      `<div style="color:#9aa0aa;font-size:11px">블록 ${count}개</div>`;
    label.style.cssText = 'padding:6px 8px;';
    card.appendChild(label);

    card.addEventListener('mouseenter', () => card.style.borderColor = '#5c88ff');
    card.addEventListener('mouseleave', () => card.style.borderColor = '#343a45');
    card.addEventListener('click', () => loadMapByName(m.name));
    galleryEl.appendChild(card);
  }
}

refreshGallery();

// ---------------- 단축키 (기어 버튼 / 키 바인딩) ----------------
const KEYBIND_STORAGE_KEY = 'lego22_keybinds_v1';

const KEYBIND_ACTIONS = [
  { id: 'undo',      label: '되돌리기 (마지막 블록 삭제)' },
  { id: 'place',     label: '예상 블록 위치에 블록 놓기' },
  { id: 'moveUp',    label: '예상 블록 이동: +Y' },
  { id: 'moveDown',  label: '예상 블록 이동: -Y' },
  { id: 'moveLeft',  label: '예상 블록 이동: -X' },
  { id: 'moveRight', label: '예상 블록 이동: +X' },
  { id: 'moveZUp',   label: '예상 블록 이동: +Z (위)' },
  { id: 'moveZDown', label: '예상 블록 이동: -Z (아래)' },
];

const DEFAULT_KEYBINDS = {
  undo:      { code: 'KeyZ',       ctrl: true,  shift: false, alt: false },
  place:     { code: 'Enter',      ctrl: false, shift: false, alt: false },
  moveUp:    { code: 'ArrowUp',    ctrl: false, shift: false, alt: false },
  moveDown:  { code: 'ArrowDown',  ctrl: false, shift: false, alt: false },
  moveLeft:  { code: 'ArrowLeft',  ctrl: false, shift: false, alt: false },
  moveRight: { code: 'ArrowRight', ctrl: false, shift: false, alt: false },
  moveZUp:   { code: 'KeyW',       ctrl: false, shift: false, alt: false },
  moveZDown: { code: 'KeyS',       ctrl: false, shift: false, alt: false },
};

function loadKeybinds() {
  try {
    const raw = localStorage.getItem(KEYBIND_STORAGE_KEY);
    if (raw) {
      const saved = JSON.parse(raw);
      const out = {};
      for (const a of KEYBIND_ACTIONS) {
        const b = saved[a.id];
        out[a.id] = (b && typeof b.code === 'string')
          ? { code: b.code, ctrl: !!b.ctrl, shift: !!b.shift, alt: !!b.alt }
          : { ...DEFAULT_KEYBINDS[a.id] };
      }
      return out;
    }
  } catch (e) { /* localStorage 사용 불가 환경이면 기본값 사용 */ }
  return JSON.parse(JSON.stringify(DEFAULT_KEYBINDS));
}

function saveKeybinds() {
  try { localStorage.setItem(KEYBIND_STORAGE_KEY, JSON.stringify(keybinds)); } catch (e) { /* 무시 */ }
}

let keybinds = loadKeybinds();
let kbCapturingAction = null; // 지금 키 입력을 기다리는 액션 id

function codeLabel(code) {
  if (code.startsWith('Key')) return code.slice(3);
  if (code.startsWith('Digit')) return code.slice(5);
  if (code.startsWith('Numpad')) return 'Num' + code.slice(6);
  const map = { ArrowUp: '↑', ArrowDown: '↓', ArrowLeft: '←', ArrowRight: '→', Space: 'Space' };
  return map[code] || code;
}

function keybindText(b) {
  const parts = [];
  if (b.ctrl) parts.push('Ctrl');
  if (b.alt) parts.push('Alt');
  if (b.shift) parts.push('Shift');
  parts.push(codeLabel(b.code));
  return parts.join(' + ');
}

function matchKeybind(e, b) {
  return e.code === b.code &&
         e.ctrlKey === !!b.ctrl &&
         e.shiftKey === !!b.shift &&
         e.altKey === !!b.alt;
}

// ----- 단축키 동작 -----
function stackTopZ(x, y) {
  // 현재 블록 footprint가 (x,y)에 놓일 때, 그 아래 겹치는 기존 블록들 중
  // 가장 높은 것 바로 위 높이를 반환 (아무것도 없으면 바닥 0)
  const [sx, sy] = getCurrentBlockSize();
  let top = -1;
  for (const b of state.blocks.values()) {
    const overlapXY = b.x < x + sx && b.x + b.sx > x &&
                      b.y < y + sy && b.y + b.sy > y;
    if (overlapXY && b.z > top) top = b.z;
  }
  return Math.min(MAX_Z, top + 1);
}

function moveGhost(dx, dy, dz) {
  dz = dz || 0;
  // 예상 블록이 없으면 현재 좌표 입력값을 시작점으로 사용
  const sel = state.selectedCell ||
    { x: numVal('input-x'), y: numVal('input-y'), z: numVal('input-z') };
  const nx = Math.max(-10, Math.min(10, sel.x + dx));
  const ny = Math.max(-10, Math.min(10, sel.y + dy));
  let nz;
  if (dz !== 0) {
    // W/S: z 수동 이동은 기존 그대로
    nz = Math.max(0, Math.min(MAX_Z, sel.z + dz));
  } else {
    // 좌우(x,y) 이동: 공중에 뜨지 않도록 그 자리 블록 더미의 맨 위로 자동 스냅
    nz = stackTopZ(nx, ny);
  }
  updatePreview(nx, ny, nz, '예상 블록 이동');
  setInputs(nx, ny, nz);
}

function placeAtGhost() {
  const sel = state.selectedCell;
  if (!sel) {
    setStatus('예상 블록이 없습니다. 칸을 클릭하거나 방향키로 위치를 지정하세요.', 'bad');
    return;
  }
  if (addBlock(sel.x, sel.y, sel.z)) {
    // 놓은 블록 바로 위(z+1)로 예상 블록 이동
    updatePreview(sel.x, sel.y, sel.z + 1, '예상 블록 위로 이동');
    setInputs(sel.x, sel.y, sel.z + 1);
  }
}

// ----- 설정 모달 -----
const keybindOverlay = document.getElementById('keybind-overlay');
const keybindListEl = document.getElementById('keybind-list');
const btnKeybindEl = document.getElementById('btn-keybind');
const kbUiReady = !!(keybindOverlay && keybindListEl && btnKeybindEl);
if (!kbUiReady) {
  console.warn('[lego-cad] 단축키 UI 요소를 찾지 못했습니다. index.html이 예전 버전(캐시)일 수 있습니다. 단축키 설정 창 없이 기본 단축키만 동작합니다.');
}

function renderKeybindList() {
  if (!kbUiReady) return;
  keybindListEl.innerHTML = '';
  for (const a of KEYBIND_ACTIONS) {
    const row = document.createElement('div');
    row.className = 'kb-row';

    const label = document.createElement('label');
    label.textContent = a.label;

    const btn = document.createElement('button');
    btn.className = 'kb-key';
    if (kbCapturingAction === a.id) {
      btn.classList.add('capturing');
      btn.textContent = '키 입력 대기중...';
    } else {
      btn.textContent = keybindText(keybinds[a.id]);
    }
    btn.addEventListener('click', () => {
      kbCapturingAction = (kbCapturingAction === a.id) ? null : a.id;
      renderKeybindList();
    });

    row.appendChild(label);
    row.appendChild(btn);
    keybindListEl.appendChild(row);
  }
}

if (kbUiReady) {
  btnKeybindEl.addEventListener('click', () => {
    kbCapturingAction = null;
    renderKeybindList();
    keybindOverlay.hidden = false;
  });
  const kbClose = document.getElementById('kb-close');
  const kbReset = document.getElementById('kb-reset');
  if (kbClose) kbClose.addEventListener('click', () => {
    kbCapturingAction = null;
    keybindOverlay.hidden = true;
  });
  if (kbReset) kbReset.addEventListener('click', () => {
    keybinds = JSON.parse(JSON.stringify(DEFAULT_KEYBINDS));
    saveKeybinds();
    kbCapturingAction = null;
    renderKeybindList();
  });
  keybindOverlay.addEventListener('click', (e) => {
    if (e.target === keybindOverlay) {
      kbCapturingAction = null;
      keybindOverlay.hidden = true;
    }
  });
}

// ----- 전역 keydown 처리 -----
const MODIFIER_CODES = new Set([
  'ControlLeft', 'ControlRight', 'ShiftLeft', 'ShiftRight',
  'AltLeft', 'AltRight', 'MetaLeft', 'MetaRight',
]);

document.addEventListener('keydown', (e) => {
  try {
  // 1) 단축키 변경(캡처) 모드
  if (kbCapturingAction) {
    e.preventDefault();
    e.stopPropagation();
    if (e.key === 'Escape') { // 변경 취소
      kbCapturingAction = null;
      renderKeybindList();
      return;
    }
    if (MODIFIER_CODES.has(e.code)) return; // 수식키 단독 입력은 대기 유지

    const newBind = { code: e.code, ctrl: e.ctrlKey, shift: e.shiftKey, alt: e.altKey };
    // 다른 액션에 같은 키가 이미 있으면 그쪽을 기본값으로 되돌려 충돌 방지
    for (const a of KEYBIND_ACTIONS) {
      if (a.id !== kbCapturingAction && matchKeybind(
            { code: newBind.code, ctrlKey: newBind.ctrl, shiftKey: newBind.shift, altKey: newBind.alt },
            keybinds[a.id])) {
        keybinds[a.id] = { ...DEFAULT_KEYBINDS[a.id] };
      }
    }
    keybinds[kbCapturingAction] = newBind;
    kbCapturingAction = null;
    saveKeybinds();
    renderKeybindList();
    return;
  }

  // 2) 단축키 설정 모달이 열려 있으면 ESC로 닫기만 허용
  if (keybindOverlay && !keybindOverlay.hidden) {
    if (e.key === 'Escape') {
      kbCapturingAction = null;
      keybindOverlay.hidden = true;
    }
    return;
  }

  // 3) 입력창에 포커스가 있거나 저장/불러오기 모달이 열려 있으면 단축키 무시
  const tag = (document.activeElement && document.activeElement.tagName) || '';
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
  if (document.querySelector('[data-dialog]')) return;

  // 4) 단축키 실행
  if (matchKeybind(e, keybinds.undo))           { e.preventDefault(); deleteLastBlock(); }
  else if (matchKeybind(e, keybinds.place))     { e.preventDefault(); placeAtGhost(); }
  else if (matchKeybind(e, keybinds.moveUp))    { e.preventDefault(); moveGhost(0, +1); }
  else if (matchKeybind(e, keybinds.moveDown))  { e.preventDefault(); moveGhost(0, -1); }
  else if (matchKeybind(e, keybinds.moveLeft))  { e.preventDefault(); moveGhost(-1, 0); }
  else if (matchKeybind(e, keybinds.moveRight)) { e.preventDefault(); moveGhost(+1, 0); }
  else if (matchKeybind(e, keybinds.moveZUp))   { e.preventDefault(); moveGhost(0, 0, +1); }
  else if (matchKeybind(e, keybinds.moveZDown)) { e.preventDefault(); moveGhost(0, 0, -1); }
  } catch (err) {
    console.error('[lego-cad] 단축키 처리 오류:', err);
  }
});

// ---------------- 출력(조립) 진행 상황 실황 표시 ----------------
// 로봇이 /current_block_status 로 "지금 이 블록 조립 시작"을 발행하면
// 서버가 버퍼링하고, 웹이 /api/progress 를 폴링해서 3D로 중계한다.
const PRINT_POLL_MS = 500;             // 진행 상황 폴링 주기
const PRINT_BLINK_MS = 400;            // 조립 중 블록 깜빡임 주기
const PRINT_LAST_FINISH_MS = 15000;    // 마지막 블록 수신 후 완료 처리까지 대기
const PRINT_IDLE_TIMEOUT_MS = 180000;  // 이 시간 동안 소식이 없으면 강제 종료 (안전장치)

const printState = {
  active: false,
  total: 0,          // 이번 출력에서 로봇이 새로 조립할 블록 수
  revealed: 0,       // 지금까지 진행 수신한 블록 수
  lastSeq: 0,        // 마지막으로 처리한 서버 seq
  blinkKey: null,    // 지금 깜빡이는 중(조립 중)인 블록 키
  pollTimer: null, blinkTimer: null, idleTimer: null, finishTimer: null,
};
// 이전 출력에서 이미 조립 완료된 블록 키 (로봇 쪽 placed_blocks와 대응)
const printedKeys = new Set();
const btnExport = document.getElementById('btn-export');

function findBlockByAnchor(ax, ay, az) {
  for (const [key, b] of state.blocks) {
    if (Math.abs(b.x - ax) < 1e-6 && Math.abs(b.y - ay) < 1e-6 && Math.abs(b.z - az) < 1e-6) {
      return key;
    }
  }
  return null;
}

async function startPrintMode() {
  // 이번에 로봇이 새로 조립할 블록 = 아직 출력된 적 없는 블록
  const pendingKeys = [...state.blocks.keys()].filter((k) => !printedKeys.has(k));
  if (pendingKeys.length === 0) {
    setStatus('새로 조립할 블록이 없습니다 (모두 이미 출력됨)', 'bad');
    return;
  }

  printState.active = true;
  printState.total = pendingKeys.length;
  printState.revealed = 0;
  printState.blinkKey = null;

  if (btnExport) {
    btnExport.disabled = true;
    btnExport.textContent = '출력중...';
  }
  clearHighlight();
  state.selectedCell = null;

  // 아직 조립 안 된 블록만 화면에서 숨김 (이미 출력된 블록은 실물이 있으므로 유지)
  for (const k of pendingKeys) {
    const b = state.blocks.get(k);
    if (b && b.group) b.group.visible = false;
  }

  // 서버의 현재 seq에 기준점을 맞춰 이전 출력의 진행 메시지를 무시
  try {
    const res = await fetch('/api/progress?after=999999999');
    const j = await res.json();
    printState.lastSeq = (j && j.latest) || 0;
  } catch (e) {
    printState.lastSeq = 0;
  }

  setStatus(`출력중... 로봇 조립 대기 (0/${printState.total})`, 'ok');

  printState.blinkTimer = setInterval(() => {
    if (!printState.blinkKey) return;
    const b = state.blocks.get(printState.blinkKey);
    if (b && b.group) b.group.visible = !b.group.visible;
  }, PRINT_BLINK_MS);

  printState.pollTimer = setInterval(pollPrintProgress, PRINT_POLL_MS);
  armPrintIdleTimer();
}

function armPrintIdleTimer() {
  if (printState.idleTimer) clearTimeout(printState.idleTimer);
  printState.idleTimer = setTimeout(() => {
    finishPrintMode('로봇 응답이 오래 없어 출력 표시를 종료합니다', 'bad');
  }, PRINT_IDLE_TIMEOUT_MS);
}

async function pollPrintProgress() {
  if (!printState.active) return;
  let j;
  try {
    const res = await fetch(`/api/progress?after=${printState.lastSeq}`);
    j = await res.json();
  } catch (e) {
    return; // 일시적 네트워크 오류는 다음 폴링에서 재시도
  }
  if (!j || !j.ok || !Array.isArray(j.items)) return;
  for (const item of j.items) {
    printState.lastSeq = item.seq;
    handlePrintProgressBlock(item.block);
    armPrintIdleTimer();
  }
}

function handlePrintProgressBlock(block) {
  // block: [타입, [gx, gy, gz]] — 로봇의 -0.5 변환 좌표는 웹 앵커 좌표와 동일
  if (!printState.active || !Array.isArray(block) || !Array.isArray(block[1])) return;
  const [gx, gy, gz] = block[1];

  // 이전에 깜빡이던(조립하던) 블록은 실색 고정
  if (printState.blinkKey) {
    const prev = state.blocks.get(printState.blinkKey);
    if (prev && prev.group) prev.group.visible = true;
    printState.blinkKey = null;
  }

  const key = findBlockByAnchor(gx, gy, gz);
  if (!key) {
    setStatus(`진행 수신: 화면에서 일치하는 블록을 찾지 못함 (${gx}, ${gy}, ${gz})`, 'bad');
    return;
  }

  printedKeys.add(key);
  printState.revealed += 1;
  printState.blinkKey = key;
  const b = state.blocks.get(key);
  if (b && b.group) b.group.visible = true; // 깜빡임 시작 (blinkTimer가 토글)

  setStatus(`출력중... ${printState.revealed}/${printState.total} — 조립 중: (${gx}, ${gy}, ${gz})`, 'ok');

  // 마지막 블록 수신 -> 일정 시간 깜빡인 뒤 완료 처리
  if (printState.revealed >= printState.total) {
    if (printState.finishTimer) clearTimeout(printState.finishTimer);
    printState.finishTimer = setTimeout(() => {
      finishPrintMode(`출력 완료! 블록 ${printState.total}개 조립됨`, 'ok');
    }, PRINT_LAST_FINISH_MS);
  }
}

function finishPrintMode(message, kind) {
  printState.active = false;
  if (printState.pollTimer) clearInterval(printState.pollTimer);
  if (printState.blinkTimer) clearInterval(printState.blinkTimer);
  if (printState.idleTimer) clearTimeout(printState.idleTimer);
  if (printState.finishTimer) clearTimeout(printState.finishTimer);
  printState.pollTimer = printState.blinkTimer = printState.idleTimer = printState.finishTimer = null;
  printState.blinkKey = null;

  // 모든 블록 실색 복구
  for (const b of state.blocks.values()) {
    if (b && b.group) b.group.visible = true;
  }

  if (btnExport) {
    btnExport.disabled = false;
    btnExport.textContent = '출력하기';
  }
  setStatus(message, kind);
}
