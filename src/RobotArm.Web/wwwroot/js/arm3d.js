import * as THREE from '../lib/three/three.module.min.js';
import { OrbitControls } from '../lib/three/OrbitControls.js';

const COLOR_NORMAL = 0x1d4ed8;
const COLOR_AMBER = 0xf59e0b;
const COLOR_RED = 0xdc2626;

const DEFAULT_CAM_POS = new THREE.Vector3(0.46, -0.48, 0.36);
const DEFAULT_CAM_TARGET = new THREE.Vector3(0.08, 0.0, 0.14);

let state = null;

function createCrosshairGroup(colorHex) {
    const group = new THREE.Group();
    const sphereMat = new THREE.MeshStandardMaterial({
        color: colorHex,
        roughness: 0.35,
        metalness: 0.1,
        transparent: true,
        opacity: 0.85
    });
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(0.009, 16, 16), sphereMat);
    group.add(sphere);

    const ringGeo = new THREE.RingGeometry(0.013, 0.016, 24);
    const ringMat = new THREE.MeshBasicMaterial({ color: colorHex, side: THREE.DoubleSide });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    group.add(ring);

    const lineMat = new THREE.LineBasicMaterial({ color: colorHex });
    const pts = [
        new THREE.Vector3(-0.022, 0, 0), new THREE.Vector3(0.022, 0, 0),
        new THREE.Vector3(0, -0.022, 0), new THREE.Vector3(0, 0.022, 0),
        new THREE.Vector3(0, 0, -0.022), new THREE.Vector3(0, 0, 0.022)
    ];
    const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
    const lines = new THREE.LineSegments(lineGeo, lineMat);
    group.add(lines);

    return group;
}

function initScene(canvasId, geom) {
    disposeScene();

    const canvas = document.getElementById(canvasId);
    if (!canvas) {
        return false;
    }

    const d1 = geom[0];  // 0.0595 m
    const a2x = geom[1]; // 0.024 m
    const a2z = geom[2]; // 0.128 m
    const l3 = geom[3];  // 0.124 m
    const l4 = geom[4];  // 0.126 m

    const width = canvas.clientWidth || 640;
    const height = canvas.clientHeight || 360;

    const renderer = new THREE.WebGLRenderer({
        canvas,
        antialias: true,
        preserveDrawingBuffer: true
    });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.setSize(width, height, false);
    renderer.setClearColor(0xf8fafc, 1);

    const scene = new THREE.Scene();
    scene.up.set(0, 0, 1);

    const camera = new THREE.PerspectiveCamera(42, width / height, 0.01, 5.0);
    camera.up.set(0, 0, 1);
    camera.position.copy(DEFAULT_CAM_POS);

    const controls = new OrbitControls(camera, canvas);
    controls.target.copy(DEFAULT_CAM_TARGET);
    controls.enableDamping = false;
    controls.minDistance = 0.15;
    controls.maxDistance = 2.0;

    // Lights
    const ambient = new THREE.AmbientLight(0xffffff, 0.85);
    scene.add(ambient);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.9);
    dirLight1.position.set(0.6, -0.7, 0.9);
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0xe2e8f0, 0.45);
    dirLight2.position.set(-0.6, 0.6, 0.5);
    scene.add(dirLight2);

    // Floor plane (z = 0) & 50 mm grid (800 mm / 16 divisions = 50 mm per cell)
    const floorGeo = new THREE.PlaneGeometry(0.8, 0.8);
    const floorMat = new THREE.MeshStandardMaterial({
        color: 0xf1f5f9,
        roughness: 0.9,
        metalness: 0.0,
        side: THREE.DoubleSide
    });
    const floorMesh = new THREE.Mesh(floorGeo, floorMat);
    floorMesh.position.set(0, 0, -0.0005);
    scene.add(floorMesh);

    const grid = new THREE.GridHelper(0.8, 16, 0x64748b, 0xcbd5e1);
    grid.rotation.x = Math.PI / 2; // Lie in X-Y plane (Z up)
    scene.add(grid);

    // Coordinate axes at origin (X: red, Y: green, Z: blue; 100 mm)
    const axes = new THREE.AxesHelper(0.10);
    axes.position.set(0, 0, 0.0005);
    scene.add(axes);

    // Base pedestal sitting on z = 0
    const basePedestalMat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.5, metalness: 0.25 });
    const basePlate = new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.040, 0.012, 24), basePedestalMat);
    basePlate.rotation.x = Math.PI / 2;
    basePlate.position.set(0, 0, 0.006);
    scene.add(basePlate);

    // Shared link materials
    const link1Mat = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.45, metalness: 0.2 });
    const link2Mat = new THREE.MeshStandardMaterial({ color: 0x1e40af, roughness: 0.4, metalness: 0.15 });
    const link3Mat = new THREE.MeshStandardMaterial({ color: 0x2563eb, roughness: 0.4, metalness: 0.15 });
    const link4Mat = new THREE.MeshStandardMaterial({ color: 0x3b82f6, roughness: 0.4, metalness: 0.15 });
    const gripperMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.4, metalness: 0.3 });

    // Per-joint materials (updated in-place each frame for normal / amber / red)
    const jointMaterials = [
        new THREE.MeshStandardMaterial({ color: COLOR_NORMAL, roughness: 0.35, metalness: 0.2 }),
        new THREE.MeshStandardMaterial({ color: COLOR_NORMAL, roughness: 0.35, metalness: 0.2 }),
        new THREE.MeshStandardMaterial({ color: COLOR_NORMAL, roughness: 0.35, metalness: 0.2 }),
        new THREE.MeshStandardMaterial({ color: COLOR_NORMAL, roughness: 0.35, metalness: 0.2 })
    ];
    const jointColorCodes = [0, 0, 0, 0];

    // 4 rigid body frame groups driven directly by ForwardKinematics.ComputeFrames
    const frameGroups = [];
    for (let i = 0; i < 4; i++) {
        const g = new THREE.Group();
        g.matrixAutoUpdate = false;
        scene.add(g);
        frameGroups.push(g);
    }

    const jointSphereGeo = new THREE.SphereGeometry(0.017, 20, 20);

    // --- Body 1 (frameGroups[0]): Origin at (0,0,0), rotated by Rz(q1). Goes up to z = d1 ---
    const j1Sphere = new THREE.Mesh(jointSphereGeo, jointMaterials[0]);
    j1Sphere.position.set(0, 0, 0.014);
    frameGroups[0].add(j1Sphere);

    const link1Cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.016, d1, 20), link1Mat);
    link1Cyl.rotation.x = Math.PI / 2;
    link1Cyl.position.set(0, 0, d1 * 0.5);
    frameGroups[0].add(link1Cyl);

    // --- Body 2 (frameGroups[1]): Origin at Joint 2, rotated by Ry(q2). L-shaped link (0,0,a2z) -> (a2x,0,a2z) ---
    const j2Sphere = new THREE.Mesh(jointSphereGeo, jointMaterials[1]);
    frameGroups[1].add(j2Sphere);

    const link2Vert = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.013, a2z, 18), link2Mat);
    link2Vert.rotation.x = Math.PI / 2;
    link2Vert.position.set(0, 0, a2z * 0.5);
    frameGroups[1].add(link2Vert);

    const elbowCorner = new THREE.Mesh(new THREE.SphereGeometry(0.013, 16, 16), link2Mat);
    elbowCorner.position.set(0, 0, a2z);
    frameGroups[1].add(elbowCorner);

    const link2Horiz = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, a2x, 16), link2Mat);
    link2Horiz.rotation.z = Math.PI / 2;
    link2Horiz.position.set(a2x * 0.5, 0, a2z);
    frameGroups[1].add(link2Horiz);

    // --- Body 3 (frameGroups[2]): Origin at Joint 3, rotated by Ry(q3). Extends along +X by L3 ---
    const j3Sphere = new THREE.Mesh(jointSphereGeo, jointMaterials[2]);
    frameGroups[2].add(j3Sphere);

    const link3Cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.012, l3, 18), link3Mat);
    link3Cyl.rotation.z = Math.PI / 2;
    link3Cyl.position.set(l3 * 0.5, 0, 0);
    frameGroups[2].add(link3Cyl);

    // --- Body 4 (frameGroups[3]): Origin at Joint 4, rotated by Ry(q4). Wrist + two-finger gripper at x = L4 ---
    const j4Sphere = new THREE.Mesh(jointSphereGeo, jointMaterials[3]);
    frameGroups[3].add(j4Sphere);

    const wristBodyLen = Math.max(0.02, l4 - 0.032);
    const link4Cyl = new THREE.Mesh(new THREE.CylinderGeometry(0.0095, 0.0105, wristBodyLen, 16), link4Mat);
    link4Cyl.rotation.z = Math.PI / 2;
    link4Cyl.position.set(wristBodyLen * 0.5, 0, 0);
    frameGroups[3].add(link4Cyl);

    // Two-finger gripper assembly around tool point x = l4
    const palmBar = new THREE.Mesh(new THREE.BoxGeometry(0.010, 0.046, 0.018), gripperMat);
    palmBar.position.set(l4 - 0.027, 0, 0);
    frameGroups[3].add(palmBar);

    const fingerGeo = new THREE.BoxGeometry(0.038, 0.005, 0.014);
    const leftFinger = new THREE.Mesh(fingerGeo, gripperMat);
    leftFinger.position.set(l4 - 0.005, 0.018, 0);
    frameGroups[3].add(leftFinger);

    const rightFinger = new THREE.Mesh(fingerGeo, gripperMat);
    rightFinger.position.set(l4 - 0.005, -0.018, 0);
    frameGroups[3].add(rightFinger);

    // Tool center point sphere (driven directly by toolPos from ComputeFrames)
    const toolMesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.006, 14, 14),
        new THREE.MeshBasicMaterial({ color: 0x0f172a })
    );
    scene.add(toolMesh);

    // Target marker (#16a34a)
    const targetGroup = createCrosshairGroup(0x16a34a);
    scene.add(targetGroup);

    // Detached PickPlace object (#ea580c) and attached payload (#ea580c)
    const payloadMat = new THREE.MeshStandardMaterial({ color: 0xea580c, roughness: 0.4, metalness: 0.1 });
    const objectMesh = new THREE.Mesh(new THREE.BoxGeometry(0.026, 0.026, 0.026), payloadMat);
    objectMesh.visible = false;
    scene.add(objectMesh);

    const attachedPayloadMesh = new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.024, 0.024), payloadMat);
    attachedPayloadMesh.visible = false;
    scene.add(attachedPayloadMesh);

    const renderNow = () => {
        renderer.render(scene, camera);
    };

    controls.addEventListener('change', renderNow);

    const onResize = () => {
        if (!state || !state.canvas) return;
        const w = state.canvas.clientWidth || 640;
        const h = state.canvas.clientHeight || 360;
        state.camera.aspect = w / h;
        state.camera.updateProjectionMatrix();
        state.renderer.setSize(w, h, false);
        renderNow();
    };
    window.addEventListener('resize', onResize);

    state = {
        canvas,
        renderer,
        scene,
        camera,
        controls,
        frameGroups,
        jointMaterials,
        jointColorCodes,
        toolMesh,
        targetGroup,
        objectMesh,
        attachedPayloadMesh,
        onResize,
        renderNow,
        updateCount: 0
    };

    renderNow();
    return true;
}

function codeToHex(code) {
    if (code === 2) return COLOR_RED;
    if (code === 1) return COLOR_AMBER;
    return COLOR_NORMAL;
}

/**
 * Single interop entry point per frame.
 * buf layout (62 doubles, zero kinematics in JS):
 *   [0..47]  4 frames x 12 values (m00, m01, m02, px, m10, m11, m12, py, m20, m21, m22, pz)
 *   [48..50] tool point (tx, ty, tz)
 *   [51..54] joint color state codes (0=normal, 1=amber, 2=red)
 *   [55..57] target position (X, Y, Z)
 *   [58..60] object position (X, Y, Z)
 *   [61]     visibility flags (bit 0 = detached object, bit 1 = attached payload)
 */
function updateScene(buf) {
    if (!state || !buf || buf.length < 62) {
        return;
    }

    for (let i = 0; i < 4; i++) {
        const base = i * 12;
        const fg = state.frameGroups[i];
        fg.matrix.set(
            buf[base + 0], buf[base + 1], buf[base + 2], buf[base + 3],
            buf[base + 4], buf[base + 5], buf[base + 6], buf[base + 7],
            buf[base + 8], buf[base + 9], buf[base + 10], buf[base + 11],
            0, 0, 0, 1
        );
        fg.matrixWorldNeedsUpdate = true;

        const cCode = buf[51 + i] | 0;
        if (state.jointColorCodes[i] !== cCode) {
            state.jointColorCodes[i] = cCode;
            state.jointMaterials[i].color.setHex(codeToHex(cCode));
        }
    }

    const tx = buf[48], ty = buf[49], tz = buf[50];
    state.toolMesh.position.set(tx, ty, tz);

    state.targetGroup.position.set(buf[55], buf[56], buf[57]);

    const flags = buf[61] | 0;
    const showDetachedObj = (flags & 1) !== 0;
    const showAttachedPayload = (flags & 2) !== 0;

    state.objectMesh.visible = showDetachedObj;
    if (showDetachedObj) {
        state.objectMesh.position.set(buf[58], buf[59], buf[60]);
    }

    state.attachedPayloadMesh.visible = showAttachedPayload;
    if (showAttachedPayload) {
        state.attachedPayloadMesh.position.set(tx, ty, tz);
    }

    state.updateCount++;
    state.renderer.render(state.scene, state.camera);
}

function resetView() {
    if (!state) return;
    state.camera.up.set(0, 0, 1);
    state.camera.position.copy(DEFAULT_CAM_POS);
    state.controls.target.copy(DEFAULT_CAM_TARGET);
    state.controls.update();
    state.renderer.render(state.scene, state.camera);
}

function getDiagnostics() {
    if (!state) return null;
    return {
        updateCount: state.updateCount,
        geometries: state.renderer.info.memory.geometries,
        textures: state.renderer.info.memory.textures,
        toolPosition: {
            x: state.toolMesh.position.x,
            y: state.toolMesh.position.y,
            z: state.toolMesh.position.z
        },
        frameOrigins: state.frameGroups.map(fg => ({
            x: fg.matrix.elements[12],
            y: fg.matrix.elements[13],
            z: fg.matrix.elements[14]
        }))
    };
}

function disposeScene() {
    if (!state) return;
    window.removeEventListener('resize', state.onResize);
    state.controls.dispose();
    state.renderer.dispose();
    state = null;
}

window.RobotArm3D = {
    initScene,
    updateScene,
    resetView,
    getDiagnostics,
    disposeScene
};
