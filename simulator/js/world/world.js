/*
 * world/world.js — 用 Three.js 把 layout.js 的数据画出来
 *
 * 世界坐标系用 Z 轴向上 (与固件文档里的地理坐标系一致: X 北 / Y 东 / Z 上),
 * 所以要先改掉 Three.js 默认的 Y-up 约定。
 *
 * 场景内容: 地形 + 起降场 + 城区高楼 + 跨河大桥 + 树林 + 河道 + 天空 + 光照
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});

    /* ---------------- 程序化贴图 ---------------- */
    function makeCanvas(w, h) {
        var c = document.createElement('canvas');
        c.width = w; c.height = h;
        return c;
    }

    /* 地面草地/土地贴图 */
    function groundTexture() {
        var c = makeCanvas(256, 256), g = c.getContext('2d');
        g.fillStyle = '#4a6b3a';
        g.fillRect(0, 0, 256, 256);
        var rng = QC.math.Mulberry32(4242);
        for (var i = 0; i < 5200; i++) {
            var x = rng() * 256, y = rng() * 256;
            var v = rng();
            g.fillStyle = v < 0.5 ? 'rgba(64,92,52,0.55)' :
                (v < 0.8 ? 'rgba(88,112,64,0.5)' : 'rgba(108,124,78,0.42)');
            g.fillRect(x, y, 1 + rng() * 2.5, 1 + rng() * 2.5);
        }
        var t = new THREE.CanvasTexture(c);
        t.wrapS = t.wrapT = THREE.RepeatWrapping;
        t.repeat.set(52, 52);
        t.anisotropy = 4;
        /* 颜色贴图必须标成 sRGB: 渲染器以 sRGB 输出, 若贴图被当成线性数据,
           颜色会被再"抬"一次, 整幅画面发白 */
        t.encoding = THREE.sRGBEncoding;
        return t;
    }

    /* 楼体窗格贴图: 3 种色调 + 1 种玻璃幕墙 */
    function buildingTexture(kind) {
        var c = makeCanvas(128, 128), g = c.getContext('2d');
        var base = ['#8d8f95', '#9a9189', '#7f8894', '#6d7f95'][kind] || '#8d8f95';
        g.fillStyle = base;
        g.fillRect(0, 0, 128, 128);
        var rng = QC.math.Mulberry32(1000 + kind * 77);
        var cols = 8, rows = 8;
        var cw = 128 / cols, ch = 128 / rows;
        for (var r = 0; r < rows; r++) {
            for (var cc = 0; cc < cols; cc++) {
                var lit = rng();
                g.fillStyle = lit < 0.22 ? 'rgba(255,238,190,0.78)' :
                    (lit < 0.5 ? 'rgba(60,74,92,0.85)' : 'rgba(96,112,132,0.7)');
                g.fillRect(cc * cw + cw * 0.18, r * ch + ch * 0.22, cw * 0.64, ch * 0.5);
            }
        }
        var bt = new THREE.CanvasTexture(c);
        bt.encoding = THREE.sRGBEncoding;
        return bt;
    }

    /* 天空渐变 */
    function skyTexture() {
        var c = makeCanvas(4, 256), g = c.getContext('2d');
        var grad = g.createLinearGradient(0, 0, 0, 256);
        grad.addColorStop(0.0, '#2e5f9e');
        grad.addColorStop(0.42, '#7fb2e0');
        grad.addColorStop(0.72, '#cfe2ee');
        grad.addColorStop(1.0, '#e8e2d2');
        g.fillStyle = grad;
        g.fillRect(0, 0, 4, 256);
        var t = new THREE.CanvasTexture(c);
        t.magFilter = THREE.LinearFilter;
        t.encoding = THREE.sRGBEncoding;
        return t;
    }

    /* ---------------- 机身模型 ---------------- */
    function buildDrone() {
        var g = new THREE.Group();

        var armMat = new THREE.MeshStandardMaterial({ color: 0x2f333a, roughness: 0.7 });
        var motorMat = new THREE.MeshStandardMaterial({ color: 0xb9bec6, roughness: 0.35, metalness: 0.8 });
        var propMat = new THREE.MeshStandardMaterial({
            color: 0x33b5e5, roughness: 0.45, metalness: 0.1,
            transparent: true, opacity: 0.6, side: THREE.DoubleSide
        });
        var bladeMat = new THREE.MeshBasicMaterial({
            color: 0xffffff, transparent: true, opacity: 0.45, side: THREE.DoubleSide
        });

        /* 机身 (X 前, Y 右, Z 上) */
        g.add(new THREE.Mesh(new THREE.BoxGeometry(0.052, 0.04, 0.014),
            new THREE.MeshStandardMaterial({ color: 0x23262b, roughness: 0.62, metalness: 0.25 })));

        /* 机头指示 */
        var nose = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.018, 0.011),
            new THREE.MeshStandardMaterial({ color: 0xef4444, roughness: 0.5 }));
        nose.position.set(0.031, 0, 0.002);
        g.add(nose);

        var L = QC.config.DRONE.armLength;
        var a = L / Math.SQRT2;
        var layout = QC.dynamics.MOTOR_LAYOUT;
        var keys = QC.dynamics.MOTOR_KEYS;
        var props = [];

        var armGeo = new THREE.BoxGeometry(1, 0.008, 0.005);
        var motorGeo = new THREE.CylinderGeometry(0.0055, 0.0055, 0.012, 10);
        motorGeo.rotateX(Math.PI / 2);   /* 圆柱轴转到 Z */
        var propGeo = new THREE.CircleGeometry(0.033, 20);
        var bladeGeo = new THREE.PlaneGeometry(0.062, 0.007);

        for (var i = 0; i < keys.length; i++) {
            var m = layout[keys[i]];
            var px = m.x * a, py = m.y * a;

            var arm = new THREE.Mesh(armGeo, armMat);
            arm.scale.x = L;
            arm.position.set(px * 0.5, py * 0.5, 0.001);
            arm.rotation.z = Math.atan2(py, px);
            g.add(arm);

            var motor = new THREE.Mesh(motorGeo, motorMat);
            motor.position.set(px, py, 0.008);
            g.add(motor);

            var prop = new THREE.Mesh(propGeo, propMat);
            prop.position.set(px, py, 0.0145);
            var blade = new THREE.Mesh(bladeGeo, bladeMat);
            blade.position.z = 0.001;
            prop.add(blade);
            g.add(prop);

            props.push({ mesh: prop, dir: m.c > 0 ? 1 : -1 });
        }

        /* 4 个 LED (固件里低电平点亮) */
        var ledMats = {};
        var ledPos = {
            leftTop: [+a, +a], leftBottom: [-a, +a], rightTop: [+a, -a], rightBottom: [-a, -a]
        };
        var ledGeo = new THREE.SphereGeometry(0.0065, 8, 6);
        Object.keys(ledPos).forEach(function (k) {
            ledMats[k] = new THREE.MeshBasicMaterial({ color: 0x241010 });
            var led = new THREE.Mesh(ledGeo, ledMats[k]);
            led.position.set(ledPos[k][0] * 0.6, ledPos[k][1] * 0.6, -0.009);
            g.add(led);
        });

        /* 起落架(沿机身方向的两条) */
        var skidGeo = new THREE.BoxGeometry(0.078, 0.012, 0.004);
        var skidMat = new THREE.MeshStandardMaterial({ color: 0x1b1d21, roughness: 0.85 });
        [-1, 1].forEach(function (s) {
            var skid = new THREE.Mesh(skidGeo, skidMat);
            skid.position.set(0, s * 0.03, -0.0125);
            g.add(skid);
        });

        return { group: g, props: props, ledMats: ledMats };
    }

    /* ---------------- 场景构建 ---------------- */
    function build() {
        THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

        var scene = new THREE.Scene();
        scene.fog = new THREE.Fog(0xc9dbe8, 300, 1000);

        var layout = QC.layout;
        var W = QC.config.WORLD;

        /* ---- 天空 ---- */
        var sky = new THREE.Mesh(
            new THREE.SphereGeometry(1700, 32, 16),
            new THREE.MeshBasicMaterial({
                map: skyTexture(), side: THREE.BackSide, fog: false, depthWrite: false
            })
        );
        scene.add(sky);

        /* ---- 光照 ---- */
        scene.add(new THREE.HemisphereLight(0xbfd6ee, 0x46503c, 0.42));
        var sun = new THREE.DirectionalLight(0xfff3dd, 0.92);
        sun.position.set(180, -150, 260);
        sun.castShadow = true;
        sun.shadow.mapSize.set(2048, 2048);
        var shc = sun.shadow.camera;
        shc.near = 1; shc.far = 950;
        shc.left = -300; shc.right = 300; shc.top = 300; shc.bottom = -300;
        scene.add(sun);
        scene.add(sun.target);

        /* ---- 地形 ----
           PlaneGeometry 默认躺在 XY 平面(z=0), 我们的世界就是 Z-up,
           所以直接把地形高度写进顶点的 z 分量即可, 不需要旋转。 */
        var seg = 120;
        var geo = new THREE.PlaneGeometry(W.size, W.size, seg, seg);
        var pos = geo.attributes.position;
        for (var i = 0; i < pos.count; i++) {
            pos.setZ(i, layout.terrainHeight(pos.getX(i), pos.getY(i)));
        }
        geo.computeVertexNormals();
        var groundMesh = new THREE.Mesh(geo, new THREE.MeshStandardMaterial({
            map: groundTexture(), roughness: 0.98, metalness: 0
        }));
        groundMesh.receiveShadow = true;
        scene.add(groundMesh);

        /* ---- 河道 ---- */
        var river = new THREE.Mesh(
            new THREE.PlaneGeometry(W.size, layout.RIVER.y1 - layout.RIVER.y0),
            new THREE.MeshStandardMaterial({
                color: 0x2f6c92, roughness: 0.2, metalness: 0.45,
                transparent: true, opacity: 0.9
            })
        );
        river.position.set(0, (layout.RIVER.y0 + layout.RIVER.y1) / 2, layout.RIVER.z);
        scene.add(river);

        /* ---- 楼房 ----
           按"高度档 x 贴图色调"分组做 InstancedMesh:
           不同档位用不同的贴图重复次数, 免得 130m 塔楼的窗户跟 15m 小楼一样大。 */
        var boxGeom = new THREE.BoxGeometry(1, 1, 1);
        var baseTex = [0, 1, 2, 3].map(buildingTexture);
        var buckets = [
            { max: 28, rows: 1 },
            { max: 62, rows: 2 },
            { max: 1e9, rows: 4 }
        ];
        var groups = [];
        buckets.forEach(function (bk, bi) {
            for (var t = 0; t < 4; t++) groups.push({ bucket: bi, rows: bk.rows, tint: t, list: [] });
        });

        layout.boxes.forEach(function (b, idx) {
            if (b.kind !== 'building' && b.kind !== 'landmark') return;
            var h = b.top - b.base;
            var bi = 0;
            while (bi < buckets.length - 1 && h > buckets[bi].max) bi++;
            var tint = layout.buildingTints[idx];
            groups[bi * 4 + tint].list.push(b);
        });

        var buildingGroup = new THREE.Group();
        groups.forEach(function (grp) {
            if (!grp.list.length) return;
            var tex = baseTex[grp.tint].clone();
            tex.needsUpdate = true;
            tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
            tex.repeat.set(1, grp.rows);
            var mat = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.72, metalness: 0.12 });
            var im = new THREE.InstancedMesh(boxGeom, mat, grp.list.length);
            im.castShadow = true;
            im.receiveShadow = true;
            var m = new THREE.Matrix4();
            grp.list.forEach(function (b, k) {
                m.makeScale(b.sizeX, b.sizeY, b.top - b.base);
                m.setPosition(b.cx, b.cy, (b.base + b.top) / 2);
                im.setMatrixAt(k, m);
            });
            im.instanceMatrix.needsUpdate = true;
            buildingGroup.add(im);
        });
        scene.add(buildingGroup);

        /* ---- 大桥 ---- */
        var boxMtx = new THREE.Matrix4();
        function instanceBoxes(list, material, castShadow) {
            var im = new THREE.InstancedMesh(boxGeom, material, list.length);
            im.castShadow = !!castShadow;
            im.receiveShadow = true;
            list.forEach(function (b, k) {
                boxMtx.makeScale(b.sizeX, b.sizeY, b.top - b.base);
                boxMtx.setPosition(b.cx, b.cy, (b.base + b.top) / 2);
                im.setMatrixAt(k, boxMtx);
            });
            im.instanceMatrix.needsUpdate = true;
            return im;
        }

        var decks = layout.boxes.filter(function (b) { return b.kind === 'deck'; });
        scene.add(instanceBoxes(decks, new THREE.MeshStandardMaterial({
            color: 0x6b6f76, roughness: 0.86
        }), true));

        var pylons = layout.boxes.filter(function (b) { return b.kind === 'pylon'; });
        scene.add(instanceBoxes(pylons, new THREE.MeshStandardMaterial({
            color: 0xa8adb5, roughness: 0.55, metalness: 0.25
        }), true));

        /* 主缆与吊索 */
        var BR = layout.BRIDGE;
        var halfW = BR.deckHalfWidth + 1.6;
        var spanY0 = BR.pylonY[0], spanY1 = BR.pylonY[1];
        var towerLift = BR.towerTopZ - BR.deckZ;
        var cablePts = [];

        function cableZ(y) {
            if (y < spanY0) {
                return BR.deckZ + towerLift * ((y - BR.yStart) / (spanY0 - BR.yStart));
            }
            if (y > spanY1) {
                return BR.deckZ + towerLift * ((BR.yEnd - y) / (BR.yEnd - spanY1));
            }
            var u = (y - spanY0) / (spanY1 - spanY0);
            return BR.deckZ + towerLift - 4 * u * (1 - u) * (towerLift * 0.82);
        }

        [-halfW, halfW].forEach(function (ox) {
            var prev = null;
            for (var t = 0; t <= 48; t++) {
                var y = BR.yStart + (BR.yEnd - BR.yStart) * (t / 48);
                var p = new THREE.Vector3(BR.x + ox, y, cableZ(y));
                if (prev) cablePts.push(prev.x, prev.y, prev.z, p.x, p.y, p.z);
                prev = p;
            }
        });
        for (var hy = spanY0 + 6; hy < spanY1 - 2; hy += 7.5) {
            [-halfW, halfW].forEach(function (ox) {
                cablePts.push(BR.x + ox, hy, cableZ(hy), BR.x + ox, hy, BR.deckZ);
            });
        }
        var cableGeo = new THREE.BufferGeometry();
        cableGeo.setAttribute('position', new THREE.Float32BufferAttribute(cablePts, 3));
        scene.add(new THREE.LineSegments(cableGeo,
            new THREE.LineBasicMaterial({ color: 0xdde3ea })));

        /* ---- 树林 ---- */
        var trunkGeo = new THREE.CylinderGeometry(0.28, 0.42, 1, 6);
        trunkGeo.rotateX(Math.PI / 2);
        trunkGeo.translate(0, 0, 0.5);
        var trunkIm = new THREE.InstancedMesh(trunkGeo,
            new THREE.MeshStandardMaterial({ color: 0x4a3625, roughness: 0.95 }), layout.trees.length);
        trunkIm.receiveShadow = true;

        var crownGeo = new THREE.ConeGeometry(1, 1, 7);
        crownGeo.rotateX(Math.PI / 2);
        crownGeo.translate(0, 0, 0.5);
        var crownIm = new THREE.InstancedMesh(crownGeo,
            new THREE.MeshStandardMaterial({ roughness: 0.92, vertexColors: true }),
            layout.trees.length);
        crownIm.receiveShadow = true;

        var crownColors = [0x2f6330, 0x396f38, 0x27542c, 0x437a3e];
        var m4 = new THREE.Matrix4();
        var col = new THREE.Color();
        layout.trees.forEach(function (t, k) {
            m4.makeScale(t.r * 0.75, t.r * 0.75, t.h * 0.36);
            m4.setPosition(t.x, t.y, t.z);
            trunkIm.setMatrixAt(k, m4);

            m4.makeScale(t.r * 2.4, t.r * 2.4, t.h * 0.76);
            m4.setPosition(t.x, t.y, t.z + t.h * 0.4);
            crownIm.setMatrixAt(k, m4);
            col.setHex(crownColors[t.tint % crownColors.length]);
            crownIm.setColorAt(k, col);
        });
        trunkIm.instanceMatrix.needsUpdate = true;
        crownIm.instanceMatrix.needsUpdate = true;
        if (crownIm.instanceColor) crownIm.instanceColor.needsUpdate = true;
        scene.add(trunkIm);
        scene.add(crownIm);

        /* ---- 起降场 ---- */
        var PAD = layout.PAD;
        var pad = new THREE.Mesh(new THREE.CircleGeometry(PAD.r, 44),
            new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.92 }));
        pad.position.set(PAD.x, PAD.y, PAD.z + 0.03);
        pad.receiveShadow = true;
        scene.add(pad);

        var ring = new THREE.Mesh(new THREE.RingGeometry(PAD.r * 0.74, PAD.r * 0.88, 44),
            new THREE.MeshBasicMaterial({ color: 0xf2c14e, side: THREE.DoubleSide }));
        ring.position.set(PAD.x, PAD.y, PAD.z + 0.04);
        scene.add(ring);

        var hMat = new THREE.MeshBasicMaterial({ color: 0xf2c14e, side: THREE.DoubleSide });
        [-1, 1].forEach(function (s) {
            var bar = new THREE.Mesh(new THREE.PlaneGeometry(0.75, 4.4), hMat);
            bar.position.set(PAD.x + s * 1.6, PAD.y, PAD.z + 0.045);
            scene.add(bar);
        });
        var cross = new THREE.Mesh(new THREE.PlaneGeometry(2.4, 0.75), hMat);
        cross.position.set(PAD.x, PAD.y, PAD.z + 0.045);
        scene.add(cross);

        /* ---- 遥控器位置标记 ---- */
        var remoteMarker = new THREE.Group();
        remoteMarker.add(new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.32, 0.16),
            new THREE.MeshStandardMaterial({ color: 0x36393f, roughness: 0.7 })));
        var ant = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.014, 0.52, 6),
            new THREE.MeshStandardMaterial({ color: 0x8a9099, metalness: 0.6 }));
        ant.position.set(0, 0.17, 0.3);
        remoteMarker.add(ant);
        remoteMarker.position.set(6, -6, 1.76);
        scene.add(remoteMarker);

        /* ---- 飞机 ---- */
        var drone = buildDrone();
        scene.add(drone.group);

        return {
            scene: scene,
            drone: drone,
            sun: sun,
            groundMesh: groundMesh,
            remoteMarker: remoteMarker
        };
    }

    QC.world = { build: build, buildDrone: buildDrone };
})(typeof globalThis !== 'undefined' ? globalThis : this);
