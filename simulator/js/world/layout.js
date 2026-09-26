/*
 * world/layout.js — 场景布局数据 (纯数据, 不依赖渲染库)
 *
 * 这份数据同时被三处使用:
 *   1. world/world.js  用 Three.js 把它画出来
 *   2. sim/dynamics.js 用它做碰撞与着陆判定
 *   3. sim/sensors.js  用它做 VL53L1X 的测距目标
 * 三者共用同一份数据与同一个高度函数, 保证"看到的"和"飞到的"完全一致。
 *
 * 地形高度是解析函数(平滑正弦叠加), 地面网格按同一个函数生成顶点高度。
 */
(function (root) {
    'use strict';

    var QC = (root.QC = root.QC || {});
    var M = QC.math;

    /* ---------------- 地形 ---------------- */
    /* 中间平坦(起降场), 西南方向缓慢抬升成丘陵, 树林长在坡上 */
    function terrainHeight(x, y) {
        var h = 0;
        /* 西南方向的主坡 */
        var r = Math.sqrt(x * x + y * y);
        var dir = Math.atan2(-y, -x); /* 指向西南 */
        var slope = M.clamp((r - 90) / 240, 0, 1);
        h += slope * 22 * (0.55 + 0.45 * Math.cos(dir - Math.PI * 0.75));
        /* 几道缓和的起伏, 让地形不那么呆板 */
        h += 2.6 * Math.sin(x / 78) * Math.cos(y / 96);
        h += 1.4 * Math.sin(x / 31 + 1.7) * Math.sin(y / 37 - 0.6);
        /* 整体抬高一点, 避免远处地形落到水平面以下 */
        h += 1.1;
        /* 河谷: 把 y 在河道内的地形压低 */
        var rv = 1 - M.clamp(Math.abs(y + 236) / 46, 0, 1);
        h -= rv * rv * 7.5;
        /* 起降场周围压平 */
        var pad = 1 - M.clamp(r / 34, 0, 1);
        h *= (1 - pad * pad);
        return h;
    }

    /* ---------------- 碰撞体 / 可着陆表面 ---------------- */
    /* 都是轴对齐长方体: minX/maxX/minY/maxY 是水平范围, base/top 是下上表面高度 */
    function makeBox(name, cx, cy, sizeX, sizeY, base, top, kind) {
        return {
            name: name, kind: kind || 'building',
            minX: cx - sizeX / 2, maxX: cx + sizeX / 2,
            minY: cy - sizeY / 2, maxY: cy + sizeY / 2,
            base: base, top: top,
            cx: cx, cy: cy, sizeX: sizeX, sizeY: sizeY
        };
    }

    /* ---------------- 生成城区 ---------------- */
    var rng = M.Mulberry32(0xC0FFEE);
    var boxes = [];
    var buildingTints = [];

    /* 城区: 东北方向, 带路网的方块阵列 */
    var CITY = { x0: 46, x1: 244, y0: -148, y1: 62, cell: 33, road: 9 };
    for (var bx = CITY.x0; bx < CITY.x1; bx += CITY.cell) {
        for (var by = CITY.y0; by < CITY.y1; by += CITY.cell) {
            var cxCell = bx + CITY.cell / 2;
            var cyCell = by + CITY.cell / 2;

            /* 留出两条主路 */
            if (Math.abs(cyCell - (-44)) < 7) continue;
            if (Math.abs(cxCell - 146) < 7) continue;

            if (rng() < 0.12) continue; /* 空地/广场 */

            var w = CITY.cell - CITY.road - rng() * 3;
            var d = CITY.cell - CITY.road - rng() * 3;
            var distFromCenter = Math.sqrt(Math.pow(cxCell - 145, 2) + Math.pow(cyCell + 43, 2));
            /* 越靠市中心越高 */
            var t = M.clamp(1 - distFromCenter / 130, 0, 1);
            var h = 12 + t * t * 74 + rng() * 16;

            boxes.push(makeBox('楼', cxCell + (rng() - 0.5) * 3, cyCell + (rng() - 0.5) * 3,
                w, d, terrainHeight(cxCell, cyCell), terrainHeight(cxCell, cyCell) + h));
            buildingTints.push(Math.floor(rng() * 3));
        }
    }

    /* 3 座地标塔楼 */
    var landmarks = [
        { x: 150, y: -30, w: 24, d: 24, h: 132 },
        { x: 190, y: -84, w: 20, d: 20, h: 108 },
        { x: 108, y: -8, w: 18, d: 22, h: 96 }
    ];
    for (var i = 0; i < landmarks.length; i++) {
        var L = landmarks[i];
        boxes.push(makeBox('地标塔楼', L.x, L.y, L.w, L.d, terrainHeight(L.x, L.y),
            terrainHeight(L.x, L.y) + L.h, 'landmark'));
        buildingTints.push(3);
    }

    /* ---------------- 大桥 ---------------- */
    /* 河道在 y ≈ -236, 桥沿南北方向跨过去 */
    var BRIDGE = {
        x: 0,
        deckZ: 13.5,       /* 桥面高度 */
        deckHalfWidth: 8,
        yStart: -292,
        yEnd: -180,
        pylonY: [-272, -200],
        pylonHeight: 52,
        pylonHalf: 2.4,
        towers: []
    };

    /* 桥面: 用若干段长方体拼成(同时也成为可着陆面) */
    var segLen = 8;
    for (var y = BRIDGE.yStart; y < BRIDGE.yEnd; y += segLen) {
        var segY = Math.min(y + segLen / 2, BRIDGE.yEnd);
        boxes.push(makeBox('桥面', BRIDGE.x, segY,
            BRIDGE.deckHalfWidth * 2, segLen * 1.02,
            BRIDGE.deckZ - 1.2, BRIDGE.deckZ, 'deck'));
    }

    /* 桥塔 */
    for (i = 0; i < BRIDGE.pylonY.length; i++) {
        var py = BRIDGE.pylonY[i];
        /* 每个桥塔两根立柱, 分列桥面两侧 */
        for (var sgn = -1; sgn <= 1; sgn += 2) {
            var px = BRIDGE.x + sgn * (BRIDGE.deckHalfWidth + 1.6);
            var base = terrainHeight(px, py);
            boxes.push(makeBox('桥塔', px, py, BRIDGE.pylonHalf * 2, BRIDGE.pylonHalf * 2,
                base, base + BRIDGE.pylonHeight, 'pylon'));
            BRIDGE.towers.push({ x: px, y: py, base: base, top: base + BRIDGE.pylonHeight });
        }
    }
    BRIDGE.towerTopZ = terrainHeight(BRIDGE.x, BRIDGE.pylonY[0]) + BRIDGE.pylonHeight;

    /* ---------------- 树林 ---------------- */
    var trees = [];
    var FOREST = { x0: -300, x1: -52, y0: -250, y1: 130, count: 520 };
    var trng = M.Mulberry32(0x5EED11);
    for (i = 0; i < FOREST.count; i++) {
        var tx = FOREST.x0 + trng() * (FOREST.x1 - FOREST.x0);
        var ty = FOREST.y0 + trng() * (FOREST.y1 - FOREST.y0);
        /* 河道里不长树 */
        if (ty > -262 && ty < -210) continue;
        var ground = terrainHeight(tx, ty);
        /* 海拔越高树越矮一点, 顺便做出高低错落 */
        var hgt = 5.5 + trng() * 7.5 - ground * 0.05;
        if (hgt < 3.5) hgt = 3.5;
        trees.push({
            x: tx, y: ty, z: ground,
            h: hgt,
            r: 1.1 + trng() * 1.5,
            tint: Math.floor(trng() * 4)
        });
    }
    /* 城区里点缀几棵行道树 */
    for (i = 0; i < 70; i++) {
        var gx = CITY.x0 + trng() * (CITY.x1 - CITY.x0);
        var gy = CITY.y0 + trng() * (CITY.y1 - CITY.y0);
        var inside = false;
        for (var b = 0; b < boxes.length; b++) {
            var B = boxes[b];
            if (gx > B.minX - 2 && gx < B.maxX + 2 && gy > B.minY - 2 && gy < B.maxY + 2) {
                inside = true;
                break;
            }
        }
        if (inside) continue;
        trees.push({
            x: gx, y: gy, z: terrainHeight(gx, gy),
            h: 4.5 + trng() * 4, r: 1.0 + trng() * 1.0, tint: Math.floor(trng() * 4)
        });
    }

    /* ---------------- 河道 ---------------- */
    var RIVER = { y0: -262, y1: -210, z: -3.2 };

    /* ---------------- 起降场 ---------------- */
    var PAD = { x: 0, y: 0, r: 7, z: terrainHeight(0, 0) };

    /* ---------------- 对外接口 ---------------- */

    /**
     * 某个位置"下方"最高的可支撑表面高度
     * @param {number} z 当前高度, 只考虑顶面不高于 z 的物体
     */
    function heightAt(x, y, z) {
        var h = terrainHeight(x, y);
        var limit = (z === undefined) ? Infinity : z + 0.06;
        for (var i = 0; i < boxes.length; i++) {
            var B = boxes[i];
            if (x >= B.minX && x <= B.maxX && y >= B.minY && y <= B.maxY) {
                if (B.top <= limit && B.top > h) h = B.top;
            }
        }
        return h;
    }

    /** 点是否落在某个建筑/桥塔内部 (撞上就坠机) */
    function hitTest(x, y, z) {
        for (var i = 0; i < boxes.length; i++) {
            var B = boxes[i];
            if (B.kind === 'deck') continue;      /* 桥面可以落在上面 */
            if (x >= B.minX && x <= B.maxX && y >= B.minY && y <= B.maxY &&
                z >= B.base && z <= B.top) {
                return B;
            }
        }
        return null;
    }

    QC.layout = {
        terrainHeight: terrainHeight,
        heightAt: heightAt,
        hitTest: hitTest,
        boxes: boxes,
        buildingTints: buildingTints,
        trees: trees,
        BRIDGE: BRIDGE,
        RIVER: RIVER,
        PAD: PAD,
        CITY: CITY,
        FOREST: FOREST
    };
})(typeof globalThis !== 'undefined' ? globalThis : this);
